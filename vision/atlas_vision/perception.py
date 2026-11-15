"""Live perception: onboard-camera frames of every AMR → one shared YOLO26n model → detections.

Pipeline per robot (an asyncio task each; one frame in flight per robot):
  1. capture: the camera render worker (camera.html, LIVE mode) renders that robot's onboard camera
     from the newest backend world state it received from the Gateway; the frame carries the
     state's simulation time and sequence (refused when the state is not online);
  2. inference: the shared model (one instance, one inference at a time, in a worker thread so the
     service loop never blocks); a frame that waited longer than ``max_frame_age_s`` for the model
     is dropped as stale instead of queued;
  3. delivery: browser clients subscribed to that robot receive the processed frame (JPEG) and its
     detections in one message, so a box can never be drawn on another frame; ROS topics
     <robot ns>/camera/image_raw, camera/camera_info and perception/detections when rclpy is available.
The rate per robot is capped (``max_inference_fps_per_robot``) and falls by itself when capture or
inference take longer (sequential loop). The lite engine and Nav2 never wait on any of this.
"""
import asyncio
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import struct
import time
import cv2
import numpy as np
from fastapi import WebSocket, WebSocketDisconnect
from .classes import names as class_names
from .models import model_identity, resolve_checkpoint, resolve_device
from .render_worker import RenderChannel, RenderError


def robot_namespace(robot_id, root='amr_01'):
    """Same convention as amr_lite_sim/fleet.py (reference robot on the root topics)."""
    return '' if robot_id == root else robot_id


class Detector:
    """The one model instance shared by every robot."""

    def __init__(self, path, device='auto', image_size=640, confidence=.35, iou=.7, threads=None):
        import torch
        from ultralytics import YOLO
        self.device = resolve_device(device)
        if self.device == 'cpu' and threads:
            torch.set_num_threads(int(threads))
        self.image_size, self.confidence, self.iou = image_size, confidence, iou
        self.load(path)

    def load(self, path):
        from ultralytics import YOLO
        checkpoint = resolve_checkpoint(path)
        model = YOLO(str(checkpoint))
        names = [model.names[i] for i in sorted(model.names)]
        if names != class_names():
            raise ValueError(f'checkpoint classes {names} differ from configs/dataset.yaml {class_names()}')
        model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=self.image_size, device=self.device, verbose=False)
        self.model, self.names, self.identity = model, names, model_identity(checkpoint)

    def predict(self, rgb):
        """Detections in the frame's own pixel coordinates (xyxy), and the timing in ms."""
        started = time.perf_counter()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)   # Ultralytics treats numpy arrays as BGR
        result = self.model.predict(bgr, imgsz=self.image_size, conf=self.confidence, iou=self.iou,
                                    device=self.device, verbose=False)[0]
        detections = [{'class_id': int(c), 'label': self.names[int(c)], 'confidence': round(float(s), 4),
                       'bbox_xyxy': [round(float(v), 1) for v in b]}
                      for c, s, b in zip(result.boxes.cls, result.boxes.conf, result.boxes.xyxy)]
        speed = {k: round(v, 2) for k, v in result.speed.items()}
        speed['total'] = round((time.perf_counter()-started)*1000, 2)
        return detections, speed


class RosOutput:
    """Optional ROS publication (Image, CameraInfo, Detection2DArray) per robot namespace."""

    def __init__(self, robots):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        from vision_msgs.msg import Detection2DArray
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('atlas_perception')
        self.publishers = {}
        for robot in robots:
            ns = robot_namespace(robot)
            base = f'/{ns}/' if ns else '/'
            self.publishers[robot] = (self.node.create_publisher(Image, base+'camera/image_raw', qos_profile_sensor_data),
                                      self.node.create_publisher(CameraInfo, base+'camera/camera_info', qos_profile_sensor_data),
                                      self.node.create_publisher(Detection2DArray, base+'perception/detections', 10))

    def publish(self, robot, frame, detections):
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import CameraInfo, Image
        from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose
        image_pub, info_pub, det_pub = self.publishers[robot]
        sec, nsec = divmod(int(round(frame.sim_time*1e9)), 1_000_000_000)
        stamp = Time(sec=sec, nanosec=nsec)
        ns = robot_namespace(robot)
        frame_id = (f'{ns}/' if ns else '')+'camera_optical_frame'
        h, w = frame.rgb.shape[:2]
        image = Image(height=h, width=w, encoding='rgb8', is_bigendian=0, step=w*3, data=frame.rgb.tobytes())
        image.header.stamp, image.header.frame_id = stamp, frame_id
        fx = (w/2)/math.tan(frame.header['camera']['horizontal_fov_rad']/2)   # square pixels
        info = CameraInfo(height=h, width=w, distortion_model='plumb_bob', d=[0.]*5,
                          k=[fx, 0., w/2, 0., fx, h/2, 0., 0., 1.], r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                          p=[fx, 0., w/2, 0., 0., fx, h/2, 0., 0., 0., 1., 0.])
        info.header = image.header
        array = Detection2DArray()
        array.header = image.header
        for d in detections:
            x0, y0, x1, y1 = d['bbox_xyxy']
            det = Detection2D()
            det.header = image.header
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x, det.bbox.center.position.y = (x0+x1)/2, (y0+y1)/2
            det.bbox.size_x, det.bbox.size_y = x1-x0, y1-y0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = d['label'], d['confidence']
            det.results.append(hyp)
            array.detections.append(det)
        image_pub.publish(image)
        info_pub.publish(info)
        det_pub.publish(array)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class RobotStats:
    def __init__(self):
        self.frames = self.dropped_stale = self.capture_errors = 0
        self.last_error = None
        self.last_published = None
        self.times = deque(maxlen=60)           # publication wall times (rate)
        self.latency = deque(maxlen=60)         # capture request → published (ms)
        self.state_age = deque(maxlen=60)       # Gateway world-state send time → published (ms)
        self.inference = deque(maxlen=60)
        self.objects = 0

    def snapshot(self):
        rate = (len(self.times)-1)/(self.times[-1]-self.times[0]) if len(self.times) > 1 and self.times[-1] > self.times[0] else 0.
        pct = lambda values, q: round(float(np.percentile(values, q)), 1) if values else None
        return {'frames': self.frames, 'dropped_stale': self.dropped_stale, 'capture_errors': self.capture_errors,
                'last_error': self.last_error, 'fps': round(rate, 2), 'latency_ms_p50': pct(self.latency, 50),
                'latency_ms_p95': pct(self.latency, 95), 'state_to_detection_ms_p50': pct(self.state_age, 50),
                'inference_ms_p50': pct(self.inference, 50), 'objects': self.objects,
                'age_s': round(time.monotonic()-self.last_published, 2) if self.last_published else None}


class Perception:
    def __init__(self, config, robots):
        self.config, self.robots = config, list(robots)
        self.channel = RenderChannel()
        self.detector = Detector(config['model_path'], config['device'], config['image_size'], config['confidence_threshold'],
                                 config['iou_threshold'], config.get('torch_threads'))
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='yolo')
        self.encoder = ThreadPoolExecutor(max_workers=1, thread_name_prefix='jpeg')
        self.model_lock = asyncio.Lock()
        self.enabled = bool(config.get('enabled', True))
        self.fps = float(config['max_inference_fps_per_robot'])
        self.clients = {r: set() for r in self.robots}
        self.latest = {}
        self.stats = {r: RobotStats() for r in self.robots}
        self.counters = {r: 0 for r in self.robots}
        self.ros = None
        if config.get('ros', {}).get('enabled'):
            try:
                self.ros = RosOutput(self.robots)
            except ImportError as error:   # ROS not sourced: the WebSocket output still works
                self.ros_error = str(error)
        self.tasks = []

    def start(self):
        self.tasks = [asyncio.create_task(self.robot_loop(r), name=f'perception-{r}') for r in self.robots]

    async def stop(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.executor.shutdown(wait=False)
        self.encoder.shutdown(wait=False)
        if self.ros:
            self.ros.close()

    async def robot_loop(self, robot):
        loop = asyncio.get_running_loop()
        stats = self.stats[robot]
        width, height = self.config['capture']['width'], self.config['capture']['height']
        while True:
            started = time.monotonic()
            if not self.enabled or not self.channel.connected:
                await asyncio.sleep(.3)
                continue
            try:
                frame = await self.channel.request({'type': 'capture', 'robot_id': robot, 'width': width, 'height': height,
                                                    'night': False}, timeout=5.)
            except (RenderError, asyncio.TimeoutError) as error:
                stats.capture_errors += 1
                stats.last_error = str(error)[:200]
                await asyncio.sleep(1.)
                continue
            captured = time.monotonic()
            async with self.model_lock:
                if time.monotonic()-captured > self.config['max_frame_age_s']:
                    stats.dropped_stale += 1
                    continue
                detections, speed = await loop.run_in_executor(self.executor, self.detector.predict, frame.rgb)
            jpeg = await loop.run_in_executor(self.encoder, self.encode, frame.rgb)
            self.counters[robot] += 1
            published_wall = time.time()
            header = {'type': 'detections', 'robot_id': robot, 'frame_id': self.counters[robot], 'stamp': frame.sim_time,
                      'world_sequence': frame.header.get('world_sequence'), 'engine_session': frame.header.get('engine_session'),
                      'width': frame.rgb.shape[1], 'height': frame.rgb.shape[0], 'model': self.detector.identity['id'],
                      'model_name': 'YOLO26n', 'confidence_threshold': self.detector.confidence, 'device': self.detector.device,
                      'detections': detections,
                      'timing_ms': {'render': round(frame.header['render_ms'], 1), 'capture': round((captured-started)*1000, 1),
                                    **{f'model_{k}': v for k, v in speed.items()},
                                    'end_to_end': round((time.monotonic()-started)*1000, 1)},
                      'state_age_ms': round(published_wall*1000-frame.header['server_wall_time']*1000, 1)
                      if frame.header.get('server_wall_time') else None}
            stats.frames += 1
            stats.last_error = None
            stats.last_published = time.monotonic()
            stats.times.append(stats.last_published)
            stats.latency.append(header['timing_ms']['end_to_end'])
            stats.inference.append(speed['total'])
            if header['state_age_ms'] is not None:
                stats.state_age.append(header['state_age_ms'])
            stats.objects = len(detections)
            header['fps'] = stats.snapshot()['fps']
            message = self.pack(header, jpeg)
            self.latest[robot] = message
            for client in list(self.clients[robot]):
                if client.full():
                    client.get_nowait()        # a slow browser only ever gets the newest frame
                client.put_nowait(message)
            if self.ros:
                try:
                    self.ros.publish(robot, frame, detections)
                except Exception as error:     # ROS output must never stop perception
                    stats.last_error = f'ROS publish: {error}'[:200]
            await asyncio.sleep(max(0., 1./self.fps-(time.monotonic()-started)))

    def encode(self, rgb):
        ok, data = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, self.config['jpeg_quality']])
        return data.tobytes()

    @staticmethod
    def pack(header, jpeg):
        head = json.dumps(header).encode()
        return struct.pack('<I', len(head))+head+jpeg

    async def serve_client(self, socket: WebSocket, robot):
        origin = socket.headers.get('origin')
        # The service also serves the built web app (render_worker.mount_worker), so a page loaded
        # from it (same origin as the Host it connects to) is accepted besides the configured ones.
        same_origin = origin == f"http://{socket.headers.get('host')}"
        if origin is not None and not same_origin and origin not in self.config['allowed_origins']:
            await socket.close(code=1008)
            return
        if robot not in self.clients:
            await socket.close(code=1008, reason=f'unknown robot {robot}')
            return
        queue = asyncio.Queue(maxsize=1)
        await socket.accept()
        self.clients[robot].add(queue)
        if robot in self.latest:
            queue.put_nowait(self.latest[robot])
        try:
            async def send():
                while True:
                    await asyncio.wait_for(socket.send_bytes(await queue.get()), 5.)

            async def receive():
                while (await socket.receive())['type'] != 'websocket.disconnect':
                    pass
            done, pending = await asyncio.wait([asyncio.create_task(send()), asyncio.create_task(receive())],
                                               return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
        except (WebSocketDisconnect, asyncio.TimeoutError, OSError):
            pass
        finally:
            self.clients[robot].discard(queue)

    def status(self):
        return {'enabled': self.enabled, 'model': self.detector.identity, 'model_name': 'YOLO26n', 'device': self.detector.device,
                'confidence_threshold': self.detector.confidence, 'max_inference_fps_per_robot': self.fps,
                'image_size': self.detector.image_size, 'classes': self.detector.names,
                'worker': self.channel.hello if self.channel.connected else None,
                'worker_disconnect_reason': self.channel.disconnect_reason,
                'ros_output': self.ros is not None, 'ros_error': getattr(self, 'ros_error', None), 'robots': {r: s.snapshot() for r, s in self.stats.items()},
                'clients': {r: len(c) for r, c in self.clients.items()}}
