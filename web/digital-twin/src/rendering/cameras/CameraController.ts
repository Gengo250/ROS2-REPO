import { PerspectiveCamera, Vector3 } from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export class CameraController {
  readonly controls: OrbitControls;
  constructor(readonly camera: PerspectiveCamera, canvas: HTMLCanvasElement, readonly span: number, readonly robot: number[]) {
    camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(camera, canvas);
    this.controls.enableDamping = true;
    this.controls.screenSpacePanning = false;
    this.controls.minDistance = .6;
    this.controls.maxDistance = span * 2;
    this.controls.maxTargetRadius = span * .6;
    this.controls.maxPolarAngle = Math.PI / 2 - .025;
    this.view('overview');
  }
  view(name: string) {
    // Flush orbit/pan inertia before applying an exact preset. Otherwise a
    // reset drifts away from the configured view on subsequent frames.
    this.controls.enableDamping = false;
    this.controls.reset();
    const target = name === 'robot' ? new Vector3(this.robot[0], this.robot[1], this.robot[2]! + .3) : new Vector3(0, 0, 0);
    this.controls.target.copy(target);
    this.camera.position.copy(target).add(name === 'robot' ? new Vector3(1.6, -1.7, 1.5)
      : name === 'top' ? new Vector3(0, -.001, this.span * 1.1)
      : new Vector3(this.span * .65, -this.span * .8, this.span * .85));
    this.controls.update();
    this.controls.enableDamping = true;
  }
  dispose() { this.controls.dispose(); }
}
