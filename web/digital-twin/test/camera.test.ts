import { describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import { staticInstances } from '../src/camera/annotation';
import type { CampusScene } from '../src/atlas/scene';

const scene = JSON.parse(readFileSync(new URL('../public/scene/atlas_campus.scene.json', import.meta.url), 'utf8')) as CampusScene;

describe('camera ground truth: labelled static objects', () => {
  const labels = staticInstances(scene);
  const cargo = scene.layers.cargo!.cube!;
  it('labels every floor pallet with its base and load, and every box on the inspection tables', () => {
    const pallets = labels.filter(l => l.kind === 'pallet'), boxes = labels.filter(l => l.kind === 'box');
    expect(pallets).toHaveLength(scene.solids.filter(s => s.tag === 'Palete no chão').length);
    // Floor pallet (atlas_campus.mjs): wooden base + three load boxes.
    for (const p of pallets) expect(p.instances).toHaveLength(4);
    expect(boxes).toHaveLength(3 * scene.solids.filter(s => s.tag === 'Mesa de inspeção').length);
    for (const b of boxes) expect(cargo[b.instances[0]! * 13 + 2]).toBeCloseTo(.99, 2);   // table-top boxes
  });
  it('never labels one instance twice', () => {
    const all = labels.flatMap(l => l.instances);
    expect(new Set(all).size).toBe(all.length);
  });
});

import { frameTransform, parseFrame } from '../src/atlas/vision/VisionView';

describe('vision view: processed frames and their detections', () => {
  const pack = (header: object, jpeg = new Uint8Array([0xff, 0xd8, 0xff, 0xd9])) => {
    const head = new TextEncoder().encode(JSON.stringify(header)), out = new Uint8Array(4 + head.length + jpeg.length);
    new DataView(out.buffer).setUint32(0, head.length, true); out.set(head, 4); out.set(jpeg, 4 + head.length);
    return out.buffer;
  };
  const header = { type: 'detections', robot_id: 'amr_02', frame_id: 7, stamp: 12.5, width: 640, height: 480,
    detections: [{ class_id: 0, label: 'person', confidence: .9, bbox_xyxy: [10, 20, 110, 220] }] };
  it('keeps the image and its detections together', async () => {
    const { header: h, jpeg } = parseFrame(pack(header));
    expect(h.robot_id).toBe('amr_02'); expect(h.detections[0]!.bbox_xyxy).toEqual([10, 20, 110, 220]);
    expect(jpeg.size).toBe(4); expect(jpeg.type).toBe('image/jpeg');
  });
  it('rejects malformed frames', () => {
    expect(() => parseFrame(pack({ ...header, detections: [{ bbox_xyxy: [1, 2, 3] }] }))).toThrow();
    expect(() => parseFrame(pack({ ...header, width: 0 }))).toThrow();
    expect(() => parseFrame(pack({ ...header, type: 'world_state' }))).toThrow();
  });
  it('scales boxes with the image into any viewport (aligned after a resize)', () => {
    for (const viewport of [{ x: 100, y: 50, width: 800, height: 600 }, { x: 0, y: 0, width: 400, height: 500 }, { x: 10, y: 10, width: 1000, height: 300 }]) {
      const t = frameTransform(viewport, 640, 480);
      // the whole image fits and keeps its aspect ratio; a pixel maps to the same relative place
      expect(640 * t.scale).toBeLessThanOrEqual(viewport.width + 1e-9); expect(480 * t.scale).toBeLessThanOrEqual(viewport.height + 1e-9);
      expect(Math.max(640 * t.scale / viewport.width, 480 * t.scale / viewport.height)).toBeCloseTo(1, 9);
      const corner = [t.x + 640 * t.scale, t.y + 480 * t.scale];
      expect(corner[0]! - viewport.x).toBeCloseTo(viewport.width - (t.x - viewport.x), 6);
      expect(corner[1]! - viewport.y).toBeCloseTo(viewport.height - (t.y - viewport.y), 6);
    }
  });
});
