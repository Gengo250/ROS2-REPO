"""Unit tests of the vision pipeline pieces that need neither a browser nor a trained model."""
import json
import re
import struct
import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'scripts'))
from atlas_vision import annotate, classes, frames, matching  # noqa: E402


def make_frame(ids, instances, width=None, height=None):
    h, w = ids.shape
    header = {'job_id': 1, 'robot_id': 'amr_01', 'sim_time': 3.5, 'width': width or w, 'height': height or h,
              'has_ids': True, 'instances': instances}
    rgba = np.zeros((h, w, 4), np.uint8)
    codes = np.zeros((h, w, 4), np.uint8)
    codes[..., 0], codes[..., 1], codes[..., 2] = ids & 255, (ids >> 8) & 255, (ids >> 16) & 255
    head = json.dumps(header).encode()
    # The worker sends rows bottom-up (WebGL order).
    return struct.pack('<I', len(head))+head+rgba[::-1].tobytes()+codes[::-1].tobytes()


def test_frame_rows_and_codes_are_restored():
    ids = np.zeros((6, 8), np.int32)
    ids[0, 0] = 300            # top-left pixel, code above one byte
    frame = frames.parse(make_frame(ids, [{'code': 300, 'id': 'COL-01', 'kind': 'person'}]))
    assert frame.ids[0, 0] == 300 and frame.ids.sum() == 300 and frame.rgb.shape == (6, 8, 3)
    with pytest.raises(ValueError):
        frames.parse(make_frame(ids, [])[:-1])


def test_boxes_are_the_visible_pixels_and_small_ones_are_ignored():
    ids = np.zeros((48, 64), np.int32)
    ids[10:30, 5:15] = 1        # person, 200 px
    ids[20:30, 12:15] = 2       # forklift in front of the person's lower right corner
    ids[40:48, 60:64] = 3       # cone cut by the image border, 32 px
    ids[0, 0] = 4               # one pixel of a far box
    instances = [{'code': 1, 'id': 'COL-01', 'kind': 'person'}, {'code': 2, 'id': 'EMP-1', 'kind': 'forklift'},
                 {'code': 3, 'id': 'obstacle_0001', 'kind': 'cone'}, {'code': 4, 'id': 'obstacle_0002', 'kind': 'box'}]
    frame = frames.parse(make_frame(ids, instances))
    labelled, ignored = annotate.boxes(frame, min_pixels=25, min_box_side=3)
    by_id = {o['id']: o for o in labelled}
    assert by_id['COL-01']['bbox_xyxy'] == [5, 10, 15, 30] and by_id['COL-01']['pixels'] == 200-30   # 10 x 3 px hidden behind the forklift
    assert by_id['EMP-1']['bbox_xyxy'] == [12, 20, 15, 30]
    assert by_id['obstacle_0001']['truncated'] and [o['id'] for o in ignored] == ['obstacle_0002']
    lines = annotate.yolo_lines(labelled, 64, 48)
    person = lines[[o['id'] for o in labelled].index('COL-01')].split()
    assert person[0] == str(classes.class_id('person'))
    assert [float(v) for v in person[1:]] == pytest.approx([10/64, 20/48, 10/64, 20/48])


def test_one_class_mapping_everywhere():
    names = classes.names()
    assert names == ['person', 'forklift', 'amr', 'pallet', 'box', 'cone']
    # The renderer reports exactly these kinds (web/digital-twin/src/camera/capture.ts).
    capture = (ROOT.parent/'web/digital-twin/src/camera/capture.ts').read_text()
    kinds = re.search(r"export type ObjectKind = ([^;]+);", capture).group(1)
    assert sorted(re.findall(r"'(\w+)'", kinds)) == sorted(names)


def test_matching_counts_true_false_positives_and_misses():
    preds = [(0, .9, [0, 0, 10, 10]), (0, .8, [1, 1, 10, 10]), (1, .7, [20, 20, 30, 30]), (2, .6, [50, 50, 60, 60])]
    truths = [(0, [0, 0, 10, 10]), (1, [20, 20, 31, 30]), (3, [70, 70, 80, 80])]
    pairs, fps, fns = matching.match(preds, truths, .5)
    assert sorted((i, j) for i, j, _ in pairs) == [(0, 0), (2, 1)]
    assert sorted(fps) == [1, 3] and fns == [2]          # duplicate box and wrong-class box are false positives
    assert matching.prf(2, 2, 1) == pytest.approx((.5, 2/3, 4/7))


def test_session_splits_are_reproducible_and_disjoint():
    from generate_dataset import assign_splits
    a, b = assign_splits(75, {'train': .7, 'val': .15, 'test': .15}, 42), assign_splits(75, {'train': .7, 'val': .15, 'test': .15}, 42)
    assert a == b and sorted(a) == list(range(75))
    counts = {s: sum(v == s for v in a.values()) for s in ('train', 'val', 'test')}
    assert counts == {'train': 52, 'val': 11, 'test': 12}
    assert assign_splits(75, {'train': .7, 'val': .15, 'test': .15}, 43) != a


def test_checkpoint_resolution_picks_the_newest_run(tmp_path, monkeypatch):
    from atlas_vision import models
    for run in ('2026-11-14_001', '2026-11-14_002'):
        (tmp_path/run/'checkpoints').mkdir(parents=True)
        (tmp_path/run/'checkpoints/best.pt').write_bytes(run.encode())
    assert models.resolve_checkpoint(tmp_path/'latest/checkpoints/best.pt').parent.parent.name == '2026-11-14_002'
    assert models.model_identity(tmp_path/'2026-11-14_001/checkpoints/best.pt')['id'].startswith('2026-11-14_001/best.pt@')


def test_frame_gate_rejects_uniform_and_repeated_frames():
    quality = {'min_gray_std': 8.0, 'min_change_from_previous': 1.0}
    wall = np.full((48, 64, 3), 90, np.uint8)
    wall[:, :2] = 95                                    # almost uniform: a wall seen point-blank
    assert annotate.frame_quality_issue(wall, None, quality)[0] == 'uniform'
    scene = np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8)
    issue, gray = annotate.frame_quality_issue(scene, None, quality)
    assert issue is None
    assert annotate.frame_quality_issue(scene, gray, quality)[0] == 'repeated'
    moved = np.roll(scene, 7, axis=1)
    assert annotate.frame_quality_issue(moved, gray, quality)[0] is None
