"""Ground-truth boxes from a rendered instance-ID image (never from model predictions).

For every instance code present in the ID pass: its visible pixels give the box (the modal box:
the visible part only; occluded parts and parts outside the image do not count). Instances with too
few visible pixels or a too-thin box are not labelled and are reported as ignored. Boxes are written
in the normalised YOLO format ``class x_centre y_centre width height`` (pixel edges: a box covering
columns c0..c1 inclusive spans [c0, c1+1)).
"""
import numpy as np
from .classes import names


def boxes(frame, min_pixels=60, min_box_side=5):
    """Labelled and ignored objects of a frame (dicts with id, kind, class_id, bbox, pixels ...)."""
    ids = frame.ids
    if ids is None:
        raise ValueError('frame has no instance-ID pass')
    h, w = ids.shape
    codes, counts = np.unique(ids[ids > 0], return_counts=True)
    labelled, ignored = [], []
    classes = names()
    for code, count in zip(codes.tolist(), counts.tolist()):
        info = frame.instance(code)
        if info is None:
            raise ValueError(f'instance code {code} missing from the frame header')
        rows, cols = np.nonzero(ids == code)
        x0, x1, y0, y1 = int(cols.min()), int(cols.max())+1, int(rows.min()), int(rows.max())+1
        entry = {'id': info['id'], 'kind': info['kind'], 'class_id': classes.index(info['kind']), 'pixels': int(count),
                 'bbox_xyxy': [x0, y0, x1, y1], 'fill': round(count/((x1-x0)*(y1-y0)), 3),
                 'truncated': x0 == 0 or y0 == 0 or x1 == w or y1 == h}
        if count < min_pixels or min(x1-x0, y1-y0) < min_box_side:
            ignored.append(entry)
        else:
            labelled.append(entry)
    return labelled, ignored


def yolo_lines(objects, width, height):
    lines = []
    for o in objects:
        x0, y0, x1, y1 = o['bbox_xyxy']
        cx, cy, bw, bh = (x0+x1)/2/width, (y0+y1)/2/height, (x1-x0)/width, (y1-y0)/height
        if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < bw <= 1 and 0 < bh <= 1):
            raise ValueError(f'box outside the image: {o}')
        lines.append(f"{o['class_id']} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    return lines


def frame_quality_issue(rgb, previous_gray, quality):
    """Why a rendered frame should be rejected (None when it is usable), plus its grey image.

    ``min_gray_std``: below it the camera faces a wall or an object point-blank (a nearly uniform
    image: nothing to learn, and difference hashes of such images are meaningless).
    ``min_change_from_previous``: mean absolute grey difference to the same robot's previous frame;
    below it a stuck robot is repeating its frame.
    """
    gray = (rgb[..., 0]*.299+rgb[..., 1]*.587+rgb[..., 2]*.114).astype(np.float32)
    if float(gray.std()) < quality['min_gray_std']:
        return 'uniform', gray
    if previous_gray is not None and float(np.abs(gray-previous_gray).mean()) < quality['min_change_from_previous']:
        return 'repeated', gray
    return None, gray
