"""Greedy IoU matching of predictions to ground truth (per class, highest confidence first)."""
import numpy as np


def iou(a, b):
    """IoU of two xyxy boxes."""
    w = max(0., min(a[2], b[2])-max(a[0], b[0]))
    h = max(0., min(a[3], b[3])-max(a[1], b[1]))
    inter = w*h
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union > 0 else 0.


def match(predictions, truths, threshold=.5):
    """``predictions``: [(class, conf, xyxy)], ``truths``: [(class, xyxy)].
    Returns (matched pairs [(pred index, truth index, iou)], unmatched predictions, unmatched truths)."""
    order = sorted(range(len(predictions)), key=lambda i: -predictions[i][1])
    used, pairs, false_positives = set(), [], []
    for i in order:
        cls, _, box = predictions[i]
        best, best_iou = None, threshold
        for j, (tcls, tbox) in enumerate(truths):
            if j in used or tcls != cls:
                continue
            value = iou(box, tbox)
            if value >= best_iou:
                best, best_iou = j, value
        if best is None:
            false_positives.append(i)
        else:
            used.add(best)
            pairs.append((i, best, best_iou))
    return pairs, false_positives, [j for j in range(len(truths)) if j not in used]


def prf(tp, fp, fn):
    p = tp/(tp+fp) if tp+fp else 0.
    r = tp/(tp+fn) if tp+fn else 0.
    return p, r, (2*p*r/(p+r) if p+r else 0.)


def percentile(values, q):
    return float(np.percentile(values, q)) if len(values) else None
