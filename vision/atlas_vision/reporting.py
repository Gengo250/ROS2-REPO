"""Figures and summaries from Ultralytics' per-epoch results.csv (measured values only)."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

PANELS = {
    'losses_box.png': (['train/box_loss', 'val/box_loss'], 'box loss'),
    'losses_cls.png': (['train/cls_loss', 'val/cls_loss'], 'classification loss'),
    'losses_l1.png': (['train/l1_loss', 'val/l1_loss'], 'L1 box loss (YOLO26)'),
    'losses_dfl.png': (['train/dfl_loss', 'val/dfl_loss'], 'distribution focal loss (YOLOv8-style heads)'),
    'precision.png': (['metrics/precision(B)'], 'precision (val)'),
    'recall.png': (['metrics/recall(B)'], 'recall (val)'),
    'map50.png': (['metrics/mAP50(B)'], 'mAP@50 (val)'),
    'map50_95.png': (['metrics/mAP50-95(B)'], 'mAP@50:95 (val)'),
    'learning_rate.png': (['lr/pg0', 'lr/pg1', 'lr/pg2'], 'learning rate'),
}


def load(path):
    frame = pd.read_csv(path)
    frame.columns = [c.strip() for c in frame.columns]
    return frame


def training_curves(csv, figures):
    frame = load(csv)
    written = []
    for name, (columns, title) in PANELS.items():
        columns = [c for c in columns if c in frame]
        if not columns:     # e.g. a loss term this model version does not report
            continue
        fig, ax = plt.subplots(figsize=(6, 3.6))
        for c in columns:
            ax.plot(frame['epoch'], frame[c], marker='.', label=c)
        ax.set_xlabel('epoch'); ax.set_title(title); ax.grid(alpha=.3); ax.legend()
        fig.tight_layout(); fig.savefig(Path(figures)/name, dpi=110); plt.close(fig)
        written.append(name)
    return written


def summary(csv):
    frame = load(csv)
    # Ultralytics' fitness: 0.1·mAP50 + 0.9·mAP50-95 (selection of best.pt).
    fitness = .1*frame['metrics/mAP50(B)']+.9*frame['metrics/mAP50-95(B)']
    best = frame.loc[fitness.idxmax()]
    keys = ['epoch', 'metrics/precision(B)', 'metrics/recall(B)', 'metrics/mAP50(B)', 'metrics/mAP50-95(B)']
    return {'epochs_run': int(frame['epoch'].max()), 'best_epoch': int(best['epoch']),
            'best': {k: round(float(best[k]), 4) for k in keys},
            'last': {k: round(float(frame.iloc[-1][k]), 4) for k in keys}}
