#!/usr/bin/env python3
"""Validate a generated dataset before training (exit code 1 on any failure).

    uv run python scripts/validate_dataset.py [--data data/synthetic]

Automatic checks: image/label/mask pairing, label syntax, class ids, normalised coordinates in
range, boxes inside the image, every label equal to the box recomputed from the saved instance mask
(image–label association and annotation alignment), metadata ↔ label agreement, exact duplicates,
near-duplicates (64-bit difference hash, confirmed on 64x48 thumbnails) and session leakage across train/val/test, nearly uniform
images (camera against a wall: the generator's frame gate), empty-image share, class balance and
object-size distribution. Reports and figures go to
<data>/metadata/validation/: report.json, class_balance.png, box_sizes.png and contact sheets
with the ground-truth boxes drawn (samples per split, plus "difficult" objects: truncated by the
border, partly occluded or tiny) for the manual inspection described in vision/README.md.
"""
import argparse
import collections
import hashlib
import json
import sys
from pathlib import Path
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atlas_vision.classes import names as class_names  # noqa: E402
from atlas_vision.paths import VISION  # noqa: E402

SPLITS = ('train', 'val', 'test')
COLOURS = [(80, 80, 255), (0, 200, 255), (120, 255, 60), (255, 170, 60), (255, 80, 200), (0, 130, 255)]


def dhash(image, size=8):
    small = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (size+1, size), interpolation=cv2.INTER_AREA)
    bits = (small[:, 1:] > small[:, :-1]).flatten()
    return int(''.join('1' if b else '0' for b in bits), 2)


def thumbnail_mad(path_a, path_b, size=(64, 48)):
    a, b = (cv2.resize(cv2.imread(str(p)), size, interpolation=cv2.INTER_AREA).astype(np.float32) for p in (path_a, path_b))
    return float(np.abs(a-b).mean())


def draw(image, objects, names):
    out = image.copy()
    for o in objects:
        x0, y0, x1, y1 = o['bbox_xyxy']
        colour = COLOURS[o['class_id'] % len(COLOURS)]
        cv2.rectangle(out, (x0, y0), (x1-1, y1-1), colour, 2)
        cv2.putText(out, names[o['class_id']], (x0, max(12, y0-3)), cv2.FONT_HERSHEY_SIMPLEX, .45, colour, 1, cv2.LINE_AA)
    return out


def sheet(entries, data, names, path, columns=4, tile=(320, 240), title_of=None):
    tiles = []
    for record, objects in entries:
        image = draw(cv2.imread(str(data/record['image'])), objects, names)
        image = cv2.resize(image, tile, interpolation=cv2.INTER_AREA)
        label = title_of(record) if title_of else Path(record['image']).stem
        cv2.putText(image, label, (5, tile[1]-6), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(image)
    if not tiles:
        return
    while len(tiles) % columns:
        tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(path), np.vstack([np.hstack(tiles[i:i+columns]) for i in range(0, len(tiles), columns)]))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--data', default=str(VISION/'data/synthetic'))
    p.add_argument('--near-duplicate-bits', type=int, default=2, help='dHash Hamming distance of a near-duplicate candidate')
    p.add_argument('--near-duplicate-mad', type=float, default=5.0,
                   help='a candidate is a near-duplicate when the mean |difference| of 64x48 colour thumbnails is below this '
                        '(consecutive frames of a moving robot: 6.7 at the 5th percentile)')
    p.add_argument('--max-empty-fraction', type=float, default=0.25)
    p.add_argument('--samples', type=int, default=12, help='images per contact sheet')
    a = p.parse_args()
    data = Path(a.data)
    names = class_names()
    out = data/'metadata/validation'
    out.mkdir(parents=True, exist_ok=True)
    errors, warnings = [], []
    records = [json.loads(line) for line in (data/'metadata/frames.jsonl').read_text().splitlines() if line.strip()]
    generation = json.loads((data/'metadata/generation.json').read_text())
    by_image = {r['image']: r for r in records}
    # ------------------------------------------------------------ files and pairing
    for split in SPLITS:
        images = {p.stem for p in (data/'images'/split).glob('*.jpg')}
        labels = {p.stem for p in (data/'labels'/split).glob('*.txt')}
        masks = {p.stem for p in (data/'masks'/split).glob('*.png')}
        listed = {Path(r['image']).stem for r in records if r['split'] == split}
        for name, other in (('label', labels), ('mask', masks), ('metadata record', listed)):
            for missing in sorted(images-other):
                errors.append(f'{split}/{missing}: image without {name}')
            for missing in sorted(other-images):
                errors.append(f'{split}/{missing}: {name} without image')
    # ------------------------------------------------------------ labels, masks, metadata
    class_counts = {s: collections.Counter() for s in SPLITS}
    sizes, empty, hashes, exact = [], collections.Counter(), {s: [] for s in SPLITS}, collections.defaultdict(list)
    difficult, uniform = [], []
    # The generator's frame gate (older datasets without one: the same default).
    min_gray_std = generation['config'].get('quality', {}).get('min_gray_std', 8.0)
    for r in records:
        split, w, h = r['split'], r['width'], r['height']
        image_path = data/r['image']
        if not image_path.is_file():
            continue
        image = cv2.imread(str(image_path))
        if image is None or image.shape[:2] != (h, w):
            errors.append(f"{r['image']}: unreadable or not {w}x{h}")
            continue
        exact[hashlib.sha256(image_path.read_bytes()).hexdigest()].append(r['image'])
        if float(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).std()) < min_gray_std-.5:
            uniform.append(r['image'])      # dHash of a nearly uniform image is noise: not hashed
        else:
            hashes[split].append((dhash(image), r['image'], r['session']))
        lines = [ln for ln in (data/r['label']).read_text().splitlines() if ln.strip()]
        if not lines:
            empty[split] += 1
        if len(lines) != len(r['objects']):
            errors.append(f"{r['label']}: {len(lines)} lines but {len(r['objects'])} labelled objects in metadata")
        mask = cv2.imread(str(data/r['mask']), cv2.IMREAD_UNCHANGED)
        if mask is None or mask.shape != (h, w):
            errors.append(f"{r['mask']}: unreadable instance mask")
            continue
        codes = {i['id']: i['code'] for i in r['instances']}
        for line, o in zip(lines, r['objects']):
            parts = line.split()
            if len(parts) != 5:
                errors.append(f"{r['label']}: malformed line {line!r}")
                continue
            cls = int(parts[0]) if parts[0].isdigit() else -1
            cx, cy, bw, bh = map(float, parts[1:])
            if not 0 <= cls < len(names):
                errors.append(f"{r['label']}: class id {parts[0]} outside 0..{len(names)-1}")
            if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < bw <= 1 and 0 < bh <= 1):
                errors.append(f"{r['label']}: coordinates out of range {line!r}")
            if cx-bw/2 < -1e-6 or cy-bh/2 < -1e-6 or cx+bw/2 > 1+1e-6 or cy+bh/2 > 1+1e-6:
                errors.append(f"{r['label']}: box outside the image {line!r}")
            if cls != o['class_id'] or names[cls] != o['kind']:
                errors.append(f"{r['label']}: class {cls} does not match metadata kind {o['kind']}")
            # The label must be exactly the visible extent of that instance in the saved mask.
            rows, cols = np.nonzero(mask == codes.get(o['id'], -1))
            if not len(rows):
                errors.append(f"{r['label']}: instance {o['id']} has no pixels in the mask")
                continue
            x0, y0, x1, y1 = cols.min(), rows.min(), cols.max()+1, rows.max()+1
            expected = ((x0+x1)/2/w, (y0+y1)/2/h, (x1-x0)/w, (y1-y0)/h)
            if max(abs(u-v) for u, v in zip((cx, cy, bw, bh), expected)) > 2e-6:
                errors.append(f"{r['label']}: label {line!r} differs from the mask box {expected}")
            class_counts[split][names[cls]] += 1
            sizes.append((names[cls], bw*w, bh*h, o.get('distance_m')))
            if o['truncated'] or o['fill'] < .35 or o['pixels'] < 150:
                difficult.append((r, o))
    # ------------------------------------------------------------ duplicates and leakage
    if uniform:
        # Checked on the JPEG (0.5 grey levels of margin for compression) against the generator's gate.
        errors.append(f'{len(uniform)} nearly uniform images (grey std < {min_gray_std-.5}): {uniform[:10]}')
    duplicates = [v for v in exact.values() if len(v) > 1]
    for group in duplicates:
        errors.append(f'identical image files: {group}')
    sessions = {s: {r['session'] for r in records if r['split'] == s} for s in SPLITS}
    for i, s in enumerate(SPLITS):
        for t in SPLITS[i+1:]:
            if sessions[s] & sessions[t]:
                errors.append(f'sessions shared by {s} and {t}: {sorted(sessions[s] & sessions[t])[:10]}')
    near = collections.Counter()
    near_examples = []
    pool = [(hv, img, sess, s) for s in SPLITS for hv, img, sess in hashes[s]]
    values = np.array([hv for hv, *_ in pool], dtype=np.uint64)
    for i, (hv, img, sess, s) in enumerate(pool):
        distance = np.array([bin(int(x)).count('1') for x in (values[i+1:] ^ np.uint64(hv))])
        for j in np.nonzero(distance <= a.near_duplicate_bits)[0]:
            other = pool[i+1+j]
            # dHash alone confuses distinct low-texture images made of horizontal bands (wall, floor):
            # confirm every candidate on the pixels.
            if thumbnail_mad(data/img, data/other[1]) >= a.near_duplicate_mad:
                near['dhash_only_rejected'] += 1
                continue
            key = 'cross_split' if other[3] != s else 'within_session' if other[2] == sess else 'within_split'
            near[key] += 1
            if key == 'cross_split' and len(near_examples) < 20:
                near_examples.append([img, other[1], int(distance[j])])
    if near['cross_split']:
        errors.append(f"{near['cross_split']} near-duplicate image pairs across splits (dHash ≤ {a.near_duplicate_bits} bits and "
                      f"thumbnail difference < {a.near_duplicate_mad}): {near_examples[:5]}")
    # ------------------------------------------------------------ balance
    totals = collections.Counter()
    for counts in class_counts.values():
        totals.update(counts)
    for name in names:
        if totals[name] == 0:
            errors.append(f'class {name} has no labelled instance')
        elif totals[name] < 100:
            warnings.append(f'class {name} has only {totals[name]} instances')
        for split in SPLITS:
            if class_counts[split][name] == 0 and sessions[split]:
                warnings.append(f'class {name} absent from the {split} split')
    images_per_split = collections.Counter(r['split'] for r in records)
    for split in SPLITS:
        if images_per_split[split] and empty[split]/images_per_split[split] > a.max_empty_fraction:
            warnings.append(f'{split}: {empty[split]/images_per_split[split]:.0%} images without labels')
    largest = max(totals.values()) if totals else 0
    imbalance = {n: round(largest/max(totals[n], 1), 2) for n in names}
    # ------------------------------------------------------------ figures and sheets
    fig, ax = plt.subplots(figsize=(8, 4))
    x = np.arange(len(names))
    for k, split in enumerate(SPLITS):
        ax.bar(x+(k-1)*.27, [class_counts[split][n] for n in names], .27, label=split)
    ax.set_xticks(x, names); ax.set_ylabel('labelled instances'); ax.legend(); ax.set_title('Class balance per split')
    fig.tight_layout(); fig.savefig(out/'class_balance.png', dpi=110); plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for name in names:
        side = [np.sqrt(bw*bh) for n, bw, bh, _ in sizes if n == name]
        dist = [d for n, _, _, d in sizes if n == name and d is not None]
        if side:
            axes[0].hist(side, bins=np.logspace(0.5, 2.8, 30), histtype='step', label=name)
        if dist:
            axes[1].hist(dist, bins=30, range=(0, 30), histtype='step', label=name)
    axes[0].set_xscale('log'); axes[0].set_xlabel('box size √(w·h) [px]'); axes[0].legend()
    axes[1].set_xlabel('distance to the camera robot [m]'); axes[1].legend()
    fig.tight_layout(); fig.savefig(out/'box_sizes.png', dpi=110); plt.close(fig)
    rng = np.random.default_rng(0)
    for split in SPLITS:
        pool = [r for r in records if r['split'] == split and r['objects']]
        if pool:
            chosen = [pool[i] for i in sorted(rng.choice(len(pool), min(a.samples, len(pool)), replace=False))]
            sheet([(r, r['objects']) for r in chosen], data, names, out/f'samples_{split}.jpg')
    night = [r for r in records if r['night'] and r['objects']]
    if night:
        chosen = [night[i] for i in sorted(rng.choice(len(night), min(8, len(night)), replace=False))]
        sheet([(r, r['objects']) for r in chosen], data, names, out/'samples_night.jpg')
    if difficult:
        chosen = [difficult[i] for i in sorted(rng.choice(len(difficult), min(a.samples, len(difficult)), replace=False))]
        sheet([(r, [o]) for r, o in chosen], data, names, out/'difficult_objects.jpg',
              title_of=None)
    bins = {'small_<32px': 0, 'medium_32-96px': 0, 'large_>96px': 0}
    for _, bw, bh, _ in sizes:
        area = np.sqrt(bw*bh)
        bins['small_<32px' if area < 32 else 'medium_32-96px' if area < 96 else 'large_>96px'] += 1
    report = {'data': str(data), 'generation_seed': generation['seed'], 'scene_sha256': generation['scene_sha256'],
              'config_sha256_16': generation['config_sha256_16'], 'images': dict(images_per_split),
              'sessions': {s: len(v) for s, v in sessions.items()}, 'empty_images': dict(empty),
              'night_images': sum(r['night'] for r in records), 'instances': {s: dict(c) for s, c in class_counts.items()},
              'instances_total': dict(totals), 'imbalance_ratio_to_largest': imbalance, 'size_bins': bins,
              'ignored_below_threshold': dict(collections.Counter(o['kind'] for r in records for o in r['ignored'])),
              'difficult_objects': len(difficult), 'exact_duplicate_groups': len(duplicates), 'near_duplicate_pairs': dict(near),
              'near_duplicate_examples_cross_split': near_examples, 'errors': errors[:200], 'error_count': len(errors),
              'warnings': warnings, 'passed': not errors}
    (out/'report.json').write_text(json.dumps(report, indent=1))
    print(json.dumps({k: report[k] for k in ('images', 'sessions', 'instances_total', 'empty_images', 'near_duplicate_pairs',
                                            'error_count', 'warnings', 'passed')}, indent=1))
    sys.exit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
