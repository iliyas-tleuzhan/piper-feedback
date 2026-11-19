"""Validate a recording, split at timing gaps, and hold out whole episodes."""
import argparse
import json
from pathlib import Path

import h5py
import numpy as np

KEYS = ('joint_pos', 'joint_vel', 'joint_cmd', 'measured_joint_torque')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'source': str(args.source.resolve()), 'history': 50,
              'max_gap_seconds': 0.03, 'seed': 0, 'episodes': [],
              'rows': {'train': 0, 'val': 0}, 'windows': {'train': 0, 'val': 0}}
    with h5py.File(args.source, 'r') as src:
        episodes = sorted(ep for ep in src if all(k in src[ep] for k in KEYS))
        if len(episodes) < 2:
            raise ValueError('Need at least two complete episodes for held-out validation.')
        val = set(np.random.default_rng(0).permutation(episodes)[:max(1, round(len(episodes)*0.2))])
        with h5py.File(args.output/'train.h5', 'w') as train, h5py.File(args.output/'val.h5', 'w') as validation:
            for ep in episodes:
                group = src[ep]
                t = group['joint_pos/timestamps'][:]
                arrays = {k: group[k]['data'][:] for k in KEYS}
                assert len(t) and np.all(np.diff(t) > 0), f'{ep}: invalid timestamps'
                for k, a in arrays.items():
                    assert a.shape == (len(t), 6) and np.isfinite(a).all(), f'{ep}/{k}: invalid data'
                    assert np.array_equal(t, group[k]['timestamps'][:]), f'{ep}/{k}: timestamps differ'
                split = 'val' if ep in val else 'train'
                dest = validation if split == 'val' else train
                edges = np.r_[0, np.flatnonzero(np.diff(t) > 30_000_000)+1, len(t)]
                entry = {'episode': ep, 'split': split, 'source_rows': len(t), 'segments': []}
                for i, (start, end) in enumerate(zip(edges[:-1], edges[1:])):
                    if end-start < 50:
                        continue
                    out = dest.create_group(f'{ep}_segment_{i:03d}')
                    out.attrs['source_episode'] = ep
                    out.attrs['source_row_start'] = int(start)
                    out.attrs['source_row_end_exclusive'] = int(end)
                    for k, a in arrays.items():
                        g = out.create_group(k)
                        g.create_dataset('data', data=a[start:end])
                        g.create_dataset('timestamps', data=t[start:end])
                    entry['segments'].append([int(start), int(end)])
                    report['rows'][split] += int(end-start)
                    report['windows'][split] += int(end-start-49)
                report['episodes'].append(entry)
            assert all(report['windows'].values()), 'Empty training or validation set'
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('rows', 'windows')}, indent=2))


if __name__ == '__main__':
    main()
