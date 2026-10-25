#!/usr/bin/env python3
"""Local CUDA notebook sweep. Standard Python only; never runs remote commands."""
import argparse
import csv
import hashlib
import json
import math
import platform
import statistics
import subprocess
from pathlib import Path

MODES = ('cpu_full_tensor', 'cpu_bins_gpu_tensor', 'gpu_event_aggregation')
ROOT = Path(__file__).resolve().parents[2]

def validate(result, events, iterations, pattern):
    if result.get('all_outputs_identical') is not True:
        raise ValueError('Tensor equivalence was not confirmed')
    if (result.get('events_per_4ms'), result.get('iterations'), result.get('pattern')) != (events, iterations, pattern):
        raise ValueError('Benchmark configuration differs from request')
    if result.get('scope') != 'decoded_event_aggregation':
        raise ValueError('Unexpected measurement scope')
    for mode in MODES:
        values = result[mode]
        for field in ('mean_ms', 'p50_ms', 'p95_ms', 'p99_ms'):
            if not isinstance(values[field], (int, float)) or not math.isfinite(values[field]) or values[field] < 0:
                raise ValueError('Invalid timing')
        if not 0 <= values['over_4ms'] <= iterations:
            raise ValueError('Invalid service-budget count')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--iterations', type=int, default=300)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--warmup', type=int, default=100)
    parser.add_argument('--events', type=int, nargs='+', default=[1000, 10000, 100000, 200000])
    parser.add_argument('--patterns', nargs='+', choices=['uniform', 'hotspot'], default=['uniform', 'hotspot'])
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.iterations < 10 or args.iterations % 3 or args.repeats < 1 or args.warmup < 10 or any(n <= 0 or n > 10000000 for n in args.events):
        parser.error('iterations>=10 divisible by 3; repeats>=1; events=1..10000000')
    jobs = [(repeat, pattern, events) for repeat in range(args.repeats)
            for pattern in args.patterns for events in (args.events if repeat % 2 == 0 else list(reversed(args.events)))]
    def command(pattern, events):
        return [str(args.binary.resolve()), '--iterations', str(args.iterations), '--events', str(events), '--pattern', pattern, '--warmup', str(args.warmup)]
    if args.dry_run:
        print(json.dumps([command(pattern, events) for _, pattern, events in jobs], indent=2)); return
    if args.output is None:
        parser.error('--output NEW_DIRECTORY is required')
    if not args.binary.is_file():
        parser.error('Build the CUDA benchmark first')
    args.output.mkdir(parents=True, exist_ok=False)
    paths = ['test/event_tensor_cuda_load_benchmark.cpp', 'src/event_tensor_cuda_backend.cu',
             'include/openeb_ros2/evt3_runs_histogram.hpp', 'include/openeb_ros2/histogram_bin_updates.hpp']
    report = {'status': 'running', 'platform': platform.platform(), 'scope': 'decoded_event_aggregation',
              'sources': {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths},
              'limitations': ['No RAW decode, USB, ROS or inference', 'Scalar CPU aggregation; not the coalesced ADDR_X decoder',
                              'Decoded input/event-array creation and D2H validation excluded',
                              'Each measured stage waits for GPU completion; excludes pipeline overlap'], 'runs': []}
    def save():
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    save()
    try:
        for repeat, pattern, events in jobs:
            result = subprocess.run(command(pattern, events), text=True, capture_output=True)
            name = f'{repeat}-{pattern}-{events}'
            (args.output/f'{name}.stdout').write_text(result.stdout)
            (args.output/f'{name}.stderr').write_text(result.stderr)
            result.check_returncode()
            decoded = json.loads(result.stdout)
            validate(decoded, events, args.iterations, pattern)
            report['runs'].append({'repeat': repeat, 'result': decoded}); save()
            print(f'{name}: verified', flush=True)
        rows = []
        for pattern in args.patterns:
            for events in args.events:
                runs = [r['result'] for r in report['runs'] if r['result']['pattern'] == pattern and r['result']['events_per_4ms'] == events]
                baseline = statistics.median(r['cpu_full_tensor']['p50_ms'] for r in runs)
                for mode in MODES:
                    p50 = statistics.median(r[mode]['p50_ms'] for r in runs)
                    rows.append({'pattern': pattern, 'events_per_second': events*250, 'mode': mode,
                                 'p50_ms_median': p50, 'p95_ms_median': statistics.median(r[mode]['p95_ms'] for r in runs),
                                 'p99_ms_median': statistics.median(r[mode]['p99_ms'] for r in runs),
                                 'speedup_vs_cpu_full_p50': baseline/p50 if p50 else None,
                                 'over_4ms_fraction': sum(r[mode]['over_4ms'] for r in runs)/(args.iterations*len(runs))})
        with (args.output/'summary.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
        report['summary'] = rows; report['status'] = 'complete'; save()
    except Exception as error:
        report['status'] = 'failed'; report['error'] = str(error); save(); raise

if __name__ == '__main__':
    main()
