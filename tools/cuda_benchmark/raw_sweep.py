#!/usr/bin/env python3
"""Repeat local RAW-to-CUDA replay; preserve timing CSV, provenance and failures."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[2]
MODES = ('events', 'evt3_runs', 'evt3_runs_gpu')

def validate(result, warmup, packet_bytes):
    if result.get('scope') != 'raw_decode_to_gpu_tensor' or result.get('all_outputs_identical') is not True:
        raise ValueError('RAW scope and exact Tensor equivalence required')
    if result.get('warmup') != warmup or result.get('packet_bytes') != packet_bytes:
        raise ValueError('Configuration mismatch')
    n = result.get('measured_frames', 0)
    if not isinstance(n, int) or n < 10 or result.get('frames') != n + warmup:
        raise ValueError('Invalid frame count')
    for mode in MODES:
        r = result[mode]
        for key in ('mean_ms', 'p50_ms', 'p95_ms', 'p99_ms', 'cpu_decode_mean_ms'):
            if not isinstance(r[key], (int, float)) or not math.isfinite(r[key]) or r[key] < 0:
                raise ValueError('Invalid timing')
        if not r['p50_ms'] <= r['p95_ms'] <= r['p99_ms']:
            raise ValueError('Invalid quantiles')
        for key in ('over_4ms', 'over_sensor_span'):
            if not isinstance(r[key], int) or not 0 <= r[key] <= n:
                raise ValueError('Invalid budget count')

def rate_rows(paths, counts):
    groups = {}
    for path, expected in zip(paths, counts):
        seen = {mode: 0 for mode in MODES}
        with path.open(newline='') as f:
            for row in csv.DictReader(f):
                if row['warmup'] != '0':
                    continue
                mode = row['mode']
                if mode not in seen:
                    raise ValueError('Unknown CSV mode')
                rate, ms = float(row['event_rate_per_second']), float(row['total_ms'])
                if not all(math.isfinite(v) and v >= 0 for v in (rate, ms)):
                    raise ValueError('Invalid CSV timing/rate')
                bucket = '<10Mev/s' if rate < 1e7 else '10..30Mev/s' if rate < 3e7 else '>=30Mev/s'
                groups.setdefault((bucket, mode), []).append(ms)
                seen[mode] += 1
        if any(n != expected for n in seen.values()):
            raise ValueError('CSV frame count mismatch')
    rows = []
    for (bucket, mode), values in sorted(groups.items()):
        values.sort()
        def q(p): return values[math.ceil(p*len(values))-1]
        rows.append(dict(event_rate_band=bucket, mode=mode, samples=len(values),
                         p50_ms=q(.5), p95_ms=q(.95), p99_ms=q(.99)))
    return rows

def write_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--warmup', type=int, default=30)
    parser.add_argument('--packet-bytes', type=int, default=65536)
    parser.add_argument('--max-bytes', type=int, default=268435456, help='0: entire RAW loaded into RAM')
    parser.add_argument('--max-frames', type=int, default=1000, help='0: all indexed windows')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if (args.repeats < 1 or args.warmup < 1 or args.packet_bytes < 2 or args.packet_bytes % 2
            or args.max_bytes < 0 or args.max_frames < 0
            or 0 < args.max_frames < args.warmup + 10):
        parser.error('Invalid limits: need >=warmup+10 frames and an even packet size')
    output = args.output or Path('raw_results')
    commands = [[str(args.binary.resolve()), '--raw', str(args.raw.resolve()),
                 '--csv', str((output/f'windows-{repeat}.csv').resolve()),
                 '--warmup', str(args.warmup), '--packet-bytes', str(args.packet_bytes),
                 '--max-bytes', str(args.max_bytes), '--max-frames', str(args.max_frames),
                 '--order-offset', str(repeat % 3)] for repeat in range(args.repeats)]
    if args.dry_run:
        print(json.dumps(commands, indent=2)); return
    if args.output is None or not args.binary.is_file() or not args.raw.is_file():
        parser.error('Require built --binary, existing --raw and --output NEW_DIRECTORY')
    output.mkdir(parents=True, exist_ok=False)
    sources = ['test/event_tensor_raw_cuda_benchmark.cpp', 'test/raw_evt3_benchmark_input.hpp',
               'src/event_tensor_cuda_backend.cu', 'include/openeb_ros2/evt3_runs_histogram.hpp',
               'include/openeb_ros2/event_tensor_cuda_backend.hpp', 'include/openeb_ros2/histogram_bin_updates.hpp',
               'tools/cuda_benchmark/CMakeLists.txt', 'tools/cuda_benchmark/raw_sweep.py']
    stat = args.raw.stat()
    # Hash the full source file outside timing; the benchmark may use a prefix.
    h = hashlib.sha256(); remaining = stat.st_size
    with args.raw.open('rb') as f:
        while remaining:
            chunk = f.read(min(1048576, remaining))
            if not chunk: break
            h.update(chunk); remaining -= len(chunk)
    report = dict(status='running', scope='raw_decode_to_gpu_tensor', platform=platform.platform(),
                  raw=dict(path=str(args.raw.resolve()), file_bytes=stat.st_size, mtime_ns=stat.st_mtime_ns,
                           sha256=h.hexdigest()),
                  binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),
                  sources={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources},
                  limitations=['RAW loading/indexing and D2H validation outside timing',
                               'USB, ROS queues/publication, inference and inter-frame pipeline overlap excluded',
                               'Same fixed-size synthetic packets; original USB packet boundaries unavailable',
                               'Frame duration may exceed 4ms due to packet quantization/gaps; see window CSV',
                               'One clock epoch only; <=4ms backward events dropped; larger jumps rejected',
                               'cpu_decode includes fused CPU histogram work for new modes',
                               'rate_summary pools repeated replay samples; no independent trials claim'], runs=[])
    def save(): (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    save()
    try:
        for repeat, command in enumerate(commands):
            process = subprocess.run(command, text=True, capture_output=True)
            (output/f'run-{repeat}.stdout').write_text(process.stdout)
            (output/f'run-{repeat}.stderr').write_text(process.stderr)
            process.check_returncode()
            result = json.loads(process.stdout); validate(result, args.warmup, args.packet_bytes)
            if report['runs']:
                previous = report['runs'][0]['result']
                for key in ('tensor_digest', 'accepted_events', 'processed_payload_bytes', 'codec_decoder_sha256', 'gpu', 'driver', 'runtime'):
                    if result[key] != previous[key]:
                        raise ValueError('Input/output/device differs between repeats: '+key)
            report['runs'].append(dict(repeat=repeat, command=command, result=result)); save()
            print(f'run {repeat}: all three tensors identical', flush=True)
        if args.raw.stat().st_mtime_ns != stat.st_mtime_ns or args.raw.stat().st_size != stat.st_size:
            raise ValueError('RAW changed during benchmark')
        runs = [r['result'] for r in report['runs']]
        base = statistics.median(r['events']['p50_ms'] for r in runs)
        rows = []
        for mode in MODES:
            median = statistics.median(r[mode]['p50_ms'] for r in runs)
            rows.append(dict(mode=mode, p50_ms_median=median,
                             p95_ms_median=statistics.median(r[mode]['p95_ms'] for r in runs),
                             p99_ms_median=statistics.median(r[mode]['p99_ms'] for r in runs),
                             speedup_vs_events_p50=base/median if median else None,
                             h2d_bytes_median=statistics.median(r[mode]['h2d_payload_bytes'] for r in runs),
                             over_4ms_fraction=sum(r[mode]['over_4ms'] for r in runs)/sum(r['measured_frames'] for r in runs)))
        rates = rate_rows([output/f'windows-{i}.csv' for i in range(args.repeats)], [r['measured_frames'] for r in runs])
        write_csv(output/'summary.csv', rows); write_csv(output/'rate_summary.csv', rates)
        report['summary'] = rows; report['status'] = 'complete'; save()
    except Exception as error:
        report['status'] = 'failed'; report['error'] = str(error); save(); raise

if __name__ == '__main__': main()
