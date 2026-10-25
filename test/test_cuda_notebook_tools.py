"""Portable orchestration checks; these never execute a CUDA benchmark."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'tools/cuda_benchmark/sweep.py'
spec=importlib.util.spec_from_file_location('sweep',SCRIPT)
sweep=importlib.util.module_from_spec(spec);spec.loader.exec_module(sweep)
class NotebookToolsTest(unittest.TestCase):
    def sample(self):
        result={'all_outputs_identical':True,'events_per_4ms':1000,'iterations':12,'pattern':'uniform','scope':'decoded_event_aggregation'}
        for mode in sweep.MODES:result[mode]={'mean_ms':1.0,'p50_ms':1.0,'p95_ms':2.0,'p99_ms':3.0,'over_4ms':0}
        return result
    def test_accepts_only_verified_finite_results(self):
        sweep.validate(self.sample(),1000,12,'uniform')
        for key,value in [('all_outputs_identical',False),('events_per_4ms',10000),('scope','unknown')]:
            r=self.sample();r[key]=value
            with self.assertRaises(ValueError):sweep.validate(r,1000,12,'uniform')
        for field,value in [('mean_ms',float('nan')),('p95_ms',-1),('over_4ms',13)]:
            r=self.sample();r['cpu_full_tensor'][field]=value
            with self.assertRaises(ValueError):sweep.validate(r,1000,12,'uniform')
    def test_dry_run_does_not_require_gpu_or_binary(self):
        data=json.loads(subprocess.check_output([sys.executable,str(SCRIPT),'--binary','/does/not/exist','--dry-run','--repeats','1']))
        self.assertEqual(len(data),8)
        self.assertEqual({int(c[c.index('--events')+1])*250 for c in data},{250000,2500000,25000000,50000000})
        self.assertEqual({c[c.index('--pattern')+1] for c in data},{'uniform','hotspot'})
spec_raw=importlib.util.spec_from_file_location('raw_sweep',ROOT/'tools/cuda_benchmark/raw_sweep.py')
raw=importlib.util.module_from_spec(spec_raw);spec_raw.loader.exec_module(raw)
class RawNotebookToolsTest(unittest.TestCase):
    def sample(self):
        r={'all_outputs_identical':True,'scope':'raw_decode_to_gpu_tensor','warmup':30,
           'packet_bytes':65536,'frames':40,'measured_frames':10}
        for mode in raw.MODES:
            r[mode]={'mean_ms':1,'p50_ms':1,'p95_ms':2,'p99_ms':3,'cpu_decode_mean_ms':0.5,
                     'over_4ms':0,'over_sensor_span':0}
        return r
    def test_raw_validation(self):
        raw.validate(self.sample(),30,65536)
        for key,value in [('scope','decoded_event_aggregation'),('all_outputs_identical',False),('frames',39)]:
            r=self.sample();r[key]=value
            with self.assertRaises(ValueError):raw.validate(r,30,65536)
        r=self.sample();r['events']['p99_ms']=float('nan')
        with self.assertRaises(ValueError):raw.validate(r,30,65536)
    def test_raw_dry_run(self):
        data=json.loads(subprocess.check_output([sys.executable,str(ROOT/'tools/cuda_benchmark/raw_sweep.py'),
            '--binary','/missing','--raw','/missing.raw','--dry-run']))
        self.assertEqual(len(data),3)
        self.assertEqual({c[c.index('--order-offset')+1] for c in data},{'0','1','2'})
        self.assertTrue(all('--packet-bytes' in c and '--csv' in c for c in data))
    def test_rate_summary_checks_csv_counts(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'windows.csv'
            path.write_text('warmup,mode,event_rate_per_second,total_ms\n'+''.join(
                '0,'+mode+',25000000,1.5\n' for mode in raw.MODES))
            self.assertEqual(len(raw.rate_rows([path],[1])),3)
            with self.assertRaises(ValueError):raw.rate_rows([path],[2])
if __name__=='__main__':unittest.main()

