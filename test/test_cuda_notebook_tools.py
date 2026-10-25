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
spec_copy=importlib.util.spec_from_file_location('copy_sweep',ROOT/'tools/cuda_benchmark/copy_sweep.py')
copy_sweep=importlib.util.module_from_spec(spec_copy);spec_copy.loader.exec_module(copy_sweep)
class CopyNotebookToolsTest(unittest.TestCase):
    def sample(self):
        r={'all_outputs_identical':True,'cpu_reference_identical':True,'scope':'raw_decode_to_gpu_tensor',
           'comparison':'histogram_cpu_copy','tensor_shape':[1,20,120,160],'tensor_dtype':'float32',
           'warmup':30,'packet_bytes':65536,'frames':40,'measured_frames':10,
           'tensor_digest':'fixture','accepted_events':1000,'processed_payload_bytes':2000,
           'codec_decoder_sha256':'fixture','gpu':'fixture','driver':0,'runtime':0}
        for i,mode in enumerate(copy_sweep.MODES):
            r[mode]={'mean_ms':1,'p50_ms':1,'p95_ms':2,'p99_ms':3,'cpu_decode_mean_ms':0.5,
                     'cpu_export_mean_ms':0.1,'gpu_submit_mean_ms':0.1,'gpu_wait_mean_ms':0.1,
                     'h2d_payload_bytes':1000,'histogram_cpu_copy_bytes':2000 if i==0 else 1000,
                     'over_4ms':0,'over_sensor_span':0}
        return r
    def test_copy_validation(self):
        copy_sweep.validate(self.sample(),30,65536)
        for key,value in [('comparison','other'),('cpu_reference_identical',False),('tensor_shape',[1,2,120,160])]:
            r=self.sample();r[key]=value
            with self.assertRaises(ValueError):copy_sweep.validate(r,30,65536)
        for key,value in [('h2d_payload_bytes',999),('histogram_cpu_copy_bytes',2000),('cpu_export_mean_ms',float('nan'))]:
            r=self.sample();r['evt3_runs_gpu_direct'][key]=value
            with self.assertRaises(ValueError):copy_sweep.validate(r,30,65536)
    def test_copy_dry_run(self):
        data=json.loads(subprocess.check_output([sys.executable,str(ROOT/'tools/cuda_benchmark/copy_sweep.py'),
            '--binary','/missing','--raw','/missing.raw','--dry-run','--repeats','4']))
        self.assertEqual([c[c.index('--order-offset')+1] for c in data],['0','1','0','1'])
    def test_copy_runner_preserves_success_and_failure(self):
        # Mock benchmark process only: this verifies reporting, NOT CUDA.
        import tempfile
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as d:
            binary=Path(d)/'binary';binary.write_text('fixture')
            raw_file=Path(d)/'raw';raw_file.write_bytes(b'fixture')
            for succeeds in [True,False]:
                output=Path(d)/str(succeeds)
                argv=['copy_sweep.py','--binary',str(binary),'--raw',str(raw_file),'--output',str(output),'--repeats','2']
                def fake_run(command,**kwargs):
                    if not succeeds:return subprocess.CompletedProcess(command,1,'','fixture failure')
                    rows='warmup,mode,event_rate_per_second,total_ms\n'+''.join(
                        '0,'+mode+',25000000,1.0\n' for _ in range(10) for mode in copy_sweep.MODES)
                    Path(command[command.index('--csv')+1]).write_text(rows)
                    return subprocess.CompletedProcess(command,0,json.dumps(self.sample()),'')
                with patch.object(sys,'argv',argv),patch.object(copy_sweep.platform,'platform',return_value='fixture'),patch.object(copy_sweep.subprocess,'run',side_effect=fake_run):
                    if succeeds:copy_sweep.main()
                    else:
                        with self.assertRaises(subprocess.CalledProcessError):copy_sweep.main()
                saved=json.loads((output/'report.json').read_text())
                self.assertEqual(saved['status'],'complete' if succeeds else 'failed')
                self.assertEqual((output/'summary.csv').exists(),succeeds)
                self.assertTrue((output/'run-0.stderr').exists())
spec_grouped=importlib.util.spec_from_file_location('grouped_sweep',ROOT/'tools/cuda_benchmark/grouped_sweep.py')
grouped=importlib.util.module_from_spec(spec_grouped);spec_grouped.loader.exec_module(grouped)
class GroupedNotebookToolsTest(unittest.TestCase):
    def sample(self):
        r=CopyNotebookToolsTest().sample()
        old=r.pop('evt3_runs_gpu');new=r['evt3_runs_gpu_direct']
        r['evt3_runs_grouped_gpu']=copy.deepcopy(new)
        r['comparison']='compressed_gpu_histogram'
        for mode in grouped.MODES:
            r[mode].update(compressed_raw_h2d_bytes=0,run_descriptor_h2d_bytes=0,runs_transferred=0)
        r['evt3_runs_grouped_gpu'].update(h2d_payload_bytes=2480,histogram_cpu_copy_bytes=0,
            compressed_raw_h2d_bytes=2000,run_descriptor_h2d_bytes=480,runs_transferred=20)
        return r
    def test_grouped_validation(self):
        grouped.validate(self.sample(),30,65536)
        for field,value in [('run_descriptor_h2d_bytes',481),('histogram_cpu_copy_bytes',1),('h2d_payload_bytes',1000)]:
            r=self.sample();r['evt3_runs_grouped_gpu'][field]=value
            with self.assertRaises(ValueError):grouped.validate(r,30,65536)
        r=self.sample();r['comparison']='histogram_cpu_copy'
        with self.assertRaises(ValueError):grouped.validate(r,30,65536)
    def test_grouped_dry_run(self):
        commands=json.loads(subprocess.check_output([sys.executable,str(ROOT/'tools/cuda_benchmark/grouped_sweep.py'),
            '--binary','/missing','--raw','/missing.raw','--dry-run']))
        self.assertEqual(len(commands),3)
        self.assertEqual([c[c.index('--order-offset')+1] for c in commands],['0','1','0'])
    def test_grouped_report(self):
        # Reporting integration with a fake process; never executes CUDA.
        import tempfile
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as d:
            binary=Path(d)/'binary';binary.write_text('fixture');raw_file=Path(d)/'raw';raw_file.write_bytes(b'fixture')
            output=Path(d)/'result'
            argv=['grouped_sweep.py','--binary',str(binary),'--raw',str(raw_file),'--output',str(output),'--repeats','2']
            def fake_run(command,**kwargs):
                rows='warmup,mode,event_rate_per_second,total_ms\n'+''.join(
                    '0,'+mode+',25000000,1.0\n' for _ in range(10) for mode in grouped.MODES)
                Path(command[command.index('--csv')+1]).write_text(rows)
                return subprocess.CompletedProcess(command,0,json.dumps(self.sample()),'')
            with patch.object(sys,'argv',argv),patch.object(grouped.platform,'platform',return_value='fixture'),patch.object(grouped.subprocess,'run',side_effect=fake_run):grouped.main()
            report=json.loads((output/'report.json').read_text())
            self.assertEqual(report['status'],'complete')
            self.assertEqual(report['summary'][0]['mode'],'evt3_runs_gpu_direct')
            self.assertEqual(report['summary'][1]['run_descriptor_h2d_bytes_median'],480)
if __name__=='__main__':unittest.main()
