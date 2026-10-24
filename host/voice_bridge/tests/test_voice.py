"""Deterministic audio / boundary / preview tests; no live microphone capture."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import wave
import numpy as np

MODULE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(MODULE))
from audio import Audio, mono16k, quality, read_wav, write_wav
import capture
from pipeline import process_wav, bounded_decode
from preview import parse_task, rule_preview, task_preview


class AudioTests(unittest.TestCase):
    def test_antiphase_channels_do_not_cancel(self):
        x=np.sin(2*np.pi*400*np.arange(1600)/16000).astype(np.float32)*.2
        a=Audio(16000,np.column_stack([x,-x]))
        np.testing.assert_allclose(mono16k(a).samples[:,0],x)
        self.assertFalse(quality(mono16k(a))['below_energy_threshold'])
        with self.assertRaises(ValueError):mono16k(a,2)

    def test_antialiased_48k_to_16k(self):
        t=np.arange(48000)/48000
        original=Audio(48000,.2*np.sin(2*np.pi*1000*t)+.2*np.sin(2*np.pi*12000*t))
        result=mono16k(original).samples[:,0]
        self.assertEqual(len(result),16000)
        spectrum=np.abs(np.fft.rfft(result))
        self.assertGreater(spectrum[1000],50*spectrum[4000])

    def test_44100_duration(self):
        self.assertEqual(len(mono16k(Audio(44100,np.ones(4410)*.1)).samples),1600)

    def test_pcm_roundtrip_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'a.wav';a=Audio(16000,np.array([[-1,.5],[1,-.5]],dtype=np.float32))
            write_wav(p,a);b=read_wav(p)
            self.assertEqual(b.channels,2)
            np.testing.assert_allclose(b.samples,np.clip(a.samples,-1,32767/32768),atol=1/32768)
            with self.assertRaises(FileExistsError):write_wav(p,a)

    def test_bad_audio_rejected(self):
        for data in ([],[float('nan')],[float('inf')]):
            with self.subTest(data=data),self.assertRaises(ValueError):Audio(16000,data)
        with self.assertRaises(ValueError):Audio(1000,[0])

    def test_wrong_wav_encoding(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'8bit.wav'
            with wave.open(str(p),'wb') as f:
                f.setnchannels(1);f.setsampwidth(1);f.setframerate(16000);f.writeframes(b'\x80'*160)
            with self.assertRaises(ValueError):read_wav(p)

    def test_clipping_diagnostic(self):
        self.assertEqual(quality(Audio(16000,np.ones(1600)))['clipping_fraction'],1)


class PipelineTests(unittest.TestCase):
    def run_file(self,values,decoder,**kwargs):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'in.wav';write_wav(p,Audio(16000,values))
            result=process_wav(p,Path(tmp)/'model',Path(tmp)/'out',decoder=decoder,**kwargs)
            saved=json.loads((Path(tmp)/'out/result.json').read_text(encoding='utf-8'))
            self.assertEqual(saved,result)
            self.assertFalse(result['execution_allowed']);self.assertFalse(result['motion_authorized'])
            self.assertTrue(result['requires_confirmation'])
            return result

    def test_silence_skips_asr(self):
        decoder=mock.Mock(side_effect=AssertionError('should not run ASR'))
        result=self.run_file(np.zeros(1600),decoder)
        self.assertEqual(result['status'],'no_speech_energy');decoder.assert_not_called()

    def test_real_format_preview(self):
        result=self.run_file(np.ones(1600)*.1,lambda *a:'找胶带')
        self.assertEqual(result['task_preview']['task'],{'i':'search','t':'tape'})

    def test_domain_policy_preserves_raw_and_blocks_general_language(self):
        result=self.run_file(np.ones(1600)*.1,lambda *a:'找交代')
        self.assertEqual(result['text'],'找交代')
        self.assertEqual(result['task_text'],'找胶带')
        self.assertEqual(result['task_preview']['task'],{'i':'search','t':'tape'})
        result=self.run_file(np.ones(1600)*.1,lambda *a:'请把事情交代清楚')
        self.assertIsNone(result['task_text'])
        self.assertEqual(result['task_preview']['source'],'domain_policy')

    def test_empty_result_cannot_create_task(self):
        result=self.run_file(np.ones(1600)*.1,lambda *a:'')
        self.assertEqual(result['status'],'empty_transcript');self.assertIsNone(result['task_preview'])

    def test_error_and_timeout_do_not_reuse_previous_task(self):
        for failure in (TimeoutError('late'),RuntimeError('model unavailable')):
            result=self.run_file(np.ones(1600)*.1,mock.Mock(side_effect=failure))
            self.assertEqual(result['status'],'error');self.assertIsNone(result['task_preview'])

    @mock.patch('pipeline.subprocess.run',side_effect=subprocess.TimeoutExpired(['asr'],.01))
    def test_worker_timeout_is_reported(self,run):
        with self.assertRaises(TimeoutError):bounded_decode('x.wav','model',.01)

    @mock.patch('pipeline.subprocess.run')
    def test_hotwords_are_forwarded_to_private_asr_worker(self,run):
        run.return_value=subprocess.CompletedProcess([],0,b'{"text":"ok"}',b'')
        self.assertEqual(bounded_decode('x.wav','model',15,2,'words.txt',1.5,'modified_beam_search'),'ok')
        command=run.call_args.args[0]
        self.assertEqual(command[command.index('--hotwords-file')+1],'words.txt')
        self.assertEqual(command[command.index('--decoding-method')+1],'modified_beam_search')

    def test_no_overwrite_results(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'in.wav';write_wav(p,Audio(16000,np.ones(1600)*.1))
            out=Path(tmp)/'out';process_wav(p,'model',out,decoder=lambda *a:'停')
            with self.assertRaises(FileExistsError):process_wav(p,'model',out,decoder=lambda *a:'找胶带')


class PreviewTests(unittest.TestCase):
    def test_negation_unknown_and_compound_are_not_flattened(self):
        for text in ('不要找胶带','找胶带和螺丝刀','先找胶带，找不到再找螺丝刀，找到后返回起点','找椅子腿','捡起胶带','不要停'):
            with self.subTest(text=text):self.assertEqual(rule_preview(text)['task'],{'i':'unknown'})

    def test_basic_stop_and_confirmation(self):
        self.assertEqual(rule_preview('停下！')['task'],{'i':'stop'})
        self.assertEqual(rule_preview('确认')['task'],{'i':'confirm'})

    def test_extra_actions_and_questions_are_not_silently_dropped(self):
        for text in ('找胶带然后前进两米','找胶带并左转','能找胶带吗','找胶带失败怎么办','平','找概念'):
            with self.subTest(text=text):self.assertEqual(rule_preview(text)['task'],{'i':'unknown'})
        self.assertEqual(rule_preview('请帮我寻找胶带，谢谢！')['task'],{'i':'search','t':'tape'})

    def test_duplicate_or_unsupported_keys_and_ungrounded_target(self):
        bad=['{"i":"stop","i":"search"}', '{"i":"search","t":"knife"}',
             '{"i":"search","t":"tape","r":1}', '{"i":"search","t":"tape","pwm":100}',
             '{"i":"stop","t":"tape"}', '{"i":"search","t":"screwdriver"}']
        for value in bad:
            with self.subTest(value=value),self.assertRaises(ValueError):parse_task(value,'找胶带')

    def test_valid_conditional_task(self):
        obj={'i':'search','t':'tape','f':'screwdriver','r':True}
        self.assertEqual(parse_task(json.dumps(obj),'找胶带，没有再找螺丝刀，找到后回起点'),obj)

    @mock.patch('preview.urllib.request.build_opener')
    def test_llm_timeout_fails_closed(self,build):
        build.return_value.open.side_effect=TimeoutError('late')
        result=task_preview('先找胶带，找到后返回', 'http://127.0.0.1:1')
        self.assertEqual(result['task'],{'i':'unknown'});self.assertEqual(result['reason'],'llm_error')

    @mock.patch('preview.urllib.request.build_opener')
    def test_incomplete_llm_result_rejected(self,build):
        response=build.return_value.open.return_value.__enter__.return_value
        response.read.return_value=json.dumps({'choices':[{'finish_reason':'length','message':{'content':'{"i":"search","t":"tape"}'}}]}).encode()
        self.assertEqual(task_preview('先找胶带','http://127.0.0.1:1')['task'],{'i':'unknown'})


class CaptureTests(unittest.TestCase):
    def test_invalid_capture_parameters(self):
        for kwargs in ({'duration':float('nan')},{'duration':100},{'rate':1},{'channels':0}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):capture.record(**kwargs)

    @mock.patch('capture.shutil.which',return_value=None)
    def test_missing_alsa_does_not_install_anything(self,which):
        self.assertFalse(capture.devices('alsa')['available'])
        with self.assertRaises(RuntimeError):capture.record(backend='alsa')

    @mock.patch('capture.shutil.which',return_value='/usr/bin/arecord')
    @mock.patch('capture.subprocess.run')
    def test_usb_disconnect_is_error(self,run,which):
        run.return_value=subprocess.CompletedProcess([],1,b'',b'Input/output error')
        with self.assertRaisesRegex(RuntimeError,'Input/output error'):
            capture.record(backend='alsa',device='plughw:CARD=USB,DEV=0')
        self.assertIsInstance(run.call_args.args[0],list)

    @mock.patch('capture.shutil.which',return_value='/usr/bin/arecord')
    @mock.patch('capture.subprocess.run',side_effect=subprocess.TimeoutExpired(['arecord'],1))
    def test_capture_timeout(self,run,which):
        with self.assertRaisesRegex(RuntimeError,'deadline'):
            capture.record(backend='alsa',duration=1)

    @mock.patch('capture.shutil.which',return_value='/usr/bin/arecord')
    @mock.patch('capture.subprocess.run')
    def test_alsa_success_selects_explicit_device_and_trims(self,run,which):
        def recorded(command,**kwargs):
            self.assertEqual(command[command.index('-D')+1],'plughw:CARD=USB,DEV=0')
            self.assertEqual(command[command.index('-d')+1],'2')
            write_wav(command[-1],Audio(48000,np.ones((96000,2))*.1))
            return subprocess.CompletedProcess(command,0,b'',b'')
        run.side_effect=recorded
        audio,source=capture.record(backend='alsa',device='plughw:CARD=USB,DEV=0',duration=1.25,rate=48000,channels=2)
        self.assertEqual(audio.samples.shape,(60000,2))
        self.assertEqual(source['actual_rate'],48000)


if __name__=='__main__':unittest.main()
