"""Separate conversational Qwen and dynamic Kokoro workers; no motion path."""
import json
import os
from pathlib import Path
import subprocess
import sys
import urllib.request
import wave


class Chat:
    def __init__(self, cfg):
        self.cfg = cfg
        self.native = None
        if cfg.get('provider') == 'rkllm':
            command = cfg.get('command')
            if not isinstance(command, list) or not command or not all(isinstance(x,str) for x in command):
                raise ValueError('RKLLM chat requires an explicit argv command')
            env = dict(os.environ)
            library = cfg.get('library_dir')
            if library:
                env['LD_LIBRARY_PATH'] = str(library) + ':' + env.get('LD_LIBRARY_PATH','')
            self.native = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=sys.stderr, text=True, encoding='utf-8', errors='replace', env=env)
            while True:
                line = self.native.stdout.readline()
                if not line:
                    raise RuntimeError('RKLLM chat adapter exited during initialization')
                if line.strip() == 'FALCONS_CHAT_READY':
                    break
            return
        self.url = cfg['url'].rstrip('/')
        if not self.url.startswith('http://127.0.0.1:'):
            raise ValueError('Chat must use the explicitly configured local Qwen server')
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def run(self, request):
        text = request['text']
        if not isinstance(text, str) or not 1 <= len(text) <= 1000:
            raise ValueError('Invalid chat utterance')
        history = request.get('history', [])[-8:]
        if any(m.get('role') not in ('user', 'assistant') or
               not isinstance(m.get('content'), str) or len(m['content']) > 1000 for m in history):
            raise ValueError('Invalid conversation history')
        messages = [{'role': 'system', 'content': self.cfg.get('prompt',
            '你是小车上的中文聊天助手。自然、简短地回答，通常不超过三句话。'
            '你只能聊天，不能执行车辆运动，不要声称已经控制电机或完成导航。')}]
        messages += history + [{'role': 'user', 'content': text}]
        if self.native is not None:
            question = 'user: '+text+' assistant: '
            system = messages[0]['content']+' '
            if len((system+question).encode())>4000:
                raise ValueError('Utterance is too long for the bounded native chat prompt')
            turns = []
            for item in reversed(history):
                turn=item['role']+': '+item['content']+' '
                if len((system+turn+''.join(turns)+question).encode())>4000:break
                turns.insert(0,turn)
            prompt = system+''.join(turns)+question
            # Line protocol is data, never a shell command or a demo command.
            prompt = prompt.replace('\r',' ').replace('\n',' ')
            self.native.stdin.write(prompt+'\n');self.native.stdin.flush()
            while True:
                line = self.native.stdout.readline()
                if not line:
                    raise RuntimeError('RKLLM chat adapter exited during generation')
                if line.startswith('FALCONS_CHAT '):
                    native = json.loads(line[len('FALCONS_CHAT '):])
                    if not native.get('ok'):
                        raise RuntimeError('RKLLM generation failed')
                    answer = native.get('text','').strip()
                    if not answer or len(answer)>1000 or '<think>' in answer:
                        raise ValueError('Invalid RKLLM chat response')
                    return {'text':answer,'engine':'Qwen3.5-0.8B / RKLLM 1.3.1','motion_executed':False}
        body = {'messages': messages, 'temperature': .6, 'max_tokens': 192,
                'chat_template_kwargs': {'enable_thinking': False}, 'stream': False}
        req = urllib.request.Request(self.url + '/v1/chat/completions',
            data=json.dumps(body, ensure_ascii=False).encode(), headers={'Content-Type': 'application/json'})
        with self.opener.open(req, timeout=self.cfg.get('timeout_s', 60)) as response:
            raw = response.read(65537)
        if len(raw) > 65536:
            raise ValueError('Oversized chat response')
        result = json.loads(raw)
        answer = result['choices'][0]['message'].get('content', '').strip()
        if not answer or len(answer) > 1000 or '<think>' in answer:
            raise ValueError('Empty, oversized or unfiltered thinking response')
        return {'text': answer, 'engine': self.cfg.get('label', 'Qwen3.5-0.8B'),
                'usage': result.get('usage'), 'motion_executed': False}

    def close(self):
        if self.native and self.native.poll() is None:
            self.native.terminate()
            try:self.native.wait(timeout=2)
            except subprocess.TimeoutExpired:self.native.kill();self.native.wait(timeout=2)


class DynamicTTS:
    def __init__(self, cfg):
        import sherpa_onnx
        self.cfg = cfg
        root = Path(cfg['model_dir']).resolve()
        self.output_root = Path(cfg['output_root']).resolve()
        config = sherpa_onnx.OfflineTtsConfig(model=sherpa_onnx.OfflineTtsModelConfig(
            kokoro=sherpa_onnx.OfflineTtsKokoroModelConfig(
                model=str(root / cfg.get('model_file', 'model.onnx')), voices=str(root / 'voices.bin'),
                tokens=str(root / 'tokens.txt'), data_dir=str(root / 'espeak-ng-data'),
                lexicon=','.join(str(root / name) for name in ('lexicon-us-en.txt', 'lexicon-zh.txt'))),
            num_threads=cfg.get('threads', 2), provider='cpu'), max_num_sentences=1)
        if not config.validate():
            raise ValueError('Dynamic TTS assets/configuration unavailable')
        self.tts = sherpa_onnx.OfflineTts(config)
        self.sid = cfg.get('sid', 3)
        if not 0 <= self.sid < self.tts.num_speakers:
            raise ValueError('Invalid TTS speaker')

    def run(self, request):
        import numpy as np
        text = request['text']
        path = Path(request['output']).resolve()
        if not isinstance(text, str) or not 1 <= len(text) <= 1000:
            raise ValueError('Invalid TTS text')
        if self.output_root not in path.parents or path.suffix != '.wav':
            raise ValueError('TTS output must stay in the private run directory')
        audio = self.tts.generate(text, sid=self.sid, speed=1.)
        samples = np.asarray(audio.samples, dtype=np.float32)
        duration = len(samples) / audio.sample_rate
        if not np.isfinite(samples).all() or not .05 <= duration <= 90:
            raise ValueError('Invalid or oversized synthesized speech')
        with wave.open(str(path), 'wb') as wav:
            wav.setparams((1, 2, audio.sample_rate, 0, 'NONE', 'not compressed'))
            wav.writeframes(np.rint(np.clip(samples, -1, 1) * 32767).astype('<i2').tobytes())
        return {'path': str(path), 'duration_s': duration, 'engine': 'Kokoro v1.1 zh', 'sid': self.sid}
