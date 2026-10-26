"""Load local models at service startup, then play one bounded readiness tone."""
import asyncio
from pathlib import Path
import time

from .announcements import ALSAPlayer
from .recording_cue import write_cue


class StartupModels:
    REQUIRED = ('asr', 'llm', 'chat', 'tts')
    ALLOWED = frozenset((*REQUIRED, 'vision'))

    def __init__(self, runtime):
        self.rt = runtime
        self.cfg = runtime.cfg.get('startup', {})
        self.enabled = self.cfg.get('enabled') is True
        self.required = tuple(self.cfg.get('required_models', self.REQUIRED))
        self.optional = tuple(self.cfg.get('optional_models', ('vision',)))
        if (not self.required or len(set(self.required + self.optional)) != len(self.required + self.optional)
                or not set(self.required + self.optional) <= self.ALLOWED):
            raise ValueError('Startup model names must be unique supported local models')
        self.ready = not self.enabled
        self.state = 'disabled' if not self.enabled else 'waiting'
        self.loaded = {}
        self.errors = {}
        self.cue_played = False
        self.cue_error = None
        self.task = None

    def worker(self, name):
        if name == 'chat':return self.rt.ai.chat_worker
        if name == 'tts':return self.rt.ai.tts_worker
        return self.rt.workers[name]

    def alive(self, name):
        proc = self.worker(name).proc
        server = self.rt.model_server.proc if name == 'llm' else None
        managed = name == 'llm' and bool(self.rt.cfg['models']['llm'].get('server_command'))
        return proc is not None and proc.returncode is None and (
            not managed or (server is not None and server.returncode is None))

    def start(self):
        if self.enabled and self.task is None:
            self.task = asyncio.create_task(self.run())

    async def load(self, name):
        if self.alive(name):return
        if self.rt.cfg.get('model_loads_enabled', True) is not True:
            raise RuntimeError('Model loading is disabled in the deployment configuration')
        if name == 'llm':await self.rt._ensure_server()
        worker = self.worker(name)
        reply = await worker.call({'op':'health'}, timeout=self.cfg.get('model_timeout_s', 150))
        if reply.get('result', {}).get('health') != 'ready':
            raise RuntimeError(name + ' model did not report readiness')
        self.loaded[name] = {'load_s':worker.load_s, 'ready_at_s':time.monotonic()}
        self.errors.pop(name, None)
        self.rt.audit({'type':'startup_model_ready', 'model':name, **self.loaded[name]})

    async def tone(self):
        device = self.cfg.get('speaker_device') or self.rt.cfg.get('ai_module', {}).get('speaker_device')
        path = Path(self.rt.cfg['root']) / 'cues' / 'models_ready.wav'
        write_cue(path, ((1175, .20),), gain=.25)
        await ALSAPlayer(device,selector=self.rt.cfg.get('audio_roles',{}).get('speaker'))({'path':path, 'duration_s':.20}, {'type':'models_ready'})

    async def run(self):
        while True:
            try:
                missing = [name for name in self.required if not self.alive(name)]
                if missing:
                    self.ready = False
                    self.state = 'loading'
                    # Do not race a task's model call or its cancellation cleanup.
                    if ((self.rt.ai.work and not self.rt.ai.work.done()) or
                            any(not task.done() for task in getattr(self.rt, 'tasks', ()))):
                        await asyncio.sleep(.2)
                        continue
                    for name in missing:
                        try:await self.load(name)
                        except asyncio.CancelledError:raise
                        except Exception as exc:
                            self.errors[name] = type(exc).__name__ + ': ' + str(exc)[:300]
                            raise
                    for name in self.optional:
                        if name not in self.rt.cfg.get('models', {}):continue
                        try:await self.load(name)
                        except asyncio.CancelledError:raise
                        except Exception as exc:
                            self.errors[name] = type(exc).__name__ + ': ' + str(exc)[:300]
                    if not all(self.alive(name) for name in self.required):
                        raise RuntimeError('A required model exited during startup')
                # The first cue finishes before controls become ready for recording.
                if not self.cue_played and not self.rt.ai.active:
                    try:
                        await self.tone()
                        self.cue_played = True
                        self.cue_error = None
                        self.rt.audit({'type':'startup_ready_tone', 'played':True})
                    except asyncio.CancelledError:raise
                    except Exception as exc:
                        self.cue_error = type(exc).__name__ + ': ' + str(exc)[:300]
                self.ready = True
                self.state = 'ready'
                await asyncio.sleep(self.cfg.get('check_interval_s', 5))
            except asyncio.CancelledError:raise
            except Exception as exc:
                self.ready = False
                self.state = 'retrying'
                self.rt.audit({'type':'startup_models_waiting', 'error':str(exc)[:300]})
                await asyncio.sleep(self.cfg.get('retry_s', 5))

    def snapshot(self):
        return {'enabled':self.enabled, 'ready':self.ready, 'state':self.state,
                'required_models':list(self.required), 'loaded':self.loaded,
                'errors':self.errors, 'ready_tone_played':self.cue_played,
                'ready_tone_error':self.cue_error}

    async def close(self):
        if self.task:
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
            self.task = None
