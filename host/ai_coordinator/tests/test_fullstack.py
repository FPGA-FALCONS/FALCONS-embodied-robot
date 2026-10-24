import asyncio
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.runtime import Runtime
from ai_coordinator.reference_peer import SimulationGateway
from test_runtime import FakeWorker,FakeServer


class FullStackTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        cfg={'root':str(self.root),'mode':'simulation','models':{'llm':{}},'input_roots':[str(self.root)]}
        self.config=self.root/'config.json';self.config.write_text(json.dumps(cfg))
        self.rt=Runtime(self.config);self.rt.model_server=FakeServer()
        self.rt.gateway=SimulationGateway(self.rt.core.session_id,self.rt.audit)
        self.rt.workers={'llm':FakeWorker({'content':'{"i":"search","t":"tape","f":"screwdriver","r":true}','finish_reason':'stop'}),
                         'asr':FakeWorker({'text':'先找胶带，找不到找螺丝刀，找到后返回起点','status':'recognized'}),
                         'vision':FakeWorker({'width':640,'height':480,'detections':[]})}
        self.wav=self.root/'sample.wav';self.wav.write_bytes(b'fixture')
        self.image=self.root/'sample.jpg';self.image.write_bytes(b'fixture')
        await self.rt.start()

    async def asyncTearDown(self):await self.rt.close();self.temp.cleanup()

    async def job(self,submitted):
        for _ in range(200):
            row=self.rt.jobs[submitted['job_id']]
            if row['status']!='running':
                self.assertEqual(row['status'],'done',row);return row
            await asyncio.sleep(.005)
        self.fail('runtime job timeout')

    async def ack(self,kind):
        self.rt.gateway.respond(self.rt.gateway.commands[-1],kind);await asyncio.sleep(.04)

    async def test_audio_fallback_vision_return_and_speech_over_fragmented_wire(self):
        await self.job(self.rt.submit_audio({'wav':str(self.wav)}))
        await self.ack('search_started')
        await self.job(self.rt.submit_image({'image':str(self.image)}))
        search=self.rt.gateway.commands[-1];self.rt.gateway.respond(search,'region_exhausted');await asyncio.sleep(.04)
        self.assertEqual(self.rt.core.target,'screwdriver');await self.ack('search_started')
        self.rt.workers['vision'].result['detections']=[{'class':'screwdriver','confidence':.9,'bbox':[1,1,30,30]}]
        for _ in range(2):await self.job(self.rt.submit_image({'image':str(self.image)}))
        self.assertEqual(self.rt.core.state,'waiting_found_ack')
        await self.ack('target_accepted');self.assertEqual(self.rt.core.state,'returning')
        await self.ack('home_arrived');self.assertEqual(self.rt.core.state,'completed')
        self.assertGreater(self.rt.gateway.wire_frames,8)
        self.assertFalse(self.rt.snapshot()['mission']['motion_authorized'])
        log=''.join(p.read_text(encoding='utf-8') for p in self.rt.run_dir.glob('events-*.jsonl'))
        self.assertIn('sim_completed',log);self.assertIn('sim_found_screwdriver',log)

    async def test_missing_home_receipt_does_not_complete(self):
        self.rt.workers['llm'].result['content']='{"i":"search","t":"tape","r":true}'
        await self.job(self.rt.submit_text('找胶带，找到后返回起点'));await self.ack('search_started')
        self.rt.workers['vision'].result['detections']=[{'class':'tape','confidence':.9,'bbox':[1,1,30,30]}]
        for _ in range(2):await self.job(self.rt.submit_image({'image':str(self.image)}))
        await self.ack('target_accepted');self.assertEqual(self.rt.core.state,'returning')
        self.rt.core.tick(self.rt.core.deadline+.1);self.rt.dispatch()
        self.assertEqual(self.rt.core.state,'failed')

    async def test_cancelled_device_action_cannot_report_late_success(self):
        await self.job(self.rt.submit_text('先找胶带，找不到找螺丝刀，找到后返回起点'))
        action=self.rt.gateway.commands[-1];self.rt.stop()
        with self.assertRaises(ValueError):self.rt.gateway.respond(action,'search_started')
        self.assertEqual(self.rt.core.state,'stopped')

    async def test_patrol_voice_and_fpga_receipts_resume_interrupted_waypoint(self):
        self.rt.cfg['patrol']={'enabled':True,'waypoints':['P1','P2'],'targets':['screwdriver'],'cycles':1,'stop_band_m':[.2,.4]}
        self.rt.workers['asr'].result['text']='开始巡航'
        await self.job(self.rt.submit_audio({'wav':str(self.wav)}))
        self.assertEqual(self.rt.core.state,'patrolling');self.assertEqual(self.rt.workers['llm'].calls,[])
        self.rt.workers['vision'].result['detections']=[{'class':'screwdriver','confidence':.9,'bbox':[1,1,30,30]}]
        for _ in range(2):await self.job(self.rt.submit_image({'image':str(self.image)}))
        await self.ack('patrol_paused');approach=self.rt.gateway.commands[-1]
        self.rt.gateway.respond(approach,'approach_arrived',metric_stop_verified=True,stationary=True,clearance_m=.3)
        await asyncio.sleep(.04);self.assertEqual(self.rt.core.state,'waiting_pickup')
        self.rt.workers['asr'].result['text']='拾取完成'
        await self.job(self.rt.submit_audio({'wav':str(self.wav)}))
        self.assertEqual(self.rt.gateway.commands[-1]['waypoint'],'P1')
        self.assertEqual(self.rt.core.operator_confirmations,1)
        self.assertFalse(self.rt.core.snapshot()['patrol']['physical_pickup_verified'])


if __name__=='__main__':unittest.main()
