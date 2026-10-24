# 中文任务播报

协调器按任务状态发出固定语音事件，播放工作独立异步执行，不调用LLM生成播报、不加载板端TTS模型。当前板端默认 `announcements.mode=log`，只记日志；真实USB音箱未连接，不能视为已通过实物发声/回声测试。

## 什么时候说什么

| 事件 | 短句 | 触发依据 |
| --- | --- | --- |
| accepted_物体 | 已接收寻找胶带的任务 | 有效任务进入状态机 |
| search_物体 | 开始寻找胶带 | 对应FPGA search_started回执 |
| found_物体 | 已找到胶带 | 连续视觉确认，随后对应target_accepted回执 |
| returning | 正在返回起点 | 状态机发起返回；不等同已经到达 |
| completed | 任务已完成 | 搜索类target_accepted；带返回任务还须home_arrived |
| stop_requested | 已请求停止任务 | 停止请求已发出；不声称电机已停 |
| timed_out / not_found / failed | 任务超时／未找到目标／任务中断 | 对应状态机失败原因，空画面不直接等于未找到 |

四个目标为胶带、螺丝刀、AD钙奶、营养快线。18条普通短句＋18条带“模拟任务”前缀的短句，共36段；shadow和simulation只使用带前缀版本。plan-only和过期模型结果不会触发播报。

播报队列最多8条、等待超过10秒丢弃；同一事件不重复。新任务打断旧任务播报，停止/失败清空旧队列。音频缺失、校验错误或声卡失败只影响语音，记录原因；缺资源时降为log。播放器超时/取消只终止自己的子进程。

第一版半双工：真实录音期间播报等待；发声及结束后300ms暂不接受新麦克风采集请求。文件音频回放不受影响。**这不是AEC，也不保证播报期间能用语音急停**；FPGA、遥控和硬件急停保持独立。未来若要随时语音打断，需实际回声消除与并发识别验收。

## 资产生成与校验

音频由电脑已有Windows中文语音 `System.Speech` 离线生成，PCM16、单声道、16kHz；不安装或下载TTS引擎。代码入仓库，生成音频留在本地AI产物目录。

```powershell
powershell.exe -NoProfile -File host/ai_coordinator/build_speech.ps1 -Python '<现有Python路径>' -OutputDirectory '<新的音频目录>'
python host/ai_coordinator/speech_assets.py check --assets '<音频目录>'
python host/ai_coordinator/speech_assets.py demo --assets '<音频目录>' --out '<模拟示例.wav>'
```

生成器拒绝覆盖已有WAV。manifest记录精确文案、文件名及SHA256；验证器同时检查PCM格式、时长、大小和目录边界。demo使用真实任务状态机、模拟图像检测/FPGA回执组装音频，并写出事件与 `physical_task_completed=false` 元数据；不是实车执行录像或ASR/LLM实测。

本轮资产位于 `D:/HITSZ_FALCONS/ai部分/voice_announcements_20261024/assets/`，36段合计4,028,536字节、125.84秒，Windows中文语音名称记在generation.json。模拟搜索和返回示例位于该目录上层。

## 板端部署与接音箱

资产位于 `/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1/speech/`。已验证全部SHA256并fsync，默认只记日志，不选择板载声卡。

USB音箱到货后先用 `aplay -l`、`aplay -L` 找出实际卡名；选支持USB音频、Linux免驱的型号。修改该部署自己的config.json中的announcements：

```json
{
  "mode": "alsa",
  "assets": "/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1/speech",
  "device": "plughw:CARD=实际USB卡名,DEV=0"
}
```

示例中的卡名是占位符，须按实际设备替换。只接受显式CARD/DEV，不允许default或按数字猜声卡；plughw可适配设备采样格式。修改后重启自己的 `falcons-ai-coordinator.service`。`/status`的announcements字段显示模式、队列、是否发声、录音互斥及最近错误。先做单段小音量发声/拔插测试，再测麦克风是否识别自己的播报；当前板端模型重负载仍HOLD，不因此恢复长测。

## 本轮验证

Windows46项通过、5项Linux串口跳过；RK3588共51/51通过。新增12项涉及状态回执与播报、替换/抢占/去重/过期、缺资源降级、录音互斥、播放失败及真实私有播放子进程取消回收；播放进程测试用受控Python子进程替代aplay，没有向声卡输出。

本轮未启动ASR/LLM/视觉或板端TTS模型，未升级环境，未提交/推送。51项是软件验证，不是36段真人听评、真实音箱播放或远场回声验收。

## 当前选定音色：Kokoro 女声A

用户试听后明确选择女声A（Kokoro v1.1-zh，zf_001，sid=3），替换原微软惠惠作为当前缓存音色；男声/女声B及旧音频保留。神经TTS仅在电脑CPU生成，未安装到RK3588。当前36段音频合计2,899,282字节、90.553秒。

板端当前assets指向 `/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1/speech_kokoro_zf001_20261024/`；mode仍log，未打开声卡。旧speech目录保留。资源清单SHA256：`aca6ac61473a25001b4b5013f0f4c190bec587ba52b4bdd5fd843f4d7d2128eb`。

新生成器为 `build_neural_speech.py`，支持audition与cache，使用现有Sherpa-ONNX/NumPy/SciPy，无自动安装或下载。例：

```sh
python host/ai_coordinator/build_neural_speech.py cache --model-dir '<已有Kokoro模型目录>' --out '<新的缓存目录>' --sid 3
```

维护者来源：[Sherpa多语种模型](https://github.com/k2-fsa/sherpa-onnx/releases/tag/tts-models)、[v1.1-zh导出说明](https://github.com/k2-fsa/sherpa-onnx/tree/master/scripts/kokoro/v1.1-zh)。模型压缩包364,816,464字节，已匹配维护者SHA256 `a3f4c73d043860e3fd2e5b06f36795eb81de0fc8e8de6df703245edddd87dbad`；包内附Apache-2.0许可。模型/来源/校验记录只在本机AI目录，不入Git。

模型原始输出24kHz；试听保留24kHz，缓存抗混叠转换16kHz以沿用现有播放器合同。Windows原生eSpeak读取中文路径失败，使用ASCII目录junction引用原模型，并保留该路径拼写；没有复制或改动系统语音库。

AD被英语前端按单词解析，生成器采用中文发音拼写“艾迪钙奶”，generation.json同时记录canonical text和spoken_text；任务合同/目标名称仍为AD钙奶。ASR只用于内容粗查，不是发音或自然度证明；保留原始候选及试听，实际音箱与真人听评仍待完成。

麦克风到货后，ASR→任务模型→状态事件可继续使用此音色。缓存短句直接播放；任意新文本的板端实时TTS尚未部署、测速，不把电脑合成速度当RK3588速度。
