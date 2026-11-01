"""Build a bounded evidence report from preserved development experiment outputs."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics


def read(path):return json.loads(path.read_text(encoding='utf-8'))
def lines(path):return [json.loads(x) for x in path.read_text(encoding='utf-8').splitlines()]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--asr-model',type=Path,required=True);args=ap.parse_args();w=args.work
    original=read(w/'e2e_run_1/summary.json');policy=read(w/'policy_after_user_request_v2/summary.json')
    stronger=read(w/'sensevoice_comparison/summary.json');audit=read(w/'export_audit.json')
    train=read(w/'training_metadata.json');download=read(args.asr_model/'download_manifest.json')
    rows=lines(w/'e2e_run_1/domain_results.jsonl');rows=[r for r in rows if r['mode']=='hotwords']
    usage=[r['parser']['usage'] for r in rows]
    estimates={str(t):round(statistics.median(.58774*u['prompt_tokens']/128+(u['completion_tokens']-1)/t for u in usage),3) for t in (10,20,27.05)}
    report='''# 四类物体语音与任务回放验证报告

日期：2026-10-24。建议保留关闭思考的四类专项模型与“交代”领域策略；SenseVoice 作为下一轮板端测试候选，小型 transducer 保留作资源对照。板端选型由项目组在 RK3588 延迟、内存和真人录音测试后决定。

## 结论及证据边界

| 待判断命题 | 状态 | 可支持的表述与证据 |
| --- | --- | --- |
| 四类目标的语音语义适配有效 | partially supported | 合成开发集上相对旧专项父模型改善；没有独立真人集，不能推断普遍能力更强 |
| 默认任务文本禁用“交代” | supported | 搜索语境纠错，含糊用法拒识，原始日志保留；27+13 项自动检查和实际回放 |
| TTS→ASR→Qwen→检测→任务流程可接通 | supported | 本机真实组件推理及固定图片任务回放；导航回执模拟 |
| 模型胜过官方 Qwen3.5 原版 | not tested | 本轮父模型已被专项微调，不是官方原版 |
| RK3588 实时语音控制可用 | not tested | 没有板端转换、部署、真人麦克风或实车证据 |
| 视觉能力保持或变强 | not tested | 视觉张量冻结只证明未改权重；检测来自另一个已有 YOLO |

## 训练与导出

父模型：`../qwen_offline_distill_pilot_20261023/merged_hf`。本轮使用555条合成模板／旧训练回放，训练128步，batch 4，LoRA rank 8、alpha 16，学习率5e-5。只训练最终JSON答案，关闭思考；可训练参数5,411,328，RTX 5090 D训练约56.7秒。本轮未从在线强模型新采集推理链，未训练声学模型和YOLO。

产物：`adapter/`、`merged_hf/`、`domain-Q8_0.gguf`（811,842,976字节）。审计186个允许的语言层张量改变、287个不变，其中153个视觉张量与父权重相同；无非有限权重，tokenizer文件一致。视觉张量不变不能替代视觉精度测试。

GGUF SHA256：`{gguf}`。

合并HF权重 SHA256：`{merged}`。

## 首轮开发对照

16个语义场景×两种Windows系统TTS声音，共32段16kHz单声道PCM16。声音为Huihui与Kangkang，直接写文件，没有扬声器／空气／麦克风链路。以下音频路线只把ASR输出送入模型；原句用于TTS与评分。独立纯文字诊断另计。

| ASR | CER | 本机CPU处理时间中位数 | pilot预期匹配 | domain预期匹配 |
| --- | ---: | ---: | ---: | ---: |
'''.replace('{gguf}',audit['gguf_sha256']).replace('{merged}',audit['merged_sha256'])
    for mode in ('greedy','beam','hotwords'):
        a=original['asr_modes'][mode];t=original['task_results']
        report+=f"| {mode} | {a['CER']:.2%} | {a['median_seconds']:.3f}s | {t['pilot'][mode]['expected_outcome_matched']}/32 | {t['domain'][mode]['expected_outcome_matched']}/32 |\n"
    report+='''
纯文字诊断为pilot 15/16、domain 16/16，不能算进音频正确率。两模型三个ASR模式的误搜索、漏停止均为0；每种模式只有16段非搜索控制语音、2段停止语音，不能据此断言误触发率为零。固定beam不加词对照用于区分解码搜索与热词的影响。

domain热词模式三处失败：女声AD钙奶→“概念”、男声AD钙奶→“奶”、男声纠正目标→“沼罗斯刀”。错误未被强行归为物体，完整原始输出保留在 `e2e_run_1/`。

## 用户要求禁用“交代”后的政策对照

`text`保留原始转写；明确搜索语境中将“交代”改为胶带，供解析的字段为`task_text`。仍含该词的普通／含糊语言拒识，不调用模型。保留否定和条件，不把“概念”或单独“奶”补成AD钙奶。这是后处理纠错／拒识，并非从声学词表删除词汇。

第一次政策回放漏掉“找到交代”模式，导致domain降到27/32。修复后重新运行于v2目录，旧失败未覆盖：

| 模型 | 相同32段热词转写＋政策 | 事后简称探测 |
| --- | ---: | ---: |
'''
    for model in ('pilot','domain'):
        report+=f"| {model} | {policy[model]['hotwords']['expected_outcome_matched']}/32 | {policy[model]['spoken_alias_probe']['expected_outcome_matched']}/10 |\n"
    report+='''
上述简称探测是在看到AD失败后追加的5句×2声音，并非盲测。小ASR依然将“钙奶”听成“概念”等，说明仅继续训练语言层不能可靠恢复已经丢失的声学信息。

## 新增SenseVoice对照（事后开发选择）

固定原来的32段与上述10段音频，使用SenseVoiceSmall INT8，CPU两线程、中文、ITN开启、不加热词，保留相同政策、Qwen权重与图片证据。它在看过失败后被选用，分数只能作为开发选择依据。

| 项目 | 原32段 | 简称10段，另计 |
| --- | ---: | ---: |
'''
    for field,label in [('cer','字错率'),('median_processing_s','识别处理时间中位数')]:
        a=stronger['asr']['main'][field];b=stronger['asr']['spoken_alias_probe'][field]
        report+=f'| {label} | '+(f'{a:.2%} | {b:.2%}' if field=='cer' else f'{a:.3f}s | {b:.3f}s')+' |\n'
    for model in ('pilot','domain'):
        values=stronger['models'][model]
        report+=f"| {model}预期结果匹配 | {values['main']['expected_outcome_matched']}/32 | {values['spoken_alias_probe']['expected_outcome_matched']}/10 |\n"
    report+=f'''
两模型在本对照中的误搜索、漏停止均为0。domain的32/32包含搜索成功和正确的停止、拒识、确认，不表示机器人执行了32次实体任务。SenseVoice仍把部分胶带识别为“交代”，规则层纠错继续发挥作用；男声AD输出“ATD钙奶”，其中钙奶语义保留。CER不为零但预期任务可全部匹配。

SenseVoice权重239,233,841字节（约228.2MiB），来自`sherpa-onnx`维护者的Hugging Face转换仓库，revision `{download['revision']}`。模型SHA256为`{download['files'][0]['sha256']}`，已对照上游LFS校验。详细来源、文件摘要保存在`{args.asr_model.as_posix()}/download_manifest.json`。没有安装包或执行上游脚本。包内许可仅指向FunASR说明，未重新分发权重。

## 图像、流程与超时

YOLO使用已有v17s候选`field_adaptation_20261021_v16_lab122/runs/v17s_replay/weights/best.pt`。四类各两张已有开发图片，选择在预测前固定；640输入、置信度阈值0.3。本轮未训练视觉模型。背景场景是AD钙奶干扰图片，不是真实空房间。匹配目标类别、置信度和合法框后认为该图片窗口找到目标；未检测到只代表窗口耗尽。

任务图保留首选／未找到才备选／成功后返回的关系。primary成功跳过fallback；没有模拟导航到家回执则不能完成return_home。`verify`节点仅检查检测结构／类别，不是独立VLM复核。导航、返回与搜索场景均为脚本回放，`simulation_only=true`、`motion_authorized=false`、`physical_task_completed=false`。

两个Qwen实际强制超时探测均终止并回收自己的私有进程。SenseVoice独立探测也证明超时路径返回，`subprocess.run`终止并等待私有worker；零能量音频在调用ASR前拒识。ASR默认15秒包含模型加载，Qwen每请求15秒、2048上下文、最大输出80token、显式关闭思考；不是全链路含录音与导航的总时限。

已实跑`demo.py`：合成条件指令→transducer→Qwen→实际YOLO→fallback→模拟返回成功。另以SenseVoice测试AD钙奶单命令入口。输出在`demos/`，不覆盖主实验。

## RK3588仅估算语言部分

本轮domain热词路线实际输入token最小／中位／最大为194／197／205，输出为6／8／20；含拒识等短答案。用前次记录的Rockchip公开满频参考TTFT=0.58774秒（128输入token）、27.05token/s进行情景计算：

`时间 = 0.58774 × 输入token/128 + (输出token−1)/TPS`

| 生成速度假设 | 本轮语言解析中位估算 |
| --- | ---: |
'''
    for t,value in estimates.items():report+=f'| {t} token/s | {value:.3f}s |\n'
    report+='''
公开参考记录见`../qwen_offline_distill_pilot_20261023/protocol.json`与`../qwen08b_threeway_20261023/threeway_report.md`。10／20TPS仅改变解码速度，是敏感性场景；线性预填充未经板校准，不能视为高负载保守上界。没有从GPU时间缩放板速。GGUF Q8与RKLLM W8A8不等价，新权重尚未转换，量化后正确率未知。

ASR、YOLO、加载、排队、CPU/NPU资源竞争和导航耗时均未得到板端测量，不能给出可信的全链路秒数。SenseVoice文件约228MiB也不是其峰值RAM。增加256个思考token，仅生成额外约9.46秒（27.05TPS）到25.6秒（10TPS）；因此当前专项链路继续关闭思考，把资源用在声学和视觉证据上。此反事实未测思考模式精度。

## 独立性、失败与待验收

配置及输入哈希在各目录protocol／manifest中。训练排除了测试原句的完全相同字符串，但相同模板、目标与设计者意味着语义重叠；16场景的两种TTS不能视为32个独立真人任务。场景标签由模板预设，没有真人标注者、盲审或一致性评估。作者同时实施与评估，没有独立验证者；通过冻结配置、保留全部失败和隔离运行提高可追溯性，不能替代独立测试。

未接真实麦克风、FPGA实时图像、RK3588或实车；未改变板端系统、Python、驱动和竞赛环境。下一步是独立真人指令与环境噪声测试，再在现有板环境分别测ASR、RKLLM W8A8和视觉。重点保留停止、否定、相近名称、未知物体和返回超时负例。只有通过这些验收，才能讨论实车启用。

仓库入口：`D:/FALCONS_workspace/FALCONS-embodied-robot/host/voice_task_demo/README.md`。本轮未提交或推送。
'''
    (w/'验证报告.md').write_text(report,encoding='utf-8')
    inputs=[w/'e2e_run_1/summary.json',w/'policy_after_user_request_v2/summary.json',
            w/'sensevoice_comparison/summary.json',w/'export_audit.json',w/'training_metadata.json',
            args.asr_model/'download_manifest.json',Path(__file__).resolve()]
    (w/'report_sources.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},indent=2),encoding='utf-8')
    print(w/'验证报告.md')


if __name__=='__main__':main()
