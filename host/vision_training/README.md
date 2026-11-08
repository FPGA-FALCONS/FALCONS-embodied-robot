# 视觉模型头部修正实验

范围：2026-10-24的离线实验与v17历史基线，以下“不替换v17”只描述该轮结论。生产模型已于10-29切换为K541 P02，现行资产见[MODELS](../MODELS.md)；本目录不改变当前板端模型。

最终追加P3对照已完成：`p3_repair.py`训练P3融合层和one2one头，比较监督训练与父Nano饮料分支保持，`evaluate_p3_repair.py`记录两支输出漂移和ONNX一致性。两臂均未改善mini工作点6/8召回，停止在这组复用开发集上继续调参，不替换v17。结果见[全栈整合报告](../../doc/archive/ai/AI整合与模型对照_20261023-24.md#report-04)。

用于电脑上现有的四类YOLO26 Nano候选，不是车辆执行模块。输入/输出和环境约束：不安装或升级包，不使用test，不替换默认模型，不访问板端。

`head_repair.py prepare`复制标签/权重、链接或复制训练图片，写出冻结协议、内容哈希、数据划分与门槛。只创建新的实验目录；训练缓存位于该目录，原数据/权重保持原样。图片hardlink只用于只读训练输入，不能在新目录里编辑图片。

`train`的两臂从相同checkpoint开始，各固定6轮、seed106、640输入。均计算相同teacher，普通对照权重0、蒸馏权重0.3；只允许one2one检测头学习，骨干和one2many参数/BN统计冻结。每轮实际检查冻结张量和teacher无梯度；最终EMA last中非one2one张量恢复为源模型精确值，避免EMA数值漂移。没有epoch挑选、阈值扫描或新增推理算子。

蒸馏限制在teacher置信度≥.25、teacher最佳类别为工具、且锚点处于同类真实标注框扩大一个stride范围内的位置。使用T=2 Bernoulli KL和原始ltrb SmoothL1；这是本项目输出层试验，不是FGD完整算法。外部标签属于提议/复核，不能写成人工金标。

```sh
python host/vision_training/head_repair.py prepare --out '<新的实验目录>' --research-root '<已有rknn研究目录>'
python host/vision_training/head_repair.py train --out '<实验目录>' --arm head_supervised
python host/vision_training/head_repair.py train --out '<实验目录>' --arm head_teacher_kd
python host/vision_training/evaluate_head_repair.py --experiment '<实验目录>'
```

评估先复现原Nano类别分支基线，再导出原有四输出合同，检查PyTorch/ONNX几何/分数一致性、one2many输出不变、参数量与算子类型计数，复用既有类别保留/NMS/COCO评价。配套sidecar不可省略，不能用现有单输出v17播放器解析新四输出。

验收不只看AP：需mini召回6→至少7/8且FP不增加，困难集8/8且FP≤1，旧类TP≥12、FP≤1；同时有冻结AP和逐类退化门槛。具体以运行前protocol.json为准。失败结果保留，不能自动用较好看的指标替换门槛。

本轮实验目录为 `D:/FALCONS_workspace/ai部分/yolo_head_repair_20261024/`。hard/mini各只有一个视频的8帧，旧类12图，且这些开发集已反复使用；没有足够独立组来给出可信的泛化区间。本轮只能报告开发回归，不能报告独立泛化、RK3588提速或实车能力。

电脑GPU由 `ai部分/AI_视觉优化协同_20261024.md` 协调；训练窗口结束前不与Claude的Qwen CUDA负载并发。板端重负载仍HOLD，电脑评估通过不构成板端验收许可。

## 2026-10-24结果

两臂各6轮及一次6轮自适应后续全部完成。最好的同锚点KD使Nano的hard AP .8175→.8759、mini .8047→.8196、old .9721→.9832；工作点误检数不增，但mini仍6/8，未通过召回门槛。按对象匹配后续hard .8804、mini .7684，因退化拒绝。**没有默认替换或新板测提名。** 完整对照与证据界限见[验证报告](../../doc/archive/ai/AI整合与模型对照_20261023-24.md#report-06)。

`semantic_kd.py`提供按GT对象匹配各自锚点的数学smoke，运行该文件可复查零损失/学生梯度/teacher无梯度与类别排除。该算法并未改善本轮小目标召回，不能作为已验证升级宣传。

后续训练入口 `--arm head_semantic_kd` 需要实验目录里已有semantic_followup.json（父protocol的SHA256与weight），本轮真实文件已保存。独立评估可用 `--arm head_semantic_kd --output-name evaluation_semantic`；新目录不能覆盖原结果。原始两臂源码另有冻结快照，便于区分自适应改动。
