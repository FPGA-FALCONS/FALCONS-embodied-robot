"""Export the vehicle's source and local dependencies without development debris."""
import argparse
import ast
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import zipfile


DOCS = {
    '竞赛源码提交说明.md',
    'HT10A接线与验收.md', 'ACG720_引脚接线手册_RevI.pdf',
    '高云 ACG720-138&60K通用管脚分配表.xlsx', '参数总表与待定清单.md',
    '避障导航操作指南.md', '导航控制方案.md', '上板指南_v1.md',
    'AI模块页面与遥控交互_20261024.md', 'AI视觉巡检.md',
    'AI巡检录像与报告.md', '七点地面单应性标定_20261028.md',
    'RK模型开机加载与手动暂停_20261025.md', 'RK相机固定对焦_20261027.md',
    '语义运动指令与FPGA运动桥_20261024.md',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def select(name, packages):
    parts = Path(name).parts
    if any(p in {'__pycache__', '.pytest_cache', 'evidence'} or p.startswith('evidence_') for p in parts):
        if name == 'doc/evidence/ai_release_20261024/MODEL_MANIFEST.json':
            return True, '历史模型资产清单（不是当前板端快照）'
        return False, '过程证据或缓存；不参与源码构建'
    if name in {'build.tcl', 'build_syn.tcl', 'build_pnr.tcl', '.gitattributes', '.gitignore'}:
        return True, '整车构建入口及文本规则'
    if parts[0] in {'rtl', 'constr', 'scripts', 'sim', 'deps', 'tools'}:
        return True, '整车源码、约束、仿真与开发工具'
    if parts[0] == 'host':
        return True, 'RK程序、标定、训练与验证源码；研究工具保留但不作为生产入口'
    if name == 'vendor/ORIGIN.md' or name == 'vendor/verilog-reusable-modules/THIRD_PARTY_NOTICES.md':
        return True, '依赖来源及第三方声明'
    if name.startswith('vendor/verilog-reusable-modules/modules/'):
        package = parts[3]
        if name == 'vendor/verilog-reusable-modules/modules/camera_fx2_direct/rtl/acg_async_fifo.v':
            return True, '生产及联仿共用的异步FIFO'
        if package not in packages:
            return False, '未接入生产构建的模块（历史相机传输或独立ToF）'
        if 'examples' in parts:
            return False, '独立开发板例程；应用使用rtl/ip及整车约束'
        return True, '生产依赖包、单元测试及其说明'
    if parts[0] == 'doc' and len(parts) == 2 and parts[1] in DOCS:
        return True, '接线、操作、标定及部署说明'
    return False, '历史过程文档、仓库维护说明或独立诊断入口'


README = '''# FALCONS 竞赛提交源码

本目录为从整车工作仓库整理出的独立源码副本，原目录结构及代码内容保持不变。
FPGA 为核心，基础感知、导航、显示及底盘闭环可独立运行；RK3588 提供四项 AI 拓展。

## 目录与入口

| 内容 | 位置 |
| --- | --- |
| 整车 FPGA 顶层、UI、字库及 Gowin IP | `rtl/`，顶层 `rtl/top/falcons_top.v` |
| 引脚及时间约束 | `constr/falcons.cst`、`constr/falcons.sdc` |
| 内嵌生产依赖 | `vendor/`；包清单 `deps/modules.list`，无需另行克隆模块库 |
| 综合与布局布线 | `build.tcl`、`scripts/`、`tools/build.ps1` |
| 应用及模块仿真 | `sim/`、各生产依赖包的 `sim/` |
| RK 四项 AI 功能、开机自启、录像与部署 | `host/ai_coordinator/` |
| 串口动作、音频、UVC、FX2 配套源码 | `host/semantic_nav/`、`voice_bridge/`、`usb_camera_v4l2/` 等 |
| Windows 相机标定及实测参数 | `host/usb_camera_win/`，参数位于 `calib_out/` |
| 项目相关模型训练、参考算法及验证代码 | `host/vision_training/`、`voice_task_demo/`、`inspection_reference/` |
| 接线、操作、巡检及模型说明 | `doc/`、`host/MODELS.md`及各模块 README |

## 构建与验证

将压缩包解压到全英文路径（例如 `D:/FALCONS_source`），避免 ModelSim 10.6d
无法处理中文路径。在本目录打开 PowerShell，安装匹配的 Gowin EDA（本项目使用 1.9.12.03）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -GowinSh '<Gowin安装目录>/IDE/bin/gw_sh.exe'
powershell -NoProfile -ExecutionPolicy Bypass -File sim/run_sim.ps1 -ModelSim '<ModelSim安装目录>/win64'
$env:PYTHONPATH = (Join-Path $PWD 'host')
python -m unittest discover -s host/ai_coordinator/tests -v
```

完整构建产生 `impl/pnr/falcons.fs`。`build_syn.tcl` 只综合，`build_pnr.tcl` 依赖已生成的综合网表。
外部工具默认路径集中于 `tools/lib_paths.ps1`，也可通过上列参数指定。
Gowin IP 及初始化字库已在源码中，EDA 软件和仿真器本身不随包分发。

RK 正式程序入口为 `host/ai_coordinator/main.py`，启动脚本为 `start_board.sh`，
部署及用户服务安装代码为 `deploy_inspection.py`、`install_autostart.py`。
运行需按现有板端环境配置 Python/原生推理库、模型路径及 USB 设备身份，
不能把 `config.example.json` 当作车辆正在使用的配置。

## 提交范围

保留全部应用层 RTL、生产依赖、测试、RK 源码、标定数据、项目训练和辅助开发代码。
研究与历史协议工具不是生产巡检入口，保留它们是为完整交付项目自研代码。
排除 Git 历史、Agent 指令、构建目录、软件安装包、日志、缓存、录像、原始采图、
历史联调过程证据、未接入整车的 ToF/旧相机传输模块及其独立开发板例程。
这些排除只发生在提交副本，原工作仓库和本地其他改动不删除。

本包是源码交付，不是 RK 系统镜像。模型权重、RK 当前 `config.json`、第三方运行库和
预合成语音缓存不在 Git 源码中，因此不包含在本包；资产说明见 `host/MODELS.md`。
所附 10-24 `MODEL_MANIFEST.json` 仅为历史资产清单，不能替代当前板端资产核对。
若比赛还要求推理权重或可直接迁移的运行镜像，需另从板端导出并核对。

## 来源与清单

`SOURCE_INFO.json` 记录源提交及工作区差异；`FILE_MANIFEST.json` 和 `SHA256SUMS`
记录交付文件；`EXCLUDED_TRACKED_FILES.csv` 逐项记录已跟踪但未纳入的文件及原因。
历史模块 README 中的独立 examples/ 和旧过程文档链接可能不在精简包内；
整车生产构建及交付测试以这里列出的入口为准。
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    out = args.out.resolve()
    archive = out.with_suffix('.zip')
    if out.is_relative_to(repo) or out.exists() or archive.exists():
        raise ValueError('Choose a new output directory outside the working repository')
    tracked = subprocess.check_output(['git', 'ls-files', '-z'], cwd=repo).decode('utf-8').split('\0')[:-1]
    tracked.extend(['tools/package_competition_source.py', 'doc/竞赛源码提交说明.md'])
    packages = set()
    for line in (repo/'deps/modules.list').read_text(encoding='utf-8-sig').splitlines():
        if line.strip() and not line.lstrip().startswith('#'):
            token = line.split()[0]
            if re.fullmatch(r'[A-Za-z0-9_]+', token):
                packages.add(token)
    out.mkdir(parents=True)
    excluded = []
    for name in sorted(set(tracked)):
        keep, reason = select(name, packages)
        if not keep:
            excluded.append((name, reason))
            continue
        source, target = repo/name, out/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        if sha(source) != sha(target):
            raise ValueError('Copy mismatch: '+name)
        if source.suffix == '.py':
            ast.parse(source.read_text(encoding='utf-8-sig'), filename=name)
    (out/'README.md').write_text(README, encoding='utf-8')
    info = {'source_commit': subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
            'source_branch':subprocess.check_output(['git','branch','--show-current'],cwd=repo,text=True).strip(),
            'source_status':subprocess.check_output(['git','status','--porcelain','-uall'],cwd=repo).decode('utf-8'),
            'scope':'source only; no model weights, live RK config, runtime SDK or hardware programming',
            'production_packages':sorted(packages)}
    (out/'SOURCE_INFO.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    stream=io.StringIO(newline=''); writer=csv.writer(stream)
    writer.writerow(['excluded_path','reason']);writer.writerows(excluded)
    (out/'EXCLUDED_TRACKED_FILES.csv').write_text(stream.getvalue(),encoding='utf-8-sig')
    manifest={p.relative_to(out).as_posix():{'bytes':p.stat().st_size,'sha256':sha(p)}
              for p in sorted(out.rglob('*')) if p.is_file()}
    (out/'FILE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    sums={**manifest,'FILE_MANIFEST.json':{'sha256':sha(out/'FILE_MANIFEST.json')}}
    (out/'SHA256SUMS').write_text(''.join(f'{v["sha256"]}  {k}\n' for k,v in sorted(sums.items())),encoding='utf-8')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(out.rglob('*')):
            if p.is_file(): z.write(p,out.name+'/'+p.relative_to(out).as_posix())
    print(json.dumps({'directory':str(out),'archive':str(archive),'files':len(manifest)+2,
                      'excluded_tracked':len(excluded),'zip_bytes':archive.stat().st_size,
                      'zip_sha256':sha(archive)},ensure_ascii=False))


if __name__ == '__main__':
    main()
