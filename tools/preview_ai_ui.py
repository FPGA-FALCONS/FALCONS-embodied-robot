"""Render the actual AI RTL with labelled simulated task data; no board access."""
import argparse
from pathlib import Path
import subprocess
import sys
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from ai_coordinator.ai_link import display_frames,font_rows


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--modelsim',type=Path,default=Path('D:/Apps/modeltech64_10.6d/win64'))
    ap.add_argument('--out',type=Path,default=ROOT/'logs/ai_ui_preview')
    ap.add_argument('--inspection',action='store_true')
    args=ap.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    names=('语义导航','目标识别','聊天机器人');counts=[];atlas=font_rows()
    if args.inspection:names=('摄像头','雷达地图','文字清单')
    for index,name in enumerate(names):
        lines=['仿真预览 AI模块 总开关:开启','CH8选择 SWA确认/返回 SWB录音/结束',
               '功能: '+name,'状态: 待命','当前指令:',
               ('前进2秒','识别胶带','你好，请介绍自己')[index],'',
               '识别目标:', '胶带 92%（示例）' if index==1 else '', '',
               '聊天回复:', '你好，我是你的机器人助手。' if index==2 else '',
               '这是页面布局示例，不是实车验收结果。','','','','','']
        frames,_=display_frames(lines,index+1,atlas)
        raw=b''.join(frames);counts.append(len(raw));assert len(raw)<120000
        (out/f'packets_{index}.hex').write_text('\n'.join('%02x'%b for b in raw)+'\n')
    (out/'packet_counts.hex').write_text('\n'.join('%08x'%n for n in counts+[0])+'\n')
    library=out/'preview_work'
    def run(argv,cwd=ROOT):
        p=subprocess.run([str(a).replace('\\','/') for a in argv],cwd=cwd,capture_output=True,text=True,errors='replace')
        with (out/'render.log').open('a',encoding='utf-8') as log:log.write(p.stdout+p.stderr)
        if p.returncode or '** Error' in p.stdout or 'FAIL:' in p.stdout:raise RuntimeError((p.stdout+p.stderr)[-3000:])
        return p.stdout
    (out/'render.log').write_text('')
    if not library.exists():run([args.modelsim/'vlib.exe',library])
    files=['rtl/common/font_rom.v','rtl/common/font_rom_lg.v','rtl/common/font_num.v',
           'rtl/common/ui_text_rom.v','rtl/common/ai_menu_font.v','rtl/ui/ui_render.v',
           'rtl/ai/ai_display_link.v','sim/ai_ui_preview_tb.v']
    run([args.modelsim/'vlog.exe','-work',library,'+incdir+'+str(ROOT/'rtl/ui'),*[ROOT/p for p in files]])
    run([args.modelsim/'vmap.exe','-c'],out)
    run([args.modelsim/'vmap.exe','-modelsimini','modelsim.ini','preview_work','preview_work'],out)
    # Old ModelSim's Windows argument parser mishandles an absolute ini path
    # containing spaces; a relative ini/fixture path preserves native argv.
    text=run([args.modelsim/'vsim.exe','-c','-modelsimini','modelsim.ini','preview_work.ai_ui_preview_tb',
              '+OUT=.', '+TASK='+str(int(args.inspection)),'-do','run -all; quit -f'],out)
    if 'PASS: ai_ui_preview_tb 4 frames' not in text:raise RuntimeError('Preview did not complete')
    for index in range(4):
        im=Image.open(out/f'ai_mode_{index}.ppm').convert('RGB')
        frozen=index<3
        selected_ink=(138,122,46) if frozen else (255,224,102)
        menu_ink=(90,97,112) if frozen else (229,231,235)
        menu_bg=(11,16,25) if frozen else (16,24,40)
        yellow=sum(pixel==selected_ink for pixel in im.crop((0,480,208,504)).getdata())
        if yellow<50:raise RuntimeError('Seventh AI menu item missing or clipped')
        for menu in range(7):
            colors={menu_ink,selected_ink}
            count=sum(pixel in colors for pixel in im.crop((0,48+72*menu,208,72+72*menu)).getdata())
            if count<50:raise RuntimeError('Menu item missing: '+str(menu))
        if im.getpixel((8,120))!=menu_bg:
            raise RuntimeError('AI menu must freeze while active and recover when closed')
        if im.getpixel((8,544))!=(32,32,32):
            raise RuntimeError('AI freeze must not change the cursor status bar')
        gap=im.crop((0,516,208,540))
        if any(pixel==(29,78,216) for pixel in gap.getdata()):raise RuntimeError('AI highlight overlaps status gap')
        for tab in range(3):
            box=im.crop((232+tab*240,64,448+tab*240,108))
            chosen=tab==index%3
            ink=(255,224,102) if chosen else (232,240,248)
            points=[(x,y) for y in range(44) for x in range(216) if box.getpixel((x,y))==ink]
            if len(points)<250:raise RuntimeError('AI large label missing')
            xs,ys=zip(*points)
            if max(ys)-min(ys)<18 or abs((min(xs)+max(xs))/2-107.5)>3 or abs((min(ys)+max(ys))/2-21.5)>3:
                raise RuntimeError('AI large label is not centered')
            if chosen and (sum(p==(29,78,216) for p in box.getdata())<7000 or (232,240,248) in box.getdata()):
                raise RuntimeError('AI selection must use blue/yellow, without a white outline')
        im.save(out/f'ai_mode_{index}.png')
    print(out)


if __name__=='__main__':main()
