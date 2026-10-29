from pathlib import Path
import hashlib,html,json,re
out=Path(__file__).resolve().parent
repo=out.parents[1]
pnr=repo/'impl/pnr'
report=(pnr/'falcons_tr_content.html').read_text(encoding='utf-8')
def rows(anchor):
    table=report.split(f'name="{anchor}"',1)[1].split('</table>',1)[0]
    return [[html.unescape(re.sub('<[^>]*>','',c)).strip() for c in re.findall(r'<t[dh][^>]*>(.*?)</t[dh]>',row,re.S)] for row in re.findall(r'<tr\b[^>]*>(.*?)</tr>',table,re.S)]
manifest=json.loads((out/'fpga_inputs.json').read_text(encoding='utf-8'))
for name,expected in manifest.items():
    assert hashlib.sha256((repo/name).read_bytes()).hexdigest()==expected,name
freq={r[1]:{'required_mhz':float(r[2].split('(')[0]),'actual_mhz':float(r[3].split('(')[0])} for r in rows('Max_Frequency_Report')[1:]}
tns=rows('Total_Negative_Slack_Report')[1:]
rsc=(pnr/'falcons.rpt.txt').read_text(encoding='utf-8')
def violated(kind):
    tail=report.split(f'Numbers of {kind} Violated Endpoints</td>',1)[1]
    return int(re.search(r'<td>(\d+)',tail).group(1))
v={'fs_sha256':hashlib.sha256((pnr/'falcons.fs').read_bytes()).hexdigest(),
   'inputs_verified':len(manifest),'frequency':freq,'same_clock_tns_rows':tns,
   'global_setup_violated_endpoints':violated('Setup'),'global_hold_violated_endpoints':violated('Hold'),
   'logic':int(re.search(r'\n\s*Logic\s*\|\s*(\d+)',rsc).group(1)),
   'registers':int(re.search(r'\n\s*Register\s*\|\s*(\d+)',rsc).group(1))}
v['same_clock_timing_pass']=all(float(r[2])==0 and int(r[3])==0 for r in tns) and all(a['actual_mhz']>=a['required_mhz'] for a in freq.values())
(out/'fpga_validation.json').write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8')
print(json.dumps(v,indent=2))
assert v['same_clock_timing_pass'],'Same-clock timing requirements not met'
