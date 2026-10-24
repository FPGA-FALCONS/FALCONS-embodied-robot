"""Conservative task previews, optionally using a separately served language model."""
import json
import re
import urllib.request

ALIASES = {
    'tape': ('胶带', '胶布', '交代', '交带', '胶袋', '胶代', 'tape'),
    'screwdriver': ('螺丝刀', '改锥', '起子', '罗丝刀', '螺丝到', '螺丝道', 'screwdriver'),
    'ad_calcium_milk': ('ad钙奶', '钙奶', '盖奶', 'a钙奶', 'a概乃', '概乃', 'ad_calcium_milk',
                         'ad calcium milk', 'adcalciummilk'),
    'nutri_express': ('营养快线', '营养块线', '营养快现', 'nutri_express',
                      'nutri express', 'nutriexpress'),
}
SYSTEM = '''把中文机器人指令转换为一行JSON，不要解释，不输出思考。
仅支持寻找胶带tape、螺丝刀screwdriver、AD钙奶ad_calcium_milk、营养快线nutri_express。
寻找格式{"i":"search","t":"tape"}。明确找不到首选才找备选，加"f":"screwdriver"。
明确找到后返回起点，加"r":true。未提及的f/r不得输出。
立即停止{"i":"stop"}；确认{"i":"confirm"}；缺少目标、不支持、仅否定或无法确定，输出{"i":"unknown"}。
纠正目标仅保留最后明确要求寻找的目标。抓取、搬运、任意坐标移动不支持。只允许i/t/f/r四个键。'''


def normalize(text):
    return re.sub(r'[\s，。！？、,.!?;；:：]', '', text).lower()


def mentioned(text):
    value = normalize(text)
    return {key for key, aliases in ALIASES.items() if any(alias in value for alias in aliases)}


def rule_preview(text):
    value = normalize(text)
    task = {'i':'unknown'}; reason = 'needs_language_model_or_clarification'
    if value in ('停', '停止', '停下', '别动', '不要动', '立即停止', '马上停止'):
        task = {'i':'stop'}; reason = 'explicit_stop'
    elif value in ('确认', '确定', '对的', '是的', '确认当前任务'):
        task = {'i':'confirm'}; reason = 'confirmation_intent_only'
    elif any(word in value for word in ('捡', '抓', '递', '搬', '送', '拿')):
        reason = 'unsupported_manipulation'
    else:
        # Match the complete utterance so an extra action or question is not lost.
        for target, aliases in ALIASES.items():
            names = '|'.join(re.escape(alias) for alias in aliases)
            pattern = r'(?:请|麻烦)?(?:帮我|给我)?(?:寻找|搜索|定位|找)(?:一下)?(?:' + names + r')(?:吧|谢谢)?'
            if re.fullmatch(pattern, value):
                task = {'i':'search', 't':target}; reason = 'simple_search_rule'
                break
    return {'task':task, 'source':'rules', 'reason':reason}


def parse_task(content, text):
    def unique(pairs):
        obj = {}
        for key, value in pairs:
            if key in obj:
                raise ValueError('duplicate JSON key')
            obj[key] = value
        return obj
    obj = json.loads(content, object_pairs_hook=unique)
    if not isinstance(obj, dict) or set(obj)-{'i','t','f','r'}:
        raise ValueError('invalid task fields')
    if obj.get('i') not in ('search','stop','confirm','unknown'):
        raise ValueError('unknown task intent')
    if obj['i'] != 'search':
        if set(obj) != {'i'}:
            raise ValueError('arguments on non-search intent')
    else:
        if obj.get('t') not in ALIASES or ('f' in obj and (obj['f'] not in ALIASES or obj['f']==obj['t'])):
            raise ValueError('invalid target/fallback')
        if 'r' in obj and obj['r'] is not True:
            raise ValueError('return flag must be true or absent')
        targets = mentioned(text)
        if obj['t'] not in targets or ('f' in obj and obj['f'] not in targets):
            raise ValueError('target not grounded in transcript')
    return obj


def task_preview(text, llm_url=None, timeout=5):
    initial = rule_preview(text)
    if not llm_url or initial['task']['i'] != 'unknown' or initial['reason']=='unsupported_manipulation':
        return initial
    if not llm_url.startswith(('http://','https://')):
        raise ValueError('LLM URL must use HTTP(S)')
    payload = {'messages':[{'role':'system','content':SYSTEM},{'role':'user','content':text}],
               'temperature':0, 'max_tokens':80, 'chat_template_kwargs':{'enable_thinking':False}}
    request = urllib.request.Request(llm_url.rstrip('/')+'/v1/chat/completions',
                data=json.dumps(payload,ensure_ascii=False).encode('utf-8'),
                headers={'Content-Type':'application/json'},method='POST')
    # Explicit endpoint only; do not leak local voice text to a configured HTTP proxy.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request,timeout=timeout) as response:
            data = response.read(65537)
        if len(data)>65536:
            raise ValueError('language model response exceeds 64 KiB')
        choice = json.loads(data)['choices'][0]
        if choice.get('finish_reason') != 'stop':
            raise ValueError('incomplete language model result')
        task = parse_task(choice['message']['content'], text)
        return {'task':task, 'source':'language_model', 'reason':'validated_preview_only'}
    except Exception as exc:
        return {'task':{'i':'unknown'}, 'source':'language_model', 'reason':'llm_error',
                'error':type(exc).__name__+': '+str(exc)[:300]}
