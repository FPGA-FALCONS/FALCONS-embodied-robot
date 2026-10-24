"""Shared Chinese coordinator admission policy; preserve raw ASR text for audit.

The existing voice guard is reused for unsupported actions and injection. Its
substring stop rule is deliberately not used for deferred/quoted stop clauses.
"""
import re
from voice_bridge.domain_vocab import command_text
from voice_bridge.preview import normalize,ALIASES
from voice_bridge.safety_rules import INJECTION, POLICY_LIMITED, UNSUPPORTED

VERSION='zh-c1b-coordinator-1'
STOP_WORDS=r'(?:停止(?:搜索|寻找|任务)?|停下|停一下|别动|不要动|不用找了|不用找|停)'
FUTURE=re.compile(r'(?:找到|找不到|到达|完成|返回|看到|如果|一旦|当|等)[^，,。；;！!？?]*?(?:后|时|再|就|才)[^，,。；;！!？?]*?'+STOP_WORDS)


def immediate_stop(text):
    value=normalize(text)
    # Never reinterpret a quoted/explanatory occurrence as an operator command.
    if any(word in value for word in ('什么意思','如何理解','解释','这个词','怎么说','不要执行')):return False
    for clause in re.split(r'[，,。；;！!？?\n]|但是|不过',text):
        remaining=FUTURE.sub('',clause)
        if re.search(r'别停|不要停|不用停',remaining):continue
        compact=normalize(remaining)
        if re.fullmatch(r'(?:请|麻烦)?(?:机器人)?(?:先|现在|马上|立即|立刻)?'+STOP_WORDS+r'(?:吧|啊|呀|了|谢谢)?',compact):return True
        if re.search(r'(?:现在|马上|立即|立刻)'+STOP_WORDS,compact):return True
    return False


def inspect(text):
    if not isinstance(text,str) or not 1<=len(text)<=1000:raise ValueError('Text must be 1..1000 characters')
    policy=command_text(text)
    hard=None;source=None
    if immediate_stop(text):hard={'i':'stop'};source='immediate_stop'
    elif any(word in text for word in ('什么意思','解释','这个词')) and re.search(STOP_WORDS,text):
        hard={'i':'unknown'};source='quoted_stop_guard'
    elif INJECTION.search(text) or POLICY_LIMITED.search(text) or UNSUPPORTED.search(text):
        hard={'i':'unknown'};source='safety_guard'
    elif policy['blocked']:hard={'i':'unknown'};source='domain_policy'
    elif policy['task_text']:
        # A narrow, fully matched idiom: finding the target already ends search.
        # No substring extraction that could drop a fallback or another action.
        compact=normalize(policy['task_text'])
        for target,names in ALIASES.items():
            aliases='|'.join(re.escape(n) for n in names)
            if re.fullmatch(r'(?:请)?找到(?:'+aliases+r')(?:后|以后)(?:立即|马上|立刻)?停止(?:搜索|寻找)?',compact):
                hard={'i':'search','t':target};source='conditional_search_rule';break
    return {'version':VERSION,'domain':policy,'hard_task':hard,'source':source}
