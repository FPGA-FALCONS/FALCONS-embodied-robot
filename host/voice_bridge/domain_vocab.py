"""User-authorized robot-command vocabulary policy; never rewrite ASR audit logs."""
import re

# Accepted ASR spellings only. Do not add 概念/单独奶/交代 here: those remain
# ambiguous under the user's policy and must not be guessed into AD calcium milk.
_ASR_ALIASES = [
    ('A概乃', 'AD钙奶'), ('a概乃', 'AD钙奶'), ('AD盖奶', 'AD钙奶'), ('概乃', 'AD钙奶'),
    ('盖奶', 'AD钙奶'),
    ('胶代', '胶带'), ('交带', '胶带'), ('胶袋', '胶带'),
    ('罗丝刀', '螺丝刀'), ('螺丝到', '螺丝刀'), ('螺丝道', '螺丝刀'),
    ('营养快綫', '营养快线'), ('营养快现', '营养快线'),
]


def _canonicalize_aliases(text):
    corrections = []
    out = text
    for source, target in _ASR_ALIASES:
        if source in out:
            count = out.count(source)
            out = out.replace(source, target)
            corrections.append({'from': source, 'to': target, 'count': count,
                                'reason': 'accepted_asr_alias'})
    return out, corrections


def command_text(raw_text):
    """Disallow 交代 in accepted task text, correcting only explicit search context.

    This is post-ASR normalization, not an acoustic decoder vocabulary constraint.
    Negation and surrounding instructions are preserved for the semantic parser.
    """
    corrections=[]
    pattern=r'(?P<verb>寻找|搜索|搜寻|定位到|定位|找到|找)(?P<gap>\s*(?:一下\s*)?)交代'
    def replace(match):
        corrections.append({'from':'交代','to':'胶带','reason':'explicit_search_context',
                            'span':[match.start(),match.end()]})
        return match.group('verb')+match.group('gap')+'胶带'
    normalized=re.sub(pattern,replace,raw_text)
    blocked='交代' in normalized
    if not blocked:
        normalized, alias_corrections = _canonicalize_aliases(normalized)
        corrections.extend(alias_corrections)
    return {'raw_text':raw_text,'task_text':None if blocked else normalized,
            'corrections':corrections,'blocked':blocked,
            'reason':'banned_ambiguous_word' if blocked else 'domain_normalized' if corrections else 'unchanged',
            'acoustic_model_modified':False}
