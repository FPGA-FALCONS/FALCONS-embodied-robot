"""Deterministic safety guard for accepted task text.

Enforces two hard policies from the task schema:
- explicit immediate stop wins over everything;
- unsupported physical actions (fetch/deliver/grasp/coordinate motion) -> unknown.

It does not parse fallback/return; those remain the language model's job.
This is post-ASR policy, not an acoustic decoder change.
"""
import re

STOP = re.compile(
    r"\bstop\b|\bhalt\b|\bfreeze\b|don'?t move|never mind|"
    r"停下|停止|停一下|立刻停|马上停|别动|不用找",
    re.IGNORECASE,
)
STOP_NEGATED = re.compile(
    r"\b(don'?t|do not)\s+stop\b|别停|不要停|不用停",
    re.IGNORECASE,
)
UNSUPPORTED = re.compile(
    r"\bfetch\b|\bretrieve\b|\bbring\b|\bdeliver\b|\bpick\s+up\b|"
    r"\bhand\s+me\b|\bgrab\b|\bput\b|\bplace\b|\bcoordinates?\b|"
    r"\bmove\s+to\b|\bdrive\b|抓|拿过来|递给我|递给|放到|坐标|搬",
    re.IGNORECASE,
)
INJECTION = re.compile(
    r"忽略.*(?:规则|指令|提示|系统)|无视.*(?:规则|指令)|"
    r"直接输出|不要输出.*json|输出\s*ok|"
    r"ignore\s+(?:all\s+)?(?:previous\s+)?instructions",
    re.IGNORECASE,
)
# Known ASR near-miss that the user's policy explicitly refuses to guess.
POLICY_LIMITED = re.compile(r"沼罗斯刀")


def guard(text):
    """Return a hard label, or None to defer to the model."""
    if STOP_NEGATED.search(text):
        return None
    if STOP.search(text):
        return {"i": "stop"}
    if INJECTION.search(text):
        return {"i": "unknown"}
    if POLICY_LIMITED.search(text):
        return {"i": "unknown"}
    if UNSUPPORTED.search(text):
        return {"i": "unknown"}
    return None
