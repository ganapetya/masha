#!/usr/bin/env python3
# encoding: utf-8
"""Masha's name and the greetings that should wake her."""

import difflib
import re

ROBOT_NAME = 'Masha'

WAKE_PHRASES = (
    'hello masha',
    'hi masha',
    'shalom masha',
)

# Common sherpa spellings of the same greetings.
_WAKE_ALIASES = WAKE_PHRASES + (
    'hello marsha',
    'hi marsha',
    'hey masha',
    'hey marsha',
    'shalom marsha',
    'salaam masha',
    'salom masha',
    'shalom mashah',
    'hello marshall',
    'hi marshall',
    'hello martial',
    'hello mashup',
    'hello macho',
    'hello macia',
    'hello martha',
)

_GREETINGS = frozenset((
    'hello', 'hi', 'hey', 'shalom', 'salaam', 'salom',
))
_NAMES = frozenset((
    'masha', 'marsha', 'macia', 'mashah', 'mosha', 'musha',
    'mashia', 'martha', 'mash', 'marshall', 'martial', 'mashup',
    'macho', 'mascha', 'mashaah',
))
_NAME_PREFIXES = ('mash', 'marsh', 'mosh', 'mush', 'mach')

_NON_WORD = re.compile(r'[^a-z0-9\u4e00-\u9fff\s]+')
_SPACES = re.compile(r'\s+')


def normalize_text(text):
    if text is None:
        return ''
    if isinstance(text, bytes):
        text = text.decode('utf-8', errors='ignore')
    text = str(text).lower().replace('-', ' ')
    text = _NON_WORD.sub(' ', text)
    return _SPACES.sub(' ', text).strip()


def _edit_distance_ok(token, target='masha', limit=1):
    if abs(len(token) - len(target)) > limit:
        return False
    return difflib.SequenceMatcher(None, token, target).ratio() >= 0.8


def token_is_name(token):
    if not token:
        return False
    if token in _NAMES:
        return True
    if token.startswith(_NAME_PREFIXES):
        return True
    return _edit_distance_ok(token)


def tokens_of(text):
    return normalize_text(text).split()


def has_greeting(text):
    return bool(set(tokens_of(text)) & _GREETINGS)


def has_name(text):
    normalized = normalize_text(text)
    if not normalized:
        return False
    if 'masha' in normalized.replace(' ', ''):
        return True
    return any(token_is_name(tok) for tok in normalized.split())


def is_wake_phrase(text):
    normalized = normalize_text(text)
    if not normalized:
        return False
    if normalized in _WAKE_ALIASES:
        return True
    if any(alias in normalized for alias in _WAKE_ALIASES):
        return True
    tokens = set(normalized.split())
    if tokens & _GREETINGS and any(token_is_name(tok) for tok in tokens):
        return True
    # "mash a" / "ma sha" often come back as two tokens.
    collapsed = normalized.replace(' ', '')
    if any(g in tokens for g in _GREETINGS) and 'masha' in collapsed:
        return True
    return False


def wake_phrase_log():
    return ' / '.join(phrase.title() for phrase in WAKE_PHRASES)


if __name__ == '__main__':
    assert is_wake_phrase('Hello Masha!')
    assert is_wake_phrase('hi  masha')
    assert is_wake_phrase('Shalom, Masha')
    assert is_wake_phrase('Please listen Hello Masha now')
    assert is_wake_phrase('hey marsha')
    assert is_wake_phrase('hi mash a')
    assert is_wake_phrase('hello marshall')
    assert is_wake_phrase('hi mashup')
    assert has_greeting('hello')
    assert has_name('marsha')
    assert not is_wake_phrase('go forward')
    assert not is_wake_phrase('hello')
    assert not is_wake_phrase('masha')
    assert not is_wake_phrase('')
    print('identity ok:', ROBOT_NAME, wake_phrase_log())
