#!/usr/bin/env python3
"""Claude Code sohbet kaydini (JSONL) okunabilir Markdown'a cevirir.

Sadece kullanici mesajlari ve asistanin metin cevaplari alinir. Arac cagrilari, arac ciktilari,
dusunme bloklari, sistem hatirlatmalari ve gorseller ATILIR. E-posta adresleri maskelenir.

Kullanim:
  python3 tools/sohbet_disa_aktar.py ~/.claude/projects/-home-lapis-ros2-ws/<oturum>.jsonl docs/sohbet/<ad>.md
"""
import json, re, sys
from datetime import datetime

EPOSTA = re.compile(r'[A-Za-z0-9._%+-]+@(?!github\.com)[A-Za-z0-9.-]+\.[A-Za-z]{2,}')
GIZLE = re.compile(r'<(system-reminder|local-command-caveat|local-command-stdout|command-name|command-message|'
                   r'command-args|ide_selection|ide_opened_file)>.*?</\1>', re.S)

def temizle(t: str) -> str:
    t = GIZLE.sub('', t)
    t = EPOSTA.sub('<e-posta>', t)
    return t.strip()

def metinler(cont):
    if isinstance(cont, str):
        return [cont]
    out = []
    for b in cont or []:
        if isinstance(b, dict) and b.get('type') == 'text':
            out.append(b.get('text', ''))
    return out

def main(src, dst):
    satirlar = []
    son_rol = None
    for l in open(src, encoding='utf-8', errors='ignore'):
        try:
            d = json.loads(l)
        except Exception:
            continue
        rol = d.get('type')
        if rol not in ('user', 'assistant') or d.get('isMeta') or d.get('isCompactSummary'):
            continue
        m = d.get('message') or {}
        parca = [temizle(x) for x in metinler(m.get('content'))]
        parca = [p for p in parca if p and not p.startswith('[Request interrupted')]
        if not parca:
            continue
        ts = d.get('timestamp', '')
        try:
            ts = datetime.fromisoformat(ts.replace('Z', '+00:00')).astimezone().strftime('%Y-%m-%d %H:%M')
        except Exception:
            ts = ''
        baslik = '## Kullanici' if rol == 'user' else '### Claude'
        if rol != son_rol or rol == 'user':
            satirlar.append(f'\n{baslik} ({ts})\n')
        satirlar.append('\n\n'.join(parca) + '\n')
        son_rol = rol
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(f'# Sohbet kaydi\n\nKaynak: `{src.split("/")[-1]}`. Sadece mesajlar ve cevaplar; arac ciktilari, '
                f'gorseller ve sistem notlari cikarildi. E-postalar maskelendi.\n')
        f.write(''.join(satirlar))
    print(dst, sum(1 for s in satirlar if s.startswith('\n## Kullanici')), 'kullanici mesaji')

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
