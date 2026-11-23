#!/usr/bin/env python3
"""Render the README hero as dark and light SVGs (no screenshots exist: the robot is hardware).

    python docs/brand/build_hero.py

Writes docs/screenshots/hero-dark.svg and hero-light.svg. Pure Python, no dependencies, no
fonts to load: SVGs shown through <img> on GitHub cannot fetch web fonts, so text uses the
viewer's monospace font. All geometry is fixed so the text fits with any common monospace.
"""
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / "screenshots"
W, H, PX = 1600, 600, 17
FONT = "'JetBrains Mono','IBM Plex Mono',ui-monospace,Menlo,Consolas,'DejaVu Sans Mono',monospace"
LIME, YELLOW, CORAL, INK, WHITE = "#B6F500", "#FFD60A", "#FF5A4F", "#0B0F14", "#FFFFFF"

# Hand-drawn robot, one character per pixel. K outline, L body, Y eyes, C status lights.
MASCOT = [
    ".......CC.......",
    ".......KK.......",
    "..KKKKKKKKKKKK..",
    ".KLLLLLLLLLLLLK.",
    ".KLKKKKLLKKKKLK.",
    ".KLKYYKLLKYYKLK.",
    ".KLKKKKLLKKKKLK.",
    ".KLLLLLLLLLLLLK.",
    "..KKKKKKKKKKKK..",
    ".....KKKKKK.....",
    "...KKLLLLLLKK...",
    "..KLLLLLLLLLLK..",
    "..KLLCCLLCCLLK..",
    "..KLLLLLLLLLLK..",
    "...KKKKKKKKKK...",
    "....KK....KK....",
    "...KLLK..KLLK...",
    "...KLLK..KLLK...",
    "..KKLLK..KLLKK..",
    "..KKKKK..KKKKK..",
]
COLORS = {"K": INK, "L": WHITE, "Y": LIME, "C": CORAL}

TILES = [("PI · 50 Hz", "policy loop"), ("USB · CRC16", "binary frames"), ("P4 · 200 Hz", "reflexes + safety")]


def build(theme: str) -> str:
    dark = theme == "dark"
    bg, fg = (INK, WHITE) if dark else (WHITE, INK)
    tile_fill = "#151B23" if dark else WHITE
    shadow = LIME if dark else INK
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" '
         f'aria-label="Dual-Brain Biped: reflexes on the chip, decisions on the Pi">']
    o.append(f'<rect x="1.5" y="1.5" width="{W-3}" height="{H-3}" fill="{bg}" stroke="{fg}" stroke-width="3"/>')
    # top bar
    o.append(f'<text x="60" y="76" font-family="{FONT}" font-size="32" font-weight="700" fill="{fg}">DUAL-BRAIN BIPED</text>')
    o.append(f'<text x="1540" y="76" text-anchor="end" font-family="{FONT}" font-size="24" fill="{fg}">ESP32-P4 + RASPBERRY PI</text>')
    o.append(f'<rect x="60" y="100" width="1480" height="3" fill="{fg}"/>')
    # headline: one highlighted word
    o.append(f'<rect x="52" y="148" width="412" height="96" fill="{LIME}"/>')
    o.append(f'<text x="60" y="226" font-family="{FONT}" font-size="80" font-weight="800" fill="{INK}">Reflexes</text>')
    o.append(f'<text x="470" y="226" font-family="{FONT}" font-size="80" font-weight="800" fill="{fg}"> on the chip.</text>')
    o.append(f'<text x="60" y="326" font-family="{FONT}" font-size="80" font-weight="800" fill="{fg}">Decisions on the Pi.</text>')
    # three-tile flow
    for i, (title, sub) in enumerate(TILES):
        x, y, w, h = 60 + i * 360, 390, 320, 150
        o.append(f'<rect x="{x+8}" y="{y+8}" width="{w}" height="{h}" fill="{shadow}"/>')
        o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{tile_fill}" stroke="{fg}" stroke-width="3"/>')
        o.append(f'<text x="{x+20}" y="{y+68}" font-family="{FONT}" font-size="30" font-weight="700" fill="{fg}">{title}</text>')
        o.append(f'<text x="{x+20}" y="{y+112}" font-family="{FONT}" font-size="26" fill="{fg}">{sub}</text>')
        if i < 2:  # arrow between tiles
            ax, ay = x + w + 6, y + h // 2
            o.append(f'<polygon points="{ax},{ay-14} {ax+28},{ay} {ax},{ay+14}" fill="{CORAL}"/>')
    # mascot on a yellow panel so the ink outline reads on both themes
    px, py, pw, ph = 1180, 120, 360, 420
    o.append(f'<rect x="{px+10}" y="{py+10}" width="{pw}" height="{ph}" fill="{shadow}"/>')
    o.append(f'<rect x="{px}" y="{py}" width="{pw}" height="{ph}" fill="{YELLOW}" stroke="{fg}" stroke-width="3"/>')
    mx, my = px + (pw - 16 * PX) // 2, py + (ph - len(MASCOT) * PX) // 2
    o.append('<g shape-rendering="crispEdges">')
    for r, row in enumerate(MASCOT):
        for c, ch in enumerate(row):
            if ch in COLORS:
                o.append(f'<rect x="{mx + c*PX}" y="{my + r*PX}" width="{PX}" height="{PX}" fill="{COLORS[ch]}"/>')
    o.append("</g></svg>")
    return "\n".join(o) + "\n"


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for t in ("dark", "light"):
        p = OUT / f"hero-{t}.svg"
        p.write_text(build(t), encoding="utf-8", newline="\n")
        print(f"wrote {p} ({p.stat().st_size // 1024} KB)")
