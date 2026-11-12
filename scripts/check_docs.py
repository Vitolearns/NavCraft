#!/usr/bin/env python3
"""Check local documentation file links, deterministic SVGs and scene media.

Standard library only. Checks file targets, not Markdown heading anchors or web
URLs. Scene URIs use the project's explicit simulation_media_path substitution;
unknown URI forms fail for review. OBJ materials and MTL textures are followed.
This structural check does not prove diagram semantics or navigation behavior.
"""
from pathlib import Path
import re
import shlex
import sys
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
from render_architecture import outputs

ROOT = Path(__file__).resolve().parents[1]


def check_links(errors):
    pages = sorted(ROOT.glob('README*.md'))
    pages += sorted((ROOT / 'docs').rglob('*.md'))
    count = 0
    for page in pages:
        text = re.sub(r'^```.*?^```[^\n]*', '', page.read_text(encoding='utf-8'), flags=re.M | re.S)
        # Inline links/images and explicit reference definitions used by this repo.
        targets = re.findall(r'!?\[[^\]\n]*\]\(\s*(<[^>]+>|[^\s)]+)', text)
        targets += re.findall(r'^\s*\[[^\]]+\]:\s*(<[^>]+>|\S+)', text, flags=re.M)
        # The centered README header uses HTML links and repository-hosted badges.
        targets += re.findall(r'<(?:a|img)\b[^>]*?\b(?:href|src)="([^"]+)"', text, flags=re.I)
        for target in targets:
            url = urlsplit(target.strip('<>'))
            if url.scheme or url.netloc or not url.path:
                continue
            path = (page.parent / unquote(url.path)).resolve()
            count += 1
            if not path.is_relative_to(ROOT) or not path.exists():
                errors.append(f'{page.relative_to(ROOT)}: missing/outside file target {target}')
    return len(pages), count


def check_diagrams(errors):
    diagrams = outputs()
    for name, expected in diagrams.items():
        path = ROOT / 'docs/images' / name
        if not path.is_file() or path.read_text(encoding='utf-8') != expected:
            errors.append(f'{path.relative_to(ROOT)}: missing/stale; run scripts/render_architecture.py')
        ET.fromstring(expected)  # Reject malformed output even on first generation.
    return len(diagrams)


def check_media(errors):
    sim = ROOT / 'src/navcraft_simulation'
    media = (sim / 'media').resolve()
    world = sim / 'worlds/indoor_room.sdf.xacro'
    refs = []
    for element in ET.parse(world).iter('uri'):
        uri = (element.text or '').strip()
        prefix = 'file://${simulation_media_path}/'
        if not uri.startswith(prefix):
            errors.append(f'{world.relative_to(ROOT)}: unhandled media URI {uri}')
            continue
        refs.append((media / uri[len(prefix):]).resolve())
    if not refs:
        errors.append('No warehouse media references found; review the scene checker.')
    seen = set()

    def visit(path):
        path = path.resolve()
        if path in seen:
            return
        seen.add(path)
        if not path.is_relative_to(media) or not path.is_file():
            errors.append(f'Missing/outside scene media: {path}')
            return
        if path.suffix.lower() not in ('.obj', '.mtl'):
            return
        lines = [shlex.split(line, comments=True) for line in path.read_text(encoding='utf-8').splitlines()]
        materials, used = set(), set()
        for tokens in lines:
            if not tokens:
                continue
            if tokens[0] == 'mtllib':
                for name in tokens[1:]:
                    library = (path.parent / name).resolve()
                    visit(library)
                    if library.is_relative_to(media) and library.is_file():
                        materials.update(re.findall(r'^newmtl\s+(.+?)\s*$', library.read_text(encoding='utf-8'), re.M))
            elif tokens[0] == 'usemtl':
                used.add(' '.join(tokens[1:]))
            elif tokens[0].lower().startswith('map_') or tokens[0].lower() in ('bump', 'disp', 'decal', 'refl'):
                # Current materials have no texture options. Fail rather than guess.
                if len(tokens) != 2:
                    errors.append(f'{path.name}: review texture arguments: {tokens}')
                else:
                    visit(path.parent / tokens[1])
        if path.suffix.lower() == '.obj' and used - materials:
            errors.append(f'{path.name}: undefined materials {sorted(used - materials)}')

    for path in refs:
        visit(path)
    return len(refs), len(seen)


def main():
    errors = []
    pages, links = check_links(errors)
    diagrams = check_diagrams(errors)
    refs, assets = check_media(errors)
    for error in errors:
        print('ERROR:', error, file=sys.stderr)
    if errors:
        return 1
    print(f'PASS: {pages} Markdown files, {links} local links, {diagrams} reproducible SVGs, '
          f'{refs} scene references / {assets} media dependencies.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
