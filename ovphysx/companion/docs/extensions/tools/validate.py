# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Validate extension design traceability; this does not execute UX scenarios."""
import argparse
import json
from pathlib import Path
import re
from urllib.parse import unquote


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--public', action='store_true', help='Validate exported docs, including checks for internal URLs')
    args = parser.parse_args()
    base = Path(__file__).resolve().parents[1]
    errors = []

    def check(ok, message):
        if not ok:
            errors.append(str(message))

    def metadata(path):
        lines = path.read_text().splitlines()
        bounds = [i for i, line in enumerate(lines) if line == '---']
        check(len(bounds) >= 2, f'{path}: missing metadata')
        if len(bounds) < 2:
            return {}
        return dict(line.split(': ', 1) for line in lines[bounds[0] + 1:bounds[1]] if ': ' in line)

    reqs = {}
    tests = {}
    coverage = {}
    for group in ('runtime', 'ux'):
        for p in (base / group / 'requirements').rglob('REQ-*.md'):
            t = p.read_text()
            m = metadata(p)
            check(m.get('id') == p.stem, f'{p}: ID mismatch')
            check(m.get('status') == 'proposed', f'{p}: unexpected REQ status')
            ac = set(map(int, re.findall(r'^- AC-(\d+):', t, re.M)))
            check(ac and ac == set(range(1, len(ac) + 1)), f'{p}: invalid AC sequence')
            check(p.stem not in reqs, f'{p}: duplicate ID')
            reqs[p.stem] = (p, ac)
        for p in (base / group / 'adr').glob('ADR-*.md'):
            m = metadata(p)
            expected = 'proposed' if group == 'ux' else ('rejected' if p.name.startswith('ADR-0002-') else 'accepted')
            check(m.get('status') == expected, f'{p}: changed decision status')
            for target in re.findall(r'\]\(([^)]+)\)', p.read_text()):
                check(Path(target.split('#')[0]).name.startswith(('ADR-', 'REQ-')), f'{p}: nonpermanent ADR reference {target}')
    for group in ('runtime', 'ux'):
        for p in (base / group / 'tests').rglob('TEST-*.md'):
            t = p.read_text()
            m = metadata(p)
            check(m.get('id') == p.stem, f'{p}: ID mismatch')
            check(m.get('maps_to') in reqs, f'{p}: unknown mapped REQ')
            check('specification only' in t.lower(), f'{p}: missing specification status')
            for heading in ('Scenario', 'Given', 'When', 'Then'):
                check('## ' + heading in t, f'{p}: missing {heading}')
            if m.get('maps_to') in reqs:
                rp, ac = reqs[m['maps_to']]
                check(p.name in rp.read_text(), f'{p}: no reciprocal REQ link')
                check(rp.name in t, f'{p}: no mapped REQ link')
                then = t.split('## Then', 1)[-1].split('\n## ', 1)[0]
                cited = set(map(int, re.findall(r'REQ(?:-[A-Z0-9-]+)? AC-(\d+)', then)))
                check(cited and cited <= ac, f'{p}: invalid AC mapping {cited - ac}')
                coverage.setdefault(m['maps_to'], set()).update(cited)
            tests[p.stem] = p
    for name, (p, ac) in reqs.items():
        check(coverage.get(name) == ac, f'{p}: uncovered ACs {ac - coverage.get(name, set())}')
        for test_id in re.findall(r'\[(TEST-[^\]]+)\]', p.read_text()):
            check(test_id in tests and metadata(tests[test_id]).get('maps_to') == name, f'{p}: wrong test backlink {test_id}')
    documents = list(base.rglob('*.md')) + list((base.parents[1] / 'plc').rglob('*.md'))
    for p in documents:
        t = p.read_text()
        check('SPDX-License-Identifier: Apache-2.0' in t, f'{p}: missing license')
        check(t.count('<!-- ' + 'ovphysx-' + 'public-strip-begin -->') == t.count('<!-- ' + 'ovphysx-' + 'public-strip-end -->'), f'{p}: unbalanced export markers')
        for target in re.findall(r'\]\(([^)]+)\)', t):
            if '://' in target or target.startswith('#'):
                continue
            path, _, anchor = unquote(target).partition('#')
            dest = (p.parent / path).resolve()
            check(dest.exists(), f'{p.relative_to(base.parents[1])}: missing {target}')
            if anchor and dest.is_file() and dest.suffix == '.md':
                headings = re.findall(r'^#+ (.+)$', dest.read_text(), re.M)
                anchors = [re.sub(r'[^\w\- ]', '', h.lower()).replace(' ', '-') for h in headings]
                check(anchor in anchors, f'{p.relative_to(base.parents[1])}: missing anchor {target}')
        if args.public:
            check('gitlab-master' + '.nvidia.com' not in t and 'docs.google.com' not in t, f'{p}: internal URL in public export')
    rows = re.findall(r'^\| (\d+) \|', (base / 'ux/FEATURE-COVERAGE.md').read_text(), re.M)
    check(len(rows) == 108 and len(set(rows)) == 108, 'Expected 108 unique feature-sheet rows')
    nodes = json.loads((base / 'ux/graph-contracts.json').read_text())['nodes']
    check(len(nodes) == 35, 'Expected 35 graph descriptors')
    if errors:
        print('\n'.join(errors))
        raise SystemExit(1)
    print(f'Validated {len(reqs)} requirements, {sum(len(ac) for _, ac in reqs.values())} ACs, {len(tests)} TESTs, 35 ADRs, 108 sheet rows and document links' + (' (public export).' if args.public else '; standalone document validation passed.'))


if __name__ == '__main__':
    main()
