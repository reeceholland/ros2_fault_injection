#!/usr/bin/env python3
"""Render observer results into the GitHub job summary, including setup failures."""
import argparse
import html
import json
from pathlib import Path


def cell(value):
    return html.escape(str(value)).replace('|', '&#124;').replace('\n', ' ').replace('\r', ' ')


def render(path):
    lines = ['\n## Resilience and collision observer', '']
    try:
        report = json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        return '\n'.join(lines + ['**NO RESULT — not a pass.**', '',
            'The observer did not produce a readable report. Check setup, player version and navigation logs.',
            '', f'Diagnostic: {cell(exc)}', ''])
    lines += [f"**{'PASS' if report.get('passed') is True else 'FAIL'}**", '',
              '| Assertion | Result | Detail |', '|---|---|---|']
    failures = report.get('failures', {})
    for name, passed in report.get('checks', {}).items():
        lines.append(f"| {cell(name)} | {'PASS' if passed and name not in failures else 'FAIL'} | {cell(failures.get(name, ''))} |")
    for name, detail in failures.items():
        if name not in report.get('checks', {}):
            lines.append(f'| {cell(name)} | FAIL | {cell(detail)} |')
    collisions = [e for e in report.get('events', []) if e.get('kind') == 'collision']
    lines += ['', f'Collision event records: **{len(collisions)}**', '',
              '| Simulation receipt time (s) | Object | Cumulative contact count | Active contacts |',
              '|---|---|---|---|']
    for event in collisions[:50]:
        lines.append('| ' + ' | '.join(cell(event.get(k, '-')) for k in ('time', 'last_object', 'count', 'active')) + ' |')
    if not collisions:
        lines.append('| — | No recorded contact events; see telemetry assertions above | — | — |')
    lines += ['', '### Measured stopping behaviour', '', '| Metric | Value |', '|---|---|']
    for name, value in report.get('metrics', {}).items():
        lines.append(f'| {cell(name)} | {cell(value)} |')
    lines += ['', 'Full reports: `navigation/observer/results.json`, `navigation/observer/junit.xml`.',
              'Live assertions: navigation step log (`OBSERVER EVENT`, `OBSERVER FAIL`).',
              'Download the CI logs artifact for Unity logs, observer limits, and collision/ground-truth ROS bag topics.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('report')
    parser.add_argument('--summary', required=True)
    args = parser.parse_args()
    text = render(args.report)
    print(text)
    with Path(args.summary).open('a', encoding='utf-8') as stream:
        stream.write(text)


if __name__ == '__main__':
    main()
