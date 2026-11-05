#!/usr/bin/env python3
"""Publish bounded regression context as a GitHub Actions error annotation."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', default='artifacts/navcraft-regression/result.json')
    parser.add_argument('--log', action='append', default=[])
    args = parser.parse_args()
    context = {}
    report = Path(args.report)
    if report.is_file():
        try:
            context['regression'] = json.loads(report.read_text(encoding='utf-8'))
        except (ValueError, OSError) as error:
            context['report_error'] = str(error)
    for name in args.log:
        path = Path(name)
        if path.is_file():
            context[name] = '\n'.join(path.read_text(encoding='utf-8', errors='replace').splitlines()[-80:])[-12000:]
    if not context:
        context['message'] = 'Regression produced no report or captured test log.'
    message = json.dumps(context, ensure_ascii=True)
    print(message, flush=True)
    escaped = message.replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')
    print('::error title=CI regression context::' + escaped, flush=True)


if __name__ == '__main__':
    main()
