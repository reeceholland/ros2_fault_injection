"""Identify a current-run final report without treating its existence as success."""
import json
from pathlib import Path
import sys


def completed(directory):
    root = Path(directory)
    try:
        report = root / 'results.json'
        if report.stat().st_mtime_ns < (root / 'run-started').stat().st_mtime_ns:
            return False
        data = json.loads(report.read_text())
        return (type(data.get('passed')) is bool and
                isinstance(data.get('goals'), list) and bool(data['goals']) and
                type(data.get('observer', {}).get('passed')) is bool)
    except (OSError, ValueError, TypeError, AttributeError):
        return False


if __name__ == '__main__':
    valid = completed(sys.argv[1])
    if valid and '--passed' in sys.argv[2:]:
        valid = json.loads((Path(sys.argv[1]) / 'results.json').read_text())['passed']
    raise SystemExit(0 if valid else 1)
