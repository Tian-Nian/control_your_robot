"""Summarize visual harness logs offline without a model call or runtime connection."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from robot.harness.diagnostics import summarize_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='*', type=Path)
    parser.add_argument('--recent', type=int, default=8)
    parser.add_argument('--out', type=Path, help='Optional JSON report file (never overwrites)')
    args = parser.parse_args()
    if args.recent < 1:
        parser.error('--recent must be positive')
    paths = args.runs or [p.parent for p in sorted((ROOT / 'save').glob('vision_*/turns.jsonl'),
                                                 key=lambda p: p.stat().st_mtime, reverse=True)[:args.recent]]
    reports = []
    for path in paths:
        try:
            reports.append(summarize_run(path))
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
            reports.append({'run': path.name, 'error': type(exc).__name__})
    text = json.dumps(reports, ensure_ascii=False, indent=2)
    if args.out:
        with args.out.open('x', encoding='utf-8') as stream:
            stream.write(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
