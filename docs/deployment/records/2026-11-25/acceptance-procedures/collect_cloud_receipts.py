"""Index DashScope query markers without copying environment, headers or request payloads."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re


def collect(root, batch):
    ledger = root / (batch + '-attempts.jsonl')
    entries = [json.loads(line) for line in ledger.read_text().splitlines()]
    labels = [e['label'] for e in entries if e['event'] == 'finish']
    if len(set(labels)) != 16 or len(labels) != 16:
        raise ValueError('Expected sixteen unique finished attempts')
    calls = {}
    for label in labels:
        if not re.fullmatch(batch + r'-(?:2\.4|1\.5)-(?:front|left45|left90|right45|right90)-[12]', label):
            raise ValueError('Unrecognized flight label')
        data = (root / 'flights' / label / 'launch.log').read_bytes()
        markers = []
        models = Counter()
        for line_no, line in enumerate(data.splitlines(), 1):
            if b'Querying DashScope (' not in line:
                continue
            known_model = b'querying dashscope (qwen3.8-omni-flash)' in line.lower()
            safe_model = 'qwen3.8-omni-flash' if known_model else 'unclassified'
            markers.append(dict(line=line_no, model=safe_model, line_sha256=hashlib.sha256(line).hexdigest()))
            models[safe_model] += 1
        calls[label] = dict(source='flights/' + label + '/launch.log', sha256=hashlib.sha256(data).hexdigest(),
                            count=len(markers), models=dict(models), query_markers=markers)
    return dict(batch=batch, total=sum(r['count'] for r in calls.values()), attempts=calls)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('batch', choices=['V03','V04'])
    args = parser.parse_args()
    print(json.dumps(collect(args.root, args.batch), indent=2))
