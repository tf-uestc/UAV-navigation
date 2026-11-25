"""Archive explicitly selected SITL measurements from a completed trial ledger."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tarfile


FLIGHT_FILES = {
    "landing-handshake.json",
    "acceptance.json", "clearance.json", "commit.txt", "contacts-ready.json", "flight.exit",
    "independent-planning.json", "local_rgb_fixture-summary.json", "moving.json",
    "native-delivery-original.json", "native-delivery-interpretation.json",
    "physics-contacts.json", "physics-contacts.txt", "physics-original-audit.json",
    "physics-grounded-shutdown.json", "postflight-state.json", "source-status.txt",
    "start.json", "stop-before.json", "result/results.json", "result/trace.json",
    "result/physics-contacts.txt", "result/physics-contacts.json", "images/observations.json",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("batch")
    parser.add_argument("--expected", type=int, required=True)
    parser.add_argument("--supplement", action="store_true", help="Read the numbered follow-up ledger")
    args = parser.parse_args()
    if not args.batch.isalnum():
        parser.error("A simple batch identifier is required")
    root = args.root.resolve()
    series = args.batch + ("-supplement" if args.supplement else "")
    ledger = root / (series + "-attempts.jsonl")
    events = [json.loads(s) for s in ledger.read_text().splitlines()]
    starts = [r["label"] for r in events if r["event"] == "start"]
    finished = [r["label"] for r in events if r["event"] == "finish"]
    if len(starts) != args.expected or len(set(starts)) != len(starts) or starts != finished:
        raise RuntimeError("Will not archive an incomplete or inconsistent batch as finished")
    destination = root / (series + "-evidence.tar.gz")
    temporary = destination.with_name(destination.name + ".partial")
    if destination.exists() or temporary.exists():
        raise FileExistsError("An archive for this batch already exists")
    selected = [ledger]
    for label in starts:
        if not label.startswith(args.batch + "-") or not all(c.isalnum() or c in ".-" for c in label):
            raise ValueError("Invalid case label")
        case = root / "flights" / label
        for rel in sorted(FLIGHT_FILES):
            path = case / rel
            if path.is_file():
                selected.append(path)
        images = case / "images"
        for pattern in ("*.png", "*-depth.npz"):
            selected.extend(sorted(images.glob(pattern)))
        for suffix in ("-ground.json", "-audit.json"):
            p = root / (label + suffix)
            if p.is_file():
                selected.append(p)
    with tarfile.open(temporary, "w:gz", compresslevel=1) as archive:
        for file in selected:
            if file.is_symlink() or root not in file.resolve().parents:
                raise ValueError("Archive member outside the experiment root")
            archive.add(file, arcname=file.relative_to(root).as_posix(), recursive=False)
    os.rename(temporary, destination)
    digest = hashlib.sha256()
    with destination.open("rb") as packed:
        for chunk in iter(lambda: packed.read(1024 * 1024), b""):
            digest.update(chunk)
    manifest = dict(batch=series, attempts=len(starts), file=destination.name,
                    bytes=destination.stat().st_size, members=len(selected), sha256=digest.hexdigest())
    (root / (series + "-evidence-manifest.json")).write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest))


if __name__ == "__main__":
    main()
