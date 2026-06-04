#!/usr/bin/env python3
"""Analyze ontology snapshot JSON or JSONL logs."""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))

from schema_validation import ValidationError, iter_jsonl, validate_snapshot  # noqa: E402


def load_snapshots(path: Path) -> list[dict[str, Any]]:
    if path.suffix == ".jsonl":
        return [validate_snapshot(item) for item in iter_jsonl(path)]
    with path.open("r", encoding="utf-8") as handle:
        return [validate_snapshot(json.load(handle))]


def edge_id(edge: dict[str, Any]) -> str:
    return edge.get("id") or f"{edge['source']}->{edge['target']}:{edge.get('label', edge.get('type', 'related_to'))}"


def analyze(snapshot: dict[str, Any]) -> dict[str, Any]:
    nodes = snapshot.get("nodes", [])
    edges = snapshot.get("edges", [])
    node_ids = [node["id"] for node in nodes]
    node_id_set = set(node_ids)
    linked = {edge["source"] for edge in edges} | {edge["target"] for edge in edges}

    return {
        "graph_id": snapshot.get("graph_id"),
        "stamp": snapshot.get("stamp"),
        "node_count": len(nodes),
        "edge_count": len(edges),
        "node_types": Counter(node.get("type", "Entity") for node in nodes),
        "edge_types": Counter(edge.get("type", edge.get("label", "related_to")) for edge in edges),
        "inferred_edges": sum(1 for edge in edges if edge.get("inferred")),
        "asserted_edges": sum(1 for edge in edges if not edge.get("inferred")),
        "duplicate_node_ids": sorted([item for item, count in Counter(node_ids).items() if count > 1]),
        "orphan_nodes": sorted(node_id_set - linked),
        "missing_references": sorted(
            {
                f"{edge_id(edge)}:{endpoint}={edge[endpoint]}"
                for edge in edges
                for endpoint in ("source", "target")
                if edge.get(endpoint) not in node_id_set
            }
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize ontology graph snapshots.")
    parser.add_argument("path", type=Path, help="Snapshot JSON or JSONL log")
    args = parser.parse_args()

    try:
        snapshots = load_snapshots(args.path)
    except (OSError, json.JSONDecodeError, ValidationError) as exc:
        print(f"failed to read snapshots: {exc}")
        return 1

    if not snapshots:
        print("no snapshots found")
        return 1

    latest = analyze(snapshots[-1])
    print(f"snapshots: {len(snapshots)}")
    print(f"latest graph: {latest['graph_id']} stamp={latest['stamp']}")
    print(f"nodes: {latest['node_count']} edges: {latest['edge_count']}")
    print(f"inferred edges: {latest['inferred_edges']} asserted edges: {latest['asserted_edges']}")
    print("node types:")
    for key, value in latest["node_types"].most_common():
        print(f"  {key}: {value}")
    print("edge types:")
    for key, value in latest["edge_types"].most_common():
        print(f"  {key}: {value}")
    if latest["orphan_nodes"]:
        print("orphan nodes: " + ", ".join(latest["orphan_nodes"]))
    if latest["duplicate_node_ids"]:
        print("duplicate node ids: " + ", ".join(latest["duplicate_node_ids"]))
    if latest["missing_references"]:
        print("missing references: " + ", ".join(latest["missing_references"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
