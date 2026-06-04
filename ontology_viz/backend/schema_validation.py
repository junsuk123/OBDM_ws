#!/usr/bin/env python3
"""Domain-neutral ontology graph snapshot validation.

The validator is intentionally lightweight so any runtime can emit graph JSON
without depending on a specific ontology stack.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable


class ValidationError(ValueError):
    """Raised when a graph snapshot violates the ontology graph contract."""


def _expect_object(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValidationError(f"{path} must be an object")
    return value


def _expect_list(value: Any, path: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValidationError(f"{path} must be a list")
    return value


def _expect_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValidationError(f"{path} must be a non-empty string")
    return value


def _validate_score(value: Any, path: str) -> None:
    if value is None:
        return
    if not isinstance(value, (int, float)):
        raise ValidationError(f"{path} must be numeric")
    if value < 0.0 or value > 1.0:
        raise ValidationError(f"{path} must be between 0.0 and 1.0")


def validate_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Validate and normalize one ontology graph snapshot.

    Required fields are deliberately small: graph_id, nodes, and edges. Optional
    fields cover inferred facts, provenance, selection, and reasoning traces.
    """
    root = _expect_object(snapshot, "$")
    _expect_string(root.get("graph_id"), "$.graph_id")
    nodes = _expect_list(root.get("nodes"), "$.nodes")
    edges = _expect_list(root.get("edges"), "$.edges")

    node_ids: set[str] = set()
    for index, node in enumerate(nodes):
        path = f"$.nodes[{index}]"
        item = _expect_object(node, path)
        node_id = _expect_string(item.get("id"), f"{path}.id")
        if node_id in node_ids:
            raise ValidationError(f"{path}.id duplicates node id '{node_id}'")
        node_ids.add(node_id)
        _expect_string(item.get("label", node_id), f"{path}.label")
        _expect_string(item.get("type", "Entity"), f"{path}.type")
        if "attributes" in item and not isinstance(item["attributes"], dict):
            raise ValidationError(f"{path}.attributes must be an object")
        if "confidence" in item:
            _validate_score(item["confidence"], f"{path}.confidence")
        if "score" in item:
            _validate_score(item["score"], f"{path}.score")

    edge_ids: set[str] = set()
    for index, edge in enumerate(edges):
        path = f"$.edges[{index}]"
        item = _expect_object(edge, path)
        source = _expect_string(item.get("source"), f"{path}.source")
        target = _expect_string(item.get("target"), f"{path}.target")
        label = _expect_string(item.get("label", item.get("type", "related_to")), f"{path}.label")
        if source not in node_ids:
            raise ValidationError(f"{path}.source references missing node '{source}'")
        if target not in node_ids:
            raise ValidationError(f"{path}.target references missing node '{target}'")
        edge_id = item.get("id", f"{source}->{target}:{label}")
        _expect_string(edge_id, f"{path}.id")
        if edge_id in edge_ids:
            raise ValidationError(f"{path}.id duplicates edge id '{edge_id}'")
        edge_ids.add(edge_id)
        if "attributes" in item and not isinstance(item["attributes"], dict):
            raise ValidationError(f"{path}.attributes must be an object")
        if "confidence" in item:
            _validate_score(item["confidence"], f"{path}.confidence")

    selected = root.get("selected")
    if selected is not None:
        selected_obj = _expect_object(selected, "$.selected")
        node_id = selected_obj.get("node_id")
        if node_id is not None and node_id not in node_ids:
            raise ValidationError(f"$.selected.node_id references missing node '{node_id}'")
        edge_id = selected_obj.get("edge_id")
        if edge_id is not None and edge_id not in edge_ids:
            raise ValidationError(f"$.selected.edge_id references missing edge '{edge_id}'")

    trace = root.get("reasoning_trace", [])
    for index, step in enumerate(_expect_list(trace, "$.reasoning_trace")):
        path = f"$.reasoning_trace[{index}]"
        item = _expect_object(step, path)
        if "nodes" in item:
            for node_id in _expect_list(item["nodes"], f"{path}.nodes"):
                if node_id not in node_ids:
                    raise ValidationError(f"{path}.nodes references missing node '{node_id}'")
        if "edges" in item:
            for edge_id in _expect_list(item["edges"], f"{path}.edges"):
                if edge_id not in edge_ids and "->" not in str(edge_id):
                    raise ValidationError(f"{path}.edges references missing edge '{edge_id}'")
        if "confidence" in item:
            _validate_score(item["confidence"], f"{path}.confidence")

    return root


def iter_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValidationError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
            yield _expect_object(value, f"{path}:{line_number}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate ontology graph JSON snapshots.")
    parser.add_argument("path", type=Path, help="JSON snapshot or JSONL snapshot log")
    args = parser.parse_args()

    try:
        if args.path.suffix == ".jsonl":
            count = 0
            for snapshot in iter_jsonl(args.path):
                validate_snapshot(snapshot)
                count += 1
        else:
            with args.path.open("r", encoding="utf-8") as handle:
                validate_snapshot(json.load(handle))
            count = 1
    except (OSError, json.JSONDecodeError, ValidationError) as exc:
        print(f"invalid: {exc}")
        return 1

    print(f"valid: {count} snapshot(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
