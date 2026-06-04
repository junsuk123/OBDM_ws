#!/usr/bin/env python3
"""Small backend adapters for serving ontology graph snapshots.

This module provides file loading, JSONL live tailing, and a minimal SSE endpoint.
Projects with ROS, MATLAB, a database, or a message bus can replace this layer
while keeping the frontend graph contract unchanged.
"""
from __future__ import annotations

import argparse
import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from schema_validation import iter_jsonl, validate_snapshot


class GraphStore:
    def __init__(self, snapshot_path: Path):
        self.snapshot_path = snapshot_path

    def latest(self) -> dict[str, Any]:
        if self.snapshot_path.suffix == ".jsonl":
            latest = None
            for latest in iter_jsonl(self.snapshot_path):
                pass
            if latest is None:
                raise ValueError(f"{self.snapshot_path} has no snapshots")
            return validate_snapshot(latest)

        with self.snapshot_path.open("r", encoding="utf-8") as handle:
            return validate_snapshot(json.load(handle))

    def replay(self) -> list[dict[str, Any]]:
        if self.snapshot_path.suffix == ".jsonl":
            return [validate_snapshot(item) for item in iter_jsonl(self.snapshot_path)]
        return [self.latest()]

    def watch(self, interval: float):
        """Yield validated snapshots whenever the source file changes."""
        if self.snapshot_path.suffix == ".jsonl":
            seen = 0
            last_size = -1
            while True:
                try:
                    snapshots = self.replay()
                    size = self.snapshot_path.stat().st_size
                except (FileNotFoundError, ValueError):
                    time.sleep(interval)
                    continue

                if size < last_size:
                    seen = 0
                last_size = size

                for snapshot in snapshots[seen:]:
                    yield snapshot
                seen = len(snapshots)
                time.sleep(interval)

        last_signature = None
        while True:
            try:
                stat = self.snapshot_path.stat()
                signature = (stat.st_mtime_ns, stat.st_size)
                if signature != last_signature:
                    last_signature = signature
                    yield self.latest()
            except (FileNotFoundError, ValueError):
                pass
            time.sleep(interval)


def append_snapshot(snapshot: dict[str, Any], log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    validate_snapshot(snapshot)
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(snapshot, sort_keys=True) + "\n")


def make_handler(store: GraphStore, watch_interval: float):
    class Handler(BaseHTTPRequestHandler):
        def _headers(self, status: int, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

        def do_GET(self) -> None:
            if self.path in ("/", "/snapshot"):
                try:
                    body = json.dumps(store.latest()).encode("utf-8")
                except Exception as exc:  # noqa: BLE001 - returned to local operator.
                    self._headers(500, "text/plain; charset=utf-8")
                    self.wfile.write(str(exc).encode("utf-8"))
                    return
                self._headers(200, "application/json")
                self.wfile.write(body)
                return

            if self.path == "/events":
                self._headers(200, "text/event-stream")
                try:
                    self.wfile.write(b": connected\n\n")
                    self.wfile.flush()
                    for snapshot in store.watch(watch_interval):
                        payload = json.dumps(snapshot)
                        self.wfile.write(f"event: graph\ndata: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                except BrokenPipeError:
                    return
                return

            self._headers(404, "text/plain; charset=utf-8")
            self.wfile.write(b"not found")

        def log_message(self, format: str, *args: Any) -> None:
            print(f"{self.address_string()} - {format % args}")

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="Serve ontology graph snapshots over REST and SSE.")
    parser.add_argument(
        "--snapshot",
        type=Path,
        default=Path("ontology_viz/frontend/data/demo_graph.json"),
        help="JSON snapshot or JSONL log to serve",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--watch-interval", type=float, default=0.5)
    args = parser.parse_args()

    store = GraphStore(args.snapshot)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(store, args.watch_interval))
    print(f"serving {args.snapshot} at http://{args.host}:{args.port}/snapshot")
    print(f"SSE live stream available at http://{args.host}:{args.port}/events")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
