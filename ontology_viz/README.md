# Ontology Visualization Framework

This directory contains a backend-agnostic ontology graph viewer. It is not tied
to robotics, simulation, ROS, MATLAB, or a specific ontology vocabulary.

## Snapshot Contract

Required top-level fields:

- `graph_id`: stable graph name
- `nodes`: entity/class instances
- `edges`: relationships/predicates between nodes

Common optional fields:

- `stamp` or version metadata
- node and edge `attributes`
- `inferred`, `confidence`, or `score`
- `selected` node or edge state
- `reasoning_trace` steps with node and edge references
- `topdown` trajectory projection with `drone`, `pad`, `target`, `actual_path`,
  and `generated_path`

See `frontend/data/demo_graph.json` for a complete `LandingOntologyTrajectoryGraph`
example using the landing ontology node names and relation names.

## Local UI

```bash
cd ontology_viz/frontend
python -m http.server 8080
```

Open `http://127.0.0.1:8080`.

## Validation

```bash
python ontology_viz/backend/schema_validation.py ontology_viz/frontend/data/demo_graph.json
```

## REST and SSE Adapter

```bash
python ontology_viz/backend/graph_provider.py \
  --snapshot ontology_viz/frontend/data/demo_graph.json \
  --port 8765
```

Endpoints:

- `http://127.0.0.1:8765/snapshot`
- `http://127.0.0.1:8765/events`

For a live `LandingOntologyTrajectoryGraph` JSONL demo, run these in separate terminals:

```bash
python ontology_viz/tools/write_demo_stream.py --truncate
python ontology_viz/backend/graph_provider.py \
  --snapshot data/runtime/ontology_snapshots.jsonl \
  --port 8765
```

Open the web UI and press `Live`. The viewer will update whenever a new
snapshot is appended to the JSONL log. Press `Poll` to refresh from a REST
snapshot URL once per second instead.

## Log Analysis

```bash
python ontology_viz/tools/analyze_ontology_logs.py data/runtime/ontology_snapshots.jsonl
```

The analyzer reports node types, edge types, inferred/asserted counts, orphan
nodes, duplicate node IDs, and missing edge references.
