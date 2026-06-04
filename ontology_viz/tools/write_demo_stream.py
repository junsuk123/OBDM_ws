#!/usr/bin/env python3
"""Append changing LandingOntologyTrajectoryGraph snapshots to a JSONL log."""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def wave(step: int, center: float, amplitude: float, speed: float, phase: float = 0.0) -> float:
    return round(clamp01(center + amplitude * math.sin(step * speed + phase)), 3)


def node(name: str, activation: float, kind: str, description: str, inferred: bool = False) -> dict:
    item = {
        "id": name,
        "label": name,
        "type": kind,
        "attributes": {
            "activation": round(clamp01(activation), 3),
            "description": description,
        },
        "confidence": round(clamp01(activation), 3),
    }
    if inferred:
        item["inferred"] = True
    return item


def edge(source: str, target: str, relation: str, weight: float, edge_type: str, inferred: bool = False) -> dict:
    item = {
        "id": f"{source}__{relation}__{target}",
        "source": source,
        "target": target,
        "label": relation,
        "type": edge_type,
        "confidence": round(clamp01(weight), 3),
    }
    if inferred:
        item["inferred"] = True
    return item


def build_snapshot(step: int) -> dict:
    wind_risk = wave(step, 0.32, 0.24, 0.23)
    drag_risk = wave(step, 0.24, 0.15, 0.19, 1.1)
    visual_stability = wave(step, 0.78, 0.18, 0.17, 0.7)
    alignment_state = wave(step, 0.70, 0.22, 0.21, 1.3)
    attitude_stability = wave(step, 0.86, 0.10, 0.15, 2.0)
    descent_stability = wave(step, 0.72, 0.16, 0.18, 0.4)
    time_efficiency = wave(step, 0.67, 0.18, 0.20, 2.4)
    energy_efficiency = wave(step, 0.80, 0.12, 0.12, 1.8)
    landing_intent = wave(step, 0.82, 0.14, 0.16, 0.2)
    pad_track = wave(step, 0.76, 0.18, 0.22, 0.5)

    operational_risk = clamp01(
        0.30 * wind_risk
        + 0.25 * drag_risk
        + 0.20 * (1.0 - energy_efficiency)
        + 0.15 * (1.0 - attitude_stability)
        + 0.10 * (1.0 - visual_stability)
    )
    track_quality = clamp01(
        0.45 * alignment_state + 0.35 * visual_stability + 0.20 * attitude_stability
    )
    efficiency = clamp01(0.55 * energy_efficiency + 0.45 * time_efficiency)
    future_pad_prediction = clamp01(0.35 + 0.55 * pad_track * track_quality)
    approach_aim_point = track_quality
    horizontal_guidance = clamp01(
        (0.45 + 0.70 * track_quality + 0.25 * time_efficiency)
        * (0.65 + 0.35 * landing_intent)
        * (1.0 - 0.30 * operational_risk)
        / 1.65
    )
    descent_authority = clamp01(
        landing_intent
        * alignment_state
        * visual_stability
        * attitude_stability
        * energy_efficiency
        * (1.0 - operational_risk)
    )
    final_land_readiness = clamp01(landing_intent * track_quality * (1.0 - operational_risk))

    selected = "DescentAuthority"
    if final_land_readiness > 0.62:
        selected = "FinalLandReadiness"
    elif operational_risk > 0.40:
        selected = "OperationalRisk"

    nodes = [
        node("VehicleState", 1.0, "evidence", "UAV pose, velocity, and attitude"),
        node("PadTrack", pad_track, "evidence", "Estimated landing pad center and velocity"),
        node("WindRisk", wind_risk, "semantic", "Wind and gust disturbance risk"),
        node("DragRisk", drag_risk, "semantic", "Aerodynamic drag disturbance risk"),
        node("VisualStability", visual_stability, "semantic", "Marker visibility and image stability"),
        node("AlignmentState", alignment_state, "semantic", "Horizontal/yaw alignment to pad"),
        node("AttitudeStability", attitude_stability, "semantic", "Roll/pitch stability"),
        node("DescentStability", descent_stability, "semantic", "Vertical-speed stability"),
        node("TimeEfficiency", time_efficiency, "semantic", "Time-to-target efficiency"),
        node("EnergyEfficiency", energy_efficiency, "semantic", "Energy cost efficiency"),
        node("OperationalRisk", operational_risk, "inference", "Unified landing risk", True),
        node("TrackQuality", track_quality, "inference", "Quality of visual/pad tracking", True),
        node("LandingIntent", landing_intent, "decision", "Posterior-weighted intent to land", True),
        node("FuturePadPrediction", future_pad_prediction, "prediction", "Multi-step future pad estimate", True),
        node("ApproachAimPoint", approach_aim_point, "trajectory", "Predicted point used for horizontal guidance", True),
        node("HorizontalGuidance", horizontal_guidance, "trajectory", "Horizontal pursuit authority", True),
        node("DescentAuthority", descent_authority, "trajectory", "Permission to descend", True),
        node("FinalLandReadiness", final_land_readiness, "terminal", "Readiness for final LAND mode", True),
    ]

    edges = [
        edge("VehicleState", "AlignmentState", "observes", 1.00, "state_observation"),
        edge("VehicleState", "AttitudeStability", "observes", 1.00, "state_observation"),
        edge("VehicleState", "DescentStability", "observes", 1.00, "state_observation"),
        edge("WindRisk", "OperationalRisk", "increases", 0.30, "risk_contribution"),
        edge("DragRisk", "OperationalRisk", "increases", 0.25, "risk_contribution"),
        edge("EnergyEfficiency", "OperationalRisk", "reduces", 0.20, "risk_mitigation"),
        edge("AttitudeStability", "OperationalRisk", "reduces", 0.15, "risk_mitigation"),
        edge("VisualStability", "OperationalRisk", "reduces", 0.10, "risk_mitigation"),
        edge("AlignmentState", "TrackQuality", "supports", 0.45, "track_quality_evidence"),
        edge("VisualStability", "TrackQuality", "supports", 0.35, "track_quality_evidence"),
        edge("AttitudeStability", "TrackQuality", "supports", 0.20, "track_quality_evidence"),
        edge("TimeEfficiency", "ApproachAimPoint", "extends_prediction", 0.45, "prediction_context", True),
        edge("PadTrack", "FuturePadPrediction", "predicts", 1.00, "prediction_source", True),
        edge("FuturePadPrediction", "ApproachAimPoint", "selects", 0.55, "prediction_selection", True),
        edge("TrackQuality", "HorizontalGuidance", "scales", 0.55, "guidance_gain", True),
        edge("LandingIntent", "HorizontalGuidance", "enables", 0.30, "guidance_gate", True),
        edge("OperationalRisk", "HorizontalGuidance", "limits", 0.45, "guidance_limit", True),
        edge("LandingIntent", "DescentAuthority", "enables", 0.20, "descent_gate", True),
        edge("AlignmentState", "DescentAuthority", "requires", 0.20, "descent_requirement", True),
        edge("VisualStability", "DescentAuthority", "requires", 0.20, "descent_requirement", True),
        edge("AttitudeStability", "DescentAuthority", "requires", 0.20, "descent_requirement", True),
        edge("DescentStability", "DescentAuthority", "requires", 0.20, "descent_requirement", True),
        edge("EnergyEfficiency", "DescentAuthority", "supports", 0.20, "descent_support", True),
        edge("OperationalRisk", "DescentAuthority", "limits", 0.20, "descent_limit", True),
        edge("DescentAuthority", "FinalLandReadiness", "supports", 0.35, "terminal_readiness", True),
        edge("TrackQuality", "FinalLandReadiness", "supports", 0.35, "terminal_readiness", True),
        edge("OperationalRisk", "FinalLandReadiness", "limits", 0.30, "terminal_limit", True),
    ]

    tick = step * 0.35
    pad = {
        "x": round(1.2 * math.cos(0.32 * tick), 3),
        "y": round(1.2 * math.sin(0.32 * tick), 3),
    }
    drone = {
        "x": round(-3.2 + min(step, 35) * 0.095 + 0.18 * math.sin(0.24 * tick), 3),
        "y": round(-2.4 + min(step, 35) * 0.075 + 0.14 * math.cos(0.22 * tick), 3),
    }
    target = {
        "x": round(drone["x"] + (pad["x"] - drone["x"]) * horizontal_guidance, 3),
        "y": round(drone["y"] + (pad["y"] - drone["y"]) * horizontal_guidance, 3),
    }
    actual_path = []
    generated_path = []
    for idx in range(max(0, step - 34), step + 1):
        t = idx * 0.35
        px = 1.2 * math.cos(0.32 * t)
        py = 1.2 * math.sin(0.32 * t)
        dx = -3.2 + min(idx, 35) * 0.095 + 0.18 * math.sin(0.24 * t)
        dy = -2.4 + min(idx, 35) * 0.075 + 0.14 * math.cos(0.22 * t)
        actual_path.append({"x": round(dx, 3), "y": round(dy, 3)})
        gain = 0.45 + 0.35 * clamp01(idx / 30.0)
        generated_path.append({
            "x": round(dx + (px - dx) * gain, 3),
            "y": round(dy + (py - dy) * gain, 3),
        })

    return {
        "stamp": time.time(),
        "graph_id": "LandingOntologyTrajectoryGraph",
        "nodes": nodes,
        "edges": edges,
        "selected": {
            "node_id": selected,
            "reason": "Selected by the live landing ontology trajectory decision state.",
        },
        "attributes": {
            "tick": step,
            "landingIntent": round(landing_intent, 3),
            "operationalRisk": round(operational_risk, 3),
            "trackQuality": round(track_quality, 3),
            "efficiency": round(efficiency, 3),
            "descentAuthority": round(descent_authority, 3),
            "finalLandReadiness": round(final_land_readiness, 3),
        },
        "topdown": {
            "drone": drone,
            "pad": pad,
            "target": target,
            "actual_path": actual_path,
            "generated_path": generated_path,
            "min_span": 6.0,
            "grid_m": 1.0,
        },
        "reasoning_trace": [
            {
                "step": 1,
                "description": "VisualStability, AlignmentState, and AttitudeStability support TrackQuality.",
                "nodes": ["VisualStability", "AlignmentState", "AttitudeStability", "TrackQuality"],
                "edges": [
                    "VisualStability__supports__TrackQuality",
                    "AlignmentState__supports__TrackQuality",
                    "AttitudeStability__supports__TrackQuality",
                ],
                "confidence": round(track_quality, 3),
            },
            {
                "step": 2,
                "description": "WindRisk, DragRisk, EnergyEfficiency, AttitudeStability, and VisualStability form OperationalRisk.",
                "nodes": [
                    "WindRisk",
                    "DragRisk",
                    "EnergyEfficiency",
                    "AttitudeStability",
                    "VisualStability",
                    "OperationalRisk",
                ],
                "edges": [
                    "WindRisk__increases__OperationalRisk",
                    "DragRisk__increases__OperationalRisk",
                    "EnergyEfficiency__reduces__OperationalRisk",
                    "AttitudeStability__reduces__OperationalRisk",
                    "VisualStability__reduces__OperationalRisk",
                ],
                "confidence": round(1.0 - operational_risk, 3),
            },
            {
                "step": 3,
                "description": "PadTrack predicts FuturePadPrediction, which selects ApproachAimPoint.",
                "nodes": ["PadTrack", "FuturePadPrediction", "ApproachAimPoint"],
                "edges": [
                    "PadTrack__predicts__FuturePadPrediction",
                    "FuturePadPrediction__selects__ApproachAimPoint",
                ],
                "confidence": round(future_pad_prediction, 3),
            },
            {
                "step": 4,
                "description": "LandingIntent, TrackQuality, and OperationalRisk determine HorizontalGuidance and DescentAuthority.",
                "nodes": [
                    "LandingIntent",
                    "TrackQuality",
                    "OperationalRisk",
                    "HorizontalGuidance",
                    "DescentAuthority",
                ],
                "edges": [
                    "TrackQuality__scales__HorizontalGuidance",
                    "LandingIntent__enables__HorizontalGuidance",
                    "OperationalRisk__limits__HorizontalGuidance",
                    "LandingIntent__enables__DescentAuthority",
                    "OperationalRisk__limits__DescentAuthority",
                ],
                "confidence": round(max(horizontal_guidance, descent_authority), 3),
            },
            {
                "step": 5,
                "description": "DescentAuthority, TrackQuality, and OperationalRisk infer FinalLandReadiness.",
                "nodes": ["DescentAuthority", "TrackQuality", "OperationalRisk", "FinalLandReadiness"],
                "edges": [
                    "DescentAuthority__supports__FinalLandReadiness",
                    "TrackQuality__supports__FinalLandReadiness",
                    "OperationalRisk__limits__FinalLandReadiness",
                ],
                "confidence": round(final_land_readiness, 3),
            },
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Write a changing landing ontology graph stream.")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/runtime/ontology_snapshots.jsonl"),
        help="JSONL path to append snapshots to",
    )
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--truncate", action="store_true", help="Start with an empty log")
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    mode = "w" if args.truncate else "a"
    with args.output.open(mode, encoding="utf-8") as handle:
        step = 1
        while True:
            handle.write(json.dumps(build_snapshot(step), sort_keys=True) + "\n")
            handle.flush()
            print(f"wrote LandingOntologyTrajectoryGraph snapshot {step} to {args.output}")
            step += 1
            time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
