# Ontology Design

The first implementation uses a lightweight YAML rule reasoner instead of OWL/RDF. This keeps the system testable while preserving a clean extension path to Protégé, RDFLib, or a formal ontology server.

## Raw Inputs

`LandingState.msg` contains:

- marker visibility and confidence
- UAV pose and velocity
- platform pose and velocity
- relative pose and distance
- wind speed and direction
- attitude error
- altitude
- battery level

Future inputs should add obstacle distance, pad boundary margin, rover body distance, and camera/estimator covariance.

## Semantic Outputs

`LandingDecision.msg` contains:

- `landing_phase`
- `selected_mode`
- `risk_level`
- `allowed_to_descend`
- `abort_required`
- `explanation`

Initial modes:

- `DIRECT_DESCENT`
- `TRACKING_APPROACH`
- `CONSERVATIVE_APPROACH`
- `MINIMUM_TIME_APPROACH`
- `HOVER_AND_ALIGN`
- `ASCEND_AND_RESEARCH`
- `ABORT_LANDING`
- `EMERGENCY_LAND`

## Rule File

Rules are stored in:

- `src/landing_ontology_reasoner/rules/landing_rules.yaml`

The reasoner evaluates rules in order. The first matching rule wins. The default mode is `HOVER_AND_ALIGN`, which is conservative when no high-confidence condition is met.

## How Decisions Affect Control

- `DIRECT_DESCENT`: final descent allowed only when marker confidence, wind, platform speed, and attitude are safe.
- `MINIMUM_TIME_APPROACH`: chosen only for low-risk, larger-distance approach.
- `CONSERVATIVE_APPROACH`: used for wind or moderate platform risk; path generator adds lateral offset and slower descent.
- `HOVER_AND_ALIGN`: holds altitude and reduces horizontal error.
- `ASCEND_AND_RESEARCH`: climbs to regain marker visibility.
- `ABORT_LANDING`: climbs toward an abort altitude.
- `EMERGENCY_LAND`: allows descent under constrained safety assumptions when battery is critical and pad is visible.

## Future Formal Ontology Mapping

The YAML rule concepts can map to ontology classes:

- `LandingSituation`
- `WindRisk`
- `MarkerReliability`
- `PlatformMotionRisk`
- `DescentSafety`
- `LandingMode`
- `AbortCondition`

The rule output can become an asserted individual such as `CurrentLandingSituation hasMode DIRECT_DESCENT`.
