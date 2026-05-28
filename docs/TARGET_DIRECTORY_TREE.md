# Target Directory Tree

```text
OBDM_ws/
├── README.md
├── .gitignore
├── setup_env.sh
├── run_sim.sh
├── run_user_stack.sh
│
├── docs/
│   ├── ARCHITECTURE.md
│   ├── RUNBOOK.md
│   ├── INTERFACE_SPEC.md
│   ├── SCENARIO_GUIDE.md
│   ├── USER_ALGORITHM_GUIDE.md
│   ├── DEPENDENCY_MAP.md
│   ├── FILE_CLASSIFICATION.md
│   ├── FILE_MIGRATION_PLAN.md
│   ├── CLEANUP_REPORT.md
│   └── TARGET_DIRECTORY_TREE.md
│
├── external/
│   └── ardu_ws/
│
├── simulator/
│   ├── launch/
│   ├── worlds/
│   ├── models/
│   ├── descriptions/
│   ├── rviz/
│   ├── terrain/
│   ├── visualization/
│   └── params/
│
├── bridges/
│   ├── ros_gz/
│   ├── mavlink/
│   ├── ardupilot/
│   └── matlab/
│
├── user_stack/
│   ├── perception/
│   ├── navigation/
│   ├── decision/
│   ├── guidance/
│   ├── control/
│   ├── matlab/
│   └── python/
│
├── scenarios/
│   ├── takeoff/
│   ├── circle_flight/
│   ├── waypoint_tracking/
│   ├── aruco_landing/
│   ├── moving_pad_landing/
│   ├── wind_disturbance/
│   └── ontology_landing/
│
├── config/
│   ├── vehicle/
│   ├── sensors/
│   ├── environment/
│   ├── autopilot/
│   ├── user_stack/
│   ├── simulation/
│   └── evaluation/
│
├── matlab/
│   ├── core/
│   ├── modules/
│   ├── algorithms/
│   ├── utils/
│   ├── examples/
│   └── tests/
│
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── matlab/
│   └── python/
│
├── scripts/
│   ├── system/
│   ├── diagnostics/
│   └── maintenance/
│
├── tools/
│   └── Micro-XRCE-DDS-Gen/
│
├── archive/
│   └── unused_candidates/
│
└── results/
    ├── logs/
    ├── rosbags/
    ├── ardupilot_logs/
    ├── matlab_logs/
    ├── plots/
    ├── reports/
    └── runtime/
```

This is the target structure. The first pass adds the runner, scenario descriptors, config seeds, and documentation without moving active source files.
