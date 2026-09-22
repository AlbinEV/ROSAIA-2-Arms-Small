# Learned model artifacts

Each validated model occupies one directory per arm and calibration, for
example:

```text
models/20260921_calibration_01/
├── arm_1/
│   ├── model.npz
│   └── metadata.json
└── arm_2/
    ├── model.npz
    └── metadata.json
```

`model.npz` contains only numeric arrays and is loaded with
`allow_pickle=False`. `metadata.json` records model structure, training and
validation sessions, row counts, seed, iterations, loss, and held-out RMSE.

Models are valid only for the encoder interval stored in the artifact. The ROS
Jacobian publisher refuses inference outside that interval. Promote an artifact
to online control only when its metadata references a completed calibration
manifest and it has passed offline replay.
