#!/usr/bin/env python3
"""Measure each so_arm joint's step response (settle time, overshoot, final error):
python3 -m so_arm_mujoco.benchmark_mujoco --output out.json.
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from so_arm_mujoco.simulation import VALID_MODELS, Simulation, build_model_xml

SETTLE_SECONDS = 2.0
SETTLE_BAND_FRACTION = 0.02  # of the joint's full range, matches pt_mujoco's benchmark convention
STEP_FRACTIONS = (0.25, 0.75)  # of the joint's [lower, upper] range, one step in each direction


def _settle_seconds(trace: np.ndarray, target: float, band: float, timestep: float):
    outside = np.flatnonzero(np.abs(trace - target) > band)
    if not outside.size:
        return 0.0
    if outside[-1] == len(trace) - 1:
        return None  # never settled within the band before the run ended
    return float((outside[-1] + 1) * timestep)


def _overshoot_fraction(trace: np.ndarray, start: float, target: float, span: float) -> float:
    direction = np.sign(target - start)
    if direction == 0 or span <= 0:
        return 0.0
    peak = np.max(trace) if direction > 0 else np.min(trace)
    overshoot = max(0.0, (peak - target) * direction)
    return float(overshoot / span)


def benchmark(model_name: str) -> list:
    """Step each joint from its current pose toward two targets (25% and 75% along its
    [lower, upper] range) while every other joint holds its start position, and record how it
    gets there - same step-response methodology as pt_mujoco/benchmark_mujoco.py, one joint at
    a time instead of pt's fixed pan/tilt pair.
    """
    sim = Simulation(mujoco.MjModel.from_xml_string(build_model_xml(model_name)))
    rows = []
    steps = round(SETTLE_SECONDS / sim.model.opt.timestep)
    for name in sim.joint_names:
        lo, hi = sim.model.joint(name).range
        span = hi - lo
        for fraction in STEP_FRACTIONS:
            sim.reset()
            start = sim.joint_state()['position'][name]
            target = float(lo + fraction * span)
            targets = sim.joint_state()['position']
            targets[name] = target

            trace = []
            for _ in range(steps):
                sim.step(count=1, targets=targets)
                trace.append(sim.joint_state()['position'][name])
            trace = np.array(trace)

            band = max(SETTLE_BAND_FRACTION * span, 1e-4)
            rows.append({
                'model': model_name,
                'joint': name,
                'target_fraction': fraction,
                'start': float(start),
                'target': target,
                'final_error': float(target - trace[-1]),
                'overshoot_fraction': _overshoot_fraction(trace, start, target, span),
                'settle_seconds': _settle_seconds(trace, target, band, sim.model.opt.timestep),
            })
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--model', choices=VALID_MODELS, action='append',
        help='Repeatable; defaults to all of %(choices)s',
    )
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)

    models = args.model or list(VALID_MODELS)
    rows = [row for model in models for row in benchmark(model)]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(rows, indent=2))
    print(f'Wrote {len(rows)} rows to {args.output}')


if __name__ == '__main__':
    main()
