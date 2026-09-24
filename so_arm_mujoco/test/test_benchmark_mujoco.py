"""so_arm_mujoco.benchmark_mujoco: headless step-response measurement, no GUI required."""
import json

import mujoco

from so_arm_mujoco.benchmark_mujoco import benchmark, main, STEP_FRACTIONS
from so_arm_mujoco.simulation import build_model_xml, Simulation


def test_benchmark_covers_every_joint_and_step_fraction():
    sim = Simulation(mujoco.MjModel.from_xml_string(build_model_xml('so101')))

    rows = benchmark('so101')

    assert len(rows) == len(sim.joint_names) * len(STEP_FRACTIONS)
    assert {row['joint'] for row in rows} == set(sim.joint_names)


def test_benchmark_reports_bounded_metrics_per_row():
    # Not every joint fully settles in 2s (e.g. shoulder_lift_joint has real steady-state
    # gravity droop under a plain P-gain position actuator near its upper range) - the
    # benchmark's job is to report that honestly, not paper over it with a loose target.
    rows = benchmark('so101')

    for row in rows:
        assert row['settle_seconds'] is None or 0.0 <= row['settle_seconds'] <= 2.0
        assert abs(row['final_error']) < abs(row['target'] - row['start']) + 1e-9
        assert row['overshoot_fraction'] >= 0.0


def test_main_writes_json_report(tmp_path):
    output = tmp_path / 'nested' / 'report.json'

    main(['--model', 'so101', '--output', str(output)])

    rows = json.loads(output.read_text())
    assert rows
    assert all(row['model'] == 'so101' for row in rows)
