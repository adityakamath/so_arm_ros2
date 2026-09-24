"""CLI entry points for so_arm_mujoco: compiling models and previewing them.

Standalone, no install needed: `python3 -m so_arm_mujoco.cli build --model so101` or
`python3 -m so_arm_mujoco.cli preview --model so101`, run from this package's root dir (`-m`
puts the cwd on sys.path, so no ROS sourcing or pip install is required). After
`pip install -e .` (or a colcon build), the same tools are also `build_mujoco_models` and
`mujoco_preview` on PATH / via `ros2 run so_arm_mujoco <name>`.
"""
import argparse
from pathlib import Path

import mujoco

from so_arm_mujoco.simulation import (
    VALID_MODELS,
    Simulation,
    build_model_xml,
    committed_mjcf_path,
    committed_model_xml,
)


def _build_sim(model: str, wrist_camera: bool) -> Simulation:
    xml = build_model_xml(model, wrist_camera=wrist_camera, scene=True)
    sim = Simulation(mujoco.MjModel.from_xml_string(xml))
    sim.reset()
    return sim


def run_headless(model: str, steps: int = 200, wrist_camera: bool = True) -> dict:
    sim = _build_sim(model, wrist_camera)
    sim.step(count=steps)
    return sim.info()


# Two label/value columns, like MuJoCo's own built-in Info overlay - set_texts' 3rd/4th tuple
# fields render as left/right-aligned columns, not one run-on wrapped line.
CONTROL_LABELS = 'Joints\nE-Stop'
CONTROL_VALUES = "Right sidebar's Control panel\nSpace"
STATUS_LABELS = 'Model\nTime\nStatus'


def run_interactive(model: str, wrist_camera: bool = True):
    import time

    import glfw
    import mujoco.viewer

    from so_arm_mujoco.keyboard import EStop, HeldKeys

    sim = _build_sim(model, wrist_camera)

    message = 'Held-key input attached (space=e-stop)'
    keys = HeldKeys({glfw.KEY_SPACE}, edge_keys={glfw.KEY_SPACE}, attached_message=message)
    estop = EStop(sim.model)

    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=keys.bootstrap) as viewer:
        while viewer.is_running():
            step_start = time.time()
            while not keys.events.empty():
                if keys.events.get() == glfw.KEY_SPACE:
                    estop.toggle(sim)

            # data.ctrl is never touched here - e-stopped, its gain/damping are already zeroed
            # so ctrl is irrelevant; otherwise the viewer's own "Control" sidebar sliders
            # (per-actuator ctrl inputs) drive the arm directly, not overwritten every frame.
            sim.step(count=1)

            viewer.sync()
            status = 'E-STOP' if estop.tripped else 'Manual'
            viewer.set_texts([
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                 STATUS_LABELS, f'{model}\n{sim.data.time:.1f} s\n{status}'),
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                 CONTROL_LABELS, CONTROL_VALUES),
            ])

            remaining = sim.model.opt.timestep - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)


def build_main(argv=None):
    parser = argparse.ArgumentParser(description='Compile so_arm.mjcf.xacro to a MuJoCo XML.')
    parser.add_argument('--model', choices=VALID_MODELS, default='so101')
    parser.add_argument('--output', type=str, default=None, help='Write compiled XML here')
    parser.add_argument('--no-wrist-camera', action='store_true')
    parser.add_argument('--bridged', action='store_true', help='Omit the free-standing scene')
    parser.add_argument('--description-package', type=str, default=None)
    parser.add_argument(
        '--commit', action='store_true',
        help='Rewrite every committed mjcf/<model>.xml in place (ignores the other flags); '
             'run this after any mjcf/*.xacro or URDF change',
    )
    args = parser.parse_args(argv)

    if args.commit:
        for model in VALID_MODELS:
            path = committed_mjcf_path(model)
            path.write_text(committed_model_xml(
                model, description_share=args.description_package))
            print(f'Wrote {path}')
        return

    xml = build_model_xml(
        args.model,
        wrist_camera=not args.no_wrist_camera,
        scene=not args.bridged,
        description_share=args.description_package,
    )
    if args.output:
        Path(args.output).write_text(xml)
        print(f'Wrote {args.output}')
    else:
        print(xml)


def preview_main(argv=None):
    parser = argparse.ArgumentParser(description='Preview an so_arm MJCF model.')
    parser.add_argument('--model', choices=VALID_MODELS, default='so101')
    parser.add_argument('--no-wrist-camera', action='store_true')
    parser.add_argument('--headless', action='store_true', help='No GUI; step and exit')
    parser.add_argument('--steps', type=int, default=200, help='--headless only')
    args = parser.parse_args(argv)

    if args.headless:
        print(run_headless(args.model, steps=args.steps, wrist_camera=not args.no_wrist_camera))
    else:
        run_interactive(args.model, wrist_camera=not args.no_wrist_camera)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['build', 'preview', 'teleop'])
    args, rest = parser.parse_known_args()
    if args.command == 'build':
        build_main(rest)
    elif args.command == 'preview':
        preview_main(rest)
    else:
        from so_arm_mujoco.teleop_ik import teleop_main
        teleop_main(rest)


if __name__ == '__main__':
    main()
