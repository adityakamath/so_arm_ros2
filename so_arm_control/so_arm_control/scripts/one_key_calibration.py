#!/usr/bin/env python3

"""Guided full-arm calibration CLI: midpoint pose -> one-key calibration -> manual range sweep.

Operator helper utility, not a long-running ROS 2 node. Unlike LeRobot's full calibration
pipeline, the range-sweep step is manual verification only - not yet recorded to a file.
"""

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from glob import glob
from pathlib import Path
from typing import List
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient

from sts_hardware_interface.srv import OneKeyCalibration
from std_srvs.srv import SetBool


def _parse_motor_ids(raw_values: List[str]) -> List[int]:
    motor_ids: List[int] = []
    for value in raw_values:
        parts = [part.strip() for part in value.split(',') if part.strip()]
        for part in parts:
            try:
                motor_id = int(part)
            except ValueError as exc:
                raise ValueError(f"Invalid motor id '{part}'. Must be an integer.") from exc
            if motor_id < 1 or motor_id > 253:
                raise ValueError(f'Invalid motor id {motor_id}. Expected range is 1..253.')
            motor_ids.append(motor_id)

    # Preserve order while removing duplicates.
    unique_ids: List[int] = []
    seen = set()
    for motor_id in motor_ids:
        if motor_id not in seen:
            seen.add(motor_id)
            unique_ids.append(motor_id)
    return unique_ids


def _check_serial_port(serial_port: str) -> None:
    if not os.path.exists(serial_port):
        raise FileNotFoundError(
            f"Serial port '{serial_port}' does not exist. Check your USB adapter path or udev rule."
        )
    if not os.path.exists('/dev'):
        return
    if not serial_port.startswith('/dev/'):
        raise ValueError(
            f"Serial port '{serial_port}' is not under /dev. Expected something like /dev/ttySERVO."
        )


def _scan_serial_ports() -> List[str]:
    candidates = [
        '/dev/ttySERVO',
        *sorted(glob('/dev/ttyUSB*')),
        *sorted(glob('/dev/ttyACM*')),
        *sorted(glob('/dev/tty.usbmodem*')),
        *sorted(glob('/dev/tty.usbserial-*')),
        *sorted(glob('/dev/cu.usbmodem*')),
        *sorted(glob('/dev/cu.usbserial-*')),
    ]
    unique: List[str] = []
    seen = set()
    for path in candidates:
        if path in seen:
            continue
        seen.add(path)
        if os.path.exists(path):
            unique.append(path)
    return unique


def _extract_hw_config_from_robot_description(robot_description: str) -> tuple[str | None, List[int], List[int]]:
    root = ET.fromstring(robot_description)
    serial_port: str | None = None
    all_motor_ids: List[int] = []
    mode0_motor_ids: List[int] = []

    for ros2_control in root.findall('.//ros2_control'):
        plugin_elem = ros2_control.find('./hardware/plugin')
        plugin_name = (plugin_elem.text or '').strip() if plugin_elem is not None else ''
        if 'sts_hardware_interface' not in plugin_name:
            continue

        if serial_port is None:
            for param in ros2_control.findall('./hardware/param'):
                if param.attrib.get('name') == 'serial_port':
                    value = (param.text or '').strip()
                    if value:
                        serial_port = value
                    break

        for joint in ros2_control.findall('./joint'):
            motor_id: int | None = None
            operating_mode = 1
            for param in joint.findall('./param'):
                name = param.attrib.get('name')
                text = (param.text or '').strip()
                if name == 'motor_id' and text:
                    try:
                        motor_id = int(text)
                    except ValueError:
                        motor_id = None
                elif name == 'operating_mode' and text:
                    try:
                        operating_mode = int(text)
                    except ValueError:
                        operating_mode = 1

            if motor_id is None:
                continue

            if motor_id not in all_motor_ids:
                all_motor_ids.append(motor_id)
            if operating_mode == 0 and motor_id not in mode0_motor_ids:
                mode0_motor_ids.append(motor_id)

    return serial_port, all_motor_ids, mode0_motor_ids


def _save_calibration_record(
    *,
    arm_id: str,
    calibration_dir: str,
    serial_port: str,
    service_name: str,
    motor_ids: List[int],
    accepted: bool,
    message: str,
) -> Path:
    output_dir = Path(calibration_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now(timezone.utc).isoformat()
    safe_arm_id = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in arm_id)
    output_path = output_dir / f'{safe_arm_id}.json'

    record = {
        'schema_version': 1,
        'arm_id': arm_id,
        'timestamp_utc': timestamp,
        'serial_port': serial_port,
        'service_name': service_name,
        'calibrated_motor_ids': motor_ids,
        'accepted': accepted,
        'message': message,
        'notes': (
            'One-key midpoint calibration completed. Min/max ranges are expected to come '
            'from URDF limits and are not recorded here by design.'
        ),
    }

    output_path.write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')
    return output_path


class OneKeyCalibrationClient(Node):
    def __init__(self, service_name: str) -> None:
        super().__init__('one_key_calibration_client')
        self._client = self.create_client(OneKeyCalibration, service_name)
        self._estop_client = self.create_client(SetBool, '/emergency_stop')

    def discover_from_controller_manager(self, controller_manager_node: str, timeout_sec: float) -> tuple[str | None, List[int], List[int]]:
        client = AsyncParameterClient(self, controller_manager_node)
        if not client.wait_for_services(timeout_sec=timeout_sec):
            raise TimeoutError(
                f"Parameter service for '{controller_manager_node}' not available within {timeout_sec:.1f}s"
            )

        future = client.get_parameters(['robot_description'])
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(
                f"Timed out while reading robot_description from '{controller_manager_node}'"
            )

        result = future.result()
        if result is None or not result.values:
            raise RuntimeError(
                f"No robot_description value returned from '{controller_manager_node}'"
            )

        robot_description = result.values[0].string_value
        if not robot_description:
            raise RuntimeError(
                f"robot_description on '{controller_manager_node}' is empty"
            )

        return _extract_hw_config_from_robot_description(robot_description)

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def call(self, motor_ids: List[int], timeout_sec: float) -> OneKeyCalibration.Response:
        request = OneKeyCalibration.Request()
        request.motor_ids = motor_ids

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            raise TimeoutError('Timed out waiting for calibration service response.')
        response = future.result()
        if response is None:
            raise RuntimeError('Calibration service call failed with no response.')
        return response

    def set_emergency_stop(self, enabled: bool, timeout_sec: float) -> SetBool.Response:
        request = SetBool.Request()
        request.data = enabled

        if not self._estop_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("Emergency stop service '/emergency_stop' is not available.")

        future = self._estop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            raise TimeoutError('Timed out waiting for /emergency_stop service response.')
        response = future.result()
        if response is None:
            raise RuntimeError('/emergency_stop service call failed with no response.')
        return response


def _wait_for_enter(prompt: str) -> None:
    input(f'\n{prompt}\nPress ENTER to continue...')


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Run guided full-arm calibration through sts_hardware_interface. '
            'If no motor IDs are provided, all mode-0 (servo) joints are targeted.'
        )
    )
    parser.add_argument(
        '--motor-id',
        action='append',
        default=[],
        help=(
            'Motor ID to calibrate. May be repeated or passed as comma-separated values '
            '(example: --motor-id 1 --motor-id 2 or --motor-id 1,2).'
        ),
    )
    parser.add_argument(
        '--service',
        default='/one_key_calibration',
        help='Service name (default: /one_key_calibration).',
    )
    parser.add_argument(
        '--wait-timeout',
        type=float,
        default=5.0,
        help='Seconds to wait for service availability (default: 5.0).',
    )
    parser.add_argument(
        '--response-timeout',
        type=float,
        default=15.0,
        help='Seconds to wait for service response (default: 15.0).',
    )
    parser.add_argument(
        '--serial-port',
        default='',
        help='Serial device override. If empty, auto-detect from /controller_manager robot_description.',
    )
    parser.add_argument(
        '--arm-id',
        default='so_arm',
        help='Arm identifier used for calibration record file naming (default: so_arm).',
    )
    parser.add_argument(
        '--calibration-dir',
        default='~/.config/so_arm_control/calibration',
        help='Directory where arm-specific calibration record JSON is written.',
    )
    parser.add_argument(
        '--expected-motor-id',
        action='append',
        default=[],
        help=(
            'Expected motor IDs for sanity checks. May be repeated or comma-separated. '
            'If omitted, IDs are auto-discovered from /controller_manager robot_description.'
        ),
    )
    parser.add_argument(
        '--controller-manager-node',
        default='/controller_manager',
        help='Controller manager node used for robot_description discovery (default: /controller_manager).',
    )
    parser.add_argument(
        '--skip-emergency-stop',
        action='store_true',
        help=(
            'Do not toggle /emergency_stop during calibration. By default this utility '
            'engages emergency stop before calibration for safe manual repositioning.'
        ),
    )
    parser.add_argument(
        '--skip-sweep-step',
        action='store_true',
        help='Skip the post-calibration full-range sweep guidance step.',
    )
    parser.add_argument(
        '--yes',
        action='store_true',
        help='Run without interactive ENTER prompts.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help=(
            'Detection-only mode. Discover USB port and motor IDs, then exit without '
            'calling /one_key_calibration or /emergency_stop.'
        ),
    )

    args = parser.parse_args()

    try:
        motor_ids = _parse_motor_ids(args.motor_id)
        expected_motor_ids = _parse_motor_ids(args.expected_motor_id)
    except ValueError as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 2

    rclpy.init()
    node = OneKeyCalibrationClient(args.service)

    try:
        detected_serial_port, detected_motor_ids, detected_mode0_motor_ids = node.discover_from_controller_manager(
            args.controller_manager_node, args.wait_timeout
        )

        if detected_motor_ids:
            node.get_logger().info('Detected motor IDs from %s: %s' % (args.controller_manager_node, detected_motor_ids))
        else:
            node.get_logger().warn('Could not detect motor IDs from %s robot_description.' % (args.controller_manager_node,))

        if detected_mode0_motor_ids:
            node.get_logger().info('Detected mode-0 motor IDs: %s' % (detected_mode0_motor_ids,))

        serial_port = args.serial_port.strip() if args.serial_port else ''
        if not serial_port and detected_serial_port:
            serial_port = detected_serial_port
            node.get_logger().info('Detected serial port from %s: %s' % (args.controller_manager_node, serial_port))

        if not serial_port:
            candidates = _scan_serial_ports()
            if len(candidates) == 1:
                serial_port = candidates[0]
                node.get_logger().warn('Could not detect serial port from robot_description, using single /dev candidate: %s' % (serial_port,))
            elif len(candidates) > 1:
                node.get_logger().error('Serial port auto-detection ambiguous. Candidates: %s. Use --serial-port.' % (candidates,))
                return 11
            else:
                node.get_logger().error(
                    'No serial port detected from robot_description and no /dev ttyUSB/ttyACM candidate found.'
                )
                return 12

        if not expected_motor_ids and detected_motor_ids:
            expected_motor_ids = detected_motor_ids

        node.get_logger().info('Using serial port: %s' % (serial_port,))
        if expected_motor_ids:
            node.get_logger().info('Using expected motor IDs: %s' % (expected_motor_ids,))

        if args.dry_run:
            node.get_logger().info('Dry run complete. Detected serial_port=%s, motor_ids=%s, mode0_motor_ids=%s' % (serial_port, expected_motor_ids, detected_mode0_motor_ids))
            return 0

        _check_serial_port(serial_port)

        if expected_motor_ids and motor_ids:
            unknown = [motor_id for motor_id in motor_ids if motor_id not in expected_motor_ids]
            if unknown:
                node.get_logger().error('Requested motor IDs not in expected set. requested=%s expected=%s unknown=%s' % (motor_ids, expected_motor_ids, unknown))
                return 10

        if not node.wait_for_service(args.wait_timeout):
            node.get_logger().error("Service '%s' not available within %.1f seconds." % (args.service, args.wait_timeout))
            return 3

        if not args.skip_emergency_stop:
            node.get_logger().info('Engaging /emergency_stop before calibration for safe manual movement...')
            estop_res = node.set_emergency_stop(True, args.wait_timeout)
            if not estop_res.success:
                node.get_logger().error('Failed to engage emergency stop: %s' % (estop_res.message,))
                return 7
            node.get_logger().info('Emergency stop active: %s' % (estop_res.message,))

        if not args.yes:
            _wait_for_enter(
                'Step 1/3: Move the arm so each target joint is near the middle of its physical range'
            )

        node.get_logger().info('Step 2/3: Sending one-key midpoint calibration request for %s' % ('all mode-0 joints' if not motor_ids else f'motor_ids={motor_ids}',))

        response = node.call(motor_ids, args.response_timeout)
        if response.accepted:
            node.get_logger().info('Calibration accepted: %s' % (response.message,))
        else:
            node.get_logger().error('Calibration rejected: %s' % (response.message,))
            return 4

        if not args.skip_sweep_step:
            if args.yes:
                node.get_logger().info('Step 3/3: Manually sweep each calibrated joint through full range (non-interactive mode).')
            else:
                _wait_for_enter(
                    'Step 3/3: Manually sweep each calibrated joint through full range of motion, one joint at a time'
                )

        if not args.skip_emergency_stop:
            if not args.yes:
                _wait_for_enter('Release emergency stop when you are ready to resume commanded control')
            node.get_logger().info('Releasing /emergency_stop...')
            estop_res = node.set_emergency_stop(False, args.wait_timeout)
            if not estop_res.success:
                node.get_logger().error('Failed to release emergency stop: %s' % (estop_res.message,))
                return 8
            node.get_logger().info('Emergency stop released: %s' % (estop_res.message,))

        calibrated_ids = motor_ids if motor_ids else expected_motor_ids
        record_path = _save_calibration_record(
            arm_id=args.arm_id,
            calibration_dir=args.calibration_dir,
            serial_port=serial_port,
            service_name=args.service,
            motor_ids=calibrated_ids,
            accepted=response.accepted,
            message=response.message,
        )
        node.get_logger().info('Wrote calibration record: %s' % (str(record_path),))

        node.get_logger().info('Guided calibration workflow finished.')
        return 0

    except TimeoutError as exc:
        node.get_logger().error(str(exc))
        return 5
    except Exception as exc:  # pragma: no cover - defensive CLI guard
        node.get_logger().error('Calibration call failed: %s' % (str(exc),))
        return 6
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
