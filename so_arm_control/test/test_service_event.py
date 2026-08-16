#!/usr/bin/env python3
"""Unit tests for the shared SetBool service-event correlation helper.

This logic used to be copy-pasted into record_replay_node, teleop_ik_node, bool_toggle_node,
and teleop_gripper_node; it now lives once in so_arm_utils.service_event and is tested once
here instead of per-node.
"""

from service_msgs.msg import ServiceEventInfo
from so_arm_control.so_arm_utils.service_event import correlate_service_event
from std_srvs.srv import SetBool, SetBool_Event


def _event(event_type, *, client_gid=b'\x01' * 16, seq=1, data=None, success=None):
    msg = SetBool_Event()
    msg.info.event_type = event_type
    msg.info.client_gid = list(client_gid)
    msg.info.sequence_number = seq
    if data is not None:
        req = SetBool.Request()
        req.data = data
        msg.request = [req]
    if success is not None:
        resp = SetBool.Response()
        resp.success = success
        msg.response = [resp]
    return msg


class TestCorrelateServiceEvent:

    def test_full_cycle_resolves_to_request_value(self):
        pending = {}
        assert correlate_service_event(
            _event(ServiceEventInfo.REQUEST_RECEIVED, data=True), pending,
        ) is None
        assert correlate_service_event(
            _event(ServiceEventInfo.RESPONSE_SENT, success=True), pending,
        ) is True

    def test_failed_response_resolves_to_none(self):
        pending = {}
        correlate_service_event(_event(ServiceEventInfo.REQUEST_RECEIVED, data=True), pending)
        assert correlate_service_event(
            _event(ServiceEventInfo.RESPONSE_SENT, success=False), pending,
        ) is None

    def test_response_without_matching_request_is_ignored(self):
        assert correlate_service_event(
            _event(ServiceEventInfo.RESPONSE_SENT, success=True), {},
        ) is None

    def test_distinct_callers_do_not_collide(self):
        """Two concurrent (client_gid, seq) pairs in the same pending dict resolve independently."""
        pending = {}
        correlate_service_event(
            _event(ServiceEventInfo.REQUEST_RECEIVED, client_gid=b'\x01' * 16, seq=1, data=True),
            pending,
        )
        correlate_service_event(
            _event(ServiceEventInfo.REQUEST_RECEIVED, client_gid=b'\x02' * 16, seq=1, data=False),
            pending,
        )
        first = correlate_service_event(
            _event(ServiceEventInfo.RESPONSE_SENT, client_gid=b'\x01' * 16, seq=1, success=True),
            pending,
        )
        second = correlate_service_event(
            _event(ServiceEventInfo.RESPONSE_SENT, client_gid=b'\x02' * 16, seq=1, success=True),
            pending,
        )
        assert (first, second) == (True, False)

    def test_pending_dict_is_capped(self):
        pending = {}
        for seq in range(20):
            correlate_service_event(
                _event(ServiceEventInfo.REQUEST_RECEIVED, seq=seq, data=True), pending,
            )
        assert len(pending) <= 16
