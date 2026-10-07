"""The bridge's stop operation must clear navigation and keep a hold setpoint."""

from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
from std_srvs.srv import Trigger

from waypoint_planner.waypoint_planner_node import FlightState, WaypointPlannerNode


@pytest.mark.parametrize('service', ['srv_hold', 'srv_abort'])
@pytest.mark.parametrize('state', [FlightState.TRACKING, FlightState.TAKEOFF])
def test_stop_clears_route_and_preserves_actual_altitude(state, service):
    node = SimpleNamespace(
        state=state, current_path=[1, 2], current_path_idx=1,
        current_gps=SimpleNamespace(latitude=40.0, longitude=-75.0),
        current_altitude_rel=12.0, fixed_altitude=True, takeoff_altitude=20.0,
        publish_path=Mock(), publish_gps_path=Mock(), set_state=Mock(),
    )
    node.srv_hold = MethodType(WaypointPlannerNode.srv_hold, node)
    response = getattr(WaypointPlannerNode, service)(node, Trigger.Request(), Trigger.Response())
    assert response.success
    assert node.current_path is None and node.current_path_idx == 0
    assert node.current_setpoint == dict(latitude=40.0, longitude=-75.0, altitude=12.0)
    node.set_state.assert_called_once_with(FlightState.TRACKING)
    node.publish_path.assert_called_once_with([])
    node.publish_gps_path.assert_called_once_with([])


@pytest.mark.parametrize('service', ['srv_hold', 'srv_abort'])
def test_stop_without_position_is_rejected(service):
    node = SimpleNamespace(state=FlightState.TRACKING, current_gps=None,
                           current_path=[1], current_altitude_rel=None)
    node.srv_hold = MethodType(WaypointPlannerNode.srv_hold, node)
    response = getattr(WaypointPlannerNode, service)(node, Trigger.Request(), Trigger.Response())
    assert not response.success
    assert node.current_path == [1]
