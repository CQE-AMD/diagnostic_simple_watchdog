import subprocess

import diagnostic_msgs.msg as diag_msg
import pytest
import rospy
import std_msgs.msg as std_msg

from diagnostic_simple_watchdog import topic_rate_watchdog_impl as wdt


@pytest.fixture
def node():
    rospy.init_node("test_node", anonymous=True)


def test_t(node: None):
    assert rospy.get_name() == "/test_node"
    params = wdt.TopicParam.get_param_list()
    # default values
    assert params[0].topic == "odom"
    assert params[0].rate == 50
    assert params[0].warn_rate_ratio == 0.1
    assert params[0].error_rate_ratio == 0.2
    assert params[0].warn_sd == None
    assert params[0].error_sd == None
    assert params[0].filter_expr == None
    assert params[0].window_size == None
    assert params[0].diag_name == None
    assert params[0].hardware_id == "none"

    # changed values
    assert params[1].topic == "laserscan"
    assert params[1].rate == 10
    assert params[1].warn_rate_ratio == 0.2
    assert params[1].error_rate_ratio == 0.3
    assert params[1].warn_sd == 0.2
    assert params[1].error_sd == 0.3
    assert params[1].filter_expr == "m.header.frame_id == 'lidar_1_frame'"
    assert params[1].window_size == 10
    assert params[1].diag_name == "dummy_diag"
    assert params[1].hardware_id == "lidar_1"


BASE_RATE = 50


@pytest.fixture(params=(i * BASE_RATE for i in (0.75, 0.85, 0.95, 1, 1.05, 1.15, 1.25)))
def rate(request: pytest.FixtureRequest):
    return request.param


def test_rate(node: None, rate: float):
    topic_name = f"dummy{int(rate*10)}"
    p = subprocess.Popen(f"rostopic pub -r {rate} {topic_name} std_msgs/Empty".split())
    rospy.sleep(1)
    wdt_param = wdt.TopicParam(topic=topic_name, rate=BASE_RATE)
    watchdog = wdt.TopicRateWatchdog(wdt_param, 1)
    rospy.sleep(1)
    res: diag_msg.DiagnosticStatus = watchdog.analyze()
    assert wdt_param.warn_rate_ratio == 0.1
    assert wdt_param.error_rate_ratio == 0.2
    if (
        BASE_RATE * (1 - wdt_param.warn_rate_ratio)
        < rate
        < BASE_RATE * (1 + wdt_param.warn_rate_ratio)
    ):
        assert res.level == res.OK
    elif (
        BASE_RATE * (1 - wdt_param.error_rate_ratio)
        < rate
        < BASE_RATE * (1 + wdt_param.error_rate_ratio)
    ):
        assert res.level == res.WARN
    else:
        assert res.level == res.ERROR
    p.kill()
    p.wait()
