#!/usr/bin/env python3
from __future__ import annotations

import dataclasses
import math
import typing

import diagnostic_msgs.msg as diag_msg
import rospy
import rostopic


class Tolerance:
    def __init__(self, rate: float, tolerance=0.1) -> None:
        self.rate = rate
        if not (math.isfinite(rate) and rate > 0):
            raise ValueError(
                f"rate must be a finite number larger than 0, but was {rate}"
            )
        self.tolerance = tolerance
        if not (0 < tolerance < 1):
            raise ValueError(
                f"tolerance_ratio must be between 0 and 1, but was {tolerance}"
            )
        self.max_rate = rate * (1 + self.tolerance)
        self.min_rate = rate * (1 - self.tolerance)


@dataclasses.dataclass
class TopicParam:
    topic: str
    rate: float
    warn_rate_ratio: float = 0.1
    error_rate_ratio: float = 0.2
    warn_sd: typing.Optional[float] = None
    error_sd: typing.Optional[float] = None
    filter_expr: typing.Optional[str] = None
    window_size: typing.Optional[int] = None
    diag_name: typing.Optional[str] = None
    hardware_id: str = "none"

    @classmethod
    def get_param_list(cls) -> typing.List[TopicParam]:
        res_buf: typing.List[TopicParam] = []
        param_list = rospy.get_param("~topics")
        if not isinstance(param_list, list):
            raise TypeError(
                f'param "{rospy.get_name()}/topics" must be a list,'
                f" but was {type(param_list)}"
            )
        if len(param_list) == 0:
            raise ValueError(f'param "{rospy.get_name()}/topics" was empty')
        for i, param in enumerate(param_list):
            if not isinstance(param, dict):
                raise TypeError(
                    f'the items in param "{rospy.get_name()}/topics" must be dicts,'
                    f" but item #{i} was {type(param)}"
                )
            buf = {}
            for field in dataclasses.fields(cls):
                key = field.name
                type_ = field.type
                default = field.default
                try:
                    val = param[key]
                except KeyError:
                    if not default == dataclasses.MISSING:
                        rospy.loginfo(
                            "failed to get param "
                            f"{rospy.get_name()}/topics[{i}]/{key},"
                            f" using default value {default}"
                        )
                        buf[key] = default
                        continue
                    raise KeyError(
                        f"failed to get param {rospy.get_name()}/topics[{i}]/{key}"
                    )
                try:
                    buf[key] = val
                except ValueError:
                    raise TypeError(
                        f"param {rospy.get_name()}/topics[{i}]/{key} should be "
                        f"a {type_}, but was {type(val)}"
                    )
            res_buf.append(cls(**buf))
        return res_buf


class TopicRateWatchdog:
    def __init__(self, param: TopicParam, publish_rate: int) -> None:
        self.param = param
        self.warn_tolerance = Tolerance(param.rate, param.warn_rate_ratio)
        self.error_tolerance = Tolerance(param.rate, param.error_rate_ratio)
        if self.param.diag_name is None:
            self.param.diag_name = "topic_watchdog_" + self.param.topic
        if self.param.window_size is None:
            self.param.window_size = int(self.param.rate * 10 / publish_rate)

        if self.param.window_size > 50000:
            rospy.logwarn(
                f"the window size for topic {self.param.topic} is too large "
                f"({self.param.window_size}), check the settings"
            )

        self._topic_handler = rostopic.ROSTopicHz(
            self.param.window_size, self.param.filter_expr
        )

        self._msg_type, self._real_topic, __ = rostopic.get_topic_class(
            self.param.topic, True
        )
        self.subscriber = rospy.Subscriber(
            self._real_topic,
            rospy.AnyMsg if self.param.filter_expr is None else self._msg_type,
            self._topic_handler.callback_hz,
        )
        rospy.loginfo("subscribed to topic %s", self._real_topic)

    def analyze(self) -> diag_msg.DiagnosticStatus:
        res_buf = diag_msg.DiagnosticStatus()
        res_buf.name = self.param.diag_name
        res_buf.hardware_id = self.param.hardware_id
        if (stat := self._topic_handler.get_hz()) is None:
            res_buf.level = res_buf.ERROR
            res_buf.message = "No new message received"
            return res_buf

        rate, min_rate, max_rate, stdev, __ = stat

        if res_buf.values is None:
            res_buf.values = []
        res_buf.values.append(diag_msg.KeyValue("rate", f"{rate:.4}"))
        res_buf.values.append(diag_msg.KeyValue("min rate", f"{min_rate:.4}"))
        res_buf.values.append(diag_msg.KeyValue("max rate", f"{max_rate:.4}"))
        res_buf.values.append(diag_msg.KeyValue("standard deviation", f"{stdev:.4}"))

        error_msg: typing.List[str] = []
        if rate < self.error_tolerance.min_rate:
            error_msg.append(
                f"rate < min_error_rate ({self.error_tolerance.min_rate:.4})"
            )
        if rate > self.error_tolerance.max_rate:
            error_msg.append(
                f"rate > max_error_rate ({self.error_tolerance.max_rate:.4})"
            )
        if self.param.error_sd is not None and self.param.error_sd < stdev:
            error_msg.append(f"standard deviation > error ({self.param.error_sd})")
        if error_msg:
            res_buf.level = res_buf.ERROR
            res_buf.message = "; ".join(error_msg)
            return res_buf

        warn_msg: typing.List[str] = []
        if rate < self.warn_tolerance.min_rate:
            warn_msg.append(f"rate < min_warn_rate ({self.warn_tolerance.min_rate:.4})")
        if rate > self.warn_tolerance.max_rate:
            warn_msg.append(f"rate > max_warn_rate ({self.warn_tolerance.max_rate:.4})")
        if self.param.warn_sd is not None and self.param.warn_sd < stdev:
            warn_msg.append(f"standard deviation > warn ({self.param.warn_sd})")
        if warn_msg:
            res_buf.level = res_buf.WARN
            res_buf.message = "; ".join(warn_msg)
            return res_buf

        res_buf.level = res_buf.OK
        res_buf.message = "OK"

        return res_buf


def main():
    rospy.init_node("topic_rate_watchdog")
    publish_rate = rospy.get_param("~publish_rate", 1)
    rate = rospy.Rate(publish_rate)
    params = TopicParam.get_param_list()
    watchdogs = [TopicRateWatchdog(param, publish_rate) for param in params]
    diag_publisher = rospy.Publisher(
        "/diagnostics", diag_msg.DiagnosticArray, queue_size=10
    )
    while not rospy.is_shutdown():
        msg = diag_msg.DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        msg.status = [wd.analyze() for wd in watchdogs]
        diag_publisher.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
