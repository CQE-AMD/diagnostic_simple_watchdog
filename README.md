# diagnostic_simple_watchdog

This package provides a simple way to add [diagnostics](https://www.ros.org/reps/rep-0107.html) to a ROS node, by adding a watchdog to its topics.

Currently, this package only supports ROS1 (noetic).

## Why?

Diagnostics is a great idea - but unfortunately, we don't live in a perfect and ideal world, and not all ROS packages are implementing it.

This is where this package comes into play - just by providing the necessary parameters, it will make a watchdog over a topic, and publish to `/diagnostics` the healthiness.

## How to use

### [topic_rate_watchdog.py](./scripts/topic_rate_watchdog.py)

With the following launch file, you will make a watchdog that checks if `odom` is publishing at 50Hz, and publish to `/diagnostics` accordingly.

```xml
<launch>
    <node name="topic_rate_watchdog" pkg="diagnostic_simple_watchdog" type="topic_rate_watchdog.py">
        <param name="topics" type="yaml" value="[{topic: odom, rate: 50}]" />
    </node>
</launch>
```

For more advanced usage, see examples in [./launch](./launch) and [./params](./params).

#### Topics

##### Subscribed Topics

- `<the provided topics from the parameter server>` (Any)
  - The provided topics to be checked for.

##### Published Topics

- `/diagnostics` ([diagnostic_msgs/DiagnosticArray](https://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html))
  - The status (healthiness) of the topic

#### Parameters

- `~<name>/topics` (array)
  - An array that contains the topics and its config to add a watchdog.
- `~<name>/topics[n]/topic` (str)
  - The topic name to add a watchdog.
- `~<name>/topics[n]/rate` (float)
  - The rate that topic should be published, in ideal conditions.
- `~<name>/topics[n]/diag_name` (str, default: "topic*watchdog*{topic name}")
  - (optional)The string used in diagnostic_msgs/DiagnosticStatus::name.
- `~<name>/topics[n]/hardware_id` (str, default: "none")
  - (optional)The string used in diagnostic_msgs/DiagnosticStatus::hardware_id.
- `~<name>/topics[n]/warn_rate_ratio` (float, default: 0.1)
  - (optional)Rates that exceeds (rate \* (1 +-warn_rate_ratio)) will be considered `WARN`.
- `~<name>/topics[n]/error_rate_ratio` (float, default: 0.2)
  - (optional)Rates that exceeds (rate \* (1 +-error_rate_ratio)) will be considered `ERROR`.
- `~<name>/topics[n]/sd_warn` (float, default: null)
  - (optional)The standard deviation (in secs) to be considered `WARN` if exceeding.
  - Will not check if not provided.
- `~<name>/topics[n]/sd_warn` (float, default: null)
  - (optional)The standard deviation (in secs) to be considered `ERROR` if exceeding.
  - Will not check if not provided.
- `~<name>/topics[n]/filter_expr` (str, default: null)
  - (optional)Filters the topic with a python eval, where `m` is the topic instance and `topic` is the topic name.
  - See `rostopic echo --help` for details (it is the same syntax).
- `~<name>/topics[n]/window_size` (float, default: (rate \* 10 / publish_rate))
  - (optional)The buffer size used for calculation.
  - The default value is fine for most use cases, but you can change this to catch short anomalies.
  - Be careful! Changing this can lead to detecting false positives!
- `~<name>/publish_rate` (float, default: 1)
  - The rate at which the `/diagnostics` message will be published.
  - Change to a smaller value if the topic monitored is slower than 5Hz.

## TODO

- Use Github actions for tests
- More documentation
