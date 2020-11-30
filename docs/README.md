ezrassor_topic_switch
---------------------
![Build Badge](https://github.com/FlaSpaceInst/ezrassor_topic_switch/workflows/Build/badge.svg) ![Style Badge](https://img.shields.io/badge/Code%20Style-black-000000.svg)

The `ezrassor_topic_switch` routes messages from one of many input topics to an output topic. The active input topic is determined by an auxiliary channel topic.

This functionality is primarily useful during operation of the EZRASSOR in "dual mode," when both autonomous control and manual control are enabled. In dual mode, a user can manually request an autonomous routine by transferring control of the robot to the autonomous controller (using topic switches under the hood). The autonomous controller executes the routine and then transfers control back to the user after the routine is finished.

Topic switches are an optional part of the system and are only employed in dual mode.

topics
------
```
topic_switch <- /input0
topic_switch <- /input1
...
topic_switch <- /inputN
topic_switch <- /channel

topic_switch -> /output
```

usage
-----
```
command:
  ros2 launch ezrassor_topic_switch topic_switch.py [argument:=value]

optional arguments:
  message_type   topic message type
  channels       number of desired channels
```

examples
--------
Launch a topic switch with the default message type and channels:
```
ros2 launch ezrassor_topic_switch topic_switch.py
```

Launch a topic switch for `Twist` messages:
```
ros2 launch ezrassor_topic_switch topic_switch.py \
  message_type:=geometry_msgs.msg.Twist
```

Launch a topic switch with a dozen channels:
```
ros2 launch ezrassor_topic_switch topic_switch.py \
  channels:=12
```
