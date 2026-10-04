#!/usr/bin/env python3
"""Publish one pilot intent command for fast_livo2_assist_planner."""

import sys
import time

import rospy
from std_msgs.msg import String


VALID = {
    "FREE",
    "FORWARD_SCAN",
    "LEFT_ORBIT",
    "RIGHT_ORBIT",
    "UP_SCAN",
    "DOWN_SCAN",
    "PULL_BACK",
    "HOLD_STABLE",
    "RESCAN_LOCAL",
}


def main():
    rospy.init_node("fast_livo2_assist_publish_intent", anonymous=True)
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print("usage: publish_intent.py INTENT [duration_s] [strength]")
        print("valid intents: {}".format(", ".join(sorted(VALID))))
        sys.exit(0)

    intent = sys.argv[1].strip().upper()
    if intent not in VALID:
        print("unknown intent: {}".format(intent))
        print("valid intents: {}".format(", ".join(sorted(VALID))))
        sys.exit(2)

    duration = float(sys.argv[2]) if len(sys.argv) >= 3 else 8.0
    strength = float(sys.argv[3]) if len(sys.argv) >= 4 else 1.0
    strength = max(0.0, min(1.0, strength))

    pub = rospy.Publisher("/fast_livo2_assist/pilot_intent", String, queue_size=1, latch=False)
    deadline = time.time() + 1.0
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)

    msg = String()
    msg.data = "intent={};strength={:.3f};duration_s={:.3f}".format(intent, strength, duration)
    pub.publish(msg)
    rospy.loginfo("[assist_intent] %s", msg.data)
    rospy.sleep(0.2)


if __name__ == "__main__":
    main()
