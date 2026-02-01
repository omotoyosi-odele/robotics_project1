#!/usr/bin/env python

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist

# Define key mappings for Option 1
MOVES = {
    'w': (2.0, 0.0),   # Forward
    's': (-2.0, 0.0),  # Backward
    'a': (0.0, 2.0),   # Rotate Left
    'd': (0.0, -2.0),  # Rotate Right
    'q': (2.0, 2.0),   # Forward Left
    'e': (2.0, -2.0),  # Forward Right
    'z': (-2.0, -2.0), # Backward Left
    'c': (-2.0, 2.0),  # Backward Right
}

def getKey(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    #rospy.init_node(turtle_name + '_controller')

    # Process Command Line Arguments
    if len(sys.argv) < 2:
        turtle_name = "turtle1"
        # FIXED: Use %s formatting instead of f-string
        rospy.logwarn("No turtle name provided. Defaulting to %s", turtle_name)
    else:
        turtle_name = sys.argv[1]

    rospy.init_node(turtle_name + '_controller')

    # Create Publisher
    # FIXED: Use + for string concatenation
    topic_name = '/' + turtle_name + '/cmd_vel'
    pub = rospy.Publisher(topic_name, Twist, queue_size=10)

    # FIXED: Use .format() or standard print
    print("Control " + turtle_name + " with the keyboard!")
    print("Use: w, a, s, d (and q, e, z, c for curves)")
    print("Release keys to STOP.")
    print("Press Ctrl+C to exit.")

    twist = Twist()

    try:
        while not rospy.is_shutdown():
            key = getKey(settings, timeout=0.1)

            if key in MOVES:
                linear_x, angular_z = MOVES[key]
                twist.linear.x = linear_x
                twist.angular.z = angular_z
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if key == '\x03': # Ctrl+C
                    break

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    main()
