#!/usr/bin/env python

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist

# Define key mappings
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
    rospy.init_node(turtle_name + '_controller')

    # Define the list of turtles you plan to spawn manually
    turtle_names = ["turtle1", "turtle2"]
    publishers = {}

    # Initialize publishers for all turtles
    for name in turtle_names:
        topic = '/' + name + '/cmd_vel'
        publishers[name] = rospy.Publisher(topic, Twist, queue_size=10)

    # Track which turtle is currently active
    current_index = 0
    current_turtle = turtle_names[current_index]

    print("\n" + "="*40)
    print("      MULTI-TURTLE CONTROLLER")
    print("="*40)
    print("Move:   W, A, S, D")
    print("Toggle: TAB   (Switch between turtles)")
    print("Halt:   SPACE (Stop ALL turtles)")
    print("Exit:   Ctrl + C")
    print("-" * 40)
    print("Currently controlling: --> " + current_turtle)

    twist = Twist()

    try:
        while not rospy.is_shutdown():
            key = getKey(settings, timeout=0.1)

            # --- STOP ALL TURTLES (SPACE KEY) ---
            if key == ' ':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # Iterate through all publishers and send stop command
                print("\n!!! EMERGENCY STOP - HALTING ALL TURTLES !!!")
                for pub in publishers.values():
                    pub.publish(twist)
                print("Currently controlling: --> " + current_turtle)
                continue # Skip the rest of the loop
            # ------------------------------------

            # --- TOGGLE TURTLE (TAB KEY) ---
            if key == '\t':
                # Stop current turtle before switching
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publishers[current_turtle].publish(twist)

                # Switch index
                current_index = (current_index + 1) % len(turtle_names)
                current_turtle = turtle_names[current_index]

                print("Switched control to: --> " + current_turtle)
                continue 
            # -------------------------------

            # --- MOVEMENT LOGIC ---
            if key in MOVES:
                linear_x, angular_z = MOVES[key]
                twist.linear.x = linear_x
                twist.angular.z = angular_z
            else:
                # If key is released or unknown, stop the CURRENT turtle
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if key == '\x03': # Ctrl+C
                    break

            # Publish ONLY to the current turtle
            publishers[current_turtle].publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Final cleanup: stop whatever turtle was active
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        publishers[current_turtle].publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    main()
