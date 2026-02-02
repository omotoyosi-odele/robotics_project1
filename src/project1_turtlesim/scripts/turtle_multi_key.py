#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist
from pynput import keyboard

# Movement settings
LINEAR_SPEED = 2.0
ANGULAR_SPEED = 2.0

# Track the state of specific keys
# We use a set to keep track of which keys are currently held down
key_state = {
    'w': False, 's': False, 'a': False, 'd': False,
    'q': False, 'e': False, 'z': False, 'c': False 
}

def on_press(key):
    """Callback when a key is pressed."""
    try:
        if hasattr(key, 'char') and key.char in key_state:
            key_state[key.char] = True
    except AttributeError:
        pass

def on_release(key):
    """Callback when a key is released."""
    try:
        if hasattr(key, 'char') and key.char in key_state:
            key_state[key.char] = False
    except AttributeError:
        pass
    
    # Allow simple exit with ESC
    if key == keyboard.Key.esc:
        return False

def calculate_twist():
    """Calculate the Twist message based on current key states."""
    twist = Twist()

    # LOGIC: Summing Forces
    # If W is held, add speed. If S is held, subtract speed.
    # If both are held, (speed - speed) = 0.
    
    linear_val = 0.0
    angular_val = 0.0

    # FORWARD (w) OR Forward-Left (q) OR Forward-Right (e)
    if key_state['w'] or key_state['q'] or key_state['e']:
        linear_val += LINEAR_SPEED

    # BACKWARD (s) OR Backward-Left (z) OR Backward-Right (c)
    if key_state['s'] or key_state['z'] or key_state['c']:
        linear_val -= LINEAR_SPEED
        
    # LEFT (a) OR Forward-Left (q) OR Backward-Right (c)
    if key_state['a'] or key_state['q'] or key_state['c']:
        angular_val += ANGULAR_SPEED

    # RIGHT (d) OR Forward-Right (e) OR Backward-Left (z)
    if key_state['d'] or key_state['e'] or key_state['z']:
        angular_val -= ANGULAR_SPEED

    twist.linear.x = linear_val
    twist.angular.z = angular_val
    
    return twist

def main():
    # Process Command Line Arguments
    if len(sys.argv) < 2:
        turtle_name = "turtle1"
        rospy.logwarn("No turtle name provided. Defaulting to %s", turtle_name)
    else:
        turtle_name = sys.argv[1]

    rospy.init_node(turtle_name + '_controller')

    topic_name = '/' + turtle_name + '/cmd_vel'
    pub = rospy.Publisher(topic_name, Twist, queue_size=10)

    print("Control " + turtle_name + " with the keyboard!")
    print("Use: w, a, s, d (Multiple keys allowed) and q, e, z, c for curves")
    print("Example: Hold 'w' + 'a' to circle left.")
    print("Press ESC or Ctrl+C to exit.")

    # Start the non-blocking keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rate = rospy.Rate(10) # 10Hz control loop

    twist = Twist()

    try:
        while not rospy.is_shutdown() and listener.is_alive():
            # Get the twist based on currently held keys
            twist = calculate_twist()
            pub.publish(twist)
            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        # Stop the listener and stop the robot
        listener.stop()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        print("\nExiting...")

if __name__ == "__main__":
    main()
