
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

msg = """
Reading from the keyboard !
---------------------------
Moving around:
   w : forward
   a : turn left
   s : backward
   d : turn right

Diagonal / Lateral movement (Shift required):
   Q : forward + strafe left
   W : forward
   E : forward + strafe right
   A : strafe left
   D : strafe right
   Z : backward + strafe left
   S : backward
   C : backward + strafe right

Anything else : stop

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0,0),
        'a':(0,0,0,1),
        's':(-1,0,0,0),
        'd':(0,0,0,-1),
        'u':(0,0,1,0),
        'v':(0,0,-1,0)
}

# For holonomic mode (strafing and diagonal movement)
moveBindings_holonomic = {
        'W':(1,0,0,0),
        'A':(0,1,0,0),
        'S':(-1,0,0,0),
        'D':(0,-1,0,0),
        'Q':(1,1,0,0),
        'E':(1,-1,0,0),
        'Z':(-1,1,0,0),
        'C':(-1,-1,0,0),
        'U':(0,0,1,0),
        'V':(0,0,-1,0),
}


speed = 0.8
turn = 1.0

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "linear: %s\tangular: %s" % (speed,turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in moveBindings_holonomic.keys(): # Check for holonomic keys
                x = moveBindings_holonomic[key][0]
                y = moveBindings_holonomic[key][1]
                z = moveBindings_holonomic[key][2]
                th = moveBindings_holonomic[key][3]
            elif key == '': # No key pressed, stop movement
                x = 0
                y = 0
                z = 0
                th = 0
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed  # For holonomic movement
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
