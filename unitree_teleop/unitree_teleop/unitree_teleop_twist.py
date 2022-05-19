from time import sleep
import rclpy
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import Twist

import sys, select, termios, tty


SETTINGS = termios.tcgetattr(sys.stdin)

CONSOLE_MSG = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

SPEED_BINDINGS = {
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def main(args=None):	
    rclpy.init(args=args)
    node = rclpy.create_node('unitree_teleop_twist')
    pub = node.create_publisher(Twist, 'key_vel', qos_profile_system_default)

    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    period = 0.1

    try:
        print(CONSOLE_MSG)
        print(vels(speed, turn))
        while rclpy.ok():
            key = getKey()
            if key in MOVE_BINDINGS.keys():
                x = MOVE_BINDINGS[key][0]
                y = MOVE_BINDINGS[key][1]
                z = MOVE_BINDINGS[key][2]
                th = MOVE_BINDINGS[key][3]
            elif key in SPEED_BINDINGS.keys():
                speed = speed * SPEED_BINDINGS[key][0]
                turn = turn * SPEED_BINDINGS[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(CONSOLE_MSG)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
            pub.publish(twist)
            sleep(period)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()