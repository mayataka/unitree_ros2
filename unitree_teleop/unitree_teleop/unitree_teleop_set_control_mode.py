import rclpy

from unitree_msgs.srv import SetControlMode

import sys, select, termios, tty

SETTINGS = termios.tcgetattr(sys.stdin)

CONSOLE_MSG = """
Set the Control Mode from Keyboard!
---------------------------
0: Zero Torque Mode
1: Standing Up Mode
2: Idling Mode
3: Contro Mode
4: Sitting Down Mode

---------------------------
Only the following mode tansitions are allowed. Otherwise, the request is denied.
0 -> 1
1 -> 2
2 -> 3
3 -> 2 or 4
4 -> 0

CTRL-C to quit
---------------------------
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
    return key

def request_to_str(mode):
    return "request mode:\t %s " % str(mode)

def response_to_str(mode):
    return "request mode:\t %s " % str(mode)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('unitree_set_control_mode')
    cli = node.create_client(SetControlMode, 'unitree_controller/set_control_mode')

    request = SetControlMode.Request()
    request.control_mode = 0
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service SetControlMode not available, waiting again...')

    print(CONSOLE_MSG)

    while rclpy.ok():
        key = getKey()
        if key in ['0', '1', '2', '3', '4']:
            request.control_mode = int(key)
            print(request_to_str(request.control_mode))

            future = cli.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            print(CONSOLE_MSG)
            node.get_logger().info('SetControlMode.accept: %s' % str(response.accept))
            node.get_logger().info('SetControlMode.current_control_mode: %s' % str(response.current_control_mode))

        elif key == '\x03':
            shutdown = True
            break

        else:
            print('Choose the ControlMode from [0, 1, 2, 3, 4]: The input key was ' + key[0])

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()