from unittest import result
import rclpy

from unitree_msgs.srv import SetControlMode

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
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
"""

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.readline(1)[0]
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def request_to_str(mode):
	return "request mode:\t %s " % (mode)

def response_to_str(mode):
	return "request mode:\t %s " % (mode)

# def vels(speed,turn):
# 	return "currently:\tspeed %s\tturn %s " % (speed,turn)


def main(args=None):	
	rclpy.init(args=args)
	node = rclpy.create_node('unitree_set_control_mode')
	cli = node.create_client(SetControlMode, 'set_control_mode')

	request = SetControlMode.Request()
	request.control_mode = 0
	status = 0

	print(msg)
	while(1):
		key = getKey()
		if key in ['0', '1', '2', '3', '4']:
			request.control_mode = int(key)
			print(request_to_str(request.control_mode))
			if (status == 14):
				print(msg)
			status = (status + 1) % 15

			future = cli.call_async(request)
			rclpy.spin_until_future_complete(node, future)
			result = future.result()
			node.get_logger().info('SetControlMode.success: ' % (result.success))
			node.get_logger().info('SetControlMode.current_control_mode: ' % (result.current_control_mode))

		else:
			print('Choose the ControlMode from [0, 1, 2, 3, 4]')

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()