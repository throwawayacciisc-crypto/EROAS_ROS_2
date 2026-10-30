#!/usr/bin/env python3
"""Keyboard teleop for RexROV2 (all body velocities) and the sonar pivot.

Publishes /rexrov2/cmd_vel continuously at 10 Hz (Gazebo's VelocityControl
latches the last command) and /rexrov2/sonar_joint_position_controller/command.
"""
import math, select, sys, termios, tty
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

HELP = """
RexROV2 teleop  (each key press changes the speed by one step)
  w / s : forward / backward      (surge x, 0.1 m/s steps)
  a / d : turn left / right       (yaw,     0.05 rad/s steps)
  q / e : strafe left / right     (sway y,  0.1 m/s steps)
  r / f : up / down               (heave z, 0.1 m/s steps)
  space : stop all motion
  i / k : sonar tilt up / down    (5 deg steps, limit +-80 deg)
  o     : sonar back to 0 deg
  x     : stop and quit
"""

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def main():
    rclpy.init()
    node = rclpy.create_node('rexrov2_teleop')
    cmd_pub = node.create_publisher(Twist, '/rexrov2/cmd_vel', 10)
    sonar_pub = node.create_publisher(Float64, '/rexrov2/sonar_joint_position_controller/command', 10)
    v = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    tilt = 0.0
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                k = sys.stdin.read(1)
                if k == 'w': v['x'] += 0.1
                elif k == 's': v['x'] -= 0.1
                elif k == 'q': v['y'] += 0.1
                elif k == 'e': v['y'] -= 0.1
                elif k == 'r': v['z'] += 0.1
                elif k == 'f': v['z'] -= 0.1
                elif k == 'a': v['yaw'] += 0.05
                elif k == 'd': v['yaw'] -= 0.05
                elif k == ' ': v = dict.fromkeys(v, 0.0)
                elif k == 'i': tilt = clamp(tilt + 5, -80, 80)
                elif k == 'k': tilt = clamp(tilt - 5, -80, 80)
                elif k == 'o': tilt = 0.0
                elif k == 'x': break
                for key in ('x', 'y', 'z'):
                    v[key] = clamp(round(v[key], 2), -1.0, 1.0)
                v['yaw'] = clamp(round(v['yaw'], 2), -0.5, 0.5)
                print(f"\rx {v['x']:+.1f}  y {v['y']:+.1f}  z {v['z']:+.1f} m/s  "
                      f"yaw {v['yaw']:+.2f} rad/s  sonar tilt {tilt:+.0f} deg   ", end='', flush=True)
            t = Twist()
            t.linear.x, t.linear.y, t.linear.z, t.angular.z = v['x'], v['y'], v['z'], v['yaw']
            cmd_pub.publish(t)
            sonar_pub.publish(Float64(data=math.radians(tilt)))
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        cmd_pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\nstopped')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
