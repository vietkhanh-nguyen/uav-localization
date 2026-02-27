import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import numpy as np

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Arrow Keys : Control Velocity (Vx, Vy)
. / ,      : Increase / Decrease Altitude (Z)
Space      : STOP (Reset all to 0)
Enter      : Hover (Vx=0, Vy=0, Keep Z)

Yaw Control:
Q: 45°   W: 0°    E: -45°
A: 90°            D: -90°
Z: 135°  X: 180°  C: -135°

CTRL-C to quit
"""

class DroneKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('drone_keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        
        self.step = 0.5
        self.angle_step = 0.1
        self.tol = 1e-2
        self.max_speed = 3.0
        
        self.vx_ref = 0.0
        self.vy_ref = 0.0
        self.z_ref = 0.0     
        self.yaw_ref = 0.0
        
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key = sys.stdin.read(2)
                if key == '[A': return 'UP'
                if key == '[B': return 'DOWN'
                if key == '[C': return 'RIGHT'
                if key == '[D': return 'LEFT'
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def approach_yaw(self, target):
        diff = self.normalize_angle(target - self.yaw_ref)
        if abs(diff) > self.tol:
            self.yaw_ref += np.clip(diff, -self.angle_step, self.angle_step)
            self.yaw_ref = self.normalize_angle(self.yaw_ref)

    def run(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                
                if key == 'UP':
                    self.vx_ref = np.clip(self.vx_ref + self.step, -self.max_speed, self.max_speed)
                elif key == 'DOWN':
                    self.vx_ref = np.clip(self.vx_ref - self.step, -self.max_speed, self.max_speed)
                elif key == 'LEFT': 
                    self.vy_ref = np.clip(self.vy_ref + self.step, -self.max_speed, self.max_speed)
                elif key == 'RIGHT':
                    self.vy_ref = np.clip(self.vy_ref - self.step, -self.max_speed, self.max_speed)

                elif key == '.':
                    self.z_ref = np.clip(self.z_ref + self.step, 0, 3.0)
                elif key == ',':
                    self.z_ref = np.clip(self.z_ref - self.step, 0, 3.0)

                elif key == ' ':
                    self.vx_ref = 0.0
                    self.vy_ref = 0.0
                    self.z_ref = 0.0
                
                elif key == '\r' or key == '\n':
                    self.vx_ref = 0.0
                    self.vy_ref = 0.0

                target_yaw = None
                if key == 'q': target_yaw = np.pi / 4
                elif key == 'w': target_yaw = 0.0
                elif key == 'e': target_yaw = -np.pi / 4
                elif key == 'a': target_yaw = np.pi / 2
                elif key == 'd': target_yaw = -np.pi / 2
                elif key == 'c': target_yaw = -3 * np.pi / 4
                elif key == 'x': target_yaw = np.pi
                elif key == 'z': target_yaw = 3 * np.pi / 4
                elif key == '\x03': 
                    break

                if target_yaw is not None:
                    self.approach_yaw(target_yaw)

                twist = Twist()
                twist.linear.x = float(self.vx_ref)
                twist.linear.y = float(self.vy_ref)
                twist.linear.z = float(self.z_ref) 
                twist.angular.z = float(self.yaw_ref) 

                self.publisher_.publish(twist)

                sys.stdout.write(f"\rVx: {self.vx_ref:.2f} | Vy: {self.vy_ref:.2f} | Alt: {self.z_ref:.2f} | Yaw: {self.yaw_ref:.2f}")
                sys.stdout.flush()

        except Exception as e:
            print(e)

        finally:
            twist = Twist() 
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = DroneKeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()