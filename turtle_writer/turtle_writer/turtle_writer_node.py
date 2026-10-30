#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

LETTERS = {
    'L': [[(0, 1), (0, 0), (1, 0)]],
    'R': [[(0, 0), (0, 1), (1, 1), (1, 0.5), (0, 0.5)], [(0.4, 0.5), (1, 0)]],
    'T': [[(0, 1), (1, 1)], [(0.5, 1), (0.5, 0)]],
    'M': [[(0, 0), (0, 1), (0.5, 0.35), (1, 1), (1, 0)]],
    'D': [[(0, 0), (0, 1), (0.65, 1), (1, 0.7), (1, 0.3), (0.65, 0), (0, 0)]],
}

LETTER_W, LETTER_H, GAP = 2.0, 3.0, 0.6
ROW_Y = {'top': 7.3, 'bottom': 2.5}

MAX_V, MIN_V = 2.0, 0.10
MAX_W, MIN_W = 3.0, 0.05
TOLERANCE_ANGLE, TOLERANCE_POS = 0.002, 0.003
K_ALIGN, K_HEADING, K_CROSS = 5.0, 6.0, 4.0

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def generate_waypoints(text, row):
    width = len(text) * LETTER_W + (len(text) - 1) * GAP
    start_x = (11.0 - width) / 2.0 + 0.04
    start_y = ROW_Y[row]
    
    waypoints = []
    for i, char in enumerate(text.upper()):
        offset_x = start_x + i * (LETTER_W + GAP)
        for stroke in LETTERS[char]:
            for j, (u, v) in enumerate(stroke):
                waypoints.append((offset_x + u * LETTER_W, start_y + v * LETTER_H, j > 0))
    return waypoints

class TurtleWriter(Node):
    def __init__(self):
        super().__init__('turtle_writer')
        self.declare_parameter('initials', 'MD')
        initials = self.get_parameter('initials').value

        self.waypoints = generate_waypoints('LRT', 'top') + generate_waypoints(initials, 'bottom')
        
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        self.current_wp_idx = 0
        self.state = 'INIT_SEGMENT'
        self.is_pen_down = None
        
        self.start_x = self.start_y = 0.0
        self.target_dist = 0.0
        self.unit_x = self.unit_y = 0.0
        self.target_angle = 0.0

    def pose_cb(self, pose_msg):
        if self.state == 'DONE':
            return
        if self.state == 'WAIT_FOR_PEN':
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()

        if self.state == 'INIT_SEGMENT':
            self.init_next_segment(pose_msg)
            return

        elif self.state == 'ALIGN':
            angle_error = normalize_angle(self.target_angle - pose_msg.theta)
            if abs(angle_error) > TOLERANCE_ANGLE:
                speed = K_ALIGN * angle_error
                cmd.angular.z = math.copysign(max(MIN_W, min(MAX_W, abs(speed))), speed)
            else:
                self.state = 'DRIVE'

        elif self.state == 'DRIVE':
            dx, dy = pose_msg.x - self.start_x, pose_msg.y - self.start_y
            
            driven_dist = dx * self.unit_x + dy * self.unit_y
            cross_error = -dx * self.unit_y + dy * self.unit_x
            
            dist_left = self.target_dist - driven_dist
            angle_error = normalize_angle(self.target_angle - pose_msg.theta)

            if dist_left <= TOLERANCE_POS:
                self.current_wp_idx += 1
                self.state = 'INIT_SEGMENT'
            elif abs(angle_error) > 0.5:
                self.state = 'ALIGN'
            else:
                cmd.linear.x = max(MIN_V, min(MAX_V, 2.0 * dist_left))
                cmd.angular.z = (K_HEADING * angle_error) - (K_CROSS * cross_error)

        self.cmd_pub.publish(cmd)

    def init_next_segment(self, pose):
        if self.current_wp_idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            self.state = 'DONE'
            self.get_logger().info('Napis gotowy.')
            return
            
        if not self.pen_client.service_is_ready():
            self.cmd_pub.publish(Twist())
            return

        target_x, target_y, should_draw = self.waypoints[self.current_wp_idx]
        dx, dy = target_x - pose.x, target_y - pose.y
        dist = math.hypot(dx, dy)
        
        if dist < 1e-3:
            self.current_wp_idx += 1
            return
            
        self.start_x, self.start_y = pose.x, pose.y
        self.target_dist = dist
        self.unit_x, self.unit_y = dx / dist, dy / dist
        self.target_angle = math.atan2(dy, dx)

        if self.is_pen_down != should_draw:
            self.state = 'WAIT_FOR_PEN'
            self.change_pen_state(should_draw)
        else:
            self.state = 'ALIGN'

    def change_pen_state(self, draw):
        req = SetPen.Request(r=255, g=255, b=255, width=4, off=0 if draw else 1)
        future = self.pen_client.call_async(req)
        future.add_done_callback(lambda _: self.on_pen_changed(draw))

    def on_pen_changed(self, draw):
        self.is_pen_down = draw
        self.state = 'ALIGN'


def main(args=None):
    rclpy.init(args=args)
    node = TurtleWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
