#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Int32
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty as Clear
from math import gcd

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')
        self.subscription = self.create_subscription(Int32, '/selected_shape', self.listener_callback, 10)

        # Services
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.clear_client = self.create_client(Clear, '/clear')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        self.timer = None
        self.running = False
        self.shape_id = None
        self.theta = 0.0
        self.scale = 0.8  # smaller scale for default window
        self.center_x = 5.5
        self.center_y = 5.5

        self.get_logger().info("TurtleCommander node started. Waiting for commands...")

        # Start with pen OFF and teleport to center without drawing
        self.set_pen(255, 255, 255, 2, 1)  # pen off
        self.teleport(self.center_x, self.center_y, 0.0)

    def listener_callback(self, msg):
        command = msg.data
        if command in [1, 2, 3]:  # Shapes
            self.start_shape(command)
        elif command == 4:  # Clear
            self.clear_screen()
        elif command == 5:  # Stop
            self.stop_drawing()

    def start_shape(self, shape_id):
        # Stop current drawing
        self.stop_drawing()

        # Clear screen
        self.clear_screen()

        # Set new shape
        self.shape_id = shape_id
        self.theta = 0.0
        self.completed_cycles = 0
        self.running = True

        # Lift pen and teleport to center BEFORE first point
        self.set_pen(255, 255, 255, 2, 1)  # pen off
        self.teleport(self.center_x, self.center_y, 0.0)

        # Start drawing timer - pen will be turned on at first draw point
        self.timer = self.create_timer(0.02, self.draw_step)
        self.get_logger().info(f"Drawing shape {shape_id}")

    def clear_screen(self):
        # Wait for service
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /clear service...')

        # Clear screen
        req = Clear.Request()
        self.clear_client.call_async(req)

        # Reset turtle to center with pen OFF (no drawing)
        self.set_pen(255, 255, 255, 2, 1)  # pen off
        self.teleport(self.center_x, self.center_y, 0.0)
        # DO NOT turn pen on here - only turn it on when actually drawing shapes

        self.get_logger().info("Screen cleared & turtle reset to center")

    def stop_drawing(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.running = False
        # Turn pen off when stopping
        self.set_pen(255, 255, 255, 2, 1)
        self.get_logger().info("Stopped drawing")

    def draw_step(self):
        if not self.running:
            return

        x, y = 0.0, 0.0

        if self.shape_id == 1:  # 🌀 Lissajous Mandala Spiral
            R, r, d = 3.0, 0.6, 1.5   # tweak these values for symmetry
            cycles = 30               # number of loops before stopping
            cycle_length = 2 * math.pi * r / math.gcd(int(R), int(r))  # ensures closed loop
            
            # continuous cycle number
            cycle_num = self.theta / cycle_length
        
            # shrink factor (1.0 until 1 cycle, then exponential decay)
            if cycle_num < 1:
                shrink = 1.0
            else:
                shrink = math.exp(-0.05 * (cycle_num - 1))  # adjust 0.05 for faster/slower shrink
        
            # apply shrinking
            x = self.scale * shrink * ((R - r) * math.cos(self.theta) + d * math.cos((R - r) / r * self.theta))
            y = self.scale * shrink * ((R - r) * math.sin(self.theta) - d * math.sin((R - r) / r * self.theta))
        
            # stop after enough cycles
            if cycle_num >= cycles:
                self.stop_drawing()

        elif self.shape_id == 2:  # Epicycloid (Spirograph with gradual hypnotic shrinking)
            R, r, d = 2.0, 0.5, 1.0
            k = (R + r) / r
            cycles = 3
            cycle_length = 2 * math.pi * (R / r)

            # progress inside the whole drawing
            progress = (self.theta % cycle_length) / cycle_length
            cycle_num = self.completed_cycles + progress

            # gradual shrink further inward (down to ~10% of size)
            shrink = max(0.1, 1.0 - (cycle_num / cycles) * 1.2)

            x = self.scale * shrink * ((R + r) * math.cos(self.theta) - d * math.cos(k * self.theta))
            y = self.scale * shrink * ((R + r) * math.sin(self.theta) - d * math.sin(k * self.theta))

            if self.theta >= cycle_length:
                self.theta = 0.0
                self.completed_cycles += 1
                if self.completed_cycles >= cycles:
                    self.stop_drawing()

        # 🔗 Torus Knot (3 inward loops)
        elif self.shape_id == 3:
            p, q = 3, 5
            cycles = 3
            cycle_length = 2 * math.pi * q   # full closed loop length

            # continuous progress across all cycles
            cycle_num = self.theta / cycle_length  

            # smooth shrink: from 1.0 → 0.7 linearly over 'cycles'
            linear_shrink = 1.0 - (cycle_num / cycles) * 0.5
            expo_shrink = 0.8 ** cycle_num
            shrink = linear_shrink * expo_shrink

            x = self.scale * shrink * ((2 + math.cos(q * self.theta)) * math.cos(p * self.theta))
            y = self.scale * shrink * ((2 + math.cos(q * self.theta)) * math.sin(p * self.theta))

            # stop after enough inward spirals
            if cycle_num >= cycles:
                self.stop_drawing()

        # 🚫 First point → teleport with pen OFF, then turn pen ON
        if abs(self.theta) < 1e-6 and self.completed_cycles == 0:
            # First point: teleport with pen off, then turn pen on
            self.set_pen(255, 255, 255, 2, 1)  # pen off
            self.teleport(self.center_x + x, self.center_y + y, 0.0)
            self.set_pen(255, 255, 255, 2, 0)  # pen on
        else:
            # Subsequent points: teleport with pen on (drawing)
            self.teleport(self.center_x + x, self.center_y + y, 0.0)

        self.theta += 0.02

    def teleport(self, x, y, theta=0.0):
        if not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Teleport service not available')
            return
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self.teleport_client.call_async(req)

    def set_pen(self, r, g, b, width, off):
        if not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SetPen service not available')
            return
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# #ros2 run turtle_controller shapeNode
# #ros2 run turtle_controller turtleCommander
# #ros2 run turtlesim turtlesim_node