#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import os
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import threading

import rclpy
from rclpy.node import Node
from hexapod_interfaces.msg import JoystickRequest

INTERVAL = 100  # milliseconds
RECONNECT_INTERVAL = 2000  # milliseconds

class NodeTeleop(Node):
    def __init__(self):
        super().__init__('node_teleop')
        self.get_logger().info('starting node_teleop')
        self.msg = JoystickRequest()
        self.joystick = None

        pygame.init()
        pygame.joystick.init()

        self.pub = self.create_publisher(JoystickRequest, 'joystick_request', 10)

        self.running = True
        self.event_thread = threading.Thread(target=self.event_loop, daemon=True)
        self.event_thread.start()

    def try_connect_joystick(self):
        """Try to connect to a joystick, return True if successful"""
        pygame.joystick.quit()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Connected to Joystick: {self.joystick.get_name()}")
            return True
        return False

    def event_loop(self):
        while self.running and rclpy.ok():
            # Try to connect if no joystick
            if self.joystick is None:
                if self.try_connect_joystick():
                    pass
                else:
                    pygame.time.wait(RECONNECT_INTERVAL)
                    continue

            # Check if joystick still connected
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                self.get_logger().warn("Joystick disconnected, waiting for reconnect...")
                self.joystick = None
                pygame.time.wait(RECONNECT_INTERVAL)
                continue
            
            # Reinit joystick after quit/init
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            for event in pygame.event.get():
                self.handle_event(event)
            pygame.time.wait(INTERVAL)

    def handle_event(self, event):
        if self.joystick is None:
            return
        if event.type not in (pygame.JOYBUTTONDOWN, pygame.JOYAXISMOTION, pygame.JOYHATMOTION):
            return
        self.publish_joystick_data()

    def publish_joystick_data(self):
        if self.joystick is None:
            return
            
        pygame.event.pump()

        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        hat_values = self.joystick.get_hat(0)

        self.msg.button_a = bool(buttons[2])
        self.msg.button_b = bool(buttons[1])
        self.msg.button_x = bool(buttons[3])
        self.msg.button_y = bool(buttons[0])
        self.msg.button_l1 = bool(buttons[4])
        self.msg.button_l2 = bool(buttons[6])
        self.msg.button_r1 = bool(buttons[5])
        self.msg.button_r2 = bool(buttons[7])
        self.msg.button_select = bool(buttons[8])
        self.msg.button_start = bool(buttons[9])

        # DualShock 4 axis mapping:
        # axes[0] = Left stick horizontal
        # axes[1] = Left stick vertical
        # axes[2] = L2 trigger (-1.0 released, +1.0 pressed)
        # axes[3] = Right stick horizontal
        # axes[4] = Right stick vertical
        # axes[5] = R2 trigger (-1.0 released, +1.0 pressed)
        self.msg.left_stick_horizontal = 0.0 if abs(axes[0]) < 0.004 else axes[0]
        self.msg.left_stick_vertical = 0.0 if abs(axes[1]) < 0.004 else axes[1]
        self.msg.right_stick_horizontal = 0.0 if abs(axes[3]) < 0.004 else axes[3]
        self.msg.right_stick_vertical = 0.0 if abs(axes[4]) < 0.004 else axes[4]

        self.msg.dpad_horizontal = hat_values[0]
        self.msg.dpad_vertical = hat_values[1]

        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

    def shutdown(self):
        self.running = False
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = NodeTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
