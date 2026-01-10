#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import os
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import threading
import time

import rclpy
from rclpy.node import Node
from hexapod_interfaces.msg import JoystickRequest

INTERVAL = 50  # milliseconds - publish rate ~20Hz

class NodeTeleop(Node):
    def __init__(self):
        super().__init__('node_teleop')
        self.get_logger().info('starting node_teleop')
        self.msg = JoystickRequest()
        self.joystick = None
        self.connected = False

        # DS4 (SDL2) default mapping. Make configurable because indices can differ per OS/driver/mode.
        self.declare_parameter('button_a_index', 2)
        self.declare_parameter('button_b_index', 1)
        self.declare_parameter('button_x_index', 3)
        self.declare_parameter('button_y_index', 0)
        self.declare_parameter('button_l1_index', 4)
        self.declare_parameter('button_l2_index', 6)
        self.declare_parameter('button_r1_index', 5)
        self.declare_parameter('button_r2_index', 7)
        self.declare_parameter('button_select_index', 8)
        self.declare_parameter('button_start_index', 9)

        self.declare_parameter('axis_left_stick_horizontal_index', 0)
        self.declare_parameter('axis_left_stick_vertical_index', 1)
        self.declare_parameter('axis_right_stick_horizontal_index', 3)
        self.declare_parameter('axis_right_stick_vertical_index', 4)

        self.declare_parameter('hat_index', 0)
        self.declare_parameter('axis_noise_threshold', 0.004)
        # DS4 BT/SDL can occasionally report all-zero axes for a short burst while stick is held.
        # This causes false "neutral" detection downstream (MOVE->MOVE_TO_STAND). Hold last good value briefly.
        self.declare_parameter('axis_zero_glitch_hold_ms', 400)
        self.declare_parameter('axis_zero_glitch_neutral_threshold', 0.05)

        self._last_good_axes = {'lh': 0.0, 'lv': 0.0, 'rh': 0.0, 'rv': 0.0}
        self._last_good_axes_time = time.monotonic()
        self._last_glitch_log_time = 0.0

        pygame.init()
        pygame.joystick.init()

        self.pub = self.create_publisher(JoystickRequest, 'joystick_request', 10)

        self.running = True
        self.event_thread = threading.Thread(target=self.event_loop, daemon=True)
        self.event_thread.start()

    def event_loop(self):
        while self.running and rclpy.ok():
            pygame.event.pump()
            
            # Check for joystick connect/disconnect
            current_count = pygame.joystick.get_count()
            
            if current_count > 0 and not self.connected:
                # Joystick connected
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.connected = True
                self.get_logger().info(f"Connected to Joystick: {self.joystick.get_name()}")
            elif current_count == 0 and self.connected:
                # Joystick disconnected
                self.connected = False
                self.joystick = None
                self.get_logger().warn("Joystick disconnected")
                pygame.joystick.quit()
                pygame.joystick.init()
            
            # Publish data if connected
            if self.connected and self.joystick:
                self.publish_joystick_data()
            
            pygame.time.wait(INTERVAL)

    def handle_event(self, event):
        pass  # Not used anymore, publishing continuously

    def publish_joystick_data(self):
        try:
            num_axes = self.joystick.get_numaxes()
            axes = [self.joystick.get_axis(i) for i in range(num_axes)]
            buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
            hat_index = int(self.get_parameter('hat_index').value)
            hat_values = (
                self.joystick.get_hat(hat_index)
                if self.joystick.get_numhats() > 0 and hat_index < self.joystick.get_numhats()
                else (0, 0)
            )
        except pygame.error:
            return

        # Log all axes once at startup to help debug mapping
        if not hasattr(self, '_axes_logged'):
            self._axes_logged = True
            self.get_logger().info(f"Joystick has {num_axes} axes")
            for i, val in enumerate(axes):
                self.get_logger().info(f"  Axis {i}: {val:.3f}")

        def btn(i: int) -> bool:
            return bool(buttons[i]) if i < len(buttons) else False

        def ax(i: int) -> float:
            return float(axes[i]) if i < len(axes) else 0.0

        self.msg.button_a = btn(int(self.get_parameter('button_a_index').value))
        self.msg.button_b = btn(int(self.get_parameter('button_b_index').value))
        self.msg.button_x = btn(int(self.get_parameter('button_x_index').value))
        self.msg.button_y = btn(int(self.get_parameter('button_y_index').value))
        self.msg.button_l1 = btn(int(self.get_parameter('button_l1_index').value))
        self.msg.button_l2 = btn(int(self.get_parameter('button_l2_index').value))
        self.msg.button_r1 = btn(int(self.get_parameter('button_r1_index').value))
        self.msg.button_r2 = btn(int(self.get_parameter('button_r2_index').value))
        self.msg.button_select = btn(int(self.get_parameter('button_select_index').value))
        self.msg.button_start = btn(int(self.get_parameter('button_start_index').value))

        # DualShock 4 axis mapping:
        # axes[0] = Left stick horizontal
        # axes[1] = Left stick vertical
        # axes[2] = L2 trigger (-1.0 released, +1.0 pressed)
        # axes[3] = Right stick horizontal
        # axes[4] = Right stick vertical
        # axes[5] = R2 trigger (-1.0 released, +1.0 pressed)
        thr = float(self.get_parameter('axis_noise_threshold').value)
        ax_lh = ax(int(self.get_parameter('axis_left_stick_horizontal_index').value))
        ax_lv = ax(int(self.get_parameter('axis_left_stick_vertical_index').value))
        ax_rh = ax(int(self.get_parameter('axis_right_stick_horizontal_index').value))
        ax_rv = ax(int(self.get_parameter('axis_right_stick_vertical_index').value))

        lh = 0.0 if abs(ax_lh) < thr else ax_lh
        lv = 0.0 if abs(ax_lv) < thr else ax_lv
        rh = 0.0 if abs(ax_rh) < thr else ax_rh
        rv = 0.0 if abs(ax_rv) < thr else ax_rv

        neutral_thr = float(self.get_parameter('axis_zero_glitch_neutral_threshold').value)
        hold_s = float(self.get_parameter('axis_zero_glitch_hold_ms').value) / 1000.0
        now_m = time.monotonic()

        is_near_zero = (abs(lh) < neutral_thr and abs(lv) < neutral_thr and
                        abs(rh) < neutral_thr and abs(rv) < neutral_thr)
        was_moving = (abs(self._last_good_axes['lh']) >= neutral_thr or
                      abs(self._last_good_axes['lv']) >= neutral_thr or
                      abs(self._last_good_axes['rh']) >= neutral_thr or
                      abs(self._last_good_axes['rv']) >= neutral_thr)

        if is_near_zero and was_moving and (now_m - self._last_good_axes_time) < hold_s:
            # Treat as glitch: keep last good values.
            lh = self._last_good_axes['lh']
            lv = self._last_good_axes['lv']
            rh = self._last_good_axes['rh']
            rv = self._last_good_axes['rv']
            if (now_m - self._last_glitch_log_time) > 1.0:
                self._last_glitch_log_time = now_m
                self.get_logger().warn("Axis zero-glitch detected; holding last axes briefly (BT/SDL jitter).")
        else:
            self._last_good_axes = {'lh': lh, 'lv': lv, 'rh': rh, 'rv': rv}
            self._last_good_axes_time = now_m

        self.msg.left_stick_horizontal = lh
        self.msg.left_stick_vertical = lv
        self.msg.right_stick_horizontal = rh
        self.msg.right_stick_vertical = rv

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
