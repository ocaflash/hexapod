from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from hexapod_interfaces.msg import MovementRequest


@dataclass
class ButtonState:
    start: bool = False
    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False
    l1: bool = False
    r1: bool = False
    dpad_up: bool = False
    dpad_down: bool = False


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("cmdvel_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("movement_request_topic", "/movement_request")

        self.declare_parameter("linear_x_limit", 0.8)
        self.declare_parameter("linear_y_limit", 0.8)
        self.declare_parameter("angular_z_limit", 2.0)

        self.declare_parameter("deadzone_start", 0.05)
        self.declare_parameter("deadzone_stop", 0.02)
        self.declare_parameter("stop_delay_ms", 200)
        self.declare_parameter("timeout_ms", 500)
        self.declare_parameter("move_to_stand_duration_ms", 300)

        self.declare_parameter("stand_duration_ms", 2000)
        self.declare_parameter("laydown_duration_ms", 2000)
        self.declare_parameter("watch_duration_ms", 3000)
        self.declare_parameter("high_five_duration_ms", 3000)
        self.declare_parameter("clap_duration_ms", 3000)
        self.declare_parameter("transport_duration_ms", 2000)
        self.declare_parameter("look_left_duration_ms", 2000)
        self.declare_parameter("look_right_duration_ms", 2000)

        self.declare_parameter("button_start_index", 9)
        self.declare_parameter("button_a_index", 2)
        self.declare_parameter("button_b_index", 1)
        self.declare_parameter("button_x_index", 3)
        self.declare_parameter("button_y_index", 0)
        self.declare_parameter("button_l1_index", 4)
        self.declare_parameter("button_r1_index", 5)

        self.declare_parameter("dpad_vertical_axis_index", 7)
        self.declare_parameter("dpad_horizontal_axis_index", 6)
        self.declare_parameter("dpad_axis_threshold", 0.5)

        self.publisher_ = self.create_publisher(
            MovementRequest,
            self.get_parameter("movement_request_topic").get_parameter_value().string_value,
            10,
        )
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            self.on_cmd_vel,
            10,
        )
        self.joy_sub_ = self.create_subscription(
            Joy,
            self.get_parameter("joy_topic").get_parameter_value().string_value,
            self.on_joy,
            10,
        )

        self.last_cmd_vel_time_: Optional[Time] = None
        self.last_non_neutral_time_: Optional[Time] = None
        self.movement_active_ = False
        self.is_standing_ = False
        self.lock_until_: Optional[Time] = None
        self.prev_buttons_ = ButtonState()
        self.name_map_ = {
            MovementRequest.MOVE: "MOVE",
            MovementRequest.MOVE_TO_STAND: "MOVE_TO_STAND",
            MovementRequest.STAND_UP: "STAND_UP",
            MovementRequest.LAYDOWN: "LAYDOWN",
            MovementRequest.WATCH: "WATCH",
            MovementRequest.HIGH_FIVE: "HIGH_FIVE",
            MovementRequest.CLAP: "CLAP",
            MovementRequest.TRANSPORT: "TRANSPORT",
            MovementRequest.LOOK_LEFT: "LOOK_LEFT",
            MovementRequest.LOOK_RIGHT: "LOOK_RIGHT",
        }

        self.timer_ = self.create_timer(0.1, self.on_timer)

        self.get_logger().info("cmd_vel bridge started (OPTIONS or DPAD UP to stand)")

    def on_cmd_vel(self, msg: Twist) -> None:
        now = self.get_clock().now()
        self.last_cmd_vel_time_ = now

        if self._is_locked(now) or not self.is_standing_:
            return

        linear_x = self._clamp(msg.linear.x, self._get_double("linear_x_limit"))
        linear_y = self._clamp(msg.linear.y, self._get_double("linear_y_limit"))
        angular_z = self._clamp(msg.angular.z, self._get_double("angular_z_limit"))

        deadzone = self._get_double("deadzone_stop") if self.movement_active_ else self._get_double("deadzone_start")
        linear_x = self._apply_deadzone(linear_x, deadzone)
        linear_y = self._apply_deadzone(linear_y, deadzone)
        angular_z = self._apply_deadzone(angular_z, deadzone)

        moving = any(abs(v) > 0.0 for v in (linear_x, linear_y, angular_z))

        if moving:
            self.last_non_neutral_time_ = now
            self.movement_active_ = True
            self._publish_move(linear_x, linear_y, angular_z)
            return

        if self.movement_active_:
            if self.last_non_neutral_time_ is None:
                self.last_non_neutral_time_ = now
                return

            if self._elapsed_ms(self.last_non_neutral_time_, now) > self._get_int("stop_delay_ms"):
                self.movement_active_ = False
                self._publish_request(
                    MovementRequest.MOVE_TO_STAND,
                    self._get_int("move_to_stand_duration_ms"),
                )

    def on_joy(self, msg: Joy) -> None:
        now = self.get_clock().now()
        if self._is_locked(now):
            return

        buttons = msg.buttons
        axes = msg.axes

        def btn(index: int) -> bool:
            return bool(buttons[index]) if index < len(buttons) else False

        start = btn(self._get_int("button_start_index"))
        a = btn(self._get_int("button_a_index"))
        b = btn(self._get_int("button_b_index"))
        x = btn(self._get_int("button_x_index"))
        y = btn(self._get_int("button_y_index"))
        l1 = btn(self._get_int("button_l1_index"))
        r1 = btn(self._get_int("button_r1_index"))

        dpad_up, dpad_down = self._dpad_vertical(axes)

        pressed = ButtonState(
            start=start and not self.prev_buttons_.start,
            a=a and not self.prev_buttons_.a,
            b=b and not self.prev_buttons_.b,
            x=x and not self.prev_buttons_.x,
            y=y and not self.prev_buttons_.y,
            l1=l1 and not self.prev_buttons_.l1,
            r1=r1 and not self.prev_buttons_.r1,
            dpad_up=dpad_up and not self.prev_buttons_.dpad_up,
            dpad_down=dpad_down and not self.prev_buttons_.dpad_down,
        )

        self.prev_buttons_ = ButtonState(
            start=start,
            a=a,
            b=b,
            x=x,
            y=y,
            l1=l1,
            r1=r1,
            dpad_up=dpad_up,
            dpad_down=dpad_down,
        )

        if pressed.start:
            if not self.is_standing_:
                self.get_logger().info("OPTIONS: Standing up...")
                self.is_standing_ = True
                self._publish_request(MovementRequest.STAND_UP, self._get_int("stand_duration_ms"))
                self.movement_active_ = False
                self.last_non_neutral_time_ = None
                self._lock(self._get_int("stand_duration_ms"))
            else:
                self.get_logger().info("OPTIONS: Laying down...")
                self.is_standing_ = False
                self._publish_request(MovementRequest.LAYDOWN, self._get_int("laydown_duration_ms"))
                self.movement_active_ = False
                self.last_non_neutral_time_ = None
                self._lock(self._get_int("laydown_duration_ms"))
            return

        if not self.is_standing_:
            if pressed.dpad_up:
                self.get_logger().info("DPAD UP: Standing up...")
                self.is_standing_ = True
                self._publish_request(MovementRequest.STAND_UP, self._get_int("stand_duration_ms"))
                self.movement_active_ = False
                self.last_non_neutral_time_ = None
                self._lock(self._get_int("stand_duration_ms"))
            return

        if pressed.dpad_down:
            self.get_logger().info("DPAD DOWN: Laying down...")
            self.is_standing_ = False
            self._publish_request(MovementRequest.LAYDOWN, self._get_int("laydown_duration_ms"))
            self.movement_active_ = False
            self.last_non_neutral_time_ = None
            self._lock(self._get_int("laydown_duration_ms"))
            return

        if pressed.a:
            self.get_logger().info("X: Watch")
            self._publish_request(MovementRequest.WATCH, self._get_int("watch_duration_ms"))
            self.movement_active_ = False
            self.last_non_neutral_time_ = None
            self.last_cmd_vel_time_ = None
            self._lock(self._get_int("watch_duration_ms"))
            return
        if pressed.b:
            self.get_logger().info("Circle: High Five")
            self._publish_request(MovementRequest.HIGH_FIVE, self._get_int("high_five_duration_ms"))
            self.movement_active_ = False
            self.last_non_neutral_time_ = None
            self.last_cmd_vel_time_ = None
            self._lock(self._get_int("high_five_duration_ms"))
            return
        if pressed.x:
            self.get_logger().info("Square: Clap")
            self._publish_request(MovementRequest.CLAP, self._get_int("clap_duration_ms"))
            self.movement_active_ = False
            self.last_non_neutral_time_ = None
            self.last_cmd_vel_time_ = None
            self._lock(self._get_int("clap_duration_ms"))
            return
        if pressed.y:
            self.get_logger().info("Triangle: Transport")
            self.is_standing_ = False
            self._publish_request(MovementRequest.TRANSPORT, self._get_int("transport_duration_ms"))
            self.movement_active_ = False
            self.last_non_neutral_time_ = None
            self.last_cmd_vel_time_ = None
            self._lock(self._get_int("transport_duration_ms"))
            return
        if pressed.l1:
            self.get_logger().info("L1: Look Left")
            self._publish_request(MovementRequest.LOOK_LEFT, self._get_int("look_left_duration_ms"))
            self.movement_active_ = False
            self.last_non_neutral_time_ = None
            self.last_cmd_vel_time_ = None  # Reset timeout
            self._lock(self._get_int("look_left_duration_ms"))
            return
        if pressed.r1:
            self.get_logger().info("R1: Look Right")
            self._publish_request(MovementRequest.LOOK_RIGHT, self._get_int("look_right_duration_ms"))
            self.movement_active_ = False
            self.last_non_neutral_time_ = None
            self.last_cmd_vel_time_ = None  # Reset timeout
            self._lock(self._get_int("look_right_duration_ms"))
            return

    def on_timer(self) -> None:
        if not self.is_standing_ or not self.movement_active_:
            return
        if self.last_cmd_vel_time_ is None:
            return

        now = self.get_clock().now()
        if self._elapsed_ms(self.last_cmd_vel_time_, now) > self._get_int("timeout_ms"):
            self.get_logger().warn("cmd_vel timeout: stopping MOVE")
            self.movement_active_ = False
            self._publish_request(MovementRequest.MOVE_TO_STAND, self._get_int("move_to_stand_duration_ms"))

    def _publish_move(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        msg = MovementRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type = MovementRequest.MOVE
        msg.duration_ms = 0
        msg.name = self.name_map_.get(MovementRequest.MOVE, "MOVE")
        msg.velocity.linear.x = linear_x
        msg.velocity.linear.y = linear_y
        msg.velocity.angular.z = angular_z
        self.publisher_.publish(msg)

    def _publish_request(self, req_type: int, duration_ms: int) -> None:
        msg = MovementRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type = req_type
        msg.duration_ms = int(duration_ms)
        msg.name = self.name_map_.get(req_type, str(req_type))
        self.publisher_.publish(msg)

    def _lock(self, duration_ms: int) -> None:
        now = self.get_clock().now()
        self.lock_until_ = now + Duration(nanoseconds=int(duration_ms) * 1_000_000)

    def _is_locked(self, now: Time) -> bool:
        return self.lock_until_ is not None and now < self.lock_until_

    def _dpad_vertical(self, axes: list[float]) -> tuple[bool, bool]:
        index = self._get_int("dpad_vertical_axis_index")
        threshold = self._get_double("dpad_axis_threshold")
        if index >= len(axes):
            return False, False
        value = axes[index]
        return value > threshold, value < -threshold

    def _get_double(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _get_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(min(value, limit), -limit)

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        return 0.0 if abs(value) < deadzone else value

    @staticmethod
    def _elapsed_ms(start: Time, end: Time) -> int:
        return int((end - start).nanoseconds / 1_000_000)


def main() -> None:
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
