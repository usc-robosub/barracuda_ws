from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class JetsonThermalWarning(Node):
    def __init__(self):
        super().__init__("jetson_thermal_warning")

        self.declare_parameter("poll_rate_hz", 1.0)
        self.declare_parameter("warning_temp_c", 70.0)
        self.declare_parameter("critical_temp_c", 80.0)
        self.declare_parameter("zone_name_filter", "")
        self.declare_parameter("status_topic", "jetson/thermal_status")
        self.declare_parameter("overheat_topic", "jetson/overheat_warning")
        self.declare_parameter("max_temp_topic", "jetson/max_temperature_c")

        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self.overheat_pub = self.create_publisher(
            Bool, self.get_parameter("overheat_topic").value, 10
        )
        self.max_temp_pub = self.create_publisher(
            Float32, self.get_parameter("max_temp_topic").value, 10
        )

        self.missing_data_warned = False

        poll_rate_hz = float(self.get_parameter("poll_rate_hz").value)
        timer_period = 1.0 / poll_rate_hz if poll_rate_hz > 0.0 else 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Jetson thermal warning node started")

    def timer_callback(self):
        zone_temps = self.read_zone_temperatures()
        if not zone_temps:
            if not self.missing_data_warned:
                self.get_logger().warning(
                    "No readable thermal zones found under /sys/class/thermal"
                )
                self.missing_data_warned = True
            return

        self.missing_data_warned = False

        max_temp_c = max(temp_c for _, temp_c in zone_temps)
        state = self.classify_state(max_temp_c)
        self.publish_state(state, max_temp_c)
        self.log_current_state(state, max_temp_c, zone_temps)

    def read_zone_temperatures(self):
        zone_name_filter = self.get_parameter("zone_name_filter").value.strip().lower()
        thermal_root = Path("/sys/class/thermal")
        zone_temps = []

        for zone_dir in sorted(thermal_root.glob("thermal_zone*")):
            type_path = zone_dir / "type"
            temp_path = zone_dir / "temp"

            try:
                zone_name = self.safe_read_text(type_path)
                raw_temp = self.safe_read_text(temp_path)
            except (OSError, TypeError, UnicodeDecodeError):
                continue

            if zone_name_filter and zone_name_filter not in zone_name.lower():
                continue

            try:
                temp_c = float(raw_temp) / 1000.0
            except ValueError:
                continue

            zone_temps.append((zone_name, temp_c))

        return zone_temps

    def safe_read_text(self, path: Path) -> str:
        try:
            raw_bytes = path.read_bytes()
        except (OSError, TypeError):
            raise

        if raw_bytes is None:
            raise OSError(f"Read returned no data for {path}")

        return raw_bytes.decode("utf-8").strip()

    def classify_state(self, max_temp_c):
        critical_temp_c = float(self.get_parameter("critical_temp_c").value)
        warning_temp_c = float(self.get_parameter("warning_temp_c").value)

        if max_temp_c >= critical_temp_c:
            return "CRITICAL"
        if max_temp_c >= warning_temp_c:
            return "WARNING"
        return "NORMAL"

    def publish_state(self, state, max_temp_c):
        status_msg = String()
        status_msg.data = state
        self.status_pub.publish(status_msg)

        overheat_msg = Bool()
        overheat_msg.data = state in ("WARNING", "CRITICAL")
        self.overheat_pub.publish(overheat_msg)

        temp_msg = Float32()
        temp_msg.data = float(max_temp_c)
        self.max_temp_pub.publish(temp_msg)

    def log_current_state(self, state, max_temp_c, zone_temps):
        hottest_zones = ", ".join(
            f"{name}={temp_c:.1f}C"
            for name, temp_c in sorted(zone_temps, key=lambda item: item[1], reverse=True)[:3]
        )

        if state == "CRITICAL":
            self.get_logger().error(
                f"Jetson temperature is CRITICAL: {max_temp_c:.1f}C ({hottest_zones})"
            )
        elif state == "WARNING":
            self.get_logger().warning(
                f"Jetson temperature is WARNING: {max_temp_c:.1f}C ({hottest_zones})"
            )
        else:
            self.get_logger().debug(
                f"Jetson temperature is NORMAL: {max_temp_c:.1f}C ({hottest_zones})"
            )


def main(args=None):
    rclpy.init(args=args)

    node = JetsonThermalWarning()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
