import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from pymavlink import mavutil

from .grid import LogOddsGrid


class MapperNode(Node):
    """Liest MAVLink (UDP), baut ein 2D-Belegungsraster und veroeffentlicht /map, /drone_path und TF map->base_link."""

    def __init__(self):
        super().__init__("drone_mapper")
        self.declare_parameter("connection", "udpin:0.0.0.0:14550")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("size_m", 20.0)
        self.declare_parameter("orientations", [0, 2, 6])   # vorn, rechts, links (Boden-Sensor 25 wird ignoriert)
        self.declare_parameter("map_rate_hz", 2.0)

        self.grid = LogOddsGrid(self.get_parameter("size_m").value, self.get_parameter("resolution").value)
        self.orients = set(int(o) for o in self.get_parameter("orientations").value)

        self.link = mavutil.mavlink_connection(self.get_parameter("connection").value)
        self.get_logger().info("Warte auf MAVLink an %s" % self.get_parameter("connection").value)

        self.north = self.east = self.yaw_ned = 0.0
        self.have_pos = self.have_att = self.have_hb = False

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", map_qos)
        self.path_pub = self.create_publisher(Path, "drone_path", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.path = Path()
        self.path.header.frame_id = "map"

        self.create_timer(0.01, self.poll)
        self.create_timer(0.05, self.publish_pose)
        self.create_timer(1.0 / self.get_parameter("map_rate_hz").value, self.publish_map)
        self.create_timer(0.2, self.append_path)

    # ---- MAVLink einlesen ----
    def poll(self):
        for _ in range(300):
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "HEARTBEAT" and not self.have_hb:
                self.have_hb = True
                self.get_logger().info("Heartbeat empfangen")
            elif t == "LOCAL_POSITION_NED":
                self.north, self.east = msg.x, msg.y
                self.have_pos = True
            elif t == "ATTITUDE_QUATERNION":
                w, x, y, z = msg.q1, msg.q2, msg.q3, msg.q4
                self.yaw_ned = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                self.have_att = True
            elif t == "DISTANCE_SENSOR" and msg.orientation in self.orients:
                if self.have_pos and self.have_att:
                    self.add_measurement(msg)

    def add_measurement(self, msg):
        d = msg.current_distance / 100.0
        dmin, dmax = msg.min_distance / 100.0, msg.max_distance / 100.0
        if d < dmin:
            return
        a = self.yaw_ned + math.radians(msg.orientation * 45.0)   # NED: von Nord im Uhrzeigersinn
        hit = d < dmax - 0.01
        self.grid.update_ray(self.east, self.north, math.sin(a), math.cos(a), min(d, dmax), hit)

    # ---- Ausgabe ----
    def yaw_enu(self):
        return math.pi / 2.0 - self.yaw_ned

    def publish_pose(self):
        if not (self.have_pos and self.have_att):
            return
        th = self.yaw_enu()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.east
        t.transform.translation.y = self.north
        t.transform.rotation.z = math.sin(th / 2.0)
        t.transform.rotation.w = math.cos(th / 2.0)
        self.tf_pub.sendTransform(t)

    def append_path(self):
        if not (self.have_pos and self.have_att):
            return
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = self.east, self.north
        th = self.yaw_enu()
        p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2.0), math.cos(th / 2.0)
        self.path.poses.append(p)
        self.path.poses = self.path.poses[-3000:]
        self.path.header.stamp = p.header.stamp
        self.path_pub.publish(self.path)

    def publish_map(self):
        g = self.grid
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.info.resolution = g.res
        m.info.width = g.n
        m.info.height = g.n
        m.info.origin.position.x = -g.half
        m.info.origin.position.y = -g.half
        m.info.origin.orientation.w = 1.0
        m.data = g.occupancy().reshape(-1).tolist()
        self.map_pub.publish(m)


def main():
    rclpy.init()
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
