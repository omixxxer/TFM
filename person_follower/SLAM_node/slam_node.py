import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import subprocess

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        # Publisher for map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Publisher for initial pose
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Timer to periodically publish map data
        self.create_timer(1.0, self.publish_map)

        # Launch SLAM process
        self.start_slam_toolbox()

    def start_slam_toolbox(self):
        try:
            # Launch slam_toolbox (online_sync mode for real-time SLAM)
            self.slam_process = subprocess.Popen(['ros2', 'launch', 'slam_toolbox', 'online_sync_launch.py'])
            self.get_logger().info("SLAM Toolbox launched successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to launch SLAM Toolbox: {e}")

    def publish_map(self):
        # Stub function to simulate map publishing
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = 0.05  # Example resolution
        map_msg.info.width = 100  # Example width
        map_msg.info.height = 100  # Example height
        map_msg.info.origin.position.x = -2.5
        map_msg.info.origin.position.y = -2.5
        map_msg.data = [0] * (100 * 100)  # Example empty map
        self.map_publisher.publish(map_msg)
        self.get_logger().info("Published dummy map.")

    def publish_static_transform(self):
        # Publish static transform between map and odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published static transform from map to odom.")

    def destroy_node(self):
        # Terminate SLAM process on shutdown
        if hasattr(self, 'slam_process') and self.slam_process:
            self.slam_process.terminate()
            self.get_logger().info("SLAM Toolbox terminated.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
