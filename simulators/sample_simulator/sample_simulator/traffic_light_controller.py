#!/usr/bin/env python3
"""Traffic light controller node.

Switches the billboard from red (stop) to green (go) after a configurable
delay (default: 75 simulation seconds) by using Gazebo's DeleteEntity and
SpawnEntity services.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose


# SDF template for the green-signal billboard.
# The pose matches the red billboard defined in the world file.
BILLBOARD_GO_SDF = """\
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="billboard">
    <static>true</static>
    <link name="billboard_link">
      <visual name="billboard_visual">
        <pose>0 0 2.0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.42 0.42 0.05</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://billboard_go/materials/scripts</uri>
            <uri>model://billboard_go/materials/textures</uri>
            <name>BillboardGo/Image</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# Position of the billboard in the world (must match the world file)
BILLBOARD_POSE_X = -37.65
BILLBOARD_POSE_Y = -25.0
BILLBOARD_POSE_Z = 0.75
BILLBOARD_ROLL = 1.5708
BILLBOARD_PITCH = 1.5708
BILLBOARD_YAW = 0.0


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert Euler angles (radians) to a geometry_msgs/Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class TrafficLightController(Node):
    """ROS2 node that switches the billboard signal after a delay."""

    def __init__(self):
        super().__init__('traffic_light_controller')

        # Declare and read the switch delay parameter (seconds of sim time)
        self.declare_parameter('switch_delay_sec', 75.0)
        self._delay = self.get_parameter('switch_delay_sec').value

        self.get_logger().info(
            f'TrafficLightController: will switch billboard to GREEN '
            f'after {self._delay:.1f} sim-seconds.'
        )

        # Service clients
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # One-shot timer (honours use_sim_time when set in the node)
        self._timer = self.create_timer(self._delay, self._switch_to_green)

    # ------------------------------------------------------------------
    def _switch_to_green(self):
        """Called once after the delay: delete red billboard, spawn green."""
        # Cancel the timer so it fires only once
        self._timer.cancel()

        self.get_logger().info('Switching billboard: RED → GREEN')

        # Step 1: delete the existing billboard model
        if not self._delete_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/delete_entity service not available!')
            return

        del_req = DeleteEntity.Request()
        del_req.name = 'billboard'
        del_future = self._delete_client.call_async(del_req)
        del_future.add_done_callback(self._on_delete_done)

    def _on_delete_done(self, future):
        """Callback after DeleteEntity completes; then spawn green billboard."""
        try:
            result = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'DeleteEntity call failed: {e}')
            return

        if not result.success:
            self.get_logger().warn(
                f'DeleteEntity reported failure: {result.status_message}'
            )

        self.get_logger().info('Red billboard deleted. Spawning green billboard…')

        # Step 2: spawn the green billboard at the same pose
        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity service not available!')
            return

        pose = Pose()
        pose.position.x = BILLBOARD_POSE_X
        pose.position.y = BILLBOARD_POSE_Y
        pose.position.z = BILLBOARD_POSE_Z
        pose.orientation = euler_to_quaternion(
            BILLBOARD_ROLL, BILLBOARD_PITCH, BILLBOARD_YAW
        )

        spawn_req = SpawnEntity.Request()
        spawn_req.name = 'billboard'
        spawn_req.xml = BILLBOARD_GO_SDF
        spawn_req.initial_pose = pose
        spawn_req.reference_frame = 'world'

        spawn_future = self._spawn_client.call_async(spawn_req)
        spawn_future.add_done_callback(self._on_spawn_done)

    def _on_spawn_done(self, future):
        """Callback after SpawnEntity completes."""
        try:
            result = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'SpawnEntity call failed: {e}')
            return

        if result.success:
            self.get_logger().info('Green billboard spawned successfully!')
        else:
            self.get_logger().error(
                f'SpawnEntity failed: {result.status_message}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
