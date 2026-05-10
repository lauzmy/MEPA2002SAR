#!/usr/bin/env python3
import os
import subprocess
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from slam_toolbox.srv import SaveMap, SerializePoseGraph


class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')
        # Rotmappe for alle kart-kjøringer. Hver kjøring får en undermappe
        # med tidsstempel slik at gamle kart ikke overskrives.
        self.declare_parameter('maps_root', '/home/ubuntu/maps')
        self.declare_parameter('map_name', 'gregor_map')
        self.declare_parameter('idle_timeout', 25.0)
        self.declare_parameter('startup_grace', 30.0)
        # Sti til script som pusher kartet til git og rydder lokalt.
        # Tom streng = ikke push, kun lagre lokalt.
        self.declare_parameter('push_script', '/home/ubuntu/push_map.sh')

        maps_root = self.get_parameter('maps_root').value
        map_name = self.get_parameter('map_name').value
        self.idle_timeout = self.get_parameter('idle_timeout').value
        self.startup_grace = self.get_parameter('startup_grace').value
        self.push_script = self.get_parameter('push_script').value

        # Lag undermappe med dato og tid: <maps_root>/YYYY-MM-DD_HHMMSS/
        self.run_stamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        self.run_dir = os.path.join(maps_root, self.run_stamp)
        os.makedirs(self.run_dir, exist_ok=True)
        # slam_toolbox lagrer som <map_path>.{pgm,yaml,posegraph,data}
        self.map_path = os.path.join(self.run_dir, map_name)

        self.last_frontier_time = None
        self.start_time = self.get_clock().now()
        self.saved = False
        self.seen_any_frontier = False

        self.create_subscription(
            MarkerArray, '/explore/frontiers', self._on_frontiers, 10)
        self.finished_pub = self.create_publisher(Bool, '/exploration_finished', 1)

        self.save_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.serialize_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'Exploration monitor started. run_dir={self.run_dir}, '
            f'idle_timeout={self.idle_timeout}s, startup_grace={self.startup_grace}s')

    def _on_frontiers(self, msg: MarkerArray):
        self.seen_any_frontier = True
        self.last_frontier_time = self.get_clock().now()

    def _tick(self):
        if self.saved:
            return
        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds * 1e-9 < self.startup_grace:
            return
        if not self.seen_any_frontier:
            return
        idle = (now - self.last_frontier_time).nanoseconds * 1e-9
        if idle < self.idle_timeout:
            return
        self.get_logger().info(f'No frontiers for {idle:.1f}s -> saving map.')
        self._save_map()

    def _save_map(self):
        self.saved = True
        if self.save_client.wait_for_service(timeout_sec=5.0):
            req = SaveMap.Request()
            req.name = String(data=self.map_path)
            fut = self.save_client.call_async(req)
            fut.add_done_callback(self._after_save)
        else:
            self.get_logger().error('/slam_toolbox/save_map not available')
            self.saved = False
            return

        if self.serialize_client.wait_for_service(timeout_sec=2.0):
            sreq = SerializePoseGraph.Request()
            sreq.filename = self.map_path
            self.serialize_client.call_async(sreq)

    def _after_save(self, fut):
        try:
            fut.result()
            self.get_logger().info(f'Map saved to {self.map_path}.{{yaml,pgm}}')
        except Exception as e:
            self.get_logger().error(f'save_map failed: {e}')
            self.saved = False
            return

        self.finished_pub.publish(Bool(data=True))

        # Kjør push-script: kopierer kjøre-mappen til maps-branch worktree,
        # commiter, pusher og rydder lokalt på Pi-en.
        if self.push_script and os.path.exists(self.push_script):
            self.get_logger().info(
                f'Running push script: {self.push_script} {self.run_dir}')
            subprocess.Popen(
                [self.push_script, self.run_dir],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        else:
            self.get_logger().warn(
                f'push_script "{self.push_script}" not found — '
                f'map kept locally only.')


def main():
    rclpy.init()
    rclpy.spin(ExplorationMonitor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()