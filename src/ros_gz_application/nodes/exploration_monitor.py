#!/usr/bin/env python3
import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from slam_toolbox.srv import SaveMap, SerializePoseGraph


# Kart fra Tailscale-IP → (ssh-bruker, ekstern mappe)
# Oppdater med riktig bruker og mappe for hver person.
OPERATOR_MAP = {
    '100.104.108.87':  ('groven',  '/home/groven/code/MEPA2002SAR/maps/'),
    '100.65.35.124':   ('lauritz', '/home/lauritz/maps/'),
    '100.110.49.100':  ('emma',    '/home/emma/maps/'),
    '100.125.96.77':   ('julie',   '/home/julie/maps/'),
}


class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')
        self.declare_parameter('map_path', '/maps/gregor_maps')
        self.declare_parameter('idle_timeout', 25.0)
        self.declare_parameter('startup_grace', 30.0)
        self.declare_parameter('remote_target', '/home/ubuntu/maps/gregor_maps')  # manuell override

        self.map_path = self.get_parameter('map_path').value
        self.idle_timeout = self.get_parameter('idle_timeout').value
        self.startup_grace = self.get_parameter('startup_grace').value

        # Auto-detect IP fra SSH_CLIENT (satt av Docker via compose SSH_CLIENT: ${SSH_CLIENT:-})
        ssh_client_env = os.environ.get('SSH_CLIENT', '').strip()
        manual_target = self.get_parameter('remote_target').value

        if manual_target:
            self.remote_target = manual_target
            self.get_logger().info(f'Using manual remote_target: {self.remote_target}')
        elif ssh_client_env:
            operator_ip = ssh_client_env.split()[0]
            if operator_ip in OPERATOR_MAP:
                user, path = OPERATOR_MAP[operator_ip]
                self.remote_target = f'{user}@{operator_ip}:{path}'
                self.get_logger().info(f'Auto-detected operator: {self.remote_target}')
            else:
                self.remote_target = ''
                self.get_logger().warn(
                    f'Unknown operator IP {operator_ip} — map will only be saved locally.')
        else:
            self.remote_target = ''
            self.get_logger().warn(
                'No SSH_CLIENT env and no remote_target set — map will only be saved locally.')

        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)

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
            f'Exploration monitor started. map_path={self.map_path}, '
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

        if self.remote_target:
            for ext in ('yaml', 'pgm', 'data', 'posegraph'):
                src = f'{self.map_path}.{ext}'
                if os.path.exists(src):
                    subprocess.Popen(
                        ['scp', '-o', 'StrictHostKeyChecking=no', src, self.remote_target],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
            self.get_logger().info(f'Sending map files to {self.remote_target}')


def main():
    rclpy.init()
    rclpy.spin(ExplorationMonitor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()