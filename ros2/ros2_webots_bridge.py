#!/usr/bin/env python3
"""Pont ROS 2 <-> Webots, pense pour : Webots + MATLAB sous Windows, ROS 2 sous WSL.

  UDP entrant (optionnel, udp_in)  -->  topic /cooling/state  -->  UDP sortant vers Webots

- /cooling/state : std_msgs/Float64MultiArray, 17 valeurs (ordre de cosim_webots.m).
- udp_in > 0 : le pont ecoute ce port UDP (paquets "M,..." envoyes par MATLAB) et les
  publie sur /cooling/state : les autres noeuds ROS 2 de WSL voient donc les donnees,
  sans decouverte DDS entre Windows et WSL.
- Chaque message du topic (venant du pont ou d'un autre noeud) est transmis a Webots.

Parametres : host (IP Windows de Webots), port (5005), udp_in (0 = desactive).
    python3 ros2_webots_bridge.py --ros-args -p host:=172.20.16.1 -p udp_in:=5006
"""
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Bridge(Node):
    def __init__(self):
        super().__init__("cooling_webots_bridge")
        host = self.declare_parameter("host", "127.0.0.1").value
        port = int(self.declare_parameter("port", 5005).value)
        udp_in = int(self.declare_parameter("udp_in", 0).value)
        self.addr = (host, port)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pub = self.create_publisher(Float64MultiArray, "/cooling/state", 10)
        self.create_subscription(Float64MultiArray, "/cooling/state", self.on_state, 10)
        self.inp = None
        if udp_in > 0:
            self.inp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.inp.bind(("0.0.0.0", udp_in))
            self.inp.setblocking(False)
            self.create_timer(0.01, self.poll_udp)
        self.get_logger().info(f"/cooling/state -> UDP {host}:{port}" + (f" ; UDP entrant :{udp_in}" if self.inp else ""))

    def poll_udp(self):
        try:
            while True:
                data, _ = self.inp.recvfrom(1024)
                try:
                    p = data.decode().strip().split(",")
                    if p[0] == "M" and len(p) >= 18:
                        msg = Float64MultiArray()
                        msg.data = [float(x) for x in p[1:18]]
                        self.pub.publish(msg)
                except ValueError:
                    self.get_logger().warn("paquet UDP illisible")
        except BlockingIOError:
            pass

    def on_state(self, msg):
        if len(msg.data) < 17:
            self.get_logger().warn(f"message trop court ({len(msg.data)} valeurs, 17 attendues)")
            return
        line = "M," + ",".join(f"{x:.4f}" for x in msg.data[:17])
        self.out.sendto(line.encode(), self.addr)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
