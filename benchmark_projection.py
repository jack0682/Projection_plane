#!/usr/bin/env python3
"""
Benchmark script for projection_plane node
Measures latency, throughput, and performance metrics
"""

import rclpy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import time
import numpy as np
from collections import deque
import signal
import sys

class ProjectionBenchmark:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('projection_benchmark')

        self.publisher = self.node.create_publisher(
            Float64MultiArray,
            '/projection/plane',
            10
        )

        self.subscription = self.node.create_subscription(
            Image,
            '/projection/image',
            self.image_callback,
            10
        )

        self.latencies = deque(maxlen=100)
        self.timestamps = {}
        self.frame_count = 0
        self.start_time = time.time()

        # Statistics
        self.total_points = 14640946  # Known from earlier

    def image_callback(self, msg):
        """Callback when image is received"""
        self.frame_count += 1

        # Check if we have a matching publish time
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)

        if key in self.timestamps:
            latency = time.time() - self.timestamps[key]
            self.latencies.append(latency * 1000)  # Convert to ms
            del self.timestamps[key]

    def publish_plane(self, a, b, c, d):
        """Publish a plane and record timestamp"""
        msg = Float64MultiArray()
        msg.data = [a, b, c, d]

        # Record timestamp
        key = (int(time.time()), int((time.time() % 1) * 1e9))
        self.timestamps[key] = time.time()

        self.publisher.publish(msg)

    def run_benchmark(self, duration=30, interval=0.5):
        """Run benchmark for specified duration"""
        print("=" * 70)
        print("🚀 PROJECTION PLANE NODE - PERFORMANCE BENCHMARK")
        print("=" * 70)
        print(f"\n📊 Test Parameters:")
        print(f"  Duration: {duration} seconds")
        print(f"  Plane update interval: {interval} seconds")
        print(f"  Total points: {self.total_points:,}")
        print(f"\nStarting benchmark...\n")

        start = time.time()
        count = 0

        try:
            while time.time() - start < duration:
                # Generate random plane
                a = np.sin(count) * 0.5
                b = np.cos(count) * 0.5
                c = np.sqrt(1 - a**2 - b**2) if (a**2 + b**2) < 1 else 0.7071
                d = np.sin(count * 0.5)

                self.publish_plane(a, b, c, d)
                count += 1

                # Spin to process callbacks
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(interval)

        except KeyboardInterrupt:
            pass

        # Final spin to collect remaining messages
        print("\nWaiting for final messages...")
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.print_results(duration, count)

    def print_results(self, duration, planes_sent):
        """Print benchmark results"""
        elapsed = time.time() - self.start_time

        print("\n" + "=" * 70)
        print("📈 BENCHMARK RESULTS")
        print("=" * 70)

        print(f"\n⏱️  TIMING:")
        print(f"  Elapsed time: {elapsed:.2f} seconds")
        print(f"  Planes sent: {planes_sent}")
        print(f"  Images received: {self.frame_count}")

        if self.latencies:
            latencies_ms = list(self.latencies)
            print(f"\n📊 LATENCY (plane → image):")
            print(f"  Min: {np.min(latencies_ms):.2f} ms")
            print(f"  Max: {np.max(latencies_ms):.2f} ms")
            print(f"  Mean: {np.mean(latencies_ms):.2f} ms")
            print(f"  Median: {np.median(latencies_ms):.2f} ms")
            print(f"  Std Dev: {np.std(latencies_ms):.2f} ms")
            print(f"  P95: {np.percentile(latencies_ms, 95):.2f} ms")
            print(f"  P99: {np.percentile(latencies_ms, 99):.2f} ms")

        if self.frame_count > 0:
            fps = self.frame_count / elapsed
            print(f"\n⚡ THROUGHPUT:")
            print(f"  Frame rate: {fps:.2f} FPS")
            print(f"  Points processed: {self.total_points * fps:,.0f} points/sec")
            print(f"  Total points processed: {self.total_points * self.frame_count:,}")

        if planes_sent > 0:
            response_rate = (self.frame_count / planes_sent) * 100
            print(f"\n📍 RESPONSE:")
            print(f"  Response rate: {response_rate:.1f}%")
            print(f"  Planes sent: {planes_sent}")
            print(f"  Images generated: {self.frame_count}")

        print("\n" + "=" * 70)

def main():
    benchmark = ProjectionBenchmark()

    # Run for 30 seconds with 0.5s interval between planes
    benchmark.run_benchmark(duration=30, interval=0.5)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
