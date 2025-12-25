#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateChecker(Node):
    def __init__(self):
        super().__init__('joint_state_checker')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.count = 0

    def listener_callback(self, msg):
        self.count += 1
        if self.count % 10 == 1:  # Print every 10th message
            print(f"\n=== Joint States ===")
            for i, name in enumerate(msg.name):
                pos = msg.position[i] if i < len(msg.position) else "N/A"
                vel = msg.velocity[i] if i < len(msg.velocity) else "N/A"
                print(f"{name:15s}: pos={pos:12.6f}, vel={vel}")

def main(args=None):
    rclpy.init(args=args)
    checker = JointStateChecker()
    print("Monitoring joint states (Ctrl+C to stop)...")
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


