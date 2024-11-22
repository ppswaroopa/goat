import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time


class OrderSimulator(Node):
    def __init__(self):
        super().__init__('order_simulator')

        # Publishers
        self.order_pub = self.create_publisher(String, '/order', 10)
        self.cancel_pub = self.create_publisher(String, '/cancel_order', 10)

        # Simulation parameters
        self.table_numbers = ['Table1', 'Table2', 'Table3']
        self.order_rate = 1  # Orders per second
        self.cancellation_probability = 0.1  # 10% chance to cancel an order

        self.get_logger().info("Order Simulator Initialized")

        # Start the simulation
        self.simulate_orders()

    def simulate_orders(self):
        order_count = 0
        while rclpy.ok() and order_count<10:
            # Generate a new order
            table = random.choice(self.table_numbers)
            order_msg = String()
            order_msg.data = table
            self.order_pub.publish(order_msg)
            self.get_logger().info(f"Published order for {table}")
            order_count += 1

            # Randomly decide whether to cancel an order
            if random.random() < self.cancellation_probability:
                cancel_msg = String()
                cancel_msg.data = table
                self.cancel_pub.publish(cancel_msg)
                self.get_logger().info(f"Published cancellation for {table}")

            # Sleep to maintain the order rate
            time.sleep(1 / self.order_rate)


def main(args=None):
    rclpy.init(args=args)
    simulator = OrderSimulator()
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info("Shutting down Order Simulator")
    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
