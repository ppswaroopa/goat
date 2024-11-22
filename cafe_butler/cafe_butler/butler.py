import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
import threading

class CafeButler(Node):
    def __init__(self):
        super().__init__('cafe_butler')
        self.get_logger().info("Cafe Butler Initialized")

        # Subscriptions
        self.create_subscription(String, '/order', self.handle_order, 10)
        self.create_subscription(String, '/cancel_order', self.cancel_order, 10)

        # Status Publisher
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # State and Task Management
        self.state = 'Idle'
        self.order_queue = deque()
        self.current_task = None
        self.task_timeout = 1  # seconds

        # Timer
        self.timer = None
        self.lock = threading.Lock()

    def handle_order(self, msg):
        table_number = msg.data
        self.get_logger().info(f"Received order for {table_number}")
        with self.lock:
            self.order_queue.append(table_number)
            self.get_logger().info(f"Remaining orders: {len(self.order_queue)}")
        if self.state == 'Idle':
            self.process_next_order()

    def cancel_order(self, msg):
        table_number = msg.data
        self.get_logger().info(f"Cancellation received for {table_number}")

        # Only removes the recent matching order.
        # TODO: Needs flexibility to cancel the correct order (order number?)
        with self.lock:
            # Cancel current task if it matches
            if self.current_task == table_number:
                self.get_logger().info(f"Canceling current task for {table_number}")
                if self.state == 'NavigateToTable':
                    self.state = 'ReturnHome'
                elif self.state == 'NavigateToKitchen':
                    self.state = 'ReturnHome'
                self.execute_task()
            else:
                # Remove from the queue
                if table_number in self.order_queue:
                    self.order_queue.remove(table_number)
                    self.get_logger().info(f"Order for {table_number} removed from queue")

    def process_next_order(self):
        # TODO: Check if the access is safe in all loads
        # with self.lock: 
        # self.get_logger().info("NEXT ORDER")
        if self.order_queue:
            self.current_task = self.order_queue.popleft()
            self.state = 'NavigateToKitchen'
            self.execute_task()
        else:
            self.state = 'Idle'
            self.get_logger().info("No more orders. Idling.")

    def execute_task(self):
        if self.state == 'NavigateToKitchen':
            self.navigate_to_kitchen()
        elif self.state == 'WaitForConfirmation':
            self.wait_for_confirmation(location="Kitchen")
        elif self.state == 'NavigateToTable':
            self.navigate_to_table()
        elif self.state == 'WaitAtTable':
            self.wait_for_confirmation(location=f"{self.current_task}")
        elif self.state == 'ReturnToKitchen':
            self.return_to_kitchen()
        elif self.state == 'ReturnHome':
            self.return_home()

    def navigate_to_kitchen(self):
        self.get_logger().info("Navigating to Kitchen...")
        self.state = 'WaitForConfirmation'
        self.execute_task()

    def wait_for_confirmation(self, location):
        self.get_logger().info(f"Waiting for confirmation at {location}...")
        self.timer = threading.Timer(self.task_timeout, self.on_timeout)
        self.timer.start()

    def on_timeout(self):
        self.get_logger().info("Timeout. Moving to next step.")
        with self.lock:
            if self.state == 'WaitForConfirmation':
                self.state = 'NavigateToTable'
            elif self.state == 'WaitAtTable':
                self.state = 'ReturnHome'
            self.execute_task()

    def navigate_to_table(self):
        self.get_logger().info(f"Delivering food to {self.current_task}...")
        self.state = 'WaitAtTable'
        self.execute_task()

    def return_to_kitchen(self):
        self.get_logger().info("Returning to Kitchen due to cancellation...")
        self.state = 'ReturnHome'
        self.execute_task()

    def return_home(self):
        self.get_logger().info("Returning to Home Position...")
        self.current_task = None
        self.timer = None
        self.state = "Idle"
        self.process_next_order()

def main(args=None):
    rclpy.init(args=args)
    cafe_butler = CafeButler()
    rclpy.spin(cafe_butler)
    cafe_butler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
