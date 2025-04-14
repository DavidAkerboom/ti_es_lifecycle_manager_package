import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from std_msgs.msg import String

class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager')

        # List of lifecycle nodes to manage
        self.lifecycle_nodes = [
            # 'temperature_humidity_node',
            # 'logger_node'
            'lc_talker'
        ]

        self.subscription = self.create_subscription(String, 'ti/es/change_state', self.listener_callback, 10)

        self.change_state_clients = {}
        self.get_logger().info('Waiting for lifecycle services...')

        for node_name in self.lifecycle_nodes:
            self.configure_and_activate_node(node_name)

#        self.timer = self.create_timer(10.0, self.deactivate_all_nodes)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        if msg.data == "activate":
            for node_name, client in self.change_state_clients.items():
                self.get_logger().info(f'Configuring {node_name}...')
                self.change_state(client, Transition.TRANSITION_CONFIGURE)
                self.get_logger().info(f'Activating {node_name}...')
                self.change_state(client, Transition.TRANSITION_ACTIVATE)
        if msg.data == "deactivate":
            for node_name, client in self.change_state_clients.items():
                self.get_logger().info(f'Deactivating {node_name}...')
                self.change_state(client, Transition.TRANSITION_DEACTIVATE)
                self.get_logger().info(f'Cleaning up {node_name}...')
                self.change_state(client, Transition.TRANSITION_CLEANUP)

    def configure_and_activate_node(self, node_name):
        change_state_client = self.create_client(ChangeState, f'{node_name}/change_state')
        self.change_state_clients[node_name] = change_state_client

        while not change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for {node_name}/change_state service...')

        self.get_logger().info(f'Configuring {node_name}...')
        self.change_state(change_state_client, Transition.TRANSITION_CONFIGURE)

        self.get_logger().info(f'Activating {node_name}...')
        self.change_state(change_state_client, Transition.TRANSITION_ACTIVATE)

    def deactivate_all_nodes(self):
            self.get_logger().info('Deactivating all lifecycle nodes...')
            for node_name, client in self.change_state_clients.items():
                self.get_logger().info(f'Deactivating {node_name}...')
                self.change_state(client, Transition.TRANSITION_DEACTIVATE)
                self.get_logger().info(f'Cleaning up {node_name}...')
                self.change_state(client, Transition.TRANSITION_CLEANUP)
            self.destroy_timer(self.timer)

    def _handle_transition_result(self, future, transition_id, node_name=None):
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Transition {transition_id} successful.')
        else:
            self.get_logger().error(f'Transition {transition_id} failed.')

    def change_state(self, client, transition_id, node_name=None):
        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        future.add_done_callback(
            lambda fut: self._handle_transition_result(fut, transition_id, node_name)
        )


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
