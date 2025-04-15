import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from std_msgs.msg import String
from dataclasses import dataclass

@dataclass
class Node:
    name: str
    active_when_moving: bool
    active_when_idle: bool
    active_when_sleep: bool


class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager')

        # List of lifecycle nodes to manage
        self.lifecycle_nodes = [
            # 'temperature_humidity_node',
            # 'logger_node'
            Node(name='lc_talker', active_when_moving=1, active_when_idle=0, active_when_sleep=0)
        ]

        self.subscription = self.create_subscription(String, 'ti/es/change_state', self.listener_callback, 10)

        self.change_state_clients = {}
        self.get_logger().info('Waiting for lifecycle services...')

        for node_name in self.lifecycle_nodes:
            self.setup_state_client(node_name)


    def configure(self, node_name):
        self.get_logger().info(f'Configuring {node_name}...')
        self.change_state(self.change_state_clients[node_name], Transition.TRANSITION_CONFIGURE)

    def activate(self, node_name):
        self.get_logger().info(f'Activating {node_name}...')
        self.change_state(self.change_state_clients[node_name], Transition.TRANSITION_ACTIVATE)
        
    def deactivate(self, node_name):
        self.get_logger().info(f'Deactivating {node_name}...')
        self.change_state(self.change_state_clients[node_name], Transition.TRANSITION_DEACTIVATE)
    
    def cleanup(self, node_name):
        self.get_logger().info(f'Cleaning up {node_name}...')
        self.change_state(self.change_state_clients[node_name], Transition.TRANSITION_CLEANUP)

    def shutdown(self, node_name):
        self.get_logger().info(f'Shutting down {node_name}...')
        self.change_state(self.change_state_clients[node_name], Transition.TRANSITION_SHUTDOWN)


    # listens for commands and sets lifecycle nodes accordingly 
    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        
        match msg.data:
            case "set_moving":
                self._update_nodes(condition="active_when_moving")
            case "set_idle":
                self._update_nodes(condition="active_when_idle")
            case "set_sleep":
                self._update_nodes(condition="active_when_sleep")
            case "configure_all":
                self._update_all_nodes(action="configure")
            case "activate_all":
                self._update_all_nodes(action="activate")
            case "deactivate_all":
                self._update_all_nodes(action="deactivate")
            case "cleanup_all":
                self._update_all_nodes(action="cleanup")
            case "shutdown_all":
                self._update_all_nodes(action="shutdown")


    # sets node states according to the mount state received from the callback
    def _update_nodes(self, condition: str):
        for node in self.lifecycle_nodes:
            if getattr(node, condition):
                self.configure(node.name)
                self.activate(node.name)
            else:
                self.deactivate(node.name)
                self.cleanup(node.name)


    # sets all nodes to the state received from the callback
    def _update_all_nodes(self, action: str):
        for node in self.lifecycle_nodes:
            method = getattr(self, action, None)
            if callable(method):
                method(node.name)
            else:
                self.get_logger().warn(f"No method named '{action}' found on self")


    # sets up a lifecycle service client for changing the state of another node
    def setup_state_client(self, node_name):
        change_state_client = self.create_client(ChangeState, f'{node_name}/change_state')
        self.change_state_clients[node_name] = change_state_client

        while not change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for {node_name}/change_state service...')


    # checks whether node state was succesful
    def _handle_transition_result(self, future, transition_id, node_name=None):
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Transition {transition_id} successful.')
        else:
            self.get_logger().error(f'Transition {transition_id} failed.')


    # handles node state transitions
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
