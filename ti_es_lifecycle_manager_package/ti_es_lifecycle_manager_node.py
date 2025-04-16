import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState, GetState, GetAvailableTransitions
from lifecycle_msgs.msg import Transition, State


from std_msgs.msg import String
from dataclasses import dataclass

TRANSITION_LABELS = {
    Transition.TRANSITION_CREATE: "CREATE",
    Transition.TRANSITION_CONFIGURE: "CONFIGURE",
    Transition.TRANSITION_CLEANUP: "CLEANUP",
    Transition.TRANSITION_ACTIVATE: "ACTIVATE",
    Transition.TRANSITION_DEACTIVATE: "DEACTIVATE",
    Transition.TRANSITION_UNCONFIGURED_SHUTDOWN: "SHUTDOWN (Unconfigured)",
    Transition.TRANSITION_INACTIVE_SHUTDOWN: "SHUTDOWN (Inactive)",
    Transition.TRANSITION_ACTIVE_SHUTDOWN: "SHUTDOWN (Active)",
    Transition.TRANSITION_DESTROY: "DESTROY",
}

@dataclass
class LifecycleNode:
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
            LifecycleNode(name='lc_talker', active_when_moving=1, active_when_idle=0, active_when_sleep=0)
        ]

        self.subscription = self.create_subscription(String, 'ti/es/change_state', self.listener_callback, 10)

        self.change_state_clients = {}
        self.get_state_clients = {}
        self.get_transitions_clients = {}
        self.get_logger().info('Waiting for lifecycle services...')

        for node in self.lifecycle_nodes:
            self.setup_state_client(node.name)

        self._update_all_nodes(action="configure")


    def configure(self, node_name):
        self.get_logger().info(f'Configuring {node_name}...')
        self.get_state(self.get_state_clients[node_name], node_name, Transition.TRANSITION_CONFIGURE)

    def activate(self, node_name):
        self.get_logger().info(f'Activating {node_name}...')
        self.get_state(self.get_state_clients[node_name], node_name, Transition.TRANSITION_ACTIVATE)
        
    def deactivate(self, node_name):
        self.get_logger().info(f'Deactivating {node_name}...')
        self.get_state(self.get_state_clients[node_name], node_name, Transition.TRANSITION_DEACTIVATE)
    
    def cleanup(self, node_name):
        self.get_logger().info(f'Cleaning up {node_name}...')
        self.get_state(self.get_state_clients[node_name], node_name, Transition.TRANSITION_CLEANUP)
    # def shutdown(self, node_name):
    #     self.get_logger().info(f'Shutting down {node_name}...')
    #     self.change_state(self.change_state_clients[node_name], Transition.TRANSITION_SHUTDOWN)


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
            # case "shutdown_all":
            #     self._update_all_nodes(action="shutdown")


    # sets node states according to the mount state received from the callback
    def _update_nodes(self, condition: str):
        for node in self.lifecycle_nodes:
            if getattr(node, condition):
                # self.configure(node.name)
                self.activate(node.name)
            else:
                self.deactivate(node.name)
                # self.cleanup(node.name)


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
        # Setup change_state client
        change_state_client = self.create_client(ChangeState, f'{node_name}/change_state')
        self.change_state_clients[node_name] = change_state_client

        # Setup get_state client
        get_state_client = self.create_client(GetState, f'{node_name}/get_state')
        self.get_state_clients[node_name] = get_state_client

        # Setup get_available_transitions client
        get_transitions_client = self.create_client(GetAvailableTransitions, f'{node_name}/get_available_transitions')
        self.get_transitions_clients[node_name] = get_transitions_client

        while not change_state_client.wait_for_service(timeout_sec=1.0) or \
            not get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for services of {node_name}...')


    # check the current state
    def get_state(self, client, node_name, transition_id):
        req = GetState.Request()

        future = client.call_async(req)
        future.add_done_callback(
            lambda fut: self._handle_get_state_result(fut, node_name, transition_id)
        )
    def _handle_get_state_result(self, future, node_name, transition_id):
        try:
            response = future.result()
            if response is not None:
                state_id = response.current_state.id
                state_label = response.current_state.label
                self.get_logger().info(f"Node '{node_name}' is currently in state [{state_id}] {state_label}")
                self.get_available_transitions(self.get_transitions_clients[node_name], node_name, transition_id)
            else:
                self.get_logger().error(f"Failed to get state for node '{node_name}'")
        except Exception as e:
            self.get_logger().error(f"Exception while getting state for node '{node_name}': {e}")


    # check the available transition and if transition is available, calls change_state
    def get_available_transitions(self, client, node_name, transition_id):
        req = GetAvailableTransitions.Request()

        future = client.call_async(req)
        future.add_done_callback(
            lambda fut: self._handle_transitions_result(fut, node_name, transition_id)
        )
    def _handle_transitions_result(self, future, node_name, transition_id):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f"Available transitions for node '{node_name}':")
                available_ids = []

                for t in response.available_transitions:
                    tid = t.transition.id
                    label = t.transition.label
                    available_ids.append(tid)
                    self.get_logger().info(f"  [{tid}] {label}")

                if transition_id in available_ids:
                    self.get_logger().info(f"Transition [{transition_id}] is available. Proceeding with change_state.")
                    self.change_state(self.change_state_clients[node_name], transition_id, node_name)
                else:
                    label = TRANSITION_LABELS.get(transition_id, "UNKNOWN")
                    self.get_logger().warn(
                        f"Requested transition [{transition_id}] {label} is NOT available for node '{node_name}'. Skipping."
                    )
            else:
                self.get_logger().error(f"Failed to get transitions for node '{node_name}'")
        except Exception as e:
            self.get_logger().error(f"Exception while getting transitions for node '{node_name}': {e}")


    # handles node state transitions
    def change_state(self, client, transition_id, node_name):
        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        future.add_done_callback(
            lambda fut: self._handle_transition_result(fut, transition_id, node_name)
        )
    def _handle_transition_result(self, future, transition_id, node_name):
        try:
            response = future.result()
            label = TRANSITION_LABELS.get(transition_id, "UNKNOWN")
            if response is not None and response.success:
                self.get_logger().info(f"Transition [{transition_id}] {label} for node '{node_name}' was successful.")
            else:
                self.get_logger().error(f"Transition [{transition_id}] {label} for node '{node_name}' failed.")
        except Exception as e:
            self.get_logger().error(f"Exception during transition [{transition_id}] for node '{node_name}': {e}")


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
