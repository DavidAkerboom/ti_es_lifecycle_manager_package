# ti_es_lifecycle_manager_package

This is a simple lifecycle manager node for ROS 2, designed to control and monitor the state transitions of other lifecycle-enabled nodes in your system (e.g., sensor drivers, logging tools, etc.).

It uses the standard ROS 2 lifecycle state machine to:
- Automatically configure and activate lifecycle nodes on startup
- Listen for commands (via a topic) to dynamically activate or deactivate those nodes during runtime

<small>

For more information about lifecycle nodes, see the following links:
- https://design.ros2.org/articles/node_lifecycle.html
- https://foxglove.dev/blog/how-to-use-ros2-lifecycle-nodes

</small>

---

## ✅ Features

- Automatically discovers and manages a list of lifecycle nodes
- Subscribes to a topic (`ti/es/change_state`) to trigger transitions based on runtime events
- Gracefully handles configuration, activation, deactivation, and cleanup transitions

---

## 🚀 Lifecycle Manager Usage

### 1. Add Lifecycle Nodes
In the `LifecycleManager` Python file, update the list of managed nodes:
```python
self.lifecycle_nodes = [
    'temperature_humidity_node',
    'logger_node'
    # other nodes
]
```

### 2. Run the Lifecycle Manager
```bash
ros2 run ti_es_lifecycle_manager_package ti_es_lifecycle_manager_node
```

### 3. Trigger Lifecycle Transitions
You can control the lifecycle nodes by publishing to the topic `ti/es/change_state`:

```bash
ros2 topic pub --once /ti/es/change_state std_msgs/msg/String "{data: 'deactivate'}"
ros2 topic pub --once /ti/es/change_state std_msgs/msg/String "{data: 'activate'}"
```

---

## 🔄 Change a Regular Node into a Lifecycle Node

To convert a standard ROS 2 node into a lifecycle node:

### 1. Import the lifecycle API
Replace `from rclpy.node import Node` with:
```python
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
```

### 2. Update the class declaration
Inherit from `rclpy.lifecycle.Node` instead of `rclpy.node.Node` by adding `**kwargs` in the following lines:

```python
class MyLifecycleNode(Node):
    def __init__(self, **kwargs):
        super().__init__('my_lifecycle_node', **kwargs)
```

### 3. Implement Lifecycle Callbacks
Define the following methods in your node class and move code from `__init__` to the according method (`on_configure` for one-time use or `on_activate` for lines which should be disabled when not needed):

```python
def on_configure(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info("Activating...")
    
    # Initialize publishers, services, or one-time setup here
    self.log_publisher = self.create_publisher(String, "ti/es/log_data", 10)        

    return TransitionCallbackReturn.SUCCESS


def on_activate(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info("Activating...")
    
    # Start timers or subscriptions
    self.timer = self.create_timer(I2C_TIMER, self.timer_callback)

    return super().on_activate(state)


def on_deactivate(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info("Deactivating...")

    # Stop timers or other activity
    self.destroy_timer(self.timer_callback)

    return super().on_deactivate(state)


def on_cleanup(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info("Cleaning up...")

    # Destroy publishers or clean resources
    self.destroy_publisher(self.log_publisher)

    return TransitionCallbackReturn.SUCCESS


def on_shutdown(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info("Shutting down...")

    # Final cleanup if needed
    self.destroy_timer(self.timer_callback)
    self.destroy_publisher(self.log_publisher)

    return TransitionCallbackReturn.SUCCESS
```

For examples, look at the [temperature_humidity_node](https://github.com/DavidAkerboom/ti_es_temperature_humidity_package/blob/lifecycle/ti_es_temperature_humidity_package/ti_es_temperature_humidity_node.py) in the branch `lifecycle` or the [ros2 lifecycle example](https://github.com/ros2/demos/tree/jazzy/lifecycle_py).


## Terminal commands for debugging
After running a lifecycle node, the state can be checked from a seperate terminal and manually changed with the commands below:

(Replace `<lifecycle_node>` with node name defined in `super().__init__`)

view all lifecycle nodes:
```bash
ros2 lifecycle nodes
```

view state:
```bash
ros2 lifecycle get /<lifecycle_node>
```

check available transitions:
```bash
ros2 lifecycle list /<lifecycle_node>
```

change state:
```bash
ros2 lifecycle set /<lifecycle_node> <state>
```
