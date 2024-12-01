import rclpy
from rclpy.node import Node
from your_msgs.msg import WorldObject  # Define your message type
from goat_controller import GoatController

class WorldStateNode(Node):
    def __init__(self):
        super().__init__('world_state_node')
        
        self.goat_controller = GoatController()
        
        # Subscribe to world state updates
        self.subscription = self.create_subscription(
            WorldObject,
            'world_objects',
            self.object_callback,
            10
        )

    def object_callback(self, msg):
        # Convert ROS2 message to GoatState format
        position = {
            'x': msg.position.x,
            'y': msg.position.y,
            'z': msg.position.z
        }
        
        properties = {
            'color': msg.color,
            'size': msg.size,
            # Add other properties as needed
        }
        
        # Update the state through GoatController
        self.goat_controller.update_world_object(
            obj_id=msg.id,
            obj_type=msg.type,
            position=position,
            properties=properties
        )

def main(args=None):
    rclpy.init(args=args)
    node = WorldStateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 