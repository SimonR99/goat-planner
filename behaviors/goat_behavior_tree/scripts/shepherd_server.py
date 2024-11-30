import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from typing import Dict, Tuple
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import open3d as o3d
import os

from shepherd import Shepherd, ShepherdConfig
from shepherd.utils.camera import CameraUtils

class LiveVideoProcessor(Node):
    def __init__(self):
        super().__init__('live_video_processor')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize Shepherd config with camera parameters for ZED camera
        self.config = ShepherdConfig(
            camera_height=0.4,  # Camera height from ground in meters
            camera_pitch=-1.57  # -90 degrees in radians
        )
        
        # Update camera parameters for ZED camera with explicit rotations
        self.config.camera = CameraUtils(
            width=1344,         # ZED camera width
            height=376,         # ZED camera height
            fov=1.88,          # ZED camera FOV in radians
            camera_height=0.4,  # Camera height from ground
            camera_pitch=3.14,
            camera_yaw=-1.57,
            camera_roll=1.57
        )
        
        # Print camera configuration
        print(f"\nZED Camera parameters:")
        print(f"Image size: {self.config.camera.width}x{self.config.camera.height}")
        print(f"FOV: {np.degrees(self.config.camera.fov):.1f} degrees")
        print(f"Focal length: fx={self.config.camera.fx:.1f}, fy={self.config.camera.fy:.1f}")
        print(f"Principal point: cx={self.config.camera.cx:.1f}, cy={self.config.camera.cy:.1f}")
        print(f"Camera height: {self.config.camera.camera_height:.2f}m")
        print(f"Camera pitch: {np.degrees(self.config.camera.camera_pitch):.1f} degrees")
        print(f"Camera yaw: {np.degrees(self.config.camera.camera_yaw):.1f} degrees")
        print(f"Camera roll: {np.degrees(self.config.camera.camera_roll):.1f} degrees")
        
        # Initialize Shepherd with config
        self.shepherd = Shepherd(config=self.config)
        
        # Create synchronized subscribers
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            '/zed/zed_node/rgb/image_rect_color'
        )
        
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/zed/zed_node/depth/depth_registered'
        )
        
        self.pose_sub = message_filters.Subscriber(
            self,
            Odometry,
            '/odometry/local'
        )
        
        # Create synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.pose_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        # Frame processing rate control
        self.frame_skip = 2
        self.frame_count = 0
        
        # Add window configuration
        cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Object Detection', 1344, 376)
        
        # Add point cloud publishers
        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            '/shepherd/object_positions',
            10
        )
        self.object_clouds_pub = self.create_publisher(
            PointCloud2,
            '/shepherd/object_point_clouds',
            10
        )
        
        # Add timer for publishing point clouds
        self.create_timer(1.0, self.publish_point_clouds)
        
        # Add color map for objects when no query is active
        self.color_map = {}  # Will store object_id -> color mapping
        self.color_palette = [  # Predefined distinct colors (BGR format)
            (255, 0, 0),    # Blue
            (0, 255, 0),    # Green
            (0, 0, 255),    # Red
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
            (128, 0, 0),    # Dark Blue
            (0, 128, 0),    # Dark Green
            (0, 0, 128),    # Dark Red
            (128, 128, 0),  # Dark Cyan
            (128, 0, 128),  # Dark Magenta
            (0, 128, 128),  # Dark Yellow
        ]
        self.next_color_idx = 0
        
        # Add output directory creation
        self.output_dir = os.path.join(os.path.dirname(__file__), '..', 'demo')
        os.makedirs(self.output_dir, exist_ok=True)

    def pose_to_dict(self, pose_msg: Odometry) -> Dict:
        """Convert Odometry to dictionary"""
        return {
            'x': pose_msg.pose.pose.position.x,
            'y': pose_msg.pose.pose.position.y,
            'z': pose_msg.pose.pose.position.z,
            'qx': pose_msg.pose.pose.orientation.x,
            'qy': pose_msg.pose.pose.orientation.y,
            'qz': pose_msg.pose.pose.orientation.z,
            'qw': pose_msg.pose.pose.orientation.w
        }

    def get_object_color(self, object_id: str, similarity: float = None) -> Tuple[int, int, int]:
        """Get color based on query similarity or assign distinct color if no query."""
        if similarity is not None:  # If we have a query and similarity score
            # Convert similarity [0,1] to intensity [0,255]
            intensity = int(similarity * 255)
            # Return black-to-red gradient (BGR format)
            return (0, 0, intensity)
        else:  # No query, use distinct colors
            if object_id not in self.color_map:
                # Assign new color
                color = self.color_palette[self.next_color_idx % len(self.color_palette)]
                self.color_map[object_id] = color
                self.next_color_idx += 1
            return self.color_map[object_id]

    def synchronized_callback(self, image_msg: Image, depth_msg: Image, pose_msg: Odometry):
        """Process synchronized messages"""
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        try:
            # Convert ROS messages to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            
            # Convert pose message to dictionary
            camera_pose = self.pose_to_dict(pose_msg)

            print(f"Camera pose: {camera_pose}")
            
            # Process frame using Shepherd
            results = self.shepherd.process_frame(cv_image, cv_depth, camera_pose)
            
            # Create visualization
            viz_frame = cv_image.copy()
            
            # Draw detections and masks
            for result in results:
                bbox = result['detection']['bbox']
                mask = result['mask']
                object_id = result.get('object_id')
                similarity = result.get('similarity')  # Get similarity if query exists
                
                # Get color based on whether we have a query
                if self.config.default_query != "":  # If we have an active query
                    color = self.get_object_color(object_id, similarity)
                else:  # No query, use distinct colors
                    color = self.get_object_color(object_id)
                
                # Draw bounding box
                x1, y1, x2, y2 = map(int, bbox)
                cv2.rectangle(viz_frame, (x1, y1), (x2, y2), color, 2)
                
                # Draw mask with object's color
                mask_overlay = viz_frame.copy()
                mask_overlay[mask] = color
                viz_frame = cv2.addWeighted(viz_frame, 0.7, mask_overlay, 0.3, 0)
                
                # Add object ID and similarity text
                if object_id is not None:
                    text = f"ID: {object_id}"
                    if similarity is not None:
                        text += f" ({similarity:.2f})"
                    cv2.putText(viz_frame, text, (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Add query information to visualization
            query_text = f"Current query: {self.config.default_query}"
            cv2.putText(viz_frame, query_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, (255, 255, 255), 2)
            cv2.putText(viz_frame, "Press 'k' to change query", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Resize and display
            viz_frame = cv2.resize(viz_frame, (1344, 376))
            cv2.imshow('Object Detection', viz_frame)
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.destroy_node()
                rclpy.shutdown()
                cv2.destroyAllWindows()
            elif key == ord('k'):              
                # Get query from terminal
                new_query = input("Enter new query: ")
                self.shepherd.update_query(new_query)  # Use the new update_query method
                print(f"Query updated to: {new_query}")
                
                # Close query input window
                cv2.destroyWindow('Query Input')
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

    def publish_point_clouds(self):
        """Publish point clouds with similarity colors and save to PLY"""
        try:
            # Query objects with current query
            results = self.shepherd.database.query_objects(
                self.config.default_query, 
                self.shepherd.embedder
            )
            
            if not results:
                return
                
            # Create headers
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'map'
            
            # Create point cloud fields
            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ]
            
            # Collect all points and their intensities
            all_points = []
            all_intensities = []
            
            for result in results:
                point_cloud = result.get('point_cloud')
                if point_cloud is not None and len(point_cloud) > 0:
                    similarity = result.get('similarity', 0.0)
                    
                    all_points.extend(point_cloud)
                    all_intensities.extend([similarity] * len(point_cloud))
            
            if all_points:
                # Convert to numpy arrays
                all_points = np.array(all_points)
                all_intensities = np.array(all_intensities)
                
                # Create and publish ROS point cloud
                points_with_intensity = np.column_stack((all_points, all_intensities))
                pc2_msg = pc2.create_cloud(header, fields, points_with_intensity)
                self.object_clouds_pub.publish(pc2_msg)
                
                # Save point cloud using database wrapper
                ply_path = os.path.join(self.output_dir, 'ros_point_cloud.ply')
                self.shepherd.database.save_point_cloud_ply(ply_path)
                self.get_logger().info(f'Saved point cloud to {ply_path}')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing point clouds: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LiveVideoProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()