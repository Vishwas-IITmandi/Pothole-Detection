import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch
import os
from ament_index_python.packages import get_package_share_directory

class YoloSubscriber(Node):
    """
    A ROS 2 node that subscribes to an image topic, performs YOLOv8 inference
    using a custom model, and displays the results.
    """
    def __init__(self):
        super().__init__('yolo_subscriber')
        
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.br = CvBridge()
        
        # --- Correctly locate and load the YOLOv8 Model ---
        package_share_directory = get_package_share_directory('yolo_subscriber')
        model_path = os.path.join(package_share_directory, 'models', 'yolov8s.pt')
        self.get_logger().info(f'Attempting to load model from: {model_path}')

        if not os.path.exists(model_path):
            self.get_logger().error(f"FATAL: Model file not found at {model_path}")
            rclpy.shutdown()
            return

        try:
            self.model = YOLO(model_path)
            # Log whether using CPU or GPU
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.get_logger().info(f'YOLOv8 model loaded successfully on {device}.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv8 model: {e}')
            rclpy.shutdown()
            return

    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        """
        try:
            current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Perform YOLOv8 inference
        results = self.model.predict(source=current_frame, verbose=False)
        
        # Use the .plot() method to get an annotated image
        rendered_frame = results[0].plot() 
        
        # Display the resulting frame in a window
        cv2.imshow("YOLOv8 Inference", rendered_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolo_subscriber = YoloSubscriber()
    try:
        rclpy.spin(yolo_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_subscriber.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

