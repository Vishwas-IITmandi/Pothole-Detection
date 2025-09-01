import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    """
    A ROS 2 node that captures video from a webcam and publishes it as Image messages.
    """
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Declare parameters that can be changed on launch
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_rate', 30.0)

        # Get the parameter values
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Create the publisher on the 'image_raw' topic
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        
        # Create a timer to trigger the callback at the specified rate
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize the webcam capture
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video stream from camera index {camera_index}')
            rclpy.shutdown() # Shutdown if camera is not available

        # Initialize the bridge between ROS 2 Images and OpenCV Images
        self.br = CvBridge()
        self.get_logger().info(f'Webcam Publisher node started. Publishing at {publish_rate} Hz.')

    def timer_callback(self):
        """
        Callback function for the timer. Reads a frame from the webcam,
        converts it to a ROS Image message, and publishes it.
        """
        ret, frame = self.cap.read()
        
        if ret:
            # Convert the OpenCV frame to a ROS Image message and publish
            image_message = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            image_message.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(image_message)
        else:
            self.get_logger().warn('Failed to capture frame from webcam.')

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources before shutting down
        webcam_publisher.cap.release()
        webcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

