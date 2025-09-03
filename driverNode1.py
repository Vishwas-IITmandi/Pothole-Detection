import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO 

class RobotDriver(Node):
    """
    A ROS 2 node that subscribes to /cmd_vel and controls robot motors via GPIO
    using two BTS7960 motor drivers.
    """
    def __init__(self):
        super().__init__('robot_driver')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.declare_parameter('speed_percentage', 70)
        
        self.get_logger().info('Setting up GPIO pins...')
        # Left side motors
        self.LEFT_RPWM = 17  # Forward
        self.LEFT_LPWM = 27  # Backward

        # Right side motors
        self.RIGHT_RPWM = 23 # Forward
        self.RIGHT_LPWM = 22 # Backward

        GPIO.setmode(GPIO.BCM)
        pins = [self.LEFT_RPWM, self.LEFT_LPWM, self.RIGHT_RPWM, self.RIGHT_LPWM]
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)

        self.pwm_left_fwd = GPIO.PWM(self.LEFT_RPWM, 1000)
        self.pwm_left_bwd = GPIO.PWM(self.LEFT_LPWM, 1000)
        self.pwm_right_fwd = GPIO.PWM(self.RIGHT_RPWM, 1000)
        self.pwm_right_bwd = GPIO.PWM(self.RIGHT_LPWM, 1000)

        for pwm in [self.pwm_left_fwd, self.pwm_left_bwd, self.pwm_right_fwd, self.pwm_right_bwd]:
            pwm.start(0)

        self.get_logger().info('Robot Driver Node Started. Listening for /cmd_vel commands.')

    def listener_callback(self, msg):
        """
        Callback function for the /cmd_vel subscriber.
        Translates Twist messages into motor commands.
        """
        speed_percentage = self.get_parameter('speed_percentage').get_parameter_value().integer_value

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Differential drive kinematics
        left_speed = (linear_vel - angular_vel) * speed_percentage
        right_speed = (linear_vel + angular_vel) * speed_percentage
        
        # --- THE FIX ---
        # Invert the right motor speed to match its physical wiring.
        # This flips the direction of the right motor to correct the jumbled movement.
        right_speed = -right_speed

        # Clamp the speeds to be between -100 and 100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        self.get_logger().info(f"Received Twist: Linear={linear_vel:.2f}, Angular={angular_vel:.2f} -> Speeds: L={left_speed:.0f}%, R={right_speed:.0f}%")

        self.set_motor_speeds(left_speed, right_speed)

    def set_motor_speeds(self, left_speed, right_speed):
        """
        Controls the speed and direction of left and right motor pairs.
        """
        # Control Left Side
        if left_speed > 0:
            self.pwm_left_fwd.ChangeDutyCycle(left_speed)
            self.pwm_left_bwd.ChangeDutyCycle(0)
        else:
            self.pwm_left_fwd.ChangeDutyCycle(0)
            self.pwm_left_bwd.ChangeDutyCycle(abs(left_speed))

        # Control Right Side
        if right_speed > 0:
            self.pwm_right_fwd.ChangeDutyCycle(right_speed)
            self.pwm_right_bwd.ChangeDutyCycle(0)
        else:
            self.pwm_right_fwd.ChangeDutyCycle(0)
            self.pwm_right_bwd.ChangeDutyCycle(abs(right_speed))
            
    def cleanup_gpio(self):
        """
        A dedicated function to stop motors and clean up GPIO pins.
        """
        self.get_logger().info('Cleaning up GPIO...')
        self.set_motor_speeds(0, 0) # Stop motors
        for pwm in [self.pwm_left_fwd, self.pwm_left_bwd, self.pwm_right_fwd, self.pwm_right_bwd]:
            pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    robot_driver = RobotDriver()
    try:
        rclpy.spin(robot_driver)
    except KeyboardInterrupt:
        pass
    finally:
        robot_driver.cleanup_gpio()
        robot_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
