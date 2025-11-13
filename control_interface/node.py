from rclpy.node import Node
from geometry_msgs.msg import Point 
from ap1_msgs.msg import VehicleSpeedStamped, TurnAngleStamped, MotorPowerStamped

class AP1SystemInterfaceNode(Node):
    def __init__(self):
        super().__init__('ap1_debug_ui')

        # publishers
        self.speed_pub = self.create_publisher(VehicleSpeedStamped, '/ap1/control/target_speed', 10)
        self.target_location_pub = self.create_publisher(Point, '/ap1/control/target_location', 10)

        # subscribers
        self.speed_sub = self.create_subscription(VehicleSpeedStamped, '/ap1/actuation/speed_actual', self.speed_callback, 10)
        self.turn_angle_sub = self.create_subscription(TurnAngleStamped, '/ap1/actuation/turn_angle_actual', self.turn_angle_callback, 10)
        self.current_motor_power = self.create_subscription(MotorPowerStamped, '/ap1/control/motor_power', self.motor_power_callback, 10)

        self.current_speed = 0.0 # m
        self.target_speed = 0.0 # m/s
        self.motor_power = 0.0 # [-1, 1]
        self.target_location = (0.0, 0.0) # m
        self.current_turn_angle = 0.0 # rads

    def speed_callback(self, msg: VehicleSpeedStamped):
        self.current_speed = msg.speed

    def turn_angle_callback(self, msg: TurnAngleStamped):
        self.current_turn_angle = msg.angle
    
    def motor_power_callback(self, msg: MotorPowerStamped):
        self.motor_power = msg.power
    
    def set_target_speed(self, speed: float):
        self.target_speed = speed

        # assemble msg
        msg = VehicleSpeedStamped()
        msg.speed = speed 
        msg.header.stamp = self.get_clock().now()

        # send out msg
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Sent target speed: {speed}')

    def set_target_location(self, x: float, y: float):
        self.target_location = (x, y)
        msg = Point()
        msg.x = x
        msg.y = y 
        msg.z = 0.0
        self.target_location_pub.publish(msg)
        self.get_logger().info(f'Sent target location: ({x}, {y})')