from rclpy.node import Node
from geometry_msgs.msg import Point 
from ap1_msgs.msg import VehicleSpeedStamped, TurnAngleStamped, MotorPowerStamped, TargetPathStamped, SpeedProfileStamped

class AP1SystemInterfaceNode(Node):
    def __init__(self):
        super().__init__('ap1_debug_ui')

        # publishers
        self.speed_pub = self.create_publisher(VehicleSpeedStamped, '/ap1/control/target_speed', 10)
        self.target_location_pub = self.create_publisher(Point, '/ap1/control/target_location', 10)

        # subscribers
        self.speed_sub = self.create_subscription(VehicleSpeedStamped, '/ap1/actuation/speed_actual', self.speed_callback, 10)
        self.turn_angle_sub = self.create_subscription(TurnAngleStamped, '/ap1/actuation/turn_angle_actual', self.turn_angle_callback, 10)
        self.current_motor_power_sub = self.create_subscription(MotorPowerStamped, '/ap1/control/motor_power', self.motor_power_callback, 10)
        self.path_sub = self.create_subscription(TargetPathStamped, '/ap1/planning/target_path', self.target_path_callback, 10)
        self.speed_profile = self.create_subscription(SpeedProfileStamped, '/ap1/planning/speed_profile', self.speed_profile_callback, 10)

        self.current_speed = 0.0 # m
        self.target_speed = 0.0 # m/s
        self.motor_power = 0.0 # [-1, 1]
        self.target_location = (0.0, 0.0) # m
        self.current_turn_angle = 0.0 # rads
        self.target_path = [] # waypoints
        self.speed_profile = [] # m/s

    def speed_callback(self, msg: VehicleSpeedStamped):
        self.current_speed = msg.speed

    def speed_profile_callback(self, msg: SpeedProfileStamped):
        self.speed_profile = msg.speeds

    def turn_angle_callback(self, msg: TurnAngleStamped):
        self.current_turn_angle = msg.angle
    
    def motor_power_callback(self, msg: MotorPowerStamped):
        self.motor_power = msg.power

    def target_path_callback(self, msg: TargetPathStamped):
        self.target_path = msg.path
    
    def set_target_speed(self, speed: float):
        self.target_speed = speed

        # assemble msg
        msg = VehicleSpeedStamped()
        msg.speed = speed 
        msg.header.stamp = self.get_clock().now().to_msg()

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

# TEMPORARY
if __name__ == '__main__':
    import rclpy

    rclpy.init()
    node = AP1SystemInterfaceNode()

    try:
        node.set_target_speed(2)
    finally:
        # teardown
        node.destroy_node()
        rclpy.shutdown()
    