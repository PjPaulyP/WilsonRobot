import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from set_mode_interface.srv import SetMode  # generated service

class ModeManager(Node):
    ALLOWED_MODES = {'idle','walk','stand','emergency_stop'}

    def __init__(self):
        super().__init__('mode_manager')
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mode_pub = self.create_publisher(String, '/mode', qos)
        self.srv = self.create_service(SetMode, '/set_mode', self.handle_set_mode)
        self.current_mode = 'idle'
        self.publish_mode()

    def publish_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Published mode: {self.current_mode}')

    def handle_set_mode(self, req, resp):
        requested = req.mode.lower()
        if requested not in self.ALLOWED_MODES:
            resp.success = False
            resp.message = f'Invalid mode: {req.mode}'
            self.get_logger().warn(resp.message)
            return resp
        # Simple transition validation example
        if self.current_mode == 'emergency_stop' and requested != 'idle':
            resp.success = False
            resp.message = 'Cannot change mode while in emergency_stop'
            return resp
        self.current_mode = requested
        self.publish_mode()
        resp.success = True
        resp.message = f'Mode set to {requested}'
        return resp

def main(args=None):
    rclpy.init(args=args)

    mode_manager = ModeManager()

    rclpy.spin(mode_manager)

    mode_manager.destroy_node()
    rclpy.shutdown()