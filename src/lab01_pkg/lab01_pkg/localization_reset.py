import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose , Point, Quaternion
from std_msgs.msg import Bool

class LocalizationResetNode(Node):
    def __init__(self):
        super().__init__('localization_reset')
        # Subscriber al topic /cmd_vel
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)

        # Publisher per al topic /pose
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)

         #subscriber al topic /reset
        self.reset_subscription = self.create_subscription(Bool, '/reset' ,self.reset_callback, 10) 
        
        self.timer = self.create_timer(1.0, self.update_pose)

        # Posizione iniziale
        self.x = 0.0
        self.y = 0.0
        self.latest_velocity = Twist()
        
        self.get_logger().info('Nodo Localization_reset avviato ')

    def cmd_vel_callback (self,msg):
        self.latest_velocity = msg
        
    def reset_callback(self,msg: Bool):
        if msg.data == True:
            self.x = 0.0
            self.y = 0.0
            self.get_logger().warn(" Reset ricevuto! Posizione riportata a (0,0).")
    
    def update_pose(self):
        
         # Periodo di aggiornamento 
        dt = 1.0
        # calcolo spostamento
        dx = self.latest_velocity.linear.x *dt
        dy = self.latest_velocity.linear.y *dt
        # Aggiorna posizione  s=s0+vt
        self.x += dx
        self.y += dy

        pose = Pose()
        pose.position = Point(x=self.x, y=self.y, z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.publisher_.publish(pose)
        self.get_logger().info(
        f'Posizione : x={self.x}, y={self.y}'
        )

def main(args=None):
    rclpy.init(args=args)
    localization_reset = LocalizationResetNode()
    rclpy.spin(localization_reset)
    localization_reset.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    