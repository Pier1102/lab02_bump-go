import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose , Point, Quaternion

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        # Subscriber al topic /cmd_vel
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
     
        # Publisher per il topic /pose
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)

        # Posizione iniziale
        self.x = 0.0
        self.y = 0.0

        # Periodo di aggiornamento 
        self.dt = 1.0

        self.get_logger().info('Nodo Localization avviato, in ascolto su /cmd_vel.')

    def cmd_vel_callback(self,msg: Twist):

        # Aggiorna posizione usando velocità e periodo dt s=s0+vt
        self.x += msg.linear.x * self.dt
        self.y += msg.linear.y * self.dt

        # Crea messaggio Pose. Il tipo geometry_msgs/Pose richiede che ci siano sia position (x,y,z) 
        # che orientation quaternion (x, y, z, w)
        pose_msg = Pose()
        pose_msg.position = Point(x=self.x, y=self.y, z=0.0)
        pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        #alternativa
        
        #pose_msg.position.x = self.x
        #pose_msg.position.y = self.y
        #pose_msg.position.z = 0.0

        #pose_msg.orientation.x = 0.0
        #pose_msg.orientation.y = 0.0
        #pose_msg.orientation.z = 0.0
        #pose_msg.orientation.w = 1.0
       
        self.publisher_.publish(pose_msg)
        # Log
        self.get_logger().info(
            f'Posizione : x={self.x}, y={self.y}'
        )
        
def main(args=None):
    rclpy.init(args=args)
    localization = LocalizationNode()
    rclpy.spin(localization)
    localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    