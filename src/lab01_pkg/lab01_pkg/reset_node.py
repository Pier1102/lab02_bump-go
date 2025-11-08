import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose , Point, Quaternion 
from std_msgs.msg import Bool
import math

class ResetNode(Node):
    def __init__(self):
        super().__init__('reset')
        self.pose_subscription = self.create_subscription(Pose, '/pose' ,self.pose_callback,10)
        self.subscription  # prevent unused variable warning
        
        # Publisher per il topic /reset
        self.publisher_ = self.create_publisher(Bool, '/reset', 10)
        self.reset_sent = False

    def pose_callback(self,msg: Pose):
        reset_msg = Bool()
        distance = math.sqrt(msg.position.x**2 + msg.position.y**2) #distanza euclidea 
        self.get_logger().info(f'Reiceved pose: x={msg.position.x:}, y={msg.position.y:}, distance={distance:}')
       
        #controllo della distanza
        if distance > 6.0:
            reset_msg.data = True   # assegno il valore True 
            self.publisher_.publish(reset_msg)  # pubblico sul topic
            self.get_logger().info(f"Distanza {distance:.2f} > 6.0 m: reset inviato!")
            self.reset_sent = True
        
        elif distance <= 6.0 and self.reset_sent:
            # se il robot torna entro i 6 m, sblocca il prossimo reset
            self.reset_sent = False
            self.get_logger().info(f'Distanza tornata a {distance:} m → pronto per nuovo reset.')   
                
def main(args=None):
    rclpy.init(args=args)
    reset = ResetNode()
    rclpy.spin(reset)
    reset.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()   