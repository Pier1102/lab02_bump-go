import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        #the “queue size” is 10. Queue size is a required QoS (quality of service) setting that limits the amount 
        # of queued messages if a subscriber is not receiving them fast enough.
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Internal state variables
        self.N = 1
        self.state = 0  # 0: +X, 1: +Y, 2: -X, 3: -Y
        self.counter = 0  # counts seconds spent in current direction

        self.get_logger().info('Nodo Controller avviato. Pubblica su /cmd_vel a 1 Hz.')

    def timer_callback(self):
        msg=Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        # imposta la direzione del movimento
        if self.state == 0:
            msg.linear.x = 1.0
        elif self.state == 1:
            msg.linear.y = 1.0
        elif self.state == 2:
            msg.linear.x = -1.0
        elif self.state == 3:
            msg.linear.y = -1.0
        # pubblica il messaggio
        self.publisher_.publish(msg)
         # stampa su terminale (logger)
        self.get_logger().info(f'Messaggio pubblicato: direzione={self.state}, N={self.N}, velocità=({msg.linear.x}, {msg.linear.y})')
        #aggiorno contatore secondi percorsi in questo stato
        self.counter +=1

        #Dopo N secondi in una direzione si passa alla successiva
        if self.counter >= self.N:
            self.counter = 0
            self.state +=1  # passa alla prossima direzion

            # se siamo tornati a state +X, aumenta N di 1
            if self.state > 3:
                self.state = 0
                self.N += 1
                self.get_logger().info(f'Ciclo completo! Nuovo N = {self.N}')

def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

