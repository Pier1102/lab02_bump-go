import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ControllerResetNode(Node):

    def __init__(self):
        super().__init__('controller_reset')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_subscription = self.create_subscription(Bool, '/reset' ,self.reset_callback, 10) 

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Internal state variables
        self.N = 1
        self.state = 0  # 0: +X, 1: +Y, 2: -X, 3: -Y
        self.counter = 0  # counts seconds spent in current direction
        
        self.get_logger().info('Nodo Controller_reset avviato ')

    def timer_callback(self):
        msg=Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0

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
            self.state +=1  # passa alla prossima direzione

            # se siamo tornati a state +X, aumenta N di 1
            if self.state > 3:
                self.state = 0
                self.N += 1
                self.get_logger().info(f'Ciclo completo! Nuovo N = {self.N}')

    def reset_callback(self,msg: Bool):
        if msg.data == True:
            self.get_logger().info('Reset ricevuto → N=1, stato=0, contatore=0')
            self.N = 1
            self.state = 0
            self.counter=0


def main(args=None):
    rclpy.init(args=args)
    controller_reset = ControllerResetNode()
    rclpy.spin(controller_reset)
    controller_reset.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

