import rclpy
from rclpy.node import Node
import numpy as np 
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
#from tf2_msgs.msg import TFMessage


class StatePub(Node):

    def __init__(self):
        super().__init__('state_pub')
        
        #initialise a quaternion array for t=0, when base_link and world are aligned 
        self.orientation=np.array([0.0,0.0,0.0,1.0])
        self.orientation2=np.array([0.0,0.0,0.0,1])
        
        #timer for the publisher function
        self.dt = 0.025 #40Hz 
        self.timer = self.create_timer(self.dt, self.publish_state) #publish_state is the master publisher node
        #tf subscriber 
        #self.tf_subscription=self.create_subscription(TFMessage, '/tf', self.tf_callback,10)      
        #publisher for publishing the estimated state
        self.pub=self.create_publisher(Odometry, '/quaternion_state',10)
        self.pub2=self.create_publisher(Odometry, '/quaternion_state_2',10)



    def quat_exp(self,q):
        v = np.asarray(q[:3], dtype=float)
        s = float(q[3])
        theta = np.linalg.norm(v)
        
        exp_s = np.exp(s)

        if theta < 1e-12: #small angle approximation 
            return np.array([
            exp_s * v[0],
            exp_s * v[1],
            exp_s * v[2],
            exp_s
            ])

        scale = exp_s * np.sin(theta) / theta

        return np.array([
            scale * v[0],
            scale * v[1],
            scale * v[2],
            exp_s * np.cos(theta)
        ])


    def quat_multiply(self,q1,q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])
    

    def quaternion_dot(self, q, omega):

        q = np.asarray(q, dtype=float)
        wx, wy, wz = omega

        Omega = np.array([
        [0.0,  wz, -wy,  wx],
        [-wz, 0.0,  wx,  wy],
        [ wy, -wx, 0.0,  wz],
        [-wx, -wy, -wz, 0.0]
    ])

        q_dot = 0.5 * Omega @ q
        return q_dot

    def publish_state(self):
        w_in=60 #in degrees per second
        wheelbase=160/1000
        radius=33/1000
        w_calculated=(2*w_in*radius/wheelbase)*np.pi/180 #in radians per second 
        omega=np.array([0.0,0.0,0.3])
        dt = self.dt  #time step 

        q_prev = self.orientation.copy()    
        rot = R.from_quat(q_prev) #rotation matrix obtained using the pre-defined function in SciPy
        R_matrix = rot.as_matrix()
        q1 = q_prev
        q2 = np.array([omega[0], omega[1], omega[2], 0])
        q2=q2*0.5*dt
        q2=self.quat_exp(q2)
        result_obj = self.quat_multiply(q1,q2)
        self.orientation = result_obj
        self.orientation/= np.linalg.norm(self.orientation)

        #####LINEAR ALGEBRA APPROACH (_la means linear algebra)
        q1_la=self.orientation2.copy()
        q_dot_la=self.quaternion_dot(q1_la,omega)
        q2_la = q1_la + dt * q_dot_la
        self.orientation2=q2_la
        self.orientation2/= np.linalg.norm(self.orientation2)


    
        # writing everything into the msg format
        msg= Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.pose.orientation.x = float(self.orientation2[0])
        msg.pose.pose.orientation.y = float(self.orientation2[1])
        msg.pose.pose.orientation.z = float(self.orientation2[2])
        msg.pose.pose.orientation.w = float(self.orientation2[3])
            
        self.pub.publish(msg)

        msg2= Odometry()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.header.frame_id = "world"

        msg2.pose.pose.orientation.x = float(self.orientation2[0])
        msg2.pose.pose.orientation.y = float(self.orientation2[1])
        msg2.pose.pose.orientation.z = float(self.orientation2[2])
        msg2.pose.pose.orientation.w = float(self.orientation2[3])
            
        self.pub2.publish(msg2)





def main(args=None):
    rclpy.init(args=args)
    state_pub=StatePub()
    rclpy.spin(state_pub) 
    state_pub.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()