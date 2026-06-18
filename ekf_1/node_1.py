#Code algorithm 
########################
#1. Define a callback that subscribes to the IMU data 
#2. Publish the estimated state (position and orientation) on a custom topic 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np 
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3Stamped

class StatePub(Node):

    def __init__(self):
        super().__init__('state_pub')

        #variables
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        # Identity quaternion
        self.orientation = np.array([0.0, 0.0, 0.0, 1])
        
        # Latest IMU measurements
        self.accln = np.array([0, 0, 9.81])
        self.w= np.zeros(3)
        #FOR DEBUGGING
        self.accln_world = np.zeros(3)

        #timer for the publisher function
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.publish_state)

        #imu subscriber 
        self.imu_susbcription=self.create_subscription(Imu, '/imu' ,self.imu_callback,10)

        #publisher for publishing the estimated state 
        self.imu_pub=self.create_publisher(Odometry, '/imu_state',10)

            #FOR DEBUGGING
        self.accel_world_pub = self.create_publisher(Vector3Stamped, '/accel_world', 10)


    def imu_callback(self,msg):
            self.w=np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            self.accln=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            self.get_logger().info("Received IMU data") #sanity check


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

    def propagate_state(self): #function for estimating the state from the IMU

        dt = self.dt  #time step 
        p_prev = self.position.copy() #previous values 
        v_prev = self.velocity.copy()
        q_prev = self.orientation.copy()
        
        rot = R.from_quat(q_prev) #rotation matrix obtained using the pre-defined function in SciPy
        R_matrix = rot.as_matrix()
        
        self.accln_world = (R_matrix @ self.accln- np.array([0.0, 0.0, 9.81]))
        self.velocity = (v_prev + (self.accln_world * dt)) #updation of position and velocity
        self.position = (p_prev+ (v_prev * dt)+ (0.5 * self.accln_world * (dt**2)))

        # 1. Define your input quaternions in [x, y, z, w] format
        q1 = q_prev
        q2 = np.array([self.w[0], self.w[1], self.w[2], 0])
        q2=q2*0.5*dt
        q2=self.quat_exp(q2)

        # 4. Perform direct algebraic multiplication (q1 * q2)
        result_obj = self.quat_multiply(q1,q2)

        self.orientation = result_obj
        self.orientation /= np.linalg.norm(self.orientation)


    def publish_state(self):
            
            self.propagate_state()

            # writing everything into the msg format
            msg_imu = Odometry()
            msg_imu.header.stamp = self.get_clock().now().to_msg()
            msg_imu.header.frame_id = "world"

            msg_imu.pose.pose.position.x = float(self.position[0])
            msg_imu.pose.pose.position.y = float(self.position[1])
            msg_imu.pose.pose.position.z = float(self.position[2])

            msg_imu.twist.twist.linear.x = float(self.velocity[0])
            msg_imu.twist.twist.linear.y = float(self.velocity[1])
            msg_imu.twist.twist.linear.z = float(self.velocity[2])

            msg_imu.pose.pose.orientation.x = float(self.orientation[0])
            msg_imu.pose.pose.orientation.y = float(self.orientation[1])
            msg_imu.pose.pose.orientation.z = float(self.orientation[2])
            msg_imu.pose.pose.orientation.w = float(self.orientation[3])
            
            self.imu_pub.publish(msg_imu)


            accel_msg = Vector3Stamped()
            accel_msg.header.stamp = self.get_clock().now().to_msg()
            accel_msg.header.frame_id = "world"
            accel_msg.vector.x = float(self.accln_world[0])
            accel_msg.vector.y = float(self.accln_world[1])
            accel_msg.vector.z = float(self.accln_world[2])
            self.accel_world_pub.publish(accel_msg)



def main(args=None):
    rclpy.init(args=args)
    state_pub=StatePub()
    rclpy.spin(state_pub) 
    state_pub.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()