#Code algorithm 
########################
#1. Define a callback that subscribes to the IMU data 
#2. Publish the estimated state (position and orientation) on a custom topic 

#two parallel pipelines will run- ekf estimate vs the prediction only from the IMU estimate 

#Pending things-
# Adding a script for velocity measurement 
#imu to data transition in first ekf function
# Checking initialisation of variables in init 
# Checking time synchronisation
#gps noise and covariance need to match 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np 
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3Stamped
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PointStamped

class StatePub(Node):

    def __init__(self):
        super().__init__('state_pub')
        
        # IMU CALLBACK FUNCTION (these are the inputs 'u')
        self.accln_bodyframe = np.array([0, 0, 9.81])
        self.w= np.zeros(3)

        #FOR DEBUGGING 
        self.accln_world_imu = np.zeros(3)
        self.accln_world_ekf = np.zeros(3)

        #VO CALLBACK FUNCTION
        self.pos_vo_prev = None
        self.t_vo_prev = None
        self.vo_velocity = np.zeros(3)


        #GPS CALLBACK FUNCTION
        self.x_gps = 0.0
        self.y_gps = 0.0
        self.z_gps = 0.0

        #VARIABLES FOR STATE ESTIMATION FROM ONLY IMU DATA (PURE PREDICTION THAT IS)
        self.position_imu = np.zeros(3)
        self.velocity_imu= np.zeros(3)
        self.orientation_imu = np.array([0.0, 0.0, 0.0, 1])

        #VARIABLES FOR STATE ESTIMATION AFTER USING KALMAN FILTER
        self.x = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[1.0]])
        self.position_ekf=np.zeros(3)
        self.velocity_ekf=np.zeros(3)
        self.orientation_ekf=np.array([0.0,0.0,0.0,1])

        #INITIALISATION OF THE MATRIX P --> can be tuned 
        self.P = np.diag([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])

        #timer for the publisher function
        self.dt = 0.1
        self.first_reading = True
        self.timer = self.create_timer(self.dt, self.publish_state) #publish_state is the master publisher node

        #imu subscriber 
        self.imu_subscription=self.create_subscription(Imu, '/imu' ,self.imu_callback,10)

        #tf subscriber to generate fake GPS data
        self.tf_subscription=self.create_subscription(TFMessage, '/tf', self.gps_callback,10)
        
        #VO data subscriber
        self.vo_subscription=self.create_subscription(Odometry, '/zed/zed_node/odom',self.vo_callback,10)
        
        #publisher for publishing the estimated state usigng only imu prediction
        self.imu_pub=self.create_publisher(Odometry, '/imu_state',10)

        #FOR DEBUGGING
        self.accel_world_pub_imu= self.create_publisher(Vector3Stamped, '/accel_world_imu', 10)
        self.accel_world_pub_ekf= self.create_publisher(Vector3Stamped, '/accel_world_ekf', 10)

        #kalman filter
        self.filtered_state=self.create_publisher(Odometry, '/filtered_state', 10)

        #Visualiser topic for fake gps data
        self.gps_pub = self.create_publisher(PointStamped,'/fake_gps',10)

        #Visualiser topic for expected dynamics purely from physics mode
        self.unicycle_pub=self.create_publisher(Odometry, '/unicycle_dynamics',10)


    def imu_callback(self,msg):
            self.w=np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            self.accln_bodyframe=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            self.get_logger().info("Received IMU data") #sanity check
    
    def gps_callback(self,msg): #this is also a callback function only 
         gps_std_x = 3   # meters
         gps_std_y = 3
         gps_std_z = 3

         for transform in msg.transforms:
            # Make sure this is the transform you want
            if transform.child_frame_id != "base_link":
                continue

            # True position from TF
            x_true = transform.transform.translation.x
            y_true = transform.transform.translation.y
            z_true = transform.transform.translation.z

            # Add Gaussian noise
            self.x_gps = x_true + np.random.normal(0.0, gps_std_x)
            self.y_gps = y_true + np.random.normal(0.0, gps_std_y)
            self.z_gps = z_true + np.random.normal(0.0, gps_std_z)

            #printing the GPS data on the terminal 
            self.get_logger().info(f"Fake GPS: ({self.x_gps:.3f}, {self.y_gps:.3f}, {self.z_gps:.3f})")

    
    def vo_callback(self, msg):

        pos_vo_current = np.array([ #one-dimensional array
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    ])
        
        t_current = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.pos_vo_prev is not None and self.t_vo_prev is not None:
            dt_vo = t_current - self.t_vo_prev
            if dt_vo > 1e-6:  # guard against duplicate/zero-dt timestamps
                self.vo_velocity = (pos_vo_current - self.pos_vo_prev) / dt_vo

        self.pos_vo_prev = pos_vo_current
        self.t_vo_prev = t_current


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

    def propagate_state_imu(self): #function for estimating the state from only the IMU

        dt = self.dt  #time step 
        p_prev = self.position_imu.copy() #previous values 
        v_prev = self.velocity_imu.copy()
        q_prev = self.orientation_imu.copy()
        
        rot = R.from_quat(q_prev) #rotation matrix obtained using the pre-defined function in SciPy
        R_matrix = rot.as_matrix()
        
        self.accln_world_imu = (R_matrix @ self.accln_bodyframe - np.array([0.0, 0.0, 9.81]))
        self.velocity_imu = (v_prev + (self.accln_world_imu * dt)) #updation of position and velocity
        self.position_imu = (p_prev+ (v_prev * dt)+ (0.5 * self.accln_world_imu * (dt**2)))

        # 1. Define your input quaternions in [x, y, z, w] format
        q1 = q_prev
        q2 = np.array([self.w[0], self.w[1], self.w[2], 0])
        q2=q2*0.5*dt
        q2=self.quat_exp(q2)

        # 4. Perform direct algebraic multiplication (q1 * q2)
        result_obj = self.quat_multiply(q1,q2)

        self.orientation_imu = result_obj
        self.orientation_imu /= np.linalg.norm(self.orientation_imu)

    def publish_state_imu(self):
            
            self.propagate_state_imu()

            # writing everything into the msg format
            msg_imu = Odometry()
            msg_imu.header.stamp = self.get_clock().now().to_msg()
            msg_imu.header.frame_id = "world"

            msg_imu.pose.pose.position.x = float(self.position_imu[0])
            msg_imu.pose.pose.position.y = float(self.position_imu[1])
            msg_imu.pose.pose.position.z = float(self.position_imu[2])

            msg_imu.twist.twist.linear.x = float(self.velocity_imu[0])
            msg_imu.twist.twist.linear.y = float(self.velocity_imu[1])
            msg_imu.twist.twist.linear.z = float(self.velocity_imu[2])

            msg_imu.pose.pose.orientation.x = float(self.orientation_imu[0])
            msg_imu.pose.pose.orientation.y = float(self.orientation_imu[1])
            msg_imu.pose.pose.orientation.z = float(self.orientation_imu[2])
            msg_imu.pose.pose.orientation.w = float(self.orientation_imu[3])
            
            self.imu_pub.publish(msg_imu)


            accel_msg = Vector3Stamped()
            accel_msg.header.stamp = self.get_clock().now().to_msg()
            accel_msg.header.frame_id = "world"
            accel_msg.vector.x = float(self.accln_world_imu[0])
            accel_msg.vector.y = float(self.accln_world_imu[1])
            accel_msg.vector.z = float(self.accln_world_imu[2])
            self.accel_world_pub_imu.publish(accel_msg)
    

    def ekf_predict(self):
        dt=self.dt

        #calculating f(x_k-1, u_k-1)
        p_prev = self.position_ekf.copy() #previous values 
        v_prev = self.velocity_ekf.copy()
        q_prev = self.orientation_ekf.copy()
        
        rot = R.from_quat(q_prev) #rotation matrix obtained using the pre-defined function in SciPy
        R_matrix = rot.as_matrix()
        
        self.accln_world_ekf = (R_matrix @ self.accln_bodyframe- np.array([0.0, 0.0, 9.81]))
        self.velocity_ekf= (v_prev + (self.accln_world_ekf * dt))
        self.position_ekf = (p_prev+ (v_prev * dt)+ (0.5 * self.accln_world_ekf * (dt**2)))

        q1 = q_prev
        q2 = np.array([self.w[0], self.w[1], self.w[2], 0])
        q2=q2*0.5*dt
        q2=self.quat_exp(q2)
        result_obj = self.quat_multiply(q1,q2)

        self.orientation_ekf = result_obj
        self.orientation_ekf /= np.linalg.norm(self.orientation_ekf) #converting the quaternion to a unit quaternion

        #by default we are dealing with column vectors 
        #Defining the state vector
        self.x = np.array([
    [self.position_ekf[0]],
    [self.position_ekf[1]],
    [self.position_ekf[2]],
    [self.velocity_ekf[0]],
    [self.velocity_ekf[1]],
    [self.velocity_ekf[2]],
    [self.orientation_ekf[0]],
    [self.orientation_ekf[1]],
    [self.orientation_ekf[2]],
    [self.orientation_ekf[3]]
])
        

        # COMPUTATION OF JACOBIAN F WITH OFF-DIAGONAL TERMS (THERE WILL BE COUPLING)
        # Current quaternion
        xq = self.orientation_ekf[0]
        yq = self.orientation_ekf[1]
        zq = self.orientation_ekf[2]
        wq = self.orientation_ekf[3]

        # Body-frame acceleration
        ax_b = self.accln_bodyframe[0]
        ay_b = self.accln_bodyframe[1]
        az_b = self.accln_bodyframe[2]

        # Aq = d(R(q)a_b)/dq

        Aq = np.array([

        [
2*yq*ay_b + 2*zq*az_b,
-4*yq*ax_b + 2*xq*ay_b + 2*wq*az_b,
-4*zq*ax_b - 2*wq*ay_b + 2*xq*az_b,
-2*zq*ay_b + 2*yq*az_b
],

[
2*yq*ax_b - 4*xq*ay_b - 2*wq*az_b,
2*xq*ax_b + 2*zq*az_b,
2*wq*ax_b - 4*zq*ay_b + 2*yq*az_b,
2*zq*ax_b - 2*xq*az_b
],

[
2*zq*ax_b + 2*wq*ay_b - 4*xq*az_b,
-2*wq*ax_b + 2*zq*ay_b - 4*yq*az_b,
2*xq*ax_b + 2*yq*ay_b,
-2*yq*ax_b + 2*xq*ay_b
]

])

    # Quaternion transition Jacobian computation
        delta_q = q2
        xd = delta_q[0]
        yd = delta_q[1]
        zd = delta_q[2]
        wd = delta_q[3]

        Fq = np.array([
    [ wd,  zd, -yd,  xd],
    [-zd,  wd,  xd,  yd],
    [ yd, -xd,  wd,  zd],
    [-xd, -yd, -zd,  wd]
])

    # Full 10x10 Jacobian
        F = np.eye(10)
    # dp/dv
        F[0,3] = dt
        F[1,4] = dt
        F[2,5] = dt
    # dp/dq
        F[0:3, 6:10] = 0.5 * dt**2 * Aq
    # dv/dq
        F[3:6, 6:10] = dt * Aq
    # dq/dq
        F[6:10, 6:10] = Fq

    #PROCESS NOISE COVARIANCE MATRIX- TO BE TUNED
        Q = np.diag([
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15
])

        # Covariance prediction
        self.P = F @ self.P @ (F.T) + Q        
         

    def ekf_update(self):

        # Measurement vector
        z = np.array([
        [self.x_gps],
        [self.y_gps],
        [self.z_gps],
        [self.vo_velocity[0]],
        [self.vo_velocity[1]],
        [self.vo_velocity[2]]
    ])

        # Measurement matrix z=H(x)
        H = np.zeros((6,10))
        H[0,0] = 1
        H[1,1] = 1
        H[2,2] = 1
        H[3,3] = 1
        H[4,4] = 1
        H[5,5] = 1

        # Measurement covariance- TUNABLE/ ARBITRARY
        R_cov = np.diag([
        10000,
        10000,
        10000,
        0.1,
        0.1,
        0.1
        ])

        # Residuals and the covariance 
        y = z - H @ self.x
        S = H @ self.P @ H.T + R_cov #6x6 matrix

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)  #10x10 10x6 6x6 gives 10x6 

        # State update
        self.x = self.x + K @ y #Ky gives 10x1 

        # STate estimation Covariance update (Updating P)
        I = np.eye(10)
        self.P = (I - K @ H) @ self.P
             
    
    def publish_filtered_state(self):

        self.ekf_predict()
        self.ekf_update()

        self.position_ekf = self.x[0:3,0] #this is 1-D array (not a column vector)
        self.velocity_ekf = self.x[3:6,0]
        self.orientation_ekf=self.x[6:,0]

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.pose.position.x = float(self.position_ekf[0])
        msg.pose.pose.position.y = float(self.position_ekf[1])
        msg.pose.pose.position.z = float(self.position_ekf[2])

        msg.twist.twist.linear.x = float(self.velocity_ekf[0])
        msg.twist.twist.linear.y = float(self.velocity_ekf[1])
        msg.twist.twist.linear.z = float(self.velocity_ekf[2])

        msg.pose.pose.orientation.x=float(self.orientation_ekf[0])
        msg.pose.pose.orientation.y=float(self.orientation_ekf[1])
        msg.pose.pose.orientation.z=float(self.orientation_ekf[2])
        msg.pose.pose.orientation.w=float(self.orientation_ekf[3])

        self.filtered_state.publish(msg)

        accel_msg = Vector3Stamped()
        accel_msg.header.stamp = self.get_clock().now().to_msg()
        accel_msg.header.frame_id = "world"
        accel_msg.vector.x = float(self.accln_world_ekf[0])
        accel_msg.vector.y = float(self.accln_world_ekf[1])
        accel_msg.vector.z = float(self.accln_world_ekf[2])
        self.accel_world_pub_ekf.publish(accel_msg) 
    
    def publish_unicycle(self):
        msg=Odometry()

        w_left=100*np.pi/180
        w_right=100*np.pi/180
        radius=50/1000
        length=526/1000

        time_elapsed=self.get_clock().now().nanoseconds/1e9

        w_uni=(w_right-w_left)*radius/length
        v_uni=0.5*(w_right + w_left)*radius

        x_uni=-1 + (v_uni*time_elapsed)

        #in the robot's current heading frame- 
        #linear equations give us v,w

        #then we bring them to world frame
        #here, since it is 2D Motion, w = w_z remains the same only

        #v_local has to be transformed to v_global using rotation matrix derived from quaternion

        #Orientation = earlier orientation * exp (1/2 * w *delta_t)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.pose.position.x = float(x_uni)
        msg.pose.pose.position.y = float(0)
        #msg.pose.pose.position.z = float(self.position_ekf[2])

        msg.twist.twist.linear.x = float(v_uni)
        #msg.twist.twist.linear.y = float(self.velocity_ekf[1])
        #msg.twist.twist.linear.z = float(self.velocity_ekf[2])

        msg.pose.pose.orientation.x=float(0)
        msg.pose.pose.orientation.y=float(0)
        msg.pose.pose.orientation.z=float(0)
        msg.pose.pose.orientation.w=float(1)
        
        self.unicycle_pub.publish(msg)

    
    def publish_state(self):
        self.get_logger().info("Iteration 12")


        if self.first_reading:
            self.first_reading = False
            #self.publish_state_imu()
            return 
        
        # Publish fake GPS
        gps_msg = PointStamped()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "map"      # or whatever frame TF uses

        gps_msg.point.x = self.x_gps
        gps_msg.point.y = self.y_gps
        gps_msg.point.z = self.z_gps

        self.gps_pub.publish(gps_msg)
        self.publish_unicycle()
        self.publish_state_imu()
        self.publish_filtered_state()
        



def main(args=None):
    rclpy.init(args=args)
    state_pub=StatePub()
    rclpy.spin(state_pub) 
    state_pub.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()