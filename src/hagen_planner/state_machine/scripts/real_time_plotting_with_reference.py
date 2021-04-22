import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import math
# from hagen_msgs import PoseCommand
from hagen_msgs.msg import PoseCommand
from geometry_msgs.msg import Vector3Stamped

class Visualiser:
    def __init__(self):
        plt.rcParams["font.size"] = "24"
        self.fig, axs = plt.subplots(4, sharex=True)
        self.ax1 = axs[0]
        self.ax2 = axs[1]
        self.ax3 = axs[2]
        self.ax4 = axs[3]
        self.ln1 = Line2D([], [], color='red', linewidth=2, label=r'$v_x$')
        self.ln2 = Line2D([], [], color='red', linewidth=2, label=r'$v_y$')
        self.ln3 = Line2D([], [], color='red', linewidth=2, label=r'$v_z$')
        self.ln4 = Line2D([], [], color='red', linewidth=2, label=r'$Yaw$')

        self.ln11 = Line2D([], [], color='blue', linewidth=2, label=r'$v^{ref}_x$')
        self.ln22 = Line2D([], [], color='blue', linewidth=2, label=r'$v^{ref}_y$')
        self.ln33 = Line2D([], [], color='blue', linewidth=2, label=r'$v^{ref}_z$')
        self.ln44 = Line2D([], [], color='blue', linewidth=2, label=r'$Yaw^{ref}_z$')

        self.ax1.add_line(self.ln1)
        self.ax1.add_line(self.ln11)
        self.ax2.add_line(self.ln2)
        self.ax2.add_line(self.ln22)
        self.ax3.add_line(self.ln3)
        self.ax3.add_line(self.ln33)
        self.ax4.add_line(self.ln4)
        self.ax4.add_line(self.ln44)

        self.fig.text(0.06, 0.6, 'm/s', ha='center', va='center', rotation='vertical')
        self.ax4.set_ylabel("rad")
        self.velocity_f = None
        
        
        
        self.x_data, self.y_data = [] , []
        self.odometry = None
        self.yaw_angles = []
        self.velocities = []
        self.velocities_com_x = []
        self.velocities_com_y = []
        self.velocities_com_z = []
        self.velocities_com_ref_x = []
        self.velocities_com_ref_y = []
        self.velocities_com_ref_z = []
        self.velocities_ref = []
        self.yaw_angles_ref = []
        self.odometry_ref = None
       
        self.save_data=open('results.txt','a')
        
   
    
    def init_plot(self):
        
        velocity_=[-0.8, 0.8]
        self.ax1.set_xlim([0, 7500])
        self.ax1.set_ylim(velocity_)
        custom_ticks = np.array([-0.4, 0, 0.4])
        self.ax1.set_yticks(custom_ticks)
        self.ax1.set_yticklabels(custom_ticks)
        ymin, ymax = self.ax1.get_ylim()
        self.ax1.vlines(x=[760, 6500], ymin=ymin, ymax=ymax, linewidth=6, color='g')

        legend = self.ax1.legend(loc='upper right', fontsize=23)
        
        self.ax2.set_xlim(0, 7500)
        self.ax2.set_ylim(velocity_)
        self.ax2.set_yticks(custom_ticks)
        self.ax2.set_yticklabels(custom_ticks)
        ymin, ymax = self.ax2.get_ylim()
        self.ax2.vlines(x=[760, 6500], ymin=ymin, ymax=ymax, linewidth=6, color='g')

        legend = self.ax2.legend(loc='upper right', fontsize=23)

        self.ax3.set_xlim(0, 7500)
        self.ax3.set_ylim(velocity_)
        self.ax3.set_yticks(custom_ticks)
        self.ax3.set_yticklabels(custom_ticks)
        ymin, ymax = self.ax3.get_ylim()
        self.ax3.vlines(x=[760, 6500], ymin=ymin, ymax=ymax, linewidth=6, color='g')

        legend = self.ax3.legend(loc='upper right', fontsize=23)

        self.ax4.set_xlim(0, 7500)
        self.ax4.set_ylim([-3.14, 3.14])
        self.ax4.set_xlabel("number of readings (15 readings per second)")
        ymin, ymax = self.ax4.get_ylim()
        self.ax4.vlines(x=[760, 6500], ymin=ymin, ymax=ymax, linewidth=6, color='g')

        legend = self.ax4.legend(loc='upper right', fontsize=23)
        
        return [self.ln1, self.ln11, self.ln2, self.ln22, self.ln3, self.ln33, self.ln4, self.ln44]
        
    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return yaw
    
    def state_handler(self, msg):
        if(self.odometry is not None):
            yaw_angle = self.getYaw(self.odometry.pose.pose)
            
            velc = msg.twist.twist.linear 
            if(math.isinf(velc.x) or math.isinf(velc.y) or math.isinf(velc.z)):
                print("can not calcutate valusss")
            else:
                if(self.odometry_ref  is not None):
                    # velocity = self.get_velocity(self.odometry)
                    self.velocities_com_x.append(self.velocity_f.x)
                    self.velocities_com_y.append(self.velocity_f.y)
                    self.velocities_com_z.append(self.velocity_f.z)
                    self.yaw_angles.append(yaw_angle)
                    x_index = len(self.x_data)
                    self.x_data.append(x_index+1)
                
                
                    ref_vel = self.odometry_ref
                    self.velocities_com_ref_x.append(ref_vel.velocity.x)
                    self.velocities_com_ref_y.append(ref_vel.velocity.y)
                    self.velocities_com_ref_z.append(ref_vel.velocity.z)
                    if(x_index>3750 and x_index<3830):
                        ref_vel.yaw = yaw_angle
                    elif(ref_vel.yaw>=np.pi):
                        ref_vel.yaw = ref_vel.yaw - 2*np.pi
                    self.yaw_angles_ref.append(ref_vel.yaw)
                    if(x_index>8000):
                        print("Saving data")
                        reference_vel = np.array([self.velocities_com_ref_x,  self.velocities_com_ref_y,  self.velocities_com_ref_z, self.yaw_angles_ref])
                        np.save("reference_velocity.npy", reference_vel)
                        actual_vel = np.array([self.velocities_com_x, self.velocities_com_y, self.velocities_com_z, self.yaw_angles])
                        np.save("actual_velocity.npy", actual_vel)
                else:
                    self.velocities_com_x.append(self.velocity_f.x)
                    self.velocities_com_y.append(self.velocity_f.y)
                    self.velocities_com_z.append(self.velocity_f.z)
                    self.yaw_angles.append(yaw_angle)
                    x_index = len(self.x_data)
                    self.x_data.append(x_index+1) 
                    self.velocities_com_ref_x.append(0)
                    self.velocities_com_ref_y.append(0)
                    self.velocities_com_ref_z.append(0)
                    self.yaw_angles_ref.append(0)


    def get_velocity(self, msg):
        velocity = msg.twist.twist.linear 
        return np.sqrt(velocity.x*2 + velocity.y*2 + velocity.z*2)  

    def odom_callback(self, msg):
        self.odometry = msg
        self.state_handler(self.odometry)
    
    def odom_ref_callback(self, msg):
        self.odometry_ref = msg
        
    def setVelocity(self, msg):
        self.velocity_f = msg.vector   
    
    def update_plot(self, frame):
        self.ln1.set_data(self.x_data, self.velocities_com_x)
        self.ln2.set_data(self.x_data, self.velocities_com_y)
        self.ln3.set_data(self.x_data, self.velocities_com_z)
        self.ln4.set_data(self.x_data, self.yaw_angles)
        self.ln11.set_data(self.x_data, self.velocities_com_ref_x)
        self.ln22.set_data(self.x_data, self.velocities_com_ref_y)
        self.ln33.set_data(self.x_data, self.velocities_com_ref_z)
        self.ln44.set_data(self.x_data, self.yaw_angles_ref)
        return [self.ln1, self.ln11, self.ln2, self.ln22, self.ln3, self.ln33, self.ln4, self.ln44]

rospy.init_node('lidar_visual_node')
vis = Visualiser()
sub = rospy.Subscriber('/dji_sdk/odometry', Odometry, vis.odom_callback)
ref_sub = rospy.Subscriber('/planning/pos_cmd', PoseCommand, vis.odom_ref_callback)
velocity_dji = rospy.Subscriber('/dji_sdk/velocity', Vector3Stamped, vis.setVelocity)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.init_plot)
plt.show(block=True)