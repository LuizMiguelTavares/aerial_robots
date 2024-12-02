#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <cmath>
#include <mutex>
#include <fstream>

class DroneController
{
public:
    DroneController() : nh_("~")
    {   
        // Perder o corpo, paredes virtuais, joystick, gráficos.

        // Params
        nh_.param("save_data", save_data_, true);
        nh_.param("pose_topic", pose_topic_, std::string("/natnet_ros/B1/pose"));
        nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/B1/cmd_vel"));
        nh_.param("takeoff_topic", takeoff_topic_, std::string("/B1/takeoff"));
        nh_.param("land_topic", land_topic_, std::string("/B1/land"));
        nh_.param("control_frequency", control_frequency_, 30.0); 

        // Gains
        nh_.param("Kp_x", Kp_(0, 0), 1.0);
        nh_.param("Kp_y", Kp_(1, 1), 1.0);
        nh_.param("Kd_x", Kd_(0, 0), 1.0);
        nh_.param("Kd_y", Kd_(1, 1), 1.0);
        nh_.param("Ki_x", Ki_(0, 0), 0.0);
        nh_.param("Ki_y", Ki_(1, 1), 0.0);
        nh_.param("K_z", K_z_, 1.0);
        nh_.param("K_yaw", K_yaw_, 5.0);
        nh_.param("alpha_velocity_gain", alpha_velocity_gain_, 0.4);
        nh_.param("theta_phi_max_deg", max_theta_phi_deg_, 15.0);
        nh_.param("z_dot_max", max_z_dot_, 1.0);
        nh_.param("yaw_dot_max_deg", max_yaw_dot_deg_, 100.0);
        nh_.param("Angular_frequency", w_, 2 * M_PI / 20); // Angular frequency

        // Task
        nh_.param("task", task_, 1); // 1: Positioning, 2: Circular trajectory, 3: Lemniscate trajectory, 4: Pringles trajectory

        pose_sub_ = nh_.subscribe(pose_topic_, 1, &DroneController::poseCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/B1/cmd_vel", 1);
        takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/B1/takeoff", 1);
        land_pub_ = nh_.advertise<std_msgs::Empty>("/B1/land", 1);
        
        if (save_data_)
        {   
            save_data();
        }

        there_is_pose_ = false;

        // Initialize control parameters
        dt_ = 1.0 / control_frequency_;
        g_ = 9.81;

        Ku_inv_control_ = (1 / g_) * Eigen::Matrix2d::Identity();

        max_theta_phi_ = deg2rad(max_theta_phi_deg_);
        max_yaw_dot_ = deg2rad(max_yaw_dot_deg_);
        
        // Set up the control loop timer
        control_timer_ = nh_.createTimer(ros::Duration(dt_), &DroneController::controlLoop, this);

        initial_time_ = ros::Time::now();
        previous_pose_time_ = ros::Time::now();

    }

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ros::Time current_time = msg->header.stamp;

        if (there_is_pose_ == false)
        {   
            current_position_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            previous_position_ = current_position_;
            previous_pose_time_ = current_time;

            current_velocity_ = Eigen::Vector3d::Zero();
            previous_velocity_ = current_velocity_;

            tf::quaternionMsgToTF(msg->pose.orientation, current_orientation_);

            there_is_pose_ = true;
            return;
        }

        double dt = (current_time - previous_pose_time_).toSec();

        current_position_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        tf::quaternionMsgToTF(msg->pose.orientation, current_orientation_);

        if (dt > 0.0)
        {
            Eigen::Vector3d new_velocity_ = (current_position_ - previous_position_) / dt;

            current_velocity_ = alpha_velocity_gain_ * new_velocity_ + (1 - alpha_velocity_gain_) * previous_velocity_;

            previous_velocity_ = current_velocity_;
            previous_position_ = current_position_;
            previous_pose_time_ = current_time;
        } 

        if (save_data_ && csv_file_.is_open())
        {
            csv_file_ << current_time << "," << dt << ","
                      << current_position_(0) << "," << current_position_(1) << "," << current_position_(2) << ","
                      << current_velocity_(0) << "," << current_velocity_(1) << "," << current_velocity_(2) << "\n";
        }

    }

    void controlLoop(const ros::TimerEvent& event)
    {   
        if (!there_is_pose_)
        {
            ROS_WARN("No pose received yet. Waiting...");
            return;
        }

        Eigen::Vector3d current_position;
        Eigen::Vector3d current_velocity;
        double current_yaw;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_position = current_position_;
            current_velocity = current_velocity_;

            double roll, pitch, yaw;
            tf::Matrix3x3(current_orientation_).getRPY(roll, pitch, yaw);
            current_yaw_ = yaw;
        }

        ros::Time current_time = ros::Time::now();
        double t = (current_time - initial_time_).toSec();

        // Compute desired states
        Eigen::Vector3d X_des = computeDesiredPosition(t, w_);
        Eigen::Vector3d X_dot_des = computeDesiredVelocity(t, w_);
        Eigen::Vector3d X_ddot_des = computeDesiredAcceleration(t, w_);

        // Calculate errors
        Eigen::Vector3d X_til = X_des - current_position;
        Eigen::Vector3d X_dot_til = X_dot_des - current_velocity;

        Eigen::Vector2d X_ddot_ref = X_ddot_des.head<2>() + Kp_ * X_til.head<2>() + Kd_ * X_dot_til.head<2>();
        Eigen::Vector2d theta_phi_ref = H_inv(current_yaw) * Ku_inv_control_ * X_ddot_ref;
        theta_phi_ref = theta_phi_ref.cwiseMax(-max_theta_phi_).cwiseMin(max_theta_phi_);

        // Theta_phi_ref_normalized
        theta_phi_ref = theta_phi_ref / max_theta_phi_;

        double Z_dot_ref = X_dot_des(2) + K_z_ * X_til(2);
        Z_dot_ref = std::max(std::min(Z_dot_ref, max_z_dot_), -max_z_dot_);

        // Control yaw
        double yaw_des = std::atan2(X_dot_des(1), X_dot_des(0));
        double yaw_dot_des = 0.0;
        double yaw_til = yaw_des - current_yaw;

        if (std::abs(yaw_til) > M_PI)
        {
            yaw_til = yaw_til - std::copysign(2 * M_PI, yaw_til);
        }

        double yaw_dot_ref = yaw_dot_des + K_yaw_ * yaw_til;
        yaw_dot_ref = std::max(std::min(yaw_dot_ref, max_yaw_dot_), -max_yaw_dot_);

        // Publish control commands
        geometry_msgs::Twist cmd_msg;

        // Assuming the drone accepts roll and pitch angles in angular.x and angular.y
        // cmd_msg.linear.x = theta_phi_ref(0); // Roll angle (theta)
        // cmd_msg.linear.y = theta_phi_ref(1); // Pitch angle (phi)
        // cmd_msg.linear.z = Z_dot_ref;         // Vertical velocity
        // cmd_msg.angular.z = yaw_dot_ref;      // Yaw rate

        // ROS_INFO("X_deg: %f, Y_deg: %f, Z_dot : %f, Yaw rate: %f", theta_phi_ref(0), theta_phi_ref(1), Z_dot_ref, yaw_dot_ref);

        // cmd_pub_.publish(cmd_msg);
    }

    void save_data()
    {   
        //getting the date and time to create a unique file name
        time_t now = time(0);
        tm *ltm = localtime(&now);
        std::string file_name = "drone_data_" + std::to_string(1900 + ltm->tm_year) + "_" + std::to_string(1 + ltm->tm_mon) + "_" + std::to_string(ltm->tm_mday) + "_" + std::to_string(ltm->tm_hour) + "_" + std::to_string(ltm->tm_min) + "_" + std::to_string(ltm->tm_sec) + ".csv";
        csv_file_.open(file_name);
        if (csv_file_.is_open())
        {
            csv_file_ << "Time,dt,Pos_X,Pos_Y,Pos_Z,Vel_X,Vel_Y,Vel_Z\n";
        }
        else
        {
            ROS_ERROR("Unable to open CSV file for writing.");
        }
        ROS_INFO_STREAM("Saving data to CSV file in the current directory with the name: " << file_name);     
    }

    Eigen::Vector3d computeDesiredPosition(double t, double w)
    {
        double x_des = 0.0;
        double y_des = 0.0;
        double z_des = 1.0;
        return Eigen::Vector3d(x_des, y_des, z_des);
    }

    Eigen::Vector3d computeDesiredVelocity(double t, double w)
    {
        double x_dot_des = 0.0;
        double y_dot_des = 0.0;
        double z_dot_des = 0.0;
        return Eigen::Vector3d(x_dot_des, y_dot_des, z_dot_des);
    }

    Eigen::Vector3d computeDesiredAcceleration(double t, double w)
    {
        double x_ddot_des = 0.0;
        double y_ddot_des = 0.0;
        double z_ddot_des = 0.0;
        return Eigen::Vector3d(x_ddot_des, y_ddot_des, z_ddot_des);
    }

    Eigen::Matrix2d H(double yaw)
    {
        Eigen::Matrix2d H;
        H(0, 0) = std::cos(yaw);
        H(0, 1) = std::sin(yaw);
        H(1, 0) = std::sin(yaw);
        H(1, 1) = -std::cos(yaw);
        return H;
    }

    Eigen::Matrix2d H_inv(double yaw)
    {
        Eigen::Matrix2d H_inv;
        H_inv(0, 0) = std::cos(yaw);
        H_inv(0, 1) = std::sin(yaw);
        H_inv(1, 0) = std::sin(yaw);
        H_inv(1, 1) = -std::cos(yaw);
        return H_inv;
    }

    double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    // Member variables
    bool there_is_pose_;
    std::string pose_topic_, cmd_vel_topic_, takeoff_topic_, land_topic_;
    int task_;
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_, odometry_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Timer control_timer_;

    // State variables
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    tf::Quaternion current_orientation_;
    double current_yaw_;

    Eigen::Vector3d previous_position_, previous_velocity_;
    ros::Time previous_pose_time_;

    // Mutex for thread safety
    std::mutex state_mutex_;

    // Control parameters
    double control_frequency_;
    double dt_;

    Eigen::Matrix2d Kp_;
    Eigen::Matrix2d Kd_;
    Eigen::Matrix2d Ki_;
    double K_z_;
    double K_yaw_;
    double alpha_velocity_gain_;
    double mass_model_;
    double mass_estimated_;
    double g_;
    double t_;
    Eigen::Matrix2d Ku_;
    Eigen::Matrix2d Ku_inv_control_;

    double max_theta_phi_deg_;
    double max_theta_phi_;
    double max_z_dot_;
    double max_yaw_dot_deg_;
    double max_yaw_dot_;

    double w_; // Angular frequency

    ros::Time initial_time_;
    bool save_data_;
    std::ofstream csv_file_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_controller_node");
    DroneController controller;
    ros::spin();
    return 0;
}