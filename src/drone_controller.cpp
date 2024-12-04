#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <cmath>
#include <mutex>
#include <fstream>
#include <ros/package.h>
#include <boost/filesystem.hpp>

class DroneController
{
public:
    DroneController() : nh_("~")
    {   
        // Perder o corpo, paredes virtuais, joystick, gráficos.

        // Params
        nh_.param("save_data", save_data_, false);
        nh_.param("pose_topic", pose_topic_, std::string("/natnet_ros/B1/pose"));
        nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/B1/cmd_vel"));
        nh_.param("joystick_topic", joystick_topic_, std::string("/joy"));
        nh_.param("takeoff_topic", takeoff_topic_, std::string("/B1/takeoff"));
        nh_.param("land_topic", land_topic_, std::string("/B1/land"));
        nh_.param("control_frequency", control_frequency_, 30.0); 
        nh_.param("time_without_pose", time_without_pose_, 0.5);
        nh_.param("x_wall_min", x_wall_min_, -1.5);
        nh_.param("x_wall_max", x_wall_max_, 1.5);
        nh_.param("y_wall_min", y_wall_min_, -1.5);
        nh_.param("y_wall_max", y_wall_max_, 1.5);
        nh_.param("z_wall_max", z_wall_max_, 2.0);
        nh_.param("dont_takeoff", dont_takeoff_, true);
        nh_.param("package_name", package_name_, std::string("aerial_robotics"));

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
        nh_.param("Angular_frequency", w_, 2 * M_PI / 20);
        nh_.param("joystick_deadzone", joystick_deadzone_, 0.05);

        // Task
        nh_.param("task", task_, 1); // 1: Positioning, 2: Circular trajectory, 3: Lemniscate trajectory, 4: Pringles trajectory

        pose_sub_ = nh_.subscribe(pose_topic_, 1, &DroneController::poseCallback, this);
        joystick_sub_ = nh_.subscribe(joystick_topic_, 1, &DroneController::joystickCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        takeoff_pub_ = nh_.advertise<std_msgs::Empty>(takeoff_topic_, 1);
        land_pub_ = nh_.advertise<std_msgs::Empty>(land_topic_, 1);

        if (save_data_)
        {   
            save_data();
        } else {
            ROS_INFO("Data will not be saved.");
        }

        there_is_pose_ = false;
        did_takeoff_ = false;
        emergency_stop_active_ = false;
        is_joy_active_ = false;
        is_landing_ = false;

        // Initialize control parameters
        dt_ = 1.0 / control_frequency_;
        g_ = 9.81;

        Ku_inv_control_ = (1 / g_) * Eigen::Matrix2d::Identity();

        max_theta_phi_ = deg2rad(max_theta_phi_deg_);
        max_yaw_dot_ = deg2rad(max_yaw_dot_deg_);
        
        // Set up the control loop timer
        control_timer_ = nh_.createTimer(ros::Duration(dt_), &DroneController::controlLoop, this);
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

            tf2::fromMsg(msg->pose.orientation, current_orientation_);

            there_is_pose_ = true;
            return;
        }

        double dt = (current_time - previous_pose_time_).toSec();

        current_position_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        tf2::fromMsg(msg->pose.orientation, current_orientation_);

        if (!(previous_position_ == current_position_)){
            last_pose_time_ = ros::Time::now();
        }

        if (dt > 0.0)
        {
            Eigen::Vector3d new_velocity_ = (current_position_ - previous_position_) / dt;

            current_velocity_ = alpha_velocity_gain_ * new_velocity_ + (1 - alpha_velocity_gain_) * previous_velocity_;

            previous_velocity_ = current_velocity_;
            previous_position_ = current_position_;
            previous_pose_time_ = current_time;
        } 
    }

    void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {   
        std::lock_guard<std::mutex> lock(joy_mutex_);
        is_joy_active_ = true;
        joy_msg_ = msg;

        if(!did_takeoff_)
        {
            ROS_INFO("Press X to takeoff, A to land, and use the left joystick to control the drone.");
        }

        if (msg->buttons[2] == 1)
        {
            if (!did_takeoff_)
            {
                takeoff();
                initial_time_ = ros::Time::now();
            }
        }
        else if (msg->buttons[0] == 1)
        {
            stop();
            land();
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
            tf2::Matrix3x3(tf2::Quaternion(current_orientation_)).getRPY(roll, pitch, yaw);
            current_yaw = yaw;
            
        }

        if ((ros::Time::now() - last_pose_time_).toSec() > time_without_pose_)
        {
            ROS_WARN_STREAM("No pose received in the last " << time_without_pose_ << " seconds. Stopping the drone.");
            emergencyStop();
        }

        if (current_position(0) < x_wall_min_ || current_position(0) > x_wall_max_ || current_position(1) < y_wall_min_ || current_position(1) > y_wall_max_ || current_position(2) > z_wall_max_)
        {
            ROS_WARN("Drone is out of the allowed area. Stopping the drone.");
            emergencyStop();
        }

        if (!is_joy_active_)
        {
            ROS_WARN("No joystick commands received yet. Waiting...");
            return;
        }

        if (!did_takeoff_)
        {
            return;
        }

        if (emergency_stop_active_)
        {
            emergencyStop();
            return;
        }

        sensor_msgs::Joy joy_msg;
        {
            std::lock_guard<std::mutex> lock(joy_mutex_);
            joy_msg = *joy_msg_;
        }

        double joy_yaw = (joy_msg.axes[5] - joy_msg.axes[2])/2;

        if (std::abs(joy_msg.axes[1]) > joystick_deadzone_ || std::abs(joy_msg.axes[0]) > joystick_deadzone_ || std::abs(joy_yaw) > joystick_deadzone_ || std::abs(joy_msg.axes[4]) > joystick_deadzone_)
        {
            geometry_msgs::Twist cmd_msg;

            cmd_msg.linear.x = joy_msg.axes[1];
            cmd_msg.linear.y = joy_msg.axes[0];
            cmd_msg.linear.z = joy_msg.axes[4];
            cmd_msg.angular.z = joy_yaw;

            ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z, cmd_msg.angular.z);

            cmd_pub_.publish(cmd_msg);
            return;
        }

        if (is_landing_)
        {
            return;
        }

        ros::Time current_time = ros::Time::now();
        double t = (current_time - initial_time_).toSec();

        // Compute desired states
        Eigen::Vector3d X_des = computeDesiredPosition(t, w_);
        Eigen::Vector3d X_dot_des = computeDesiredVelocity(t, w_);
        Eigen::Vector3d X_ddot_des = computeDesiredAcceleration(t, w_);

        if (save_data_ && csv_file_.is_open())
        {
            csv_file_ << t << ","
                      << current_position(0) << "," << current_position(1) << "," << current_position(2) << ","
                      << current_velocity(0) << "," << current_velocity(1) << "," << current_velocity(2) << ", "
                      << X_des(0) << "," << X_des(1) << "," << X_des(2) << ","
                      << X_dot_des(0) << "," << X_dot_des(1) << "," << X_dot_des(2) << "\n";
        }

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
        Z_dot_ref = Z_dot_ref / max_z_dot_;

        // Control yaw
        double yaw_des = std::atan2(X_dot_des(1), X_dot_des(0));
        if (std::abs(yaw_des) > M_PI)
        {
            yaw_des = yaw_des - std::copysign(2 * M_PI, yaw_des);
        }

        double yaw_dot_des = 0.0;
        double yaw_til = yaw_des - current_yaw;

        if (std::abs(yaw_til) > M_PI)
        {
            yaw_til = yaw_til - std::copysign(2 * M_PI, yaw_til);
        }

        double yaw_dot_ref = yaw_dot_des + K_yaw_ * yaw_til;

        yaw_dot_ref = std::max(std::min(yaw_dot_ref, max_yaw_dot_), -max_yaw_dot_);
        yaw_dot_ref = yaw_dot_ref / max_yaw_dot_;

        // Publish control commands
        geometry_msgs::Twist cmd_msg;

        // Assuming the drone accepts roll and pitch angles in angular.x and angular.y
        cmd_msg.linear.x = theta_phi_ref(0); // Roll angle (theta)
        cmd_msg.linear.y = -theta_phi_ref(1); // Pitch angle (phi)
        cmd_msg.linear.z = Z_dot_ref;         // Vertical velocity
        cmd_msg.angular.z = yaw_dot_ref;      // Yaw rate

        ROS_INFO("X: %f, Y: %f, Z : %f, Yaw: %f", theta_phi_ref(0), -theta_phi_ref(1), Z_dot_ref, yaw_dot_ref);

        cmd_pub_.publish(cmd_msg);
    }

    void emergencyStop()
    {   
        geometry_msgs::Twist cmd_msg;
        emergency_stop_active_ = true;
        cmd_msg.linear.x = 0.0;
        cmd_msg.linear.y = 0.0;
        cmd_msg.linear.z = 0.0;
        cmd_msg.angular.z = 0.0;

        for (int i = 0; i < 10; i++)
        {
            cmd_pub_.publish(cmd_msg);
            ros::Duration(0.1).sleep();
        }

        ROS_WARN("Emergency stop. All control commands set to zero.");

        if (save_data_ && csv_file_.is_open())
        {
            csv_file_.close();
            ROS_INFO("CSV file closed.");
        }

        land();
    }

    void land()
    {   
        is_landing_ = true;
        ROS_INFO("Landing the drone.");
        std_msgs::Empty msg;
        for (int i = 0; i < 10; i++)
        {
            land_pub_.publish(msg);
            ros::Duration(0.1).sleep();
        }
    }

    void stop()
    {
        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = 0.0;
        cmd_msg.linear.y = 0.0;
        cmd_msg.linear.z = 0.0;
        cmd_msg.angular.z = 0.0;

        for (int i = 0; i < 10; i++)
        {
            cmd_pub_.publish(cmd_msg);
            ros::Duration(0.1).sleep();
        }
    }

    void takeoff()
    {
        did_takeoff_ = true;
        if (dont_takeoff_)
        {
            ROS_WARN("Takeoff is disabled. Please set the 'dont_takeoff' parameter to false to enable takeoff.");
            return;
        }
        
        std_msgs::Empty msg;
        for (int i = 0; i < 10; i++)
        {
            takeoff_pub_.publish(msg);
            ros::Duration(0.1).sleep();
        }
    }

    void save_data()
    {   
        //getting the date and time to create a unique file name
        
        std::string path = ros::package::getPath(package_name_) + "/data/";

        if (!boost::filesystem::exists(path))
        {
            if (boost::filesystem::create_directory(path))
            {
            ROS_INFO("Data folder created successfully.");
            }
            else
            {
            ROS_ERROR("Failed to create data folder.");
            }
        }

        time_t now = time(0);
        tm *ltm = localtime(&now);
        std::string file_name = path + "drone_data_" + std::to_string(1900 + ltm->tm_year) + "_" + std::to_string(1 + ltm->tm_mon) + "_" + std::to_string(ltm->tm_mday) + "_" + std::to_string(ltm->tm_hour) + "_" + std::to_string(ltm->tm_min) + "_" + std::to_string(ltm->tm_sec) + ".csv";
        csv_file_.open(file_name);
        if (csv_file_.is_open())
        {
            csv_file_ << "Time,Pos_X,Pos_Y,Pos_Z,Vel_X,Vel_Y,Vel_Z,Pos_X_des,Pos_Y_des,Pos_z_des,Vel_X_des,Vel_Y_des,Vel_Z_des\n";
        }
        else
        {
            ROS_ERROR("Unable to open CSV file for writing.");
        }
        ROS_INFO_STREAM("Saving data to CSV file in the current directory with the name: " << file_name);     
    }

    Eigen::Vector3d computeDesiredPosition(double t, double w)
    {
        if (task_ == 1)
        {
            double x_des = 0.0;
            double y_des = 0.0;
            double z_des = 1.0;
            return Eigen::Vector3d(x_des, y_des, z_des);
        }
        else if (task_ == 2)
        {
            double x_des = std::cos(w * t);
            double y_des = std::sin(w * t);
            double z_des = 1.5 + 0.25 * std::sin(w * t);
            return Eigen::Vector3d(x_des, y_des, z_des);
        }
        else if (task_ == 3)
        {
            double x_des = std::cos(w * t);
            double y_des = std::sin(2 * w * t);
            double z_des = 1.5 + 0.25 * std::sin(w * t);
            return Eigen::Vector3d(x_des, y_des, z_des);
        }
        else if (task_ == 4)
        {
            double x_des = std::cos(w * t);
            double y_des = std::sin(w * t);
            double z_des = 1.5 + 0.25 * std::sin(2 * w * t);
            return Eigen::Vector3d(x_des, y_des, z_des);
        }
        else
        {
            ROS_ERROR("Invalid task number. Please set the task parameter to 1, 2, 3, or 4. Defaulting to task 1.");
            double x_des = 0.0;
            double y_des = 0.0;
            double z_des = 1.0;
            return Eigen::Vector3d(x_des, y_des, z_des);
        }
    }

    Eigen::Vector3d computeDesiredVelocity(double t, double w)
    {
        if (task_ == 1) {
            double x_dot_des = 0.0;
            double y_dot_des = 0.0;
            double z_dot_des = 0.0;
            return Eigen::Vector3d(x_dot_des, y_dot_des, z_dot_des);
        }
        else if (task_ == 2)
        {
            double x_dot_des = -w * std::sin(w * t);
            double y_dot_des = w * std::cos(w * t);
            double z_dot_des = 0.25 * w * std::cos(w * t);
            return Eigen::Vector3d(x_dot_des, y_dot_des, z_dot_des);
        }
        else if (task_ == 3)
        {
            double x_dot_des = -w * std::sin(w * t);
            double y_dot_des = 2 * w * std::cos(2 * w * t);
            double z_dot_des = 0.25 * w * std::cos(w * t);
            return Eigen::Vector3d(x_dot_des, y_dot_des, z_dot_des);
        }
        else if (task_ == 4)
        {
            double x_dot_des = -w * std::sin(w * t);
            double y_dot_des = w * std::cos(w * t);
            double z_dot_des = 0.5 * w * std::cos(2 * w * t);
            return Eigen::Vector3d(x_dot_des, y_dot_des, z_dot_des);
        }
        else
        {
            ROS_ERROR("Invalid task number. Please set the task parameter to 1, 2, 3, or 4. Defaulting to task 1.");
            double x_dot_des = 0.0;
            double y_dot_des = 0.0;
            double z_dot_des = 0.0;
            return Eigen::Vector3d(x_dot_des, y_dot_des, z_dot_des);
        }
    }

    Eigen::Vector3d computeDesiredAcceleration(double t, double w)
    {
        if (task_ == 1) {
            double x_ddot_des = 0.0;
            double y_ddot_des = 0.0;
            double z_ddot_des = 0.0;
            return Eigen::Vector3d(x_ddot_des, y_ddot_des, z_ddot_des);
        }
        else if (task_ == 2)
        {
            double x_ddot_des = -w * w * std::cos(w * t);
            double y_ddot_des = -w * w * std::sin(w * t);
            double z_ddot_des = -0.25 * w * w * std::sin(w * t);
            return Eigen::Vector3d(x_ddot_des, y_ddot_des, z_ddot_des);
        }
        else if (task_ == 3)
        {
            double x_ddot_des = -w * w * std::cos(w * t);
            double y_ddot_des = -4 * w * w * std::sin(2 * w * t);
            double z_ddot_des = -0.25 * w * w * std::sin(w * t);
            return Eigen::Vector3d(x_ddot_des, y_ddot_des, z_ddot_des);
        }
        else if (task_ == 4)
        {
            double x_ddot_des = -w * w * std::cos(w * t);
            double y_ddot_des = -w * w * std::sin(w * t);
            double z_ddot_des = -w * w * std::sin(2 * w * t);
            return Eigen::Vector3d(x_ddot_des, y_ddot_des, z_ddot_des);
        }
        else
        {
            ROS_ERROR("Invalid task number. Please set the task parameter to 1, 2, 3, or 4. Defaulting to task 1.");
            double x_ddot_des = 0.0;
            double y_ddot_des = 0.0;
            double z_ddot_des = 0.0;
            return Eigen::Vector3d(x_ddot_des, y_ddot_des, z_ddot_des);
        }
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
    bool there_is_pose_, did_takeoff_, emergency_stop_active_, is_joy_active_, dont_takeoff_, is_landing_;
    std::string pose_topic_, cmd_vel_topic_, takeoff_topic_, land_topic_, joystick_topic_;
    std::string package_name_;
    sensor_msgs::Joy::ConstPtr joy_msg_;
    int task_;
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_, odometry_sub_, joystick_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Timer control_timer_;

    // State variables
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    tf2::Quaternion current_orientation_;
    double current_yaw_;

    Eigen::Vector3d previous_position_, previous_velocity_;
    ros::Time previous_pose_time_;

    // Mutex for thread safety
    std::mutex state_mutex_;
    std::mutex joy_mutex_; 

    // Control parameters
    double control_frequency_;
    double time_without_pose_;
    double x_wall_min_, x_wall_max_, y_wall_min_, y_wall_max_, z_wall_max_;
    ros::Time last_pose_time_;
    double dt_;

    Eigen::Matrix2d Kp_;
    Eigen::Matrix2d Kd_;
    Eigen::Matrix2d Ki_;
    double K_z_;
    double K_yaw_;
    double alpha_velocity_gain_;
    double joystick_deadzone_;
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