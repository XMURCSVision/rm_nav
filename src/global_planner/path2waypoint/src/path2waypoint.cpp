// ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

// std
#include <string>
#include <vector>

std::string odom_topic = "/state_estimation";
std::string path_topic = "/global_path";
float distance_thre = 1.2;
float distance_tole = 0.6;

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;
double way_point_temp_x = 0;
double way_point_temp_y = 0;

bool Flag_get_new_path;
bool Flag_finish_path;
bool Flag_switch_goal;

rclcpp::Node::SharedPtr node;

std::vector<geometry_msgs::msg::PoseStamped> way_point_array;

void poseHandler(const nav_msgs::msg::Odometry::SharedPtr msg)
{
        curTime = msg->header.stamp.sec;

        vehicleX = msg->pose.pose.position.x;
        vehicleY = msg->pose.pose.position.y;
        vehicleZ = msg->pose.pose.position.z;
}

void pathHandler(const nav_msgs::msg::Path::SharedPtr msg)
{
    if(!msg->poses.empty())
    {
        Flag_get_new_path = true;
        way_point_array.clear();
        std::vector<geometry_msgs::msg::PoseStamped> path_array = msg->poses;
        geometry_msgs::msg::PoseStamped first_point = *path_array.begin();
        geometry_msgs::msg::PoseStamped last_point = *(path_array.end() - 1);

        float distance_2d = 0;
        double temp_x = vehicleX;
        double temp_y = vehicleY;

        for(const auto& pose_stamped: path_array)
        {
            distance_2d = sqrt(pow(pose_stamped.pose.position.x - temp_x, 2) + pow(pose_stamped.pose.position.y - temp_y, 2));
            if(distance_2d > distance_thre)
            {
                way_point_array.push_back(pose_stamped);
                temp_x = pose_stamped.pose.position.x;
                temp_y = pose_stamped.pose.position.y;
                RCLCPP_INFO(node->get_logger(), "choosen point: %f, %f", temp_x, temp_y);
            }
        }
        way_point_array.push_back(last_point);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("path2waypoint");

    node->declare_parameter("odom_topic", odom_topic);
    node->declare_parameter("path_topic", path_topic);
    node->declare_parameter("distance_thre", distance_thre);
    node->declare_parameter("distance_tole", distance_tole);
    
    odom_topic = node->get_parameter("odom_topic").as_string();
    path_topic = node->get_parameter("path_topic").as_string();
    distance_thre = node->get_parameter("distance_thre").as_double();
    distance_tole = node->get_parameter("distance_tole").as_double();

    auto pose_sub = node->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, poseHandler);
    auto path_sub = node->create_subscription<nav_msgs::msg::Path>(path_topic, 10, pathHandler);

    auto waypoint_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("way_point", 10);
    
    geometry_msgs::msg::PointStamped waypointMsgs;
    waypointMsgs.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped goal_point;
    int path_index = 0;
    float goal_point_distance = 0;

    rclcpp::Rate rate(100);
    bool state = rclcpp::ok();
    while(state)
    {
        rclcpp::spin_some(node);

        if(!way_point_array.empty())
        {
            if (Flag_get_new_path)
            {
                goal_point = way_point_array.front();
                path_index = 0;
                Flag_switch_goal = true;
                Flag_get_new_path = false;
            }
            else
            {
                
                goal_point_distance = sqrt(pow(goal_point.pose.position.x - vehicleX, 2) 
                                        + pow(goal_point.pose.position.y - vehicleY, 2));
                
                if((goal_point_distance < distance_tole)&&(path_index < static_cast<int>(way_point_array.size())-1))
                {
                    path_index ++;
                    goal_point = way_point_array.at(path_index);
                    Flag_switch_goal = true;
                }
                else if(path_index == static_cast<int>(way_point_array.size())-1)
                {
                    Flag_finish_path = true;
                }
                
            }
        }

        if(Flag_switch_goal)
        {
            waypointMsgs.header.stamp = node->now();
            waypointMsgs.point.x = goal_point.pose.position.x;
            waypointMsgs.point.y = goal_point.pose.position.y;
            waypointMsgs.point.z = 0;
            waypoint_pub->publish(waypointMsgs);
        }

        state = rclcpp::ok();
        rate.sleep();
    }

    return 0;
}