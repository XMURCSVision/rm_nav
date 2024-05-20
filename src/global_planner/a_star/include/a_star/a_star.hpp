#ifndef GLOBAL_PLANNER__A_STAR_HPP_
#define GLOBAL_PLANNER__A_STAR_HPP_

// ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

// std
#include <vector>
#include <queue>

namespace global_planner
{
    struct Point
    {
        int x, y;
        int g, h, f;
        Point *parent;

        Point(int x = -1, int y = -1, int g = 0, int h = 0, int f = 0)
            : x(x), y(y), g(g), h(h), f(f), parent(nullptr) {}
    };

    class aStar : public rclcpp::Node
    {
    public:
        explicit aStar(const rclcpp::NodeOptions &options);
        ~aStar();
    private:
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        std::vector<Point *> getNeighbours(Point *p);
        std::vector<Point *> getPath(Point *start, Point *goal);

        // define parameters
        int cost = 1;
        int dilate_radius_ = 8;

        std::vector<std::vector<int>> map;
        int width, height;
        float resolution;
        int origin_x, origin_y;
        bool isEndPointUpdated = false;

        Point startPoint, endPoint;

        // subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

        // publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dilated_map_pub_;

    };
} // namespace global_planner

#endif  // GLOBAL_PLANNER__A_STAR_HPP_