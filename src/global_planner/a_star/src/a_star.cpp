#include "a_star/a_star.hpp"

namespace global_planner
{
    aStar::aStar(const rclcpp::NodeOptions &options) : Node("global_planner", options)
    {
        // initialize parameters

        // initialize subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&aStar::mapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/state_estimation", 10, std::bind(&aStar::odomCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&aStar::goalCallback, this, std::placeholders::_1));

        // initialize publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
        dilated_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("dilated_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    }

    aStar::~aStar()
    {
    }

    void aStar::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Map is loading...");
        width = msg->info.width;                             // map width in pixels
        height = msg->info.height;                           // map height in pixels
        resolution = msg->info.resolution;                   // map resolution in meters/pixel
        origin_x = msg->info.origin.position.x / resolution; // map origin x in pixels
        origin_y = msg->info.origin.position.y / resolution; // map origin y in pixels
        RCLCPP_INFO(get_logger(), "Map is loaded! Map info: width = %d, height = %d, resolution = %f", width, height, resolution);
        
        map.clear(); // clear the map data

        for (int i = 0; i < height; i++)
            map.push_back(std::vector<int>(width));
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                // calculate the index of the current cell in the data array
                int index = i * width + j;

                // get the value of the current cell
                int value = msg->data[index];

                // store the value of the current cell in the 2D vector
                map[i][j] = value;
            }
        }

        // Dilate the map
        std::vector<std::vector<int>> new_map = map;

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                if (map[i][j] == 100) // If the cell is an obstacle
                {
                    // Dilate the obstacle
                    for (int di = -dilate_radius_; di <= dilate_radius_; di++)
                    {
                        for (int dj = -dilate_radius_; dj <= dilate_radius_; dj++)
                        {
                            // Check if the cell is within the map boundaries
                            if (i + di >= 0 && i + di < height && j + dj >= 0 && j + dj < width)
                            {
                                // Set the cell as an obstacle in the new map
                                new_map[i + di][j + dj] = 100;
                            }
                        }
                    }
                }
            }
        }

        map = new_map; // Copy the dilated map back to the original map

        // Publish the dilated map
        nav_msgs::msg::OccupancyGrid dilated_map;
        dilated_map.header = msg->header;
        dilated_map.info = msg->info;
        dilated_map.data = std::vector<signed char>(width * height);
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                dilated_map.data[i * width + j] = map[i][j];
            }
        }
        dilated_map_pub_->publish(dilated_map);
        RCLCPP_INFO(get_logger(), "Dilated map is published!");
    }

    void aStar::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (msg == nullptr)
            return;
        startPoint.y = msg->pose.pose.position.y / resolution - origin_y;
        startPoint.x = msg->pose.pose.position.x / resolution - origin_x;
        // RCLCPP_INFO(get_logger(), "Odometry is received!");
    }

    void aStar::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg == nullptr)
            return;
        // Check if the goal pose is within the map boundaries and not on an obstacle
        if (msg->pose.position.x / resolution < origin_x || msg->pose.position.x / resolution >= origin_x + width || msg->pose.position.y / resolution < origin_y || msg->pose.position.y / resolution >= origin_y + height)
        {
            RCLCPP_INFO(get_logger(), "Goal pose is out of map!");
            return;
        }
        if (map[msg->pose.position.y / resolution - origin_y][msg->pose.position.x / resolution - origin_x] == 100)
        {
            RCLCPP_INFO(get_logger(), "Goal pose is on the obstacle!");
            return;
        }
        endPoint.y = msg->pose.position.y / resolution - origin_y;
        endPoint.x = msg->pose.position.x / resolution - origin_x;
        startPoint.h = (abs(startPoint.x - endPoint.x) + abs(startPoint.y - endPoint.y)) / resolution;
        isEndPointUpdated = true;

        RCLCPP_INFO(get_logger(), "Goal posi: x = %f, y = %f", msg->pose.position.x, msg->pose.position.y);
        // RCLCPP_INFO(get_logger(), "Goal pose: x = %d, y = %d", endPoint.x, endPoint.y);

        // get the path
        if (map.size() > 0)
        {
            if (startPoint.x != -1 && startPoint.y != -1 && endPoint.x != -1 && endPoint.y != -1 && isEndPointUpdated)
            {
                std::vector<Point *> path = getPath(&startPoint, &endPoint);
                std::vector<Point *> my_path = path;

                if (my_path.size() > 0)
                {
                    // path publisher
                    nav_msgs::msg::Path path_msg;
                    path_msg.header.stamp = rclcpp::Clock().now();
                    path_msg.header.frame_id = "map";
                    for (auto it = my_path.rbegin(); it != my_path.rend(); it++)
                    {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = path_msg.header;
                        pose.pose.position.x = ((*it)->x + origin_x) * resolution;
                        pose.pose.position.y = ((*it)->y + origin_y) * resolution;
                        pose.pose.position.z = 0;
                        path_msg.poses.push_back(pose);
                    }
                    RCLCPP_INFO(get_logger(), "Path is published!");
                    path_pub_->publish(path_msg);
                }
                isEndPointUpdated = false;
            }
        }
    }

    std::vector<Point *> aStar::getNeighbours(Point *p){
        std::vector<Point *> neighbors;
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 && j == 0)
                    continue;
                if (i != 0 && j != 0)
                    continue;
                neighbors.push_back(new Point(p->x + i, p->y + j));
            }
        }
        return neighbors;
    }

    std::vector<Point *> aStar::getPath(Point *start, Point *end)
    {
        rclcpp::Time start_time = rclcpp::Clock().now();
        // 初始化table，0-未访问，-1-已访问（在closelist中），>0-代价g
        std::vector<std::vector<int>> table(height, std::vector<int>(width));
        for (auto &row : table)
        {
            std::fill(row.begin(), row.end(), 0);
        }

        std::priority_queue<Point *, std::vector<Point *>, bool (*)(Point *, Point *)> openList([](Point *a, Point *b)
                                                                                                { return a->f > b->f; });
        openList.push(start);
        table[start->y][start->x] = start->g;

        while (!openList.empty())
        {
            // 取出f值最小的点
            Point *current = openList.top();

            // 将当前点放入closelist中
            openList.pop();
            table[current->y][current->x] = -1;

            // 判断当前点是否目标点
            if (current->x == end->x && current->y == end->y)
            {
                rclcpp::Duration duration = rclcpp::Clock().now() - start_time;
                RCLCPP_INFO(get_logger(), "Time taken by found path: %f", duration.seconds());
                // 回溯路径
                end->parent = current;
                std::vector<Point *> path;
                Point *p = end;
                while (p)
                {
                    path.push_back(p);
                    p = p->parent;
                }
                // RCLCPP_INFO(get_logger(), "path's length is : %ld", path.size());
                return path;
            }

            // 获取周围点
            std::vector<Point *> neighbors = getNeighbours(current);

            // 遍历周围点
            for (auto neighbor : neighbors)
            {
                // 判断是否在网格内
                if (neighbor->x < 0 || neighbor->x >= width || neighbor->y < 0 || neighbor->y >= height)
                    continue;

                // 判断是否是障碍物
                if (map[neighbor->y][neighbor->x] == 100)
                    continue;

                // 判断是否在closelist
                if (table[neighbor->y][neighbor->x] == -1)
                    continue;

                // 计算代价
                neighbor->g = current->g + cost;
                neighbor->h = abs(neighbor->x - end->x) + abs(neighbor->y - end->y);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;

                // 判断否在openlist中，如果在，需要比较当前是否更优
                if (table[neighbor->y][neighbor->x] > 0)
                {
                    if (neighbor->g < table[neighbor->y][neighbor->x])
                    {
                        table[neighbor->y][neighbor->x] = neighbor->g;
                        neighbor->parent = current;
                    }
                }
                // 如果不在openlist中，将其放入openlist
                else
                {
                    openList.push(neighbor);
                    table[neighbor->y][neighbor->x] = neighbor->g;
                }
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("getPath"), "Path not found!");
        return {};
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(global_planner::aStar)