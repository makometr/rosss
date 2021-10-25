#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

auto GRID_WIDTH = 30;
auto GRID_HEIGHT = 30;
auto GRID_RESOLUTION = 0.1;

ros::Publisher original;
ros::Publisher noOutliers;
ros::Publisher occupGrid;

auto polarToCart = [](float r, float theta) -> std::pair<float, float>
{
    return {r * cos(theta), r * sin(theta)};
};

auto dist = [](auto &p1, auto &p2) -> float
{
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
};

auto removeOutliers = [](auto &ranges, float angle_min, float angle_step) -> std::vector<float>
{
    auto OVERSTEOP = 3;
    auto MAX_FLOAT = 5000.0;
    auto MAX_DISTANCE = 0.05;
    std::vector<float> results(OVERSTEOP, MAX_FLOAT);

    for (int i = OVERSTEOP; i < ranges.size() - OVERSTEOP; i++)
    {
        auto angle_l = angle_min + angle_step * (i + OVERSTEOP);
        auto angle = angle_min + angle_step * i;
        auto angle_r = angle_min + angle_step * (i - OVERSTEOP);

        auto pos_l = polarToCart(ranges[i + OVERSTEOP], angle_l);
        auto pos = polarToCart(ranges[i], angle);
        auto pos_r = polarToCart(ranges[i - OVERSTEOP], angle_r);

        if (dist(pos_l, pos) + dist(pos_r, pos) < MAX_DISTANCE * OVERSTEOP * 2.0)
            results.push_back(ranges[i]);
        else
            results.push_back(MAX_FLOAT);
    }

    for (int i = 0; i < OVERSTEOP; i++)
        results.push_back(MAX_FLOAT);

    return results;
};

void processLaserScan(const sensor_msgs::LaserScanConstPtr &msg)
{
    std::cout << msg->header << "\n";
    original.publish(msg);

    sensor_msgs::LaserScan newMsg = *msg;
    newMsg.ranges = removeOutliers(msg->ranges, msg->angle_min, msg->angle_increment);
    noOutliers.publish(newMsg);

    auto gridMsg = nav_msgs::OccupancyGrid{};

    auto width = int(GRID_WIDTH / GRID_RESOLUTION);
    auto height = int(GRID_HEIGHT / GRID_RESOLUTION);
    gridMsg.header.frame_id = "base_link";
    gridMsg.info.width = width;
    gridMsg.info.height = height;
    gridMsg.info.resolution = GRID_RESOLUTION;
    gridMsg.info.origin.position.x = -GRID_WIDTH / 2.0;
    gridMsg.info.origin.position.y = -GRID_HEIGHT / 2.0;
    gridMsg.data = std::vector<int8_t>(-1, (width * height));

    occupGrid.publish(gridMsg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/base_scan", 0, processLaserScan);
    original = n.advertise<sensor_msgs::LaserScan>("/base_scan_original", 10);
    noOutliers = n.advertise<sensor_msgs::LaserScan>("/base_scan_no_outliers", 10);
    occupGrid = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 10);

    while (n.ok())
    {
        ros::spinOnce();
    }

    return 0;
}