#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>
#include <tf/transform_broadcaster.h>
#include <mutex>
#include <shared_mutex>

const double PI = 3.141592654;

class TurtlePosition
{
    geometry_msgs::Pose2D pose;
    mutable std::shared_mutex mutex_;

public:
    geometry_msgs::Pose2D get() const
    {
        std::shared_lock lock(mutex_);
        return pose;
    }

    void set(const geometry_msgs::Pose2D &newPose)
    {
        std::unique_lock lock(mutex_);
        pose = newPose;
    }
};

ros::Publisher chatter_pub;
TurtlePosition TurtleOnePos;
TurtlePosition LeoPos;

void getLeoPos(const turtlesim::PoseConstPtr &msg)
{
    // ROS_INFO("Leo Pos: [%f, %f, %f]", msg->x, msg->y, msg->theta);
    geometry_msgs::Pose2D kek;
    kek.x = msg->x;
    kek.y = msg->y;
    kek.theta = msg->theta;
    LeoPos.set(kek);
}

void getTurtle1Position(const turtlesim::PoseConstPtr &msg)
{
    // ROS_INFO("Turtle Pos: [%f, %f, %f]", msg->x, msg->y, msg->theta);
    geometry_msgs::Pose2D kek;
    kek.x = msg->x;
    kek.y = msg->y;
    kek.theta = msg->theta;
    TurtleOnePos.set(kek);
}

double getDistance(const geometry_msgs::Pose2D &cur, const geometry_msgs::Pose2D &goal)
{
    return sqrt(pow(goal.x - cur.x, 2) + pow(goal.y - cur.y, 2));
}

double getHeadingError(const geometry_msgs::Pose2D &cur, const geometry_msgs::Pose2D &goal)
{

    double deltaX = goal.x - cur.x;
    double deltaY = goal.y - cur.y;
    double waypointHeading = atan2(deltaY, deltaX);
    double headingError = waypointHeading - cur.theta;

    // Make sure heading error falls within -PI to PI range
    if (headingError > PI)
    {
        headingError = headingError - (2 * PI);
    }
    if (headingError < -PI)
    {
        headingError = headingError + (2 * PI);
    }

    return headingError;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("turtle1/pose", 0, getTurtle1Position);
    ros::Subscriber sub_2 = n.subscribe("leo/pose", 0, getLeoPos);
    chatter_pub = n.advertise<geometry_msgs::Twist>("/leo/cmd_vel", 10);

    ros::Rate loop_rate{10}; // max fps
    while (n.ok())
    {
        ros::spinOnce();
        geometry_msgs::Twist leo_msg; // создаем сообщение

        auto cur = LeoPos.get();
        auto goal = TurtleOnePos.get();

        double distanceToWaypoint = getDistance(cur, goal);
        double headingError = getHeadingError(cur, goal);

        if ((abs(distanceToWaypoint) > 0.1) || abs(headingError) > 0.1)
        {
            leo_msg.angular.z = 2 * headingError;
            leo_msg.linear.x = 2 * distanceToWaypoint;
        }
        else
        {
            ROS_INFO("Goal has been reached!");
            leo_msg.linear.x = 0.0;
            leo_msg.angular.z = 0.0;
        }

        chatter_pub.publish(leo_msg);
        loop_rate.sleep();
    }

    return 0;
}
