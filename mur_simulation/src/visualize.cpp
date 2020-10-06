#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"viualization");
    ros::NodeHandle n;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::Point p_1;
    geometry_msgs::Point p_2;
    geometry_msgs::Point p_3;

    p_1.x = 0.0;
    p_1.y = 0.0;
    p_1.z = 0.0;

    p_2.x = 1.0;
    p_2.y = 1.0;
    p_2.z = 1.0;

    p_3.x = 2.0;
    p_3.y = 2.0;
    p_3.z = 2.0;

    std::vector<geometry_msgs::Point> my_points;
    my_points.push_back(p_1);
    my_points.push_back(p_2);
    my_points.push_back(p_3);

    while(ros::ok()){

        for (int ii = 0; ii < my_points.size(); ++ii)
        {
            std_msgs::ColorRGBA c;
            if( ii == 0)
                c.r = 1.0;
            else if(ii == 1)
                c.g = 1.0;
            else
                c.b = 1.0;
            c.a = 1.0;

            marker.points.push_back(my_points[ii]);
            // Here, the field colors is populated with a specific color per point.
            marker.colors.push_back(c);
        }

        vis_pub.publish(marker);
    }
    ros::spin();
}