#include <stdio.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/OccupancyGrid.h>



int main(int argc, char** argv)
{


    ros::init(argc, argv, "grid");
    ros::NodeHandle n;

    ros::Rate loop_rate(1.0);

    ros::Publisher map_pub;
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_out", 1);

    nav_msgs::OccupancyGrid map_topic_msg;


    pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (argv[1], *src);

    //cloud_to_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg);

    while(ros::ok())
    {
        map_pub.publish(map_topic_msg);
        loop_rate.sleep();
        ros::spinOnce();    	
    }


	printf("test\n");
	return 0;
}