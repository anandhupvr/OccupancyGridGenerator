#include <stdio.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/OccupancyGrid.h>


void setMapTopicMsg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg)
{
    msg.header.seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = ros::Time::now();
    msg.info.resolution = 0.05;

    double x_min, x_max, y_min, y_max;

    printf("before loop map \n");

    for (int i = 0; i < cloud->points.size() - 1; i++)
    {
        if ((cloud->points[i].z < 1) && (cloud->points[i].z < 2))
        {
            if (i == 0)
            {
                x_min = x_max = cloud->points[i].x;
                y_min = y_max = cloud->points[i].y;
            }


            double x = cloud->points[i].x;
            double y = cloud->points[i].y;

            if (x < x_min) x_min = x;
            if(x > x_max) x_max = x;

            if(y < y_min) y_min = y;
            if(y > y_max) y_max = y;
        }
    }

    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 0.0;

    msg.info.width = int((x_max - x_min) / msg.info.resolution);
    msg.info.height = int((y_max - y_min) / msg.info.resolution);

    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 0);

    for (int iter = 0; iter < cloud->points.size(); iter++)
    {
        if((cloud->points[iter].z > 1) && (cloud->points[iter].z < 3))
        {
            int i = int((cloud->points[iter].x - x_min) / msg.info.resolution);
            if (i < 0 || i >= msg.info.width) continue;

            int j = int((cloud->points[iter].y - y_min) / msg.info.resolution);
            if(j < 0 || j >= msg.info.height - 1) continue;

            msg.data[i + j * msg.info.width] = 100;
        }
        else
        {

            int i = int((cloud->points[iter].x - x_min) / msg.info.resolution);
            if (i < 0 || i >= msg.info.width) continue;

            int j = int((cloud->points[iter].y - y_min) / msg.info.resolution);
            if(j < 0 || j >= msg.info.height - 1) continue;


            msg.data[i + j * msg.info.width] = 0;
        }
    }

}

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

    // convert point to map msg
    setMapTopicMsg(src, map_topic_msg);

    while(ros::ok())
    {
        map_pub.publish(map_topic_msg);
        loop_rate.sleep();
        ros::spinOnce();    	
    }


	printf("test\n");
	return 0;
}