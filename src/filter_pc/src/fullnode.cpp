#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <cstdlib> 


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <pcl/console/parse.h>
#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <thread>


ros::Subscriber pcl_sub;
ros::Publisher lane_pcl_pub;
float min = 5.0;


void pointCloudCallback(const sensor_msgs::PointCloud2 &cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
  int pointBytes = cloud_msg.point_step;
  int offset_x;
  int offset_y;
  int offset_z;
  int offset_int;
  float y_threshold = 2.0;

  float z_threshold = 12.0;
  //So basically what we need to do here, is that first obtain the PointCloud data coming in from the Zed Node as XYZrgb,
  //and then convert the resultant point into XYZI using inbuilt functions from the point cloud library 
  cloud.header.frame_id = cloud_msg.header.frame_id;
  for (int f=0; f<cloud_msg.fields.size(); ++f)
  {
    if (cloud_msg.fields[f].name == "x")
      offset_x = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "y")
      offset_y = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "z")
      offset_z = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "rgb")
      offset_int = cloud_msg.fields[f].offset;
  } 
    float max_y = 0.0 , min_y = 0.0;

    for (int p=0; p<cloud_msg.width; ++p)
    {
      pcl::PointXYZRGB pt;

      pt.x = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + offset_x);
      pt.y = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + offset_y);
      pt.z = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + offset_z);
      pt.rgb =  *(float*)(&cloud_msg.data[0] + (pointBytes*p) + offset_int);
      if(pt.y > max_y)
        max_y = pt.y;
      else if(pt.y < min_y)
        min_y = pt.y;
      temp_cloud.points.push_back(pt);
    }
    max_y = 0.6*max_y;
    min_y = 0.8*min_y;
    for(pcl::PointXYZRGB point:temp_cloud.points)
    {
      //Getting the 'r' 'g' 'b' components
      std::uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
      std::uint8_t r = (rgb >> 16) & 0x0000ff;
      std::uint8_t g = (rgb >> 8)  & 0x0000ff;
      std::uint8_t b = (rgb)       & 0x0000ff;
      
      bool is_white = ((r>100) && (g>100)  && (b>100));
      if(point.z < z_threshold)
      {
        if(is_white || point.y<max_y)
        {
            cloud.points.push_back(point);
        }
        else
        {
            point.y = point.y - 200;
            cloud.points.push_back(point);
        }
      }
    }
    // cloud.header.frame_id = "base_link";
    lane_pcl_pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LanePointCloudGen");

    ros::NodeHandle nh;

    pcl_sub = nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered",10,pointCloudCallback);

    lane_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/altered_point_cloud",10);
    
    while(ros::ok())
    ros::spinOnce();
    
    return 0;    
}