/*
 * @Author: asdleng lengjianghao2006@163.com
 * @Date: 2023-04-05 21:49:32
 * @LastEditors: asdleng lengjianghao2006@163.com
 * @LastEditTime: 2023-04-05 22:30:17
 * @FilePath: /vio_in_lidar_map/src/vio_in_lidar_map/include/visualization.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vio_in_lidar_map.h>
extern ros::Publisher pub_key_poses;
class CameraPoseVisualization {
public:
	std::string m_marker_ns;

	CameraPoseVisualization(float r, float g, float b, float a);
	
	void setImageBoundaryColor(float r, float g, float b, float a=1.0);
	void setOpticalCenterConnectorColor(float r, float g, float b, float a=1.0);
	void setScale(double s);
	void setLineWidth(double width);

	void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
	void reset();

	void publish_by(ros::Publisher& pub, const std_msgs::Header& header);
	void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
	void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
private:
	std::vector<visualization_msgs::Marker> m_markers;
	std_msgs::ColorRGBA m_image_boundary_color;
	std_msgs::ColorRGBA m_optical_center_connector_color;
	double m_scale;
	double m_line_width;

	static const Eigen::Vector3d imlt;
	static const Eigen::Vector3d imlb;
	static const Eigen::Vector3d imrt;
	static const Eigen::Vector3d imrb;
	static const Eigen::Vector3d oc  ;
	static const Eigen::Vector3d lt0 ;
	static const Eigen::Vector3d lt1 ;
	static const Eigen::Vector3d lt2 ;
};

void registerPub(ros::NodeHandle &n);
void pubKeyPoses(const lvo::Map &map, const std_msgs::Header &header);