/*
 * @Author: asdleng lengjianghao2006@163.com
 * @Date: 2023-02-22 14:50:12
 * @LastEditors: asdleng lengjianghao2006@163.com
 * @LastEditTime: 2023-09-02 12:58:21
 * @FilePath: /vio_in_lidar_map/src/vio_in_lidar_map/src/vio_in_lidar_map_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <sstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <map.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>
#include <vio_in_lidar_map.h>
#include <visualization.h>
sensor_msgs::Imu last_imu;
double process_img_time;
double current_imu_time;
bool pure_imu = false;
bool give_init_v_a = false;
image_transport::Publisher img_pub;
std::string encoding;
bool pub_pcl = true;
ifstream fpath;
nav_msgs::Path true_path;
int publish_count;
double solve_time;
bool new_image = false;
bool imu_init=false;
int init_num = 100;
M3D Eye3d(M3D::Identity());
M3F Eye3f(M3F::Identity());
V3D Zero3d(0, 0, 0);
V3F Zero3f(0, 0, 0);
double cov_acc_scale;
double cov_gyr_scale;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
vector<double> cameraextrinT(3, 0.0);
vector<double> cameraextrinR(9, 0.0);
vector<double> origin_pose(6, 0.0);
vector<double> init_v_a(6, 0.0);
vector<double> init_ba_bg(6, 0.0);
vector<double> init_g(3, 0.0);
double cam_fx, cam_fy, cam_cx, cam_cy;
geometry_msgs::Quaternion geoQuat;
pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud<PointType>);
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
int NUM_MAX_ITERATIONS = 20;
string img_topic;
string imu_topic;
shared_ptr<ImuProcess> p_imu(new ImuProcess());
lvo::Map map;
lvo::FramePtr new_frame;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<std::pair<cv::Mat,double>> img_buffer; 
mutex mtx_buffer_imu;
mutex mtx_buffer_img;
mutex mtx_img_pub;
condition_variable sig_buffer;
lvo::violm vio_l_m;
nav_msgs::Odometry odomAftMapped;
nav_msgs::Odometry camera_pose;
geometry_msgs::PoseStamped msg_body_pose;
nav_msgs::Path path;
ofstream o_pose;
    V3D extT;
    M3D extR;
void wirteEuRoc(double current_time, nav_msgs::Odometry odom){
    o_pose<< fixed<<setprecision(4) << current_time << " " << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " " << odom.pose.pose.position.z  << " " 
  << odom.pose.pose.orientation.x  << " " <<  odom.pose.pose.orientation.y << " " <<odom.pose.pose.orientation.z << " " << odom.pose.pose.orientation.w << std::endl;
}
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg) {
  cv::Mat img;
    if(encoding=="bgr8"){
        img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        //img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    }
    else if(encoding=="rgb8"){
        img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8)->image;
        //img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    }
    else if(encoding=="mono8"){
        img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8)->image;
        //img = cv_bridge::toCvShare(img_msg, "mono8")->image;
    }
    else{
        std::cout<<"Wrong encoding, please check the image encoding"<<std::endl;
    }

  return img;
}
void publish_img(const image_transport::Publisher pubImg){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    // out_msg.header.frame_id = "camera_init";
    if(encoding=="bgr8"){
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    }
    else if(encoding=="rgb8"){
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    }
    else if(encoding=="mono8"){
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    }
    out_msg.image = vio_l_m.img_cp;
    pubImg.publish(out_msg.toImageMsg());
    //mtx_img_pub.unlock();
}
void img_cbk(const sensor_msgs::Image::ConstPtr &msg){
    mtx_buffer_img.lock();
    cv::Mat img = getImageFromMsg(msg);
    //cv::imshow("fuck",img);
    std::pair<cv::Mat,double> img_pair;
    img_pair.first = img; img_pair.second = msg->header.stamp.toSec();
    //cv::imshow("fuck1",img_pair.first);
    img_buffer.push_back(img_pair);
    //cv::imshow("fuck2",img_buffer.back().first);
    //vio_l_m.debug_file<<"callback中:"<<img_buffer.back().first.channels()<<std::endl;
    //vio_l_m.debug_file<<"callback中:"<<img_buffer.back().first.depth()<<std::endl;
    mtx_buffer_img.unlock();
    //sig_buffer.notify_all();
    new_image = true;
            
}
/*** imu数据处理回调***/
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_imu.lock();  // 加锁

    imu_buffer.push_back(msg);  // 压入imu_buffer
    mtx_buffer_imu.unlock();    // 解锁
    //sig_buffer.notify_all();    // 避免死锁
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data){
    //std::cout<<"2.0"<<std::endl;
    bool valid = true;
    vio_l_m.Updatekf(kf);//std::cout<<"                                         2.1"<<std::endl;
    //vio_l_m.UpdateCamPose();
    //vio_l_m.reprojection();//std::cout<<"                                             2.2"<<std::endl;
    //valid = vio_l_m.CalculateJandRes();//std::cout<<"                                    2.3"<<std::endl;
    if(valid==false){
        vio_l_m.debug_file<<"无参考点"<<std::endl;
        vio_l_m.iterate_num++;
        return;
    }
    //std::cout<<"Measurement Dimension: "<<vio_l_m.res_sub.size()<<std::endl;
    
   int effect_num = vio_l_m.res_sub.size();
   ekfom_data.h = VectorXd::Zero(effect_num);
   ekfom_data.h_x = MatrixXd::Zero(effect_num, 12);
   ekfom_data.h = vio_l_m.res_sub;
   ekfom_data.h_x = vio_l_m.J_sub;
   				double means = 0;
				int k = 0;
				for(int j=0;j<ekfom_data.h.rows();j++){
					k++;
					means+=fabs(ekfom_data.h(j));
				}
				means = means/k;
				vio_l_m.debug_file<<"第"<<vio_l_m.iterate_num<<"次迭代的平均error为："<<means<<std::endl;
//    for(int i=0;i<effect_num;i++){
//         ekfom_data.h += vio_l_m.res_sub.block<1,1>(i,0);
//         ekfom_data.h_x += vio_l_m.J_sub.block<1,12>(i,0);
//    }
    // std::cout<<"res: "<<ekfom_data.h.transpose()<<std::endl;
    // std::cout<<"J: "<<ekfom_data.h_x<<std::endl;
    vio_l_m.debug_file<<"res: "<<setprecision(8)<<ekfom_data.h.transpose()<<std::endl;
    vio_l_m.debug_file<<"J: "<<setprecision(8)<<ekfom_data.h_x<<std::endl;
    ekfom_data.valid = valid;
    // if(ekfom_data.h_x.hasNaN()==true||ekfom_data.h.hasNaN()==true){
    //     std::cout<<"存在NaN"<<std::endl;
    //     ekfom_data.valid = false;
    // }
    vio_l_m.iterate_num++;
    //std::cout<<"2.2"<<std::endl;
}
template<typename T>
void set_posestamp(T & out)
{
    #ifdef USE_IKFOM
    //state_ikfom stamp_state = kf.get_x();
    out.position.x = state_point.pos(0);
    out.position.y = state_point.pos(1);
    out.position.z = state_point.pos(2);
    #else
    out.position.x = state.pos_end(0);
    out.position.y = state.pos_end(1);
    out.position.z = state.pos_end(2);
    #endif
    auto euler_cur = RotMtoEuler(state_point.rot.toRotationMatrix());
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}
void publish_odometry(const ros::Publisher & pubOdomAftMapped,const ros::Publisher & pubCamera)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time().fromSec(process_img_time);//.ros::Time()fromSec(last_timestamp_lidar);
    set_posestamp(odomAftMapped.pose.pose);
    pubOdomAftMapped.publish(odomAftMapped);

   static tf::TransformBroadcaster br;
    static tf::TransformBroadcaster br_lidar_map;
    static tf::TransformBroadcaster br_camera;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped"));
    tf::Transform transform_lidar;
    transform_lidar.setOrigin(tf::Vector3(extT[0], extT[1], extT[2]));
    //M3D R_lidar_imu = Map<Matrix3d>(extrinR.data(),3,3);
    Quaterniond q_lidar_imu_(extR);
    tf::Quaternion q_lidar_imu;
    q_lidar_imu.setW(q_lidar_imu_.w());
    q_lidar_imu.setX(q_lidar_imu_.x());
    q_lidar_imu.setY(q_lidar_imu_.y());
    q_lidar_imu.setZ(q_lidar_imu_.z());
    transform_lidar.setRotation(q_lidar_imu);
    br_lidar_map.sendTransform(tf::StampedTransform(transform_lidar, odomAftMapped.header.stamp, "camera_init", "lidar"));
    tf::Transform transform_camera;
    V3D Pic = - vio_l_m.Rci.transpose()* vio_l_m.Pci;
    transform_camera.setOrigin(tf::Vector3(Pic[0],Pic[1],Pic[2]));
     Quaterniond q_camera_imu_(vio_l_m.Rci.transpose());
    tf::Quaternion q_camera_imu;
    q_camera_imu.setW(q_camera_imu_.w());
    q_camera_imu.setX(q_camera_imu_.x());
    q_camera_imu.setY(q_camera_imu_.y());
    q_camera_imu.setZ(q_camera_imu_.z());
    transform_camera.setRotation(q_camera_imu);
    br_camera.sendTransform(tf::StampedTransform(transform_camera, odomAftMapped.header.stamp, "aft_mapped", "camera"));

}
void publish_observed_world(const ros::Publisher & pubLaserCloudObserved)
{
    uint size = vio_l_m.pcl_projected->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*vio_l_m.pcl_projected, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudObserved.publish(laserCloudmsg);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void publish_frame_world_Triangulate(const ros::Publisher & pub)
{
    uint size = vio_l_m.triangulate_pts_world->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*vio_l_m.triangulate_pts_world, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pub.publish(laserCloudmsg);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)
{
    uint size = vio_l_m.pcl_down->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*vio_l_m.pcl_down, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void publish_local_map(const ros::Publisher & pubLocalMap)
{
    uint size = vio_l_m.local_map->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*vio_l_m.local_map, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLocalMap.publish(laserCloudmsg);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void publish_frame_world_3DLine(const ros::Publisher & pub3DLine)
{
    uint size = vio_l_m.pcl_down->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        visualization_msgs::Marker line_list;
        line_list.id = 2;
        line_list.color.b = 1.0;
        line_list.color.g = 1.0;
        line_list.color.r = 0.0;
        line_list.color.a = 1.0;
        line_list.scale.x = 0.01;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        for(int i=0;i<vio_l_m.lines.size();i++){
            geometry_msgs::Point p1,p2;
            p1.x = vio_l_m.lines[i][0].x;
            p1.y = vio_l_m.lines[i][0].y;
            p1.z = vio_l_m.lines[i][0].z;
            p2.x = vio_l_m.lines[i][1].x;
            p2.y = vio_l_m.lines[i][1].y;
            p2.z = vio_l_m.lines[i][1].z;
            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
        }

        line_list.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        line_list.header.frame_id = "camera_init";
        pub3DLine.publish(line_list);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void publish_frame_world_line(const ros::Publisher & pubLaserCloudFullRes)
{
    uint size = vio_l_m.pcl_line->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*vio_l_m.pcl_line, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "camera_init";
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}
void publish_true_path(const ros::Publisher pubTruePath){
    true_path.header.stamp = ros::Time::now();
    pubTruePath.publish(true_path);
}

void publish_depth_img(const image_transport::Publisher pubDepthImg){
    //if(!new_image) return;
    cv::Mat img_depth = vio_l_m.img_depth;
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    // out_msg.header.frame_id = "camera_init";
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    out_msg.image = img_depth;
    pubDepthImg.publish(out_msg.toImageMsg());
}
string filename = "/home/i/vio_in_lidar_map/src/vio_in_lidar_map/map/scans.pcd";
string path_file = "path.txt";
string lines_file = "line.txt";
string euroc_file = "euroc.txt";
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vio");
    ros::NodeHandle nh;
    registerPub(nh);
    nh.param<string>("camera/img_topic", img_topic, "/img");
    nh.param<string>("common/imu_topic", imu_topic, "/imu");
    nh.param<double>("theta_thre",vio_l_m.theta_thre,0.8);
    nh.param<bool>("enable_line",vio_l_m.enable_line,true);
    nh.param<bool>("cut_points_above_ground",vio_l_m.cut_points_above_ground,true);
    nh.param<double>("cam_fx",cam_fx,453.483063);
    nh.param<double>("cam_fy",cam_fy,453.254913);
    nh.param<double>("cam_cx",cam_cx,318.908851);
    nh.param<double>("cam_cy",cam_cy,234.238189);
    nh.param<double>("img_scale",vio_l_m.img_scale,1.0);
    nh.param<int>("window_size",vio_l_m.window_size,10);
    nh.param<int>("max_iteration",vio_l_m.max_iteration,10);
    nh.param<int>("grid_size",vio_l_m.grid_size,40);
    nh.param<int>("patch_size",vio_l_m.patch_size,4);
    nh.param<int>("init_num",init_num,100);
    nh.param<double>("filter_size",vio_l_m.filter_size,0.2);
    nh.param<double>("score_threshold",vio_l_m.score_threshold,50.0);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<vector<double>>("camera/Pcl", cameraextrinT, vector<double>());
    nh.param<vector<double>>("camera/Rcl", cameraextrinR, vector<double>());
    nh.param<double>("outlier_threshold", vio_l_m.outlier_threshold, 100.0);
    nh.param<double>("mapping/gyr_cov_scale",cov_gyr_scale,1.0);
    nh.param<double>("mapping/acc_cov_scale",cov_acc_scale,1.0);
    nh.param<double>("mapping/gyr_Q_scale",p_imu->gyr_Q_scale,10.0);
    nh.param<double>("mapping/acc_Q_scale",p_imu->acc_Q_scale,10.0);
    nh.param<double>("img_point_cov",vio_l_m.IMG_COV,100.0);
    nh.param<double>("projection_point_cov",vio_l_m.POINT_COV,100.0);
    nh.param<double>("triangulate_point_cov",vio_l_m.TRIAN_COV,10.0);
    nh.param<double>("line_cov",vio_l_m.LINE_COV,100.0);
    nh.param<double>("camera/blind",vio_l_m.blind,0.0);
    nh.param<double>("camera/max_blind",vio_l_m.max_blind,100);
    nh.param<bool>("read_enable",vio_l_m.read_enable,true);
    nh.param<bool>("enable_BA",vio_l_m.enable_BA,true);
    nh.param<bool>("pub_pcl",pub_pcl,true);
    nh.param<bool>("left_coord",vio_l_m.left_coord,false);
    nh.param<bool>("need_down_size",vio_l_m.need_down_size,true);
    nh.param<bool>("equalize", vio_l_m.equalize,false);
    nh.param<string>("map_dir", filename,"/home/i/vio_in_lidar_map/src/vio_in_lidar_map/map/scans.pcd");
    nh.param<string>("path_file", path_file,"path.txt");
    nh.param<string>("lines_file", vio_l_m.lines_file,"line.txt");
    nh.param<vector<double>>("origin_pose", origin_pose,vector<double>());
    nh.param<string>("encoding", encoding, "bgr8");
    nh.param<bool>("give_init_v_a",give_init_v_a,false);
    nh.param<vector<double>>("init_v_a", init_v_a,vector<double>());
    nh.param<vector<double>>("init_ba_bg", init_ba_bg,vector<double>());
    nh.param<vector<double>>("init_g", init_g,vector<double>());
    nh.param<bool>("map_is_based_on_LiDAR",vio_l_m.map_is_based_on_LiDAR,false);
    nh.param<bool>("mapping/normalized",p_imu->normalized,true);
    nh.param<bool>("pure_imu",pure_imu,false);
    nh.param<bool>("enable_projection",vio_l_m.enable_projection,true);
    nh.param<bool>("enable_triangulate",vio_l_m.enable_triangulate,false);
    nh.param<int>("down_sample_manner",vio_l_m.down_sample_manner,0);
    nh.param<int>("down_sample_num",vio_l_m.down_sample_num,50);
    nh.param<bool>("ncc_en",vio_l_m.ncc_en,false);
    nh.param<double>("ncc_thre",vio_l_m.ncc_thre,0.0);
    nh.param<int>("depth_search_radius",vio_l_m.depth_search_radius,4);
    nh.param<double>("trans_thre",vio_l_m.trans_thre,0.1);
    nh.param<double>("rot_thre",vio_l_m.rot_thre,PI_M/12);
    nh.param<int>("grid_pts_num",vio_l_m.grid_pts_num, 1);
    nh.param<double>("linethre3d",vio_l_m.linethre3d, 2.0);
    nh.param<int>("linethre3d_proj",vio_l_m.linethre3d_proj, 10);
    nh.param<int>("linethre2d",vio_l_m.linethre2d, 20);
    nh.param<double>("lamda",vio_l_m.lamda, 0.0394);
    nh.param<double>("line_threshold",vio_l_m.line_threshold, 50);
    nh.param<double>("edgescore_threshold",vio_l_m.edgescore_threshold, 100.0);
    nh.param<int>("pyr",vio_l_m.pyr,1);
    nh.param<double>("delta_dist_thre",vio_l_m.delta_dist_thre,1.5);
    nh.param<double>("skip_depth",vio_l_m.skip_depth,1.0);
    p_imu->cov_gyr_scale = V3D(cov_gyr_scale,cov_gyr_scale,cov_gyr_scale);
    p_imu->cov_acc_scale = V3D(cov_acc_scale,cov_acc_scale,cov_acc_scale);
    extT<<VEC_FROM_ARRAY(extrinT);
    extR<<MAT_FROM_ARRAY(extrinR);
    
    vio_l_m.set_extrinsic(extT,extR);
    vio_l_m.set_camera2lidar(cameraextrinR, cameraextrinT);
    if (pcl::io::loadPCDFile<PointType>(filename, *src) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    //vio_l_m.ikdtree.Build((*src).points);

    
    double epsi[24] = {0.001};
    fill(epsi, epsi + 24, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 1000, imu_cbk);
    ros::Subscriber sub_img = nh.subscribe(img_topic, 1000, img_cbk);
    ros::Publisher pubTriangulate = nh.advertise<sensor_msgs::PointCloud2>("/triangulate_cloud", 100000);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/origin_cloud", 100000);
    ros::Publisher pubLaserCloudFullLine = nh.advertise<sensor_msgs::PointCloud2>("/origin_cloud_line", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudObserved = nh.advertise<sensor_msgs::PointCloud2>("/cloud_observed", 100000);
    ros::Publisher pubLocalMap = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubCamera = nh.advertise<nav_msgs::Odometry>("/camera_pose", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
    ros::Publisher pubTruePath = nh.advertise<nav_msgs::Path>("/true_path", 100000);
    ros::Publisher pubExt = nh.advertise<geometry_msgs::PoseStamped>("/ext", 100000);
    ros::Publisher pub3DLine = nh.advertise<visualization_msgs::Marker>("/3DLine",1000);

    o_pose.open(FILE_DIR(euroc_file), ios::out);
    image_transport::ImageTransport it(nh);
    img_pub = it.advertise("/rgb_img", 1);
    image_transport::Publisher img_depth_pub = it.advertise("/depth_img", 1);
    // 读取相机参数
      if(!vk::camera_loader::loadFromRosNs("vio_in_lidar_map", vio_l_m.cam))
    throw std::runtime_error("Camera model not correctly specified.");
    vio_l_m.Init(src);
    vio_l_m.fx = cam_fx;
    vio_l_m.fy = cam_fy;
    vio_l_m.cx = cam_cx;
    vio_l_m.cy = cam_cy;
    
    // 赋起始坐标
    state_ikfom state0 = kf.get_x();
    state0.pos[0] = origin_pose[0];
    state0.pos[1] = origin_pose[1];
    state0.pos[2] = origin_pose[2];
    Eigen::Vector3d ea(origin_pose[3], origin_pose[4], origin_pose[5]);
    state0.rot = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ());
    kf.change_x(state0);
    // 如果提供初始速度，不进行静止初始化
    if(give_init_v_a){
            state_ikfom init_state = kf.get_x();
            init_state.vel[0] = init_v_a[0];
            init_state.vel[1] = init_v_a[1];
            init_state.vel[2] = init_v_a[2];
            init_state.ba[0] = init_ba_bg[0];
            init_state.ba[1] = init_ba_bg[1];
            init_state.ba[2] = init_ba_bg[2];
            init_state.bg[0] = init_ba_bg[3];
            init_state.bg[1] = init_ba_bg[4];
            init_state.bg[2] = init_ba_bg[5];
            init_state.grav[0] = init_g[0];
            init_state.grav[1] = init_g[1];
            init_state.grav[2] = -init_g[2];
            kf.change_x(init_state);
            // 协方差 全是自己定的
            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf.get_P();
            init_P.setIdentity();
            init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.001; // 旋转外参
            init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.001; // 平移外参
            init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.01;  // bg
            init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.01; // ba
            init_P(21, 21) = init_P(22, 22) = init_P(23, 23) = 0.01;  //g
            kf.change_P(init_P);
            give_init_v_a = false;
            imu_init = true;
            ROS_INFO("Give Initial States");
            vio_l_m.debug_file<<"Initial x: "<<kf.get_x()<<std::endl;
            vio_l_m.debug_file<<"Initial P: "<<kf.get_P()<<std::endl;
    }
    int frame_count_pub=0;
    int odom_count_pub=0;
    ros::Rate rate(500);   // 高过图像速度

    ros::Time start = ros::Time::now();


    //publish_frame_world_line(pubLaserCloudFullLine);
    fpath.open(FILE_DIR(path_file), ios::in);
    
    std::string s;
    true_path.header.frame_id = "camera_init";
    double true_path_length = 0;
    geometry_msgs::PoseStamped last_p;
    while(getline(fpath,s)){
        std::stringstream ss(s);
        double x, y, z;
        ss>>x>>y>>z;
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "camera_init";
        p.pose.position.x = x; p.pose.position.y = y; p.pose.position.z = z;
        true_path.poses.push_back(p);
        true_path_length+=sqrt((p.pose.position.x-last_p.pose.position.x)*(p.pose.position.x-last_p.pose.position.x)+
        (p.pose.position.y-last_p.pose.position.y)*(p.pose.position.y-last_p.pose.position.y)+
        (p.pose.position.z-last_p.pose.position.z)*(p.pose.position.z-last_p.pose.position.z));
        last_p = p;
    }
    // cv::namedWindow("fuck",CV_WINDOW_NORMAL);
    // cv::namedWindow("fuck1",CV_WINDOW_NORMAL);
    // cv::namedWindow("fuck2",CV_WINDOW_NORMAL);
    // cv::namedWindow("fuck3",CV_WINDOW_NORMAL);
    // cv::namedWindow("fuck4",CV_WINDOW_NORMAL);
    state_point = kf.get_x();
    double t0 = omp_get_wtime();
    while(ros::ok()){
        double t1 = omp_get_wtime();
        if((t1-t0)<1){
            if(pub_pcl)    
                publish_frame_world(pubLaserCloudFull);
                publish_frame_world_3DLine(pub3DLine);
                publish_true_path(pubTruePath);
        }
        
        


        // count_map++;
        // if ((ros::Time::now()-start).toSec()<2&&count_map>100){
        //     publish_frame_world(pubLaserCloudFull);
        //     publish_frame_world_line(pubLaserCloudFullLine);
        //     count_map = 0;
        // }
        //publish_frame_world_3DLine(pub3DLine);

        ros::spinOnce();
        
        if(img_buffer.size()==0){
            //std::cout<<"No IMAGE"<<endl;
            continue;   // 等待图像
        }
        if(imu_buffer.size()==0){
            //std::cout<<"NO IMU!!!"<<std::endl;
            continue;   // 没有IMU信息
        }
        current_imu_time = imu_buffer.back().get()->header.stamp.toSec();
        process_img_time = img_buffer.front().second;

        if(process_img_time>current_imu_time){
            //std::cout<<"Wait for imu data"<<std::endl;  
            continue;  // IMU落后了
        }
        //std::cout<<"1"<<std::endl;

        mtx_buffer_imu.lock();
        if(imu_init==false){
            if(imu_buffer.size()>init_num){
                int init_iter_num=0;
                while(init_iter_num<=init_num){
                    p_imu->IMU_init(imu_buffer,kf,init_iter_num);
                }
                imu_init = true;
                ROS_INFO("IMU Initial Done");
                vio_l_m.debug_file<<"Initial x: "<<kf.get_x()<<std::endl;
                vio_l_m.debug_file<<"Initial P: "<<kf.get_P()<<std::endl;
                //std::cout<<"s.gra: "<< kf.get_x().grav<<std::endl;
            }
            else{
                //std::cout<<"Wait for more imu to initialize."<<std::endl;
                mtx_buffer_imu.unlock();
                continue;
            }
        }
        //std::cout<<fixed<<setprecision(10)<<"imu尾："<<imu_buffer.back()->header.stamp.toSec()<<std::endl;
        //std::cout<<fixed<<setprecision(10)<<"imu头："<<imu_buffer.front()->header.stamp.toSec()<<std::endl;
        //std::cout<<"imu个数："<<imu_buffer.size()<<std::endl;
        //std::cout<<fixed<<setprecision(10)<<"相机时间："<<process_img_time<<std::endl;
        for(auto it = imu_buffer.front();it->header.stamp.toSec()<process_img_time;){
            auto imu1 = *it;    //std::cout<<"imu1: "<<imu1<<std::endl;
            last_imu = imu1;
            imu_buffer.pop_front();
            it = imu_buffer.front(); 
            auto imu2 = *it;    //std::cout<<"imu2: "<<imu2<<std::endl;
            p_imu->intergrate(imu1,imu2,kf,process_img_time);
               // 得到状态(位姿、外参、速度、零偏、重力)
            //auto P = kf.get_P();
        }
        //std::cout<<"pos:"<<state_point.pos.x()<<","<<state_point.pos.y()<<","<<state_point.pos.z()<<std::endl;  
            //vio_l_m.debug_file<<"propagated cov: "<<P0.8<state_point.pos.y()<<","<<state_point.pos.z()<<std::endl;  
        mtx_buffer_imu.unlock();
        //std::cout<<"2"<<std::endl;
        mtx_buffer_img.lock();

        // vio_l_m.debug_file<<"main中:"<<img.channels()<<std::endl;
        // vio_l_m.debug_file<<"main中:"<<img.depth()<<std::endl;
        //cv::imshow("fuck3",img_buffer.back().first);
        if(!pure_imu){
            vio_l_m.Process(img_buffer.front().first,kf);
        }
        
        

        state_point = kf.get_x();
        
        publish_odometry(pubOdomAftMapped,pubCamera);
        publish_path(pubPath);

        wirteEuRoc(img_buffer.front().second,odomAftMapped);
        publish_img(img_pub);
        img_buffer.pop_front();
        //cv::waitKey(100);
        
        mtx_buffer_img.unlock();
        // mtx_buffer_imu.lock();
        // p_imu->intergrate2(last_imu,*imu_buffer.front(),kf,process_img_time);   // 处理图像后的那一小段
        // mtx_buffer_imu.unlock();
        //std::cout<<"3"<<std::endl;
        
        //vio_l_m.debug_file<<"x: "<<kf.get_x()<<std::endl;
        
        if(vio_l_m.enable_triangulate){
            publish_frame_world_Triangulate(pubTriangulate);
        }

        if(vio_l_m.need_keyframe==true){
            std_msgs::Header header;
            header.frame_id = "camera_init";
            header.stamp = ros::Time::now();
            pubKeyPoses(vio_l_m.map,header);
            publish_observed_world(pubLaserCloudObserved);
            //publish_local_map(pubLocalMap);
        }
        rate.sleep();
    }
    std::cout<<"true path length: "<<true_path_length<<std::endl;
    auto pos = odomAftMapped.pose.pose.position;
    std::cout<<"distance to origin: "<<sqrt(pos.x*pos.x+pos.y*pos.y+pos.z*pos.z)<<std::endl;
    std::cout<<"ATE rate (%): "<<100*sqrt(pos.x*pos.x+pos.y*pos.y+pos.z*pos.z)/true_path_length<<std::endl;
}
