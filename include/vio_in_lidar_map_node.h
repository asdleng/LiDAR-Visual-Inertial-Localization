#ifndef VIO_IN_LIDAR_NODE_H_
#define VIO_IN_LIDAR_NODE_H_
#include <mutex>
#include <math.h>
#include <thread>
#include <vector>
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
#include <vio_in_lidar_map.h>
#include <visualization.h>
#include <functional>
#include "preprocess.h"

class vio_in_lidar_map_node
{
public:
    static vio_in_lidar_map_node* instance;
    ros::NodeHandle nh;
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
    bool add_stock = false;
    int init_num = 100;
    double cov_acc_scale;
    double cov_gyr_scale;
    vector<double> extrinT;
    vector<double> extrinR;
    vector<double> cameraextrinT;
    vector<double> cameraextrinR;
    vector<double> origin_pose;
    vector<double> init_v_a;
    vector<double> init_ba_bg;
    vector<double> init_g;
    double cam_fx, cam_fy, cam_cx, cam_cy;
    geometry_msgs::Quaternion geoQuat;
    pcl::PointCloud<PointType>::Ptr src;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;
    int NUM_MAX_ITERATIONS = 20;
    string img_topic;
    string imu_topic;
    string lidar_topic;
    shared_ptr<ImuProcess> p_imu;
    lvo::Map map;
    lvo::FramePtr new_frame;
    deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    deque<sensor_msgs::Imu::ConstPtr> imu_buffer_lidar; // 专为lidar去畸变的imubuffer
    deque<std::pair<cv::Mat,double>> img_buffer; 
    deque<std::pair<PointCloudXYZI::Ptr,double>> lidar_buffer; 
    mutex mtx_buffer_imu;
    mutex mtx_buffer_img;
    mutex mtx_buffer_lidar;
    mutex mtx_img_pub;
    double lidar_time;
    condition_variable sig_buffer;
    std::shared_ptr<lvo::violm> vio_l_m;
    std::shared_ptr<Preprocess> preprocess;
    nav_msgs::Odometry odomAftMapped;
    nav_msgs::Odometry camera_pose;
    geometry_msgs::PoseStamped msg_body_pose;
    nav_msgs::Path path;
    double t_shift;
    ofstream o_pose;
    V3D extT;
    M3D extR;
    string filename = "/home/weilin/vio_in_lidar_map/src/vio_in_lidar_map/map/scans.pcd";
    string path_file = "path.txt";
    string lines_file = "line.txt";
    string euroc_file = "euroc.txt";

    PointCloudXYZI::Ptr feats_down_body;
    PointCloudXYZI::Ptr feats_down_world;
    PointCloudXYZI::Ptr normvec;
    PointCloudXYZI::Ptr laserCloudOri;
    PointCloudXYZI::Ptr corr_normvect;
    vector<PointVector> Nearest_Points;
    double res_mean_last = 0.05, total_residual = 0.0;
    int iterCount = 0, \
    feats_down_size = 0, \
    laserCloudValidNum = 0, \
    pcd_save_interval = -1,\
    pcd_index = 0,\
    effct_feat_num = 0;
    double match_time = 0;
    bool point_selected_surf[100000] = {0};
    float res_last[100000] = {0.0};
    
    ros::Subscriber sub_lidar;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_img;
    ros::Publisher pubTriangulate;
    ros::Publisher pubLaserCloudFull;
    ros::Publisher pubLaserCloudFullLine;
    ros::Publisher pubLaserCloudFull_body;
    ros::Publisher pubLaserCloudObserved;
    ros::Publisher pubLocalMap;
    ros::Publisher pubLaserCloudMap;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubCamera;
    ros::Publisher pubPath;
    ros::Publisher pubTruePath;
    ros::Publisher pubExt;
    ros::Publisher pub3DLine;

    void wirteEuRoc(double current_time, nav_msgs::Odometry odom);
    cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg);
    void publish_img(const image_transport::Publisher pubImg);
    void img_cbk(const sensor_msgs::Image::ConstPtr &msg);
    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void lidar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in);
    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
    template<typename T>
    void set_posestamp(T & out);
    void publish_odometry(const ros::Publisher & pubOdomAftMapped,const ros::Publisher & pubCamera);
    void publish_observed_world(const ros::Publisher & pubLaserCloudObserved);
    void publish_frame_world_Triangulate(const ros::Publisher & pub);
    void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes);
    void publish_local_map(const ros::Publisher & pubLocalMap);
    void publish_frame_world_3DLine(const ros::Publisher & pub3DLine);
    void publish_frame_world_line(const ros::Publisher & pubLaserCloudFullRes);
    void publish_path(const ros::Publisher pubPath);
    void publish_true_path(const ros::Publisher pubTruePath);
    void publish_depth_img(const image_transport::Publisher pubDepthImg);
    void para();
    void run();
    static void static_h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<state_ikfom::scalar> &ekfom_data);

    vio_in_lidar_map_node(ros::NodeHandle* nodehandle):
        nh(*nodehandle),
        extrinT(3, 0.0),             // 使用构造函数初始化列表初始化
        extrinR(9, 0.0),
        cameraextrinT(3, 0.0),
        cameraextrinR(9, 0.0),
        origin_pose(6, 0.0),
        init_v_a(6, 0.0),
        init_ba_bg(6, 0.0),
        init_g(3, 0.0)
    {
        instance = this;
        src.reset(new pcl::PointCloud<PointType>);
        p_imu.reset(new ImuProcess());
        vio_l_m.reset(new lvo::violm());
        preprocess.reset(new Preprocess());
        registerPub(nh);
        para();
        sub_lidar = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1000, boost::bind(&vio_in_lidar_map_node::lidar_cbk, this, _1));
        sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1000, boost::bind(&vio_in_lidar_map_node::imu_cbk, this, _1));
        sub_img = nh.subscribe<sensor_msgs::Image>(img_topic, 1000, boost::bind(&vio_in_lidar_map_node::img_cbk, this, _1));
        pubTriangulate = nh.advertise<sensor_msgs::PointCloud2>("/triangulate_cloud", 100000);
        pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/origin_cloud", 100000);
        pubLaserCloudFullLine = nh.advertise<sensor_msgs::PointCloud2>("/origin_cloud_line", 100000);
        pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
        pubLaserCloudObserved = nh.advertise<sensor_msgs::PointCloud2>("/cloud_observed", 100000);
        pubLocalMap = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 100000);
        pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
        pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
        pubCamera = nh.advertise<nav_msgs::Odometry>("/camera_pose", 100000);
        pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
        pubTruePath = nh.advertise<nav_msgs::Path>("/true_path", 100000);
        pubExt = nh.advertise<geometry_msgs::PoseStamped>("/ext", 100000);
        pub3DLine = nh.advertise<visualization_msgs::Marker>("/3DLine",1000);

        feats_down_body.reset(new PointCloudXYZI());
        feats_down_world.reset(new PointCloudXYZI());
        normvec.reset(new PointCloudXYZI(100000, 1));
        laserCloudOri.reset(new PointCloudXYZI(100000, 1));
        corr_normvect.reset(new PointCloudXYZI(100000, 1));
    
    }
};
template<typename T>
void vio_in_lidar_map_node::set_posestamp(T & out)
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
#endif
