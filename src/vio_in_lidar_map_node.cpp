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
#include "vio_in_lidar_map_node.h"
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

/* 全局变量、静态变量 */

vio_in_lidar_map_node* vio_in_lidar_map_node::instance = nullptr;

void vio_in_lidar_map_node::wirteEuRoc(double current_time, nav_msgs::Odometry odom){
    o_pose<< fixed<<setprecision(4) << current_time << " " << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " " << odom.pose.pose.position.z  << " " 
  << odom.pose.pose.orientation.x  << " " <<  odom.pose.pose.orientation.y << " " <<odom.pose.pose.orientation.z << " " << odom.pose.pose.orientation.w << std::endl;
}
cv::Mat vio_in_lidar_map_node::getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg) {
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
    else if(encoding=="bgra8"){
        img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGRA8)->image;
        //img = cv_bridge::toCvShare(img_msg, "mono8")->image;
    }
    else{
        std::cout<<"Wrong encoding, please check the image encoding"<<std::endl;
    }
  return img;
}
void vio_in_lidar_map_node::publish_img(const image_transport::Publisher pubImg){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    // out_msg.header.frame_id = "camera_init";
    if(encoding=="bgr8"){
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    }
    else if(encoding=="bgra8"){
        out_msg.encoding = sensor_msgs::image_encodings::BGRA8;
    }
    else if(encoding=="rgb8"){
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    }
    else if(encoding=="mono8"){
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    }
    out_msg.image = vio_l_m->img_cp;
    pubImg.publish(out_msg.toImageMsg());
    //mtx_img_pub.unlock();
}
void vio_in_lidar_map_node::img_cbk(const sensor_msgs::Image::ConstPtr &msg){
    mtx_buffer_img.lock();
    cv::Mat img = getImageFromMsg(msg);
    //cv::imshow("fuck",img);
    std::pair<cv::Mat,double> img_pair;
    img_pair.first = img; img_pair.second = msg->header.stamp.toSec();
    //cv::imshow("fuck1",img_pair.first);
    img_buffer.push_back(img_pair);
    //cv::imshow("fuck2",img_buffer.back().first);
    //vio_l_m->debug_file<<"callback中:"<<img_buffer.back().first.channels()<<std::endl;
    //vio_l_m->debug_file<<"callback中:"<<img_buffer.back().first.depth()<<std::endl;
    mtx_buffer_img.unlock();
    //sig_buffer.notify_all();
    new_image = true;
            
}
/*** imu数据处理回调***/
void vio_in_lidar_map_node::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec()-t_shift;

    mtx_buffer_imu.lock();  // 加锁

    imu_buffer.push_back(msg);  // 压入imu_buffer
    mtx_buffer_imu.unlock();    // 解锁
    //sig_buffer.notify_all();    // 避免死锁
}
/*** LiDAR数据处理回调***/
void vio_in_lidar_map_node::lidar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in){
    mtx_buffer_lidar.lock();
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    preprocess->process(msg_in,ptr);
    vio_l_m->debug_file<<"新进lidar有"<<ptr->size()<<"个点"<<std::endl;
    std::pair<PointCloudXYZI::Ptr,double> lidar_pair;
    lidar_pair.first = ptr; lidar_pair.second = msg_in->header.stamp.toSec();
    //cv::imshow("fuck1",img_pair.first);
    lidar_buffer.push_back(lidar_pair);
    //cv::imshow("fuck2",img_buffer.back().first);
    //vio_l_m->debug_file<<"callback中:"<<img_buffer.back().first.channels()<<std::endl;
    //vio_l_m->debug_file<<"callback中:"<<img_buffer.back().first.depth()<<std::endl;
    mtx_buffer_lidar.unlock();
}

void vio_in_lidar_map_node::run(){
    p_imu->cov_gyr_scale = V3D(cov_gyr_scale,cov_gyr_scale,cov_gyr_scale);
    p_imu->cov_acc_scale = V3D(cov_acc_scale,cov_acc_scale,cov_acc_scale);
    extT<<VEC_FROM_ARRAY(extrinT);
    extR<<MAT_FROM_ARRAY(extrinR);
    std::cout<<"LiDAR外参为："<<std::endl;
    std::cout<<extR<<std::endl;
    std::cout<<extT<<std::endl;
    vio_l_m->set_extrinsic(extT,extR);
    vio_l_m->set_cameraext(cameraextrinR, cameraextrinT);
    vio_l_m->set_mapext(mapextR,mapextT);
    if (pcl::io::loadPCDFile<PointType>(filename, *src) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return;
    }
    double epsi[24] = {0.001};
    fill(epsi, epsi + 24, 0.001);
    std::function<void(state_ikfom&, esekfom::dyn_share_datastruct<state_ikfom::scalar>&)> func = std::bind(&vio_in_lidar_map_node::h_share_model, this, std::placeholders::_1, std::placeholders::_2);
    kf.init_dyn_share(get_f, df_dx, df_dw, vio_in_lidar_map_node::static_h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** ROS subscribe initialization ***/
    o_pose.open(FILE_DIR(euroc_file), ios::out);
    image_transport::ImageTransport it(nh);
    img_pub = it.advertise("/rgb_img", 1);
    image_transport::Publisher img_depth_pub = it.advertise("/depth_img", 1);
    // 读取相机参数
      if(!vk::camera_loader::loadFromRosNs("vio_in_lidar_map", vio_l_m->cam))
    throw std::runtime_error("Camera model not correctly specified.");
    
    vio_l_m->Init(src);
    vio_l_m->fx = cam_fx;
    vio_l_m->fy = cam_fy;
    vio_l_m->cx = cam_cx;
    vio_l_m->cy = cam_cy;
    
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

    if(add_stock){
        double radius = 0.3;
        double points_per_unit = 5000;
        float area = M_PI * radius * radius;
        // 总点数
        int total_points = static_cast<int>(area * points_per_unit);

        // 生成圆盘上的点
        int ii = vio_l_m->pcl_down->size()-1;
        for (int i = 0; i < total_points; ++i) {
            float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
            float r = radius * sqrt(static_cast<float>(rand()) / RAND_MAX);
            float x = r * cos(angle);
            float y = r * sin(angle);

            PointType point;
            point.x = state0.pos[0]+x;
            point.y = state0.pos[1]+y;
            point.z = state0.pos[2]-0.1;
            point.intensity = 1.0;  // 可以根据需要设置强度值

            // 添加点到点云
            vio_l_m->pcl_down->push_back(point);

        }
        for(;ii<vio_l_m->pcl_down->size();ii++){
            V3D pos(vio_l_m->pcl_down->at(ii).x,
            vio_l_m->pcl_down->at(ii).y,
            vio_l_m->pcl_down->at(ii).z);

            lvo::PointPtr p(new lvo::Point(pos));
            p->id_ = ii;
            vio_l_m->map.addPoint(p);  
        }
    }

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
            vio_l_m->debug_file<<"Initial x: "<<kf.get_x()<<std::endl;
            vio_l_m->debug_file<<"Initial P: "<<kf.get_P()<<std::endl;
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
                vio_l_m->debug_file<<"Initial x: "<<kf.get_x()<<std::endl;
                vio_l_m->debug_file<<"Initial P: "<<kf.get_P()<<std::endl;
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
            imu_buffer_lidar.push_back(it);
            auto imu1 = *it;    //std::cout<<"imu1: "<<imu1<<std::endl;
            last_imu = imu1;
            imu_buffer.pop_front();
            it = imu_buffer.front();
            auto imu2 = *it;    //std::cout<<"imu2: "<<imu2<<std::endl;
            p_imu->intergrate(imu1,imu2,kf,process_img_time);
            if(lidar_buffer.size()>0){
                // 判断是否有LiDAR点云存在
                if(lidar_buffer.front().second<imu2.header.stamp.toSec()){
                    undistort();
                    double solve_H_time = 0;
                    Nearest_Points.resize(feats_down_size);
                    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                    publish_lidar_frame_world(pubLaserCloudLiDARFull);
                }
            }
        }
        //std::cout<<"pos:"<<state_point.pos.x()<<","<<state_point.pos.y()<<","<<state_point.pos.z()<<std::endl;  
        //vio_l_m->debug_file<<"propagated cov: "<<P0.8<state_point.pos.y()<<","<<state_point.pos.z()<<std::endl;  
        mtx_buffer_imu.unlock();
        //std::cout<<"2"<<std::endl;
        mtx_buffer_img.lock();

        //cv::imshow("fuck3",img_buffer.back().first);
        if(!pure_imu){
            vio_l_m->Process(img_buffer.front().first,kf);
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
        
        //vio_l_m->debug_file<<"x: "<<kf.get_x()<<std::endl;
        
        if(vio_l_m->enable_triangulate){
            publish_frame_world_Triangulate(pubTriangulate);
        }

        if(vio_l_m->need_keyframe==true){
            std_msgs::Header header;
            header.frame_id = "camera_init";
            header.stamp = ros::Time::now();
            pubKeyPoses(vio_l_m->map,header);
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


int main(int argc, char **argv){
    //std::cout << "Initializing ROS..." << std::endl;
    ros::init(argc, argv, "vio");
    ros::NodeHandle nh;
    //std::cout << "ROS initialized." << std::endl;
    
    //std::cout << "Creating node..." << std::endl;
    vio_in_lidar_map_node node(&nh);
    //std::cout << "Node created." << std::endl;

    node.run();
    return 0;
}
