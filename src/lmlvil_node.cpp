#include "lmlvil_node.h"
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>


lmlvil_node* lmlvil_node::instance = nullptr;

void lmlvil_node::wirteTUM(double current_time, nav_msgs::Odometry odom){
    o_pose<< fixed<<setprecision(4) << current_time << " " << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " " << odom.pose.pose.position.z  << " " 
  << odom.pose.pose.orientation.x  << " " <<  odom.pose.pose.orientation.y << " " <<odom.pose.pose.orientation.z << " " << odom.pose.pose.orientation.w << std::endl;
}
cv::Mat lmlvil_node::getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg) {
  cv::Mat img;
    if(encoding=="bgr8"){
        img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
 
    }
    else if(encoding=="rgb8"){
        img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8)->image;

    }
    else if(encoding=="mono8"){
        img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8)->image;

    }
    else if(encoding=="bgra8"){
        img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGRA8)->image;

    }
    else{
        std::cout<<"Wrong encoding, please check the image encoding"<<std::endl;
    }
  return img;
}
void lmlvil_node::publish_img(const image_transport::Publisher pubImg){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
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
    out_msg.image = lm_lvil->img_cp;
    pubImg.publish(out_msg.toImageMsg());
}
void lmlvil_node::img_cbk(const sensor_msgs::Image::ConstPtr &msg){
    mtx_buffer_img.lock();
    cv::Mat img = getImageFromMsg(msg);
    std::pair<cv::Mat,double> img_pair;
    img_pair.first = img; img_pair.second = msg->header.stamp.toSec();
    img_buffer.push_back(img_pair);
    mtx_buffer_img.unlock();
    new_image = true;
            
}

void lmlvil_node::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec()-t_shift;

    mtx_buffer_imu.lock();  

    imu_buffer.push_back(msg);  
    mtx_buffer_imu.unlock();    
}

void lmlvil_node::lidar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in){
    mtx_buffer_lidar.lock();
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    preprocess->process(msg_in,ptr);
    std::pair<PointCloudXYZI::Ptr,double> lidar_pair;
    lidar_pair.first = ptr; lidar_pair.second = msg_in->header.stamp.toSec();
    
    lidar_buffer.push_back(lidar_pair);
    mtx_buffer_lidar.unlock();
}

void lmlvil_node::run(){
    p_imu->cov_gyr_scale = V3D(cov_gyr_scale,cov_gyr_scale,cov_gyr_scale);
    p_imu->cov_acc_scale = V3D(cov_acc_scale,cov_acc_scale,cov_acc_scale);
    extT<<VEC_FROM_ARRAY(extrinT);
    extR<<MAT_FROM_ARRAY(extrinR);
    lm_lvil->set_extrinsic(extT,extR);
    lm_lvil->set_cameraext(cameraextrinR, cameraextrinT);
    lm_lvil->set_mapext(mapextR,mapextT);
    if (pcl::io::loadPCDFile<PointType>(filename, *src) == -1) 
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return;
    }
    double epsi[24] = {0.001};
    fill(epsi, epsi + 24, 0.001);
    std::function<void(state_ikfom&, esekfom::dyn_share_datastruct<state_ikfom::scalar>&)> func = std::bind(&lmlvil_node::h_share_model, this, std::placeholders::_1, std::placeholders::_2);
    kf.init_dyn_share(get_f, df_dx, df_dw, lmlvil_node::static_h_share_model, NUM_MAX_ITERATIONS, epsi);

    o_pose.open(FILE_DIR(euroc_file), ios::out);
    image_transport::ImageTransport it(nh);
    img_pub = it.advertise("/rgb_img", 1);
    image_transport::Publisher img_depth_pub = it.advertise("/depth_img", 1);
      if(!vk::camera_loader::loadFromRosNs("lmlvil", lm_lvil->cam))
    throw std::runtime_error("Camera model not correctly specified.");
    
    lm_lvil->Init(src);
    lm_lvil->fx = cam_fx;
    lm_lvil->fy = cam_fy;
    lm_lvil->cx = cam_cx;
    lm_lvil->cy = cam_cy;
    
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
        int total_points = static_cast<int>(area * points_per_unit);

        int ii = lm_lvil->pcl_down->size()-1;
        for (int i = 0; i < total_points; ++i) {
            float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
            float r = radius * sqrt(static_cast<float>(rand()) / RAND_MAX);
            float x = r * cos(angle);
            float y = r * sin(angle);

            PointType point;
            point.x = state0.pos[0]+x;
            point.y = state0.pos[1]+y;
            point.z = state0.pos[2]-0.1;
            point.intensity = 1.0;  

            lm_lvil->pcl_down->push_back(point);

        }
        for(;ii<lm_lvil->pcl_down->size();ii++){
            V3D pos(lm_lvil->pcl_down->at(ii).x,
            lm_lvil->pcl_down->at(ii).y,
            lm_lvil->pcl_down->at(ii).z);

            lvo::PointPtr p(new lvo::Point(pos));
            p->id_ = ii;
            lm_lvil->map.addPoint(p);  
        }
    }

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
        esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf.get_P();
        init_P.setIdentity();
        init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.001; // 旋转外参
        init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.001; // 平移外参
        init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.01;  // bg
        init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.01; // ba
        init_P(21, 21) = init_P(22, 22) = init_P(23, 23) = 0.01;  //g
        kf.change_P(init_P);
        give_init_v_a = false;
        p_imu->give_init = true;
        ROS_INFO("Give Initial States");
    }
    int frame_count_pub=0;
    int odom_count_pub=0;
    ros::Rate rate(500);   

    ros::Time start = ros::Time::now();
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

        ros::spinOnce();
        
        if(img_buffer.size()==0){
            continue;   
        }
        if(imu_buffer.size()==0){
            continue;   
        }
        current_imu_time = imu_buffer.back().get()->header.stamp.toSec();
        process_img_time = img_buffer.front().second;
        
        if(process_img_time>current_imu_time){
            continue;  
        }

        
        if(imu_init==false){
            if(imu_buffer.size()>init_num){
                mtx_buffer_imu.lock();
                int init_iter_num=0;
                while(init_iter_num<=init_num){
                    p_imu->IMU_init(imu_buffer,kf,init_iter_num);
                }
                imu_init = true;
                ROS_INFO("IMU Initial Done");
                last_imu_ = imu_buffer.back();
                mtx_buffer_imu.unlock();
            }
            else{
                continue;
            }
        }
        mtx_buffer_imu.lock();
        if(imu_buffer.size()<=2) continue;
        while(imu_buffer.size()>2){
            auto it_ptr = imu_buffer.begin()+1;
            if(it_ptr->get()->header.stamp.toSec()>process_img_time)
                break;
            auto imu1 = *(it_ptr-1);
            last_imu = *imu1;
            auto imu2 = *it_ptr;
            ms->imu.push_back(imu2);
            if(lidar_buffer.size()>0 && enable_lidar){
                if(lidar_buffer.front().second<imu2->header.stamp.toSec()){
                    auto t1 = omp_get_wtime();
                    mtx_buffer_lidar.lock();
                    ms->lidar = lidar_buffer.front().first;
                    p_imu->intergrate(*imu1,*imu2,kf,process_img_time);
                    downsample(*ms);
                    double solve_H_time = 0;
                    Nearest_Points.resize(feats_down_size);
                    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                    publish_lidar_frame_world(pubLaserCloudLiDARFull);
                    lidar_buffer.pop_front();
                    ms->lidar->clear();       
                    ms->imu.clear();          
                    mtx_buffer_lidar.unlock();
                    auto t2 = omp_get_wtime();
                }
                else{
                    p_imu->intergrate(*imu1,*imu2,kf,process_img_time);
                }
            }
            else{ 
                p_imu->intergrate(*imu1,*imu2,kf,process_img_time);
            }
            imu_buffer.pop_front();
        }
        mtx_buffer_imu.unlock();

        mtx_buffer_img.lock();
        if(enable_vis){
            auto t1 = omp_get_wtime();
            lm_lvil->Process(img_buffer.front().first,kf);
            auto t2 = omp_get_wtime();
        }
        
        
        state_point = kf.get_x();
        publish_odometry(pubOdomAftMapped,pubCamera);
        publish_path(pubPath);

        wirteTUM(img_buffer.front().second,odomAftMapped);
        publish_img(img_pub);
        img_buffer.pop_front();
        
        mtx_buffer_img.unlock();
        
        if(lm_lvil->need_keyframe==true){
            std_msgs::Header header;
            header.frame_id = "camera_init";
            header.stamp = ros::Time::now();
            pubKeyPoses(lm_lvil->map,header);
            publish_observed_world(pubLaserCloudObserved);
            //publish_local_map(pubLocalMap);
            if(lm_lvil->enable_triangulate){
                publish_frame_world_Triangulate(pubTriangulate);
            }
        }
        rate.sleep();
    }
    auto pos = odomAftMapped.pose.pose.position;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "lmlvil");
    ros::NodeHandle nh;
    lmlvil_node node(&nh);
    node.run();
    return 0;
}
