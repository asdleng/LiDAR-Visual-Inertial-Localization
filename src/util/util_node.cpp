#include "lmlvil_node.h"

void lmlvil_node::para(){
    nh.param<string>("camera/img_topic", img_topic, "/img");
    nh.param<string>("common/imu_topic", imu_topic, "/imu");
    nh.param<string>("common/lid_topic", lidar_topic, "/lidar");
    nh.param<double>("theta_thre",lm_lvil->theta_thre,0.8);
    nh.param<bool>("enable_line",lm_lvil->enable_line,true);
    nh.param<bool>("cut_points_above_ground",lm_lvil->cut_points_above_ground,true);
    nh.param<double>("cam_fx",cam_fx,453.483063);
    nh.param<double>("cam_fy",cam_fy,453.254913);
    nh.param<double>("cam_cx",cam_cx,318.908851);
    nh.param<double>("cam_cy",cam_cy,234.238189);
    nh.param<double>("img_scale",lm_lvil->img_scale,1.0);
    nh.param<int>("window_size",lm_lvil->window_size,10);
    nh.param<int>("max_iteration",lm_lvil->max_iteration,10);
    nh.param<int>("grid_size",lm_lvil->grid_size,40);
    nh.param<int>("patch_size",lm_lvil->patch_size,4);
    nh.param<int>("init_num",init_num,100);
    nh.param<double>("filter_size",lm_lvil->filter_size,0.2);
    nh.param<double>("score_threshold",lm_lvil->score_threshold,50.0);
    nh.param<vector<double>>("mapextT", mapextT, vector<double>());
    nh.param<vector<double>>("mapextR", mapextR, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<vector<double>>("camera/Pci", cameraextrinT, vector<double>());
    nh.param<vector<double>>("camera/Rci", cameraextrinR, vector<double>());
    nh.param<double>("outlier_threshold", lm_lvil->outlier_threshold, 100.0);
    nh.param<double>("mapping/gyr_cov_scale",cov_gyr_scale,1.0);
    nh.param<double>("mapping/acc_cov_scale",cov_acc_scale,1.0);
    nh.param<double>("mapping/gyr_Q_scale",p_imu->gyr_Q_scale,10.0);
    nh.param<double>("mapping/acc_Q_scale",p_imu->acc_Q_scale,10.0);
    nh.param<double>("mapping/filter_size_surf_min",filter_size_surf_min,0.2);
    nh.param<double>("img_point_cov",lm_lvil->IMG_COV,100.0);
    nh.param<double>("projection_point_cov",lm_lvil->POINT_COV,100.0);
    nh.param<double>("triangulate_point_cov",lm_lvil->TRIAN_COV,10.0);
    nh.param<double>("line_cov",lm_lvil->LINE_COV,100.0);
    nh.param<double>("camera/blind",lm_lvil->blind,0.0);
    nh.param<double>("camera/max_blind",lm_lvil->max_blind,100);
    nh.param<bool>("read_enable",lm_lvil->read_enable,true);
    nh.param<bool>("enable_BA",lm_lvil->enable_BA,true);
    nh.param<bool>("pub_pcl",pub_pcl,true);
    nh.param<bool>("left_coord",lm_lvil->left_coord,false);
    nh.param<bool>("need_down_size",lm_lvil->need_down_size,true);
    nh.param<bool>("equalize", lm_lvil->equalize,false);
    nh.param<string>("map_dir", filename,string(string(ROOT_DIR)+"/map/scans.pcd"));
    nh.param<string>("path_file", path_file,"path.txt");
    nh.param<string>("lines_file", lm_lvil->lines_file,"line.txt");
    nh.param<vector<double>>("origin_pose", origin_pose,vector<double>());
    nh.param<string>("encoding", encoding, "bgr8");
    nh.param<bool>("give_init_v_a",give_init_v_a,false);
    nh.param<vector<double>>("init_v_a", init_v_a,vector<double>());
    nh.param<vector<double>>("init_ba_bg", init_ba_bg,vector<double>());
    nh.param<vector<double>>("init_g", init_g,vector<double>());
    nh.param<bool>("convert_map",lm_lvil->convert_map,false);
    nh.param<bool>("mapping/normalized",p_imu->normalized,true);
    nh.param<bool>("enable_vis",enable_vis,true);
    nh.param<bool>("pure_lidar",pure_lidar,false);
    nh.param<bool>("enable_projection",lm_lvil->enable_projection,true);
    nh.param<bool>("enable_triangulate",lm_lvil->enable_triangulate,false);
    nh.param<int>("down_sample_manner",lm_lvil->down_sample_manner,0);
    nh.param<int>("down_sample_num",lm_lvil->down_sample_num,50);
    nh.param<bool>("add_stock",add_stock,false);
    nh.param<bool>("ncc_en",lm_lvil->ncc_en,false);
    nh.param<bool>("enable_lidar",enable_lidar,false);
    nh.param<double>("ncc_thre",lm_lvil->ncc_thre,0.0);
    nh.param<int>("depth_search_radius",lm_lvil->depth_search_radius,4);
    nh.param<double>("trans_thre",lm_lvil->trans_thre,0.1);
    nh.param<double>("rot_thre",lm_lvil->rot_thre,PI_M/12);
    nh.param<int>("grid_pts_num",lm_lvil->grid_pts_num, 1);
    nh.param<double>("linethre3d",lm_lvil->linethre3d, 2.0);
    nh.param<int>("linethre3d_proj",lm_lvil->linethre3d_proj, 10);
    nh.param<int>("linethre2d",lm_lvil->linethre2d, 20);
    nh.param<double>("lamda",lm_lvil->lamda, 0.0394);
    nh.param<double>("line_threshold",lm_lvil->line_threshold, 50);
    nh.param<double>("edgescore_threshold",lm_lvil->edge_threshold, 100.0);
    nh.param<int>("pyr",lm_lvil->pyr,1);
    nh.param<double>("delta_dist_thre",lm_lvil->delta_dist_thre,1.5);
    nh.param<double>("skip_depth",lm_lvil->skip_depth,1.0);
    nh.param<double>("t_shift",t_shift,0.0);
    nh.param<int>("low_pyr",lm_lvil->low_pyr,0);
    nh.param<double>("preprocess/blind", preprocess->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", preprocess->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", preprocess->N_SCANS, 16);
    nh.param<int>("preprocess/scan_rate", preprocess->SCAN_RATE, 10);
    nh.param<int>("preprocess/point_filter_num", preprocess->point_filter_num, 2);
    nh.param<bool>("preprocess/feature_extract_enable", preprocess->feature_enabled, false);
}
void lmlvil_node::static_h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<state_ikfom::scalar> &ekfom_data) {
    if (instance) {
        instance->h_share_model(s, ekfom_data);
    }
}
void lmlvil_node::publish_odometry(const ros::Publisher & pubOdomAftMapped,const ros::Publisher & pubCamera)
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
    Quaterniond q_lidar_imu_(extR);
    tf::Quaternion q_lidar_imu;
    q_lidar_imu.setW(q_lidar_imu_.w());
    q_lidar_imu.setX(q_lidar_imu_.x());
    q_lidar_imu.setY(q_lidar_imu_.y());
    q_lidar_imu.setZ(q_lidar_imu_.z());
    transform_lidar.setRotation(q_lidar_imu);
    br_lidar_map.sendTransform(tf::StampedTransform(transform_lidar, odomAftMapped.header.stamp, "camera_init", "lidar"));
    tf::Transform transform_camera;
    V3D Pic = - lm_lvil->Rci.transpose()* lm_lvil->Pci;
    transform_camera.setOrigin(tf::Vector3(Pic[0],Pic[1],Pic[2]));
     Quaterniond q_camera_imu_(lm_lvil->Rci.transpose());
    tf::Quaternion q_camera_imu;
    q_camera_imu.setW(q_camera_imu_.w());
    q_camera_imu.setX(q_camera_imu_.x());
    q_camera_imu.setY(q_camera_imu_.y());
    q_camera_imu.setZ(q_camera_imu_.z());
    transform_camera.setRotation(q_camera_imu);
    br_camera.sendTransform(tf::StampedTransform(transform_camera, odomAftMapped.header.stamp, "aft_mapped", "camera"));

}
void lmlvil_node::publish_observed_world(const ros::Publisher & pubLaserCloudObserved)
{
    uint size = lm_lvil->pcl_projected->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*lm_lvil->pcl_projected, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudObserved.publish(laserCloudmsg);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void lmlvil_node::publish_frame_world_Triangulate(const ros::Publisher & pub)
{
    uint size = lm_lvil->triangulate_pts_world->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*lm_lvil->triangulate_pts_world, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pub.publish(laserCloudmsg);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void lmlvil_node::publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)
{
    uint size = lm_lvil->pcl_down->points.size();

    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*lm_lvil->pcl_down, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}
void lmlvil_node::publish_lidar_frame_world(const ros::Publisher &pubLaserCloudFull)
{
    int size = feats_down_body->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&feats_down_body->points[i],
                                &laserCloudWorld->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull.publish(laserCloudmsg);
}
void lmlvil_node::publish_local_map(const ros::Publisher & pubLocalMap)
{
    uint size = lm_lvil->local_map->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*lm_lvil->local_map, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLocalMap.publish(laserCloudmsg);

    }

}
void lmlvil_node::publish_frame_world_3DLine(const ros::Publisher & pub3DLine)
{
    uint size = lm_lvil->pcl_down->points.size();
    if (1)
    {
        visualization_msgs::Marker line_list;
        line_list.id = 2;
        line_list.color.b = 1.0;
        line_list.color.g = 1.0;
        line_list.color.r = 0.0;
        line_list.color.a = 1.0;
        line_list.scale.x = 0.01;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        for(int i=0;i<lm_lvil->lines.size();i++){
            geometry_msgs::Point p1,p2;
            p1.x = lm_lvil->lines[i][0].x;
            p1.y = lm_lvil->lines[i][0].y;
            p1.z = lm_lvil->lines[i][0].z;
            p2.x = lm_lvil->lines[i][1].x;
            p2.y = lm_lvil->lines[i][1].y;
            p2.z = lm_lvil->lines[i][1].z;
            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
        }

        line_list.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        line_list.header.frame_id = "camera_init";
        pub3DLine.publish(line_list);

    }

}
void lmlvil_node::publish_frame_world_line(const ros::Publisher & pubLaserCloudFullRes)
{
    uint size = lm_lvil->pcl_line->points.size();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*lm_lvil->pcl_line, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);

    }
    // mtx_buffer_pointcloud.unlock();
}
void lmlvil_node::publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "camera_init";
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}
void lmlvil_node::publish_true_path(const ros::Publisher pubTruePath){
    true_path.header.stamp = ros::Time::now();
    pubTruePath.publish(true_path);
}

void lmlvil_node::publish_depth_img(const image_transport::Publisher pubDepthImg){
    cv::Mat img_depth = lm_lvil->img_depth;
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    out_msg.image = img_depth;
    pubDepthImg.publish(out_msg.toImageMsg());
}
void lmlvil_node::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (extR * p_body + extT) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}