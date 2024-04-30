#include "vio_in_lidar_map_node.h"
bool vio_in_lidar_map_node::undistort(){
    mtx_buffer_lidar.lock();
    auto points = lidar_buffer.front().first;
    downSizeFilterSurf.setInputCloud(points);  // 输入去畸变点云，输出降采样点云(雷达坐标系)
    downSizeFilterSurf.filter(*feats_down_body); 
    feats_down_size = feats_down_body->points.size();
    vio_l_m->debug_file<<"降采样后点云数"<<feats_down_size<<std::endl;
    feats_down_world->resize(feats_down_size);
    for(int i=0;i<feats_down_size;i++){
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    }
    lidar_buffer.pop_front();
    mtx_buffer_lidar.unlock();
//     auto time = lidar_buffer.back().second;
//     auto begin_time = time-0.1; // 根据lidar频率
//     auto lidar = lidar_buffer.back().first;
//     auto it_pcl = lidar->points.end() - 1;

//     for(auto it=imu_buffer_lidar.begin();it!=imu_buffer_lidar.end();it++){
//             if(it->get()->header.stamp.toSec()<time-0.1)
//                 continue;
//             if(it->get()->header.stamp.toSec()>time)
//                 break;

//             auto head = it - 1;
//             auto tail = it;
//             head->get().
//             R_imu << MAT_FROM_ARRAY(head->rot);
//             vel_imu << VEC_FROM_ARRAY(head->vel);
//             pos_imu << VEC_FROM_ARRAY(head->pos);
//             acc_imu << VEC_FROM_ARRAY(tail->acc);
//             angvel_avr << VEC_FROM_ARRAY(tail->gyr);
//             for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
//             {
//             // cout<<it_pcl->curvature/ double(1000)<<",";
//             auto dt = it_pcl->curvature / double(1000) - head->offset_time;

//             M3D R_i(R_imu * Exp(angvel_avr, dt));

//             // 本质上是将P_i转换到全局坐标系，再转换到end的坐标系下
//             V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
//             V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
//             V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I); // not accurate!

//             // save Undistorted points and their rotation
//             it_pcl->x = P_compensate(0);
//             it_pcl->y = P_compensate(1);
//             it_pcl->z = P_compensate(2);

//             if (it_pcl == pcl_out.points.begin())
//                 break;
//             }

//         }
//     }
//     /*** undistort each lidar point (backward propagation) ***/
//     auto it_pcl = pcl_out.points.end() - 1;
//     for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
//     {
//         auto head = it_kp - 1;
//         auto tail = it_kp;
//         R_imu << MAT_FROM_ARRAY(head->rot);
//         vel_imu << VEC_FROM_ARRAY(head->vel);
//         pos_imu << VEC_FROM_ARRAY(head->pos);
//         acc_imu << VEC_FROM_ARRAY(tail->acc);
//         angvel_avr << VEC_FROM_ARRAY(tail->gyr);
        
        
//         // cout<<endl;
    return true;
}
void vio_in_lidar_map_node::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data){
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (extR * p_body + extT) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];
        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            vio_l_m->ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4)
        pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //状态是23，实际只有12个状态优化了，即自身旋转平移和雷达外参
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = extR * point_this_be + extT;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (0)
        {
            V3D B(point_be_crossmat * extR.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }   // h_x: 残差的雅可比

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;    // h: 残差
    }
    // cout<<"h:";
    // cout<< fixed<<setprecision(3)<< ekfom_data.h.transpose()<<endl;
    solve_time += omp_get_wtime() - solve_start_;
    
    return;
}

/***将点从lidar坐标系边到地面坐标系***/
void vio_in_lidar_map_node::pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (extR * p_body + extT) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}