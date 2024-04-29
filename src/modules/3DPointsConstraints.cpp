#include "vio_in_lidar_map.h"
namespace lvo{

void violm::triangulate(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    triangulate_pts_body->clear();
    // debug_file<<"上一关键帧提取点的个数"<<tracked_points.size()<<std::endl;
    // debug_file<<"当前关键帧跟踪点个数"<<pts_last.size()<<std::endl;
    for(int i=0;i<tracked_points.size();i++){
        if(is_tran[i] == true) continue;
            auto dx = pts_last.at(i).x - tracked_points[i].x;
            auto dy = pts_last.at(i).y - tracked_points[i].y;
            if(sqrt(dx*dx+dy*dy)<20){
                continue;   // 视差太小
            }
            auto tracked_num = all_tracked_points.size() - tracked_start_index[i];
            Eigen::MatrixXd svd_A(2*tracked_num, 4);
            int f_num = 0;
            int svd_idx = 0;
            M3D R0; V3D t0;
            M3D R1; V3D t1;
            //debug_file<<"该点三角化时总共被"<<tracked_num<<"帧看到"<<std::endl;
            R0 = new_frame->T_f_w_.rotation_matrix().transpose();
            t0 = new_frame->pos();
            Eigen::Matrix<double, 3, 4> P0;
            P0.leftCols<3>() = Eigen::Matrix3d::Identity();
            P0.rightCols<1>() = Eigen::Vector3d::Zero();
            V3D f0 = new_frame->c2f(V2D{pts_last.at(i).x,pts_last.at(i).y});
            f0 = f0.normalized();
            svd_A.row(svd_idx++) = f0[0] * P0.row(2) - f0[2] * P0.row(0);
            svd_A.row(svd_idx++) = f0[1] * P0.row(2) - f0[2] * P0.row(1);
            //debug_file<<"f0: "<<f0<<std::endl;
            //debug_file<<"P0: "<<P0<<std::endl;
            for(int j=tracked_start_index[i];j<all_tracked_points.size()-1;j++){
                R1 = all_pts_T[j][i].rotation_matrix().transpose();
                t1 = all_pts_T[j][i].inverse().translation();
                V3D t = R0.transpose() * (t1 - t0);
                M3D R = R0.transpose() * R1;
                Eigen::Matrix<double, 3, 4> P;
                P.leftCols<3>() = R.transpose();
                P.rightCols<1>() = -R.transpose() * t;
                V3D f1 = cam->cam2world(V2D{all_tracked_points[j][i].x,all_tracked_points[j][i].y});
                f1 = f1.normalized();
                svd_A.row(svd_idx++) = f1[0] * P.row(2) - f1[2] * P.row(0);
                svd_A.row(svd_idx++) = f1[1] * P.row(2) - f1[2] * P.row(1);
                //debug_file<<"f1: "<<f1<<std::endl;
                //debug_file<<"P: "<<P<<std::endl;
            }
            

            ROS_ASSERT(svd_idx == svd_A.rows());
            Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
            PointType p,pf_;
            V3D pf;
            pf[0] = svd_V[0]/ svd_V[3];
            pf[1] = svd_V[1]/ svd_V[3];
            pf[2] = svd_V[2]/ svd_V[3];
            pf_.x = pf.x();
            pf_.y = pf.y();
            pf_.z = pf.z();
            triangulate_pts_body->push_back(pf_);
            is_tran[i] = true;
            // V3D pt = new_frame->f2w(pf);
            // p.x = pt.x();
            // p.y = pt.y();
            // p.z = pt.z();
            // // //debug_file<<"第"<<index<<"号点的三角化坐标为: "<<p.x<<","<<p.y<<","<<p.z<<std::endl;
            // triangulate_pts_world->push_back(p);
    }
    // 视差满足条件，开始迭代
    if(enable_triangulate){
        normvec->resize(triangulate_pts_body->size());
        StatesGroup * state_before = new StatesGroup(*state);
        iterate_num_T = 0;
        double last_error=100000;
        trian_outlier.resize(triangulate_pts_body->size(),1);
        body_pts_size = triangulate_pts_body->size();
        for(int i=0;i<max_iteration;i++){
            auto old_state = *state;
            auto valid = CalculateJTandResT();
            if(!valid){
                //debug_file<<"无点面匹配"<<std::endl;
                continue;
            }
            //debug_file<<"JT_sub: "<<JT_sub<<std::endl;
            //debug_file<<"resT_sub: "<<resT_sub.transpose()<<std::endl;
            const int eff_num = resT_sub.size();
            Eigen::MatrixXd HT_sub = MatrixXd::Zero(eff_num, DIM_STATE);
            for(int l=0;l<6;l++){
                HT_sub.col(l) = JT_sub.col(l);
            }
            
            //debug_file<<"H_sub: "<<H_sub<<std::endl; 
            auto HT_sub_T = HT_sub.transpose();    
            HT_T_HT = HT_sub_T * HT_sub;
            
            
            // debug_file<<"IMG_COV: "<<IMG_COV<<std::endl;
            MD(DIM_STATE, DIM_STATE) &&KT_1 = (HT_T_HT + (state->cov / TRIAN_COV).inverse()).inverse();
            //debug_file<<"H_T_H: "<<H_T_H<<std::endl;
            //debug_file<<"state->cov: "<<state->cov<<std::endl;
            // debug_file<<"K_1: "<<K_1<<std::endl;
            auto &&HTTz = HT_sub_T * resT_sub;
            // K = K_1.block<DIM_STATE,6>(0,0) * H_sub_T;
            auto vecT = (*state_before) - (*state);
            GT = KT_1 * HT_T_HT;
            auto solutionT = - KT_1 * HTTz + vecT - GT* vecT;
            (*state) += solutionT;
            auto &&rotT_add = solutionT.block<3,1>(3,0);
            auto &&tT_add   = solutionT.block<3,1>(0,0);
            
			int k = 0;
            double meansT = 0;
			for(int j=0;j<resT_sub.rows();j++){
				k++;
				meansT+=fabs(resT_sub(j));
			}
			meansT = meansT/k;
			//debug_file<<"第"<<iterate_num_T<<"次面约束迭代的平均error为："<<meansT<<std::endl;
            
            if(meansT>last_error){
                //debug_file<<"损失增大，回退"<<std::endl;
                *state = old_state; 
                GT = lastGT;
            }
            else 
            if(state->pos_end.hasNaN()==true){
                *state = old_state;
            }
            else{
            // 成功了
                last_error = meansT; 
                lastGT = GT;
            }   
            if ((rotT_add.norm() * 57.3f < 0.001f) && (tT_add.norm() * 100.0f < 0.001f))
            {
                //debug_file<<"迭代"<<iterate_num_T<<"次收敛"<<std::endl;
                break;
            }
            iterate_num_T++;
        }
        state->cov -= GT*state->cov;
        updateFrameState();
        setKF(kf);
        Updatekf(kf);
        UpdateCamPose();
    }
    int ii=-1;
    for (auto it = triangulate_pts_body->begin();it!=triangulate_pts_body->end();it++)
    {
        ii++;
        if(it->curvature<1) continue;
        V3D p_body(it->x, it->y, it->z);
        V3D p_world = Rwc*p_body+Pwc;
        PointType p_world_;
        p_world_.x = p_world.x();
        p_world_.y = p_world.y();
        p_world_.z = p_world.z();
        triangulate_pts_world->push_back(p_world_);
        V3D pos(p_world.x(),p_world.y(),p_world.z());
        PointPtr  p(new Point(pos));
        p->id_ = map.map_points_.size();
        map.addPoint(p);  
    }
}
bool violm::CalculateJTandResT(){
    //debug_file<<"第"<<frame_nums<<"帧的第"<<iterate_num_T<<"次面特征迭代"<<std::endl;
    Rwi = state->rot_end;                //debug_file<<"state->rot_end: "<<state->rot_end<<std::endl;
    Pwi = state->pos_end;               //debug_file<<"state->pos_end: "<<state->pos_end.transpose()<<std::endl;
    Rcw = Rci * Rwi.transpose();    //debug_file<<"state->vel_end: "<<state->vel_end.transpose()<<std::endl;
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;   //debug_file<<"state->bias_a: "<<state->bias_a.transpose()<<std::endl;
    Rwc = Rcw.transpose();
    Pwc = -Rwc*Pcw;
    laserCloudOri->clear();
    corr_normvect->clear();
    memset(point_selected_surf, false, sizeof(point_selected_surf));
    bool valid = true;

    //triangulate_pts_world->clear();
    if(iterate_num_T==0){
    //debug_file<<"新来了"<<triangulate_pts_body->size()<<"个三角化点"<<std::endl;
        // 将地面系下的点转移到body系下，用于优化
        for(auto it = triangulate_pts_world->begin();it!=triangulate_pts_world->end();it++){
            V3D p_world(it->x, it->y, it->z);
            V3D p_body = Rcw * p_world + Pcw;
            PointType p_body_;
            p_body_.x = p_body.x();
            p_body_.y = p_body.y();
            p_body_.z = p_body.z();
            p_body_.curvature = 2;  // 标记一下这是老点
            triangulate_pts_body->push_back(p_body_);
        }
        triangulate_pts_world->clear();
    }
    int ii=-1;
    for(auto it = triangulate_pts_body->begin();it!=triangulate_pts_body->end();it++){
        ii++;
        V3D p_body(it->x, it->y, it->z);
        V3D p_world = Rwc*p_body+Pwc;
        PointType p_world_;
        p_world_.x = p_world.x();
        p_world_.y = p_world.y();
        p_world_.z = p_world.z();
        
        PointVector points_near;
        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        double max_dis;
        if(it->curvature==2){
            max_dis = 0.1;
        }
        else{
            max_dis = 0.1;
        }
        ikdtree.Nearest_Search(p_world_, NUM_MATCH_POINTS, points_near, pointSearchSqDis, max_dis);
        /***Check if it is an effective point***/
        point_selected_surf[ii] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > max_dis ? false
                                                                                                                                : true;
        if (!point_selected_surf[ii])
            continue;
        VD(4) pabcd;
        pabcd.setZero();
        
        if (esti_plane(pabcd, points_near, 0.1)) //(planeValid)
        {
            float pd2 = pabcd(0) * p_world_.x + pabcd(1) * p_world_.y + pabcd(2) * p_world_.z +
                        pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9) {
                point_selected_surf[ii] = true;
                normvec->points[ii].x = pabcd(0);
                normvec->points[ii].y = pabcd(1);
                normvec->points[ii].z = pabcd(2);
                normvec->points[ii].intensity = pd2;
                res_last[ii] = abs(pd2);
            }
        }
        
    }
    
    effct_feat_num = 0;
    ii=-1;
    for (auto it = triangulate_pts_body->begin();it!=triangulate_pts_body->end();it++)
    {
        ii++;
        if (point_selected_surf[ii])
        {   // 有效观测
            it->curvature = 1;  // 标记有效
            V3D p_body(it->x, it->y, it->z);
            V3D p_world = Rwc*p_body+Pwc;
            PointType p_world_;
            p_world_.x = p_world.x();
            p_world_.y = p_world.y();
            p_world_.z = p_world.z();
            // triangulate_pts_world->push_back(p_world_);
            laserCloudOri->points[effct_feat_num] = triangulate_pts_body->points[ii];
            corr_normvect->points[effct_feat_num] = normvec->points[ii];
            effct_feat_num++;
            
        }
    }
    //debug_file<<"面约束有效点个数为："<<effct_feat_num<<std::endl;
    if (effct_feat_num < 1)
    {
        valid = false;
        return valid;
    }
    

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    JT_sub = MatrixXd::Zero(effct_feat_num, 24); //23
    resT_sub = MatrixXd::Zero(effct_feat_num, 1);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = Rci * point_this_be + Pci;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(state->rot_end.conjugate() * norm_vec);
        V3D A(point_crossmat * C);

        JT_sub.block<1, 6>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A);
        /*** Measuremnt: distance to the closest surface/corner ***/
        resT_sub(i) = -norm_p.intensity;    
    }
    return valid;
}
}