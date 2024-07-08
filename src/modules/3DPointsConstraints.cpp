#include "lmlvil.h"
namespace lvo{

void lmlvil::triangulate(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    triangulate_pts_body->clear();
    for(int i=0;i<tracked_points.size();i++){
        if(is_tran[i] == true) continue;
            auto dx = pts_last.at(i).x - tracked_points[i].x;
            auto dy = pts_last.at(i).y - tracked_points[i].y;
            if(sqrt(dx*dx+dy*dy)<20){
                continue;   
            }
            auto tracked_num = all_tracked_points.size() - tracked_start_index[i];
            Eigen::MatrixXd svd_A(2*tracked_num, 4);
            int f_num = 0;
            int svd_idx = 0;
            M3D R0; V3D t0;
            M3D R1; V3D t1;
            R0 = new_frame->T_f_w_.rotation_matrix().transpose();
            t0 = new_frame->pos();
            Eigen::Matrix<double, 3, 4> P0;
            P0.leftCols<3>() = Eigen::Matrix3d::Identity();
            P0.rightCols<1>() = Eigen::Vector3d::Zero();
            V3D f0 = new_frame->c2f(V2D{pts_last.at(i).x,pts_last.at(i).y});
            f0 = f0.normalized();
            svd_A.row(svd_idx++) = f0[0] * P0.row(2) - f0[2] * P0.row(0);
            svd_A.row(svd_idx++) = f0[1] * P0.row(2) - f0[2] * P0.row(1);
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
    }

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

                continue;
            }

            const int eff_num = resT_sub.size();
            Eigen::MatrixXd HT_sub = MatrixXd::Zero(eff_num, DIM_STATE);
            for(int l=0;l<6;l++){
                HT_sub.col(l) = JT_sub.col(l);
            }

            auto HT_sub_T = HT_sub.transpose();    
            HT_T_HT = HT_sub_T * HT_sub;
            
            MD(DIM_STATE, DIM_STATE) &&KT_1 = (HT_T_HT + (state->cov / TRIAN_COV).inverse()).inverse();
            auto &&HTTz = HT_sub_T * resT_sub;
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
            
            if(meansT>last_error){
                *state = old_state; 
                GT = lastGT;
            }
            else 
            if(state->pos_end.hasNaN()==true){
                *state = old_state;
            }
            else{
                last_error = meansT; 
                lastGT = GT;
            }   
            if ((rotT_add.norm() * 57.3f < 0.001f) && (tT_add.norm() * 100.0f < 0.001f))
            {
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
    }
}
bool lmlvil::CalculateJTandResT(){
    Rwi = state->rot_end;                
    Pwi = state->pos_end;               
    Rcw = Rci * Rwi.transpose();    
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;   
    Rwc = Rcw.transpose();
    Pwc = -Rwc*Pcw;
    laserCloudOri->clear();
    corr_normvect->clear();
    memset(point_selected_surf, false, sizeof(point_selected_surf));
    bool valid = true;

    if(iterate_num_T==0){
        for(auto it = triangulate_pts_world->begin();it!=triangulate_pts_world->end();it++){
            V3D p_world(it->x, it->y, it->z);
            V3D p_body = Rcw * p_world + Pcw;
            PointType p_body_;
            p_body_.x = p_body.x();
            p_body_.y = p_body.y();
            p_body_.z = p_body.z();
            p_body_.curvature = 2;  
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

        point_selected_surf[ii] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > max_dis ? false
                                                                                                                                : true;
        if (!point_selected_surf[ii])
            continue;
        VD(4) pabcd;
        pabcd.setZero();
        
        if (esti_plane(pabcd, points_near, 0.1)) 
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
        {   
            it->curvature = 1;  
            V3D p_body(it->x, it->y, it->z);
            V3D p_world = Rwc*p_body+Pwc;
            PointType p_world_;
            p_world_.x = p_world.x();
            p_world_.y = p_world.y();
            p_world_.z = p_world.z();
            laserCloudOri->points[effct_feat_num] = triangulate_pts_body->points[ii];
            corr_normvect->points[effct_feat_num] = normvec->points[ii];
            effct_feat_num++;
            
        }
    }
    if (effct_feat_num < 1)
    {
        valid = false;
        return valid;
    }
    
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

        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(state->rot_end.conjugate() * norm_vec);
        V3D A(point_crossmat * C);

        JT_sub.block<1, 6>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A);
        resT_sub(i) = -norm_p.intensity;    
    }
    return valid;
}
}