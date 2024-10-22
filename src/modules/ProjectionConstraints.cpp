#include "lmlvil.h"
namespace lvo{
void lmlvil::projectionConstraint(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    vector<uchar> status;
    vector<float> err;
    if(first_frame){
        img_last = new_frame->img();
        featAdd();
    }
    else{
        img_cur = new_frame->img();
        
        cv::calcOpticalFlowPyrLK(img_last,img_cur,pts_last,pts_cur,status,err,cv::Size(21, 21), 3);

        reduceVector(pts_last,status);
        reduceVector(pts_cur,status);
        reduceVector2(ptws_last,status);
        reduceVector(is_tran,status);
        reduceVector(pts_T0,status);
        reduceVector(tracked_points,status);
        reduceVector(tracked_start_index,status);
        for(int i=0;i<all_tracked_points.size();i++){
            reduceVector(all_tracked_points[i],status);
        }
        for(int i=0;i<all_pts_T.size();i++){
            reduceVector(all_pts_T[i],status);
        }

        img_last = img_cur;
        pts_last = pts_cur;
        iterate_num_P = 0;
        last_error = 10000;
        StatesGroup * state_before = new StatesGroup(*state);
        const int pts_num = pts_last.size();
        track_outlier.resize(pts_num);
        std::fill(track_outlier.begin(),track_outlier.end(),0);
        int wrong_time = 0;
        for(int i=0;i<max_iteration;i++){
            auto old_state = *state;
            auto valid = CalculateJPandResP();
            if(!valid){
                continue;
            }
            const int eff_num = resP_sub.size();
            Eigen::MatrixXd HP_sub = MatrixXd::Zero(eff_num, DIM_STATE);
            for(int l=0;l<6;l++){
                HP_sub.col(l) = JP_sub.col(l);
            }
            auto HP_sub_T = HP_sub.transpose();
            HP_T_HP = HP_sub_T * HP_sub;
            

            MD(DIM_STATE, DIM_STATE) &&KP_1 = (HP_T_HP + (state->cov / POINT_COV).inverse()).inverse();
            auto &&HPTz = HP_sub_T * resP_sub;
            auto vecP = (*state_before) - (*state);
            GP = KP_1 * HP_T_HP;
            auto solutionP = - KP_1 * HPTz + vecP - GP* vecP;
            (*state) += solutionP;
            auto &&rotP_add = solutionP.block<3,1>(3,0);
            auto &&tP_add   = solutionP.block<3,1>(0,0);
            
			int k = 0;
            double meansP = 0;
			for(int j=0;j<resP_sub.rows();j++){
				k++;
				meansP+=fabs(resP_sub(j));
			}
			meansP = meansP/k;
            
            if(meansP>last_error){
                wrong_time++;
                *state = old_state; 
                GP = lastGP;
            }
            else 
                if(state->pos_end.hasNaN()==true){
                    *state = old_state;
                }
            else{
            // 成功了
                last_error = meansP; 
                lastGP = GP;
            }   
            if ((rotP_add.norm() * 57.3f < 0.001f) && (tP_add.norm() * 100.0f < 0.001f) || wrong_time>2)
            {
                if(wrong_time>2){
                }
                else{
                }
                break;
            }
            iterate_num_P++;
        }
        state->cov -= GP*state->cov;
        Eigen::Quaterniond rotation(state->rot_end);
        rotation.normalize();
        state->rot_end = rotation.toRotationMatrix();
        updateFrameState();
        setKF(kf);
        if(1) {
            featAdd();
            
        }
    }
}
bool lmlvil::featAdd(){
    if(pts_last.size()>=10000) return true;
    auto mask = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
    
    for(int i=0;i<pts_last.size();i++){
        cv::Point p(pts_last[i].x,pts_last[i].y);
        cv::circle(mask, p, 30, 0, -1);
    }
    for(int i=0;i<length;i++){
        if(grid.cells[i]->size()==0) continue;
        for(auto it=grid.cells[i]->begin();it!=grid.cells[i]->end();it++){
            auto it_ptr = it->pt;
            if (it_ptr->value<score_threshold&&!first_frame) continue;
            V3D pt_w = it_ptr->pos_;                    // 空间点坐标
            V3D pf = Rcw * pt_w + Pcw;              // 相机系坐标
            V2D px = cam->world2cam(pf);    // 像素坐标
            cv::Point2f pp(px[0],px[1]);
            if (mask.at<uchar>(pp) == 0) continue;
            pts_last.push_back(pp);
            ptws_last.push_back(pt_w);
            pts_T0.push_back(new_frame->T_f_w_);
            tracked_points.push_back(pp);
            tracked_start_index.push_back(all_pts_T.size());
            is_tran.push_back(false);
            if(pts_last.size()>=10000) break;
            cv::circle(mask, pp, 30, 0, -1);
        }
    }
    all_tracked_points.push_back(pts_last);
    auto cur_pts_T = std::vector<Sophus::SE3>(pts_last.size(),new_frame->T_f_w_);
    all_pts_T.push_back(cur_pts_T);

    return true;
}
bool lmlvil::CalculateJPandResP(){
    const int H_DIM = ptws_last.size();  
    Rwi = state->rot_end;                
    Pwi = state->pos_end;               
    Rcw = Rci * Rwi.transpose();    
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;   
    JP.resize(H_DIM*2,6);  
    resP.resize(H_DIM*2,1);
    JP.setZero();
    resP.setZero();
    std::vector<int> index_table;
    int kk = -2;
    for(int i=0;i<H_DIM;i++){
        V2D pc_ref;
        pc_ref[0] = pts_last[i].x;pc_ref[1] = pts_last[i].y;
        V3D pt_w = ptws_last[i]; 
        V3D pf = Rcw * pt_w + Pcw;  
        V2D pc = cam->world2cam(pf);    
        kk = kk+2;
        double du = pc[0] - pc_ref[0];
        double dv = pc[1] - pc_ref[1];
        if(iterate_num_P==0){

            if((du*du+dv*dv)>10){
                track_outlier[i] = 1;
                continue;
            }   
            cv::Point gird_point(pc[0],pc[1]);
            cv::circle(img_cp,gird_point,2,cv::Scalar(0, 255, 0), -1, 8);  
        }
        if(track_outlier[i]==1) continue;
        MD(2,3) Jdpi;
        MD(2,3) Jdphi, Jdp, JdR, Jdt;
        M3D p_hat;
        M3D Jdp_dt;
        Jdp_dt = Rci * Rwi.transpose(); 
        dpi(pf, Jdpi);
        p_hat << SKEW_SYM_MATRX(pf);

        Jdphi = Jdpi * p_hat;    
        Jdp = -Jdpi;                 
        JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
        Jdt = Jdp * Jdp_dt;
        int row_index = kk;
        resP(row_index) = du;
        resP(row_index+1) = dv;
        JP.block<2,6>(row_index,0)<< Jdt, JdR;
        index_table.push_back(row_index);
        index_table.push_back(row_index+1);
    }
    if(index_table.size()==0) return false;
    JP_sub = MatrixXd::Zero(index_table.size(),6);
    resP_sub = VectorXd::Zero(index_table.size(),1);
    for(int j=0;j<index_table.size();j++){
        JP_sub.block<1,6>(j,0) =  JP.block<1,6>(index_table[j],0);
        resP_sub.block<1,1>(j,0) =  resP.block<1,1>(index_table[j],0);
    }
    return true;
}
}