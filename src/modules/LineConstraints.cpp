#include "vio_in_lidar_map.h"
namespace lvo{
/*线约束相关内容*/

void violm::lineConstraint(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    boundingbox_t bbox;
    bbox.height = height;
    bbox.width = width;
    bbox.x = bbox.y = 0;
    std::vector<line_float_t> lines1;
    LsdLineDetector(img.data, width, height,
	1.0,1.0, bbox, lines1);
    auto it = lines1.begin();
    while(it!=lines1.end()){
        double dis = std::sqrt((it->startx-it->endx)*(it->startx-it->endx)+
        (it->starty-it->endy)*(it->starty-it->endy));
        if(dis<linethre2d)  it = lines1.erase(it);
        else  it++;
    }
    lines_2d.clear();
    for(int i=0;i<lines1.size();i++){
        cv::Point pt1,pt2;
        pt1.x =  lines1[i].startx; pt1.y = lines1[i].starty;
        pt2.x = lines1[i].endx; pt2.y = lines1[i].endy;
        //cv::line(img_cp, pt1, pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_4);
        line2d line_2d_tmp(Vector4d(pt1.x,pt2.x,pt2.x,pt2.y));
        lines_2d.push_back(line_2d_tmp);
    }
    
    matches2d3d.clear();
    matches2d3d = updatecorrespondence(lines_3d,lines_2d,pin_cam->K(),
    Rcw,Pcw,lamda,line_threshold);
    debug_file<<"线段匹配情况："<<std::endl;
    debug_file<<"共"<<lines_3d.size()<<"条3d线段"<<std::endl;
    debug_file<<"共"<<lines_2d.size()<<"条2d线段"<<std::endl;
    debug_file<<"共"<<matches2d3d.size()<<"个匹配"<<std::endl;
    if(matches2d3d.size()==0) return;
    for(auto it = matches2d3d.begin();it!=matches2d3d.end();it++){
        const auto line3d_tmp = it->line3dt;
        const auto line2d_tmp = it->line2dt;
        cv::Point pt1,pt2,pt3,pt4;
        V3D pt_w1(line3d_tmp.ptstart.x(),line3d_tmp.ptstart.y(),line3d_tmp.ptstart.z());
        V3D pt_w2(line3d_tmp.ptend.x(),line3d_tmp.ptend.y(),line3d_tmp.ptend.z());
        V3D pf1 = Rcw * pt_w1 + Pcw;
        V3D pf2 = Rcw * pt_w2 + Pcw;
        V2D px1 = cam->world2cam(pf1);    // 像素坐标
        V2D px2 = cam->world2cam(pf2);    // 像素坐标
        pt1.x = px1.x(); pt1.y = px1.y(); pt2.x = px1.x(); pt2.y = px2.y();
        cv::line(img_cp, pt1, pt2, cv::Scalar(255, 0, 0), 3, cv::LINE_4);
        pt3.x = line2d_tmp.ptstart.x();pt3.y = line2d_tmp.ptstart.y();
        pt4.x = line2d_tmp.ptend.x();pt4.y = line2d_tmp.ptend.y();
        cv::line(img_cp, pt3, pt4, cv::Scalar(0, 255, 0), 2, cv::LINE_4);
        auto d13x = (pt1-pt3).x;auto d13y = (pt1-pt3).y;
        double dista= d13x*d13x+d13y*d13y;
        auto d14x = (pt1-pt4).x;auto d14y = (pt1-pt4).y;
        double distb= d14x*d14x+d14y*d14y;
        if(dista<distb){
            cv::line(img_cp, pt1, pt3, cv::Scalar(0, 0, 0), 1, cv::LINE_4);
            cv::line(img_cp, pt2, pt4, cv::Scalar(0, 0, 0), 1, cv::LINE_4);
        }
        else{
            cv::line(img_cp, pt1, pt4, cv::Scalar(0, 0, 0), 1, cv::LINE_4);
            cv::line(img_cp, pt2, pt3, cv::Scalar(0, 0, 0), 1, cv::LINE_4);
        }

        //debug_file<<"第"<<it->index<<"对: "<<it->distance.transpose()<<std::endl;
    }
        StatesGroup * state_before = new StatesGroup(*state);
        iterate_num_L = 0;
        double last_error=100000;

        for(int i=0;i<max_iteration;i++){
            auto old_state = *state;
            auto valid = CalculateJLandResL();
            if(!valid){
                debug_file<<"无线段匹配"<<std::endl;
                continue;
            }
            //debug_file<<"JL_sub: "<<JL_sub<<std::endl;
            //debug_file<<"resL_sub: "<<resL_sub.transpose()<<std::endl;
            const int eff_num = resL_sub.size();
            Eigen::MatrixXd HL_sub = MatrixXd::Zero(eff_num, DIM_STATE);
            for(int l=0;l<6;l++){
                HL_sub.col(l) = JL_sub.col(l);
            }
            
            //debug_file<<"H_sub: "<<H_sub<<std::endl; 
            auto HL_sub_T = HL_sub.transpose();    
            HL_T_HL = HL_sub_T * HL_sub;
            
            
            // debug_file<<"IMG_COV: "<<IMG_COV<<std::endl;
            MD(DIM_STATE, DIM_STATE) &&KL_1 = (HL_T_HL + (state->cov / LINE_COV).inverse()).inverse();
            //debug_file<<"H_T_H: "<<H_T_H<<std::endl;
            //debug_file<<"state->cov: "<<state->cov<<std::endl;
            // debug_file<<"K_1: "<<K_1<<std::endl;
            auto &&HLTz = HL_sub_T * resL_sub;
            // K = K_1.block<DIM_STATE,6>(0,0) * H_sub_T;
            auto vecL = (*state_before) - (*state);
            GL = KL_1 * HL_T_HL;
            auto solutionL = - KL_1 * HLTz + vecL - GL * vecL;
            // debug_file<<"vecL"<<vecL<<std::endl;
            // debug_file<<"KL_1: "<<KL_1<<std::endl;
            // debug_file<<"HLTz: "<<HLTz<<std::endl;
            // debug_file<<"GL: "<<GL<<std::endl;
            // debug_file<<"- KL_1 * HLTz: "<<(- KL_1 * HLTz).transpose()<<std::endl;
            // debug_file<<"solutionL: "<<solutionL.transpose()<<std::endl;
            (*state) += solutionL;
            auto &&rotL_add = solutionL.block<3,1>(3,0);
            auto &&tL_add   = solutionL.block<3,1>(0,0);
            
			int k = 0;
            double meansL = 0;
			for(int j=0;j<resL_sub.rows();j++){
				k++;
				meansL+=fabs(resL_sub(j));
			}
			meansL = meansL/k;
			debug_file<<"第"<<iterate_num_L<<"次线约束迭代的平均error为："<<meansL<<std::endl;
            
            if(meansL>last_error){
                // 损失增大，回退
                *state = old_state; 
                GL = lastGL;
            }
            else 
            if(state->pos_end.hasNaN()==true){
                *state = old_state;
            }
            else{
            // 成功了
                last_error = meansL; 
                lastGL = GL;
            }   
            if ((rotL_add.norm() * 57.3f < 0.001f) && (tL_add.norm() * 100.0f < 0.001f))
            {
                debug_file<<"迭代"<<iterate_num_L<<"次收敛"<<std::endl;
                break;
            }
            iterate_num_L++;
        }
        state->cov -= GL*state->cov;
        Eigen::Quaterniond rotation(state->rot_end);
        rotation.normalize();
        state->rot_end = rotation.toRotationMatrix();
        updateFrameState();
        setKF(kf);
        Updatekf(kf);
        UpdateCamPose();
}
void violm::projectionLine(){
    lines_3d.clear();
    for(auto it = lines.begin();it!=lines.end();it++){
        V3D pt_w1(it->at(0).x,it->at(0).y,it->at(0).z);
        V3D pt_w2(it->at(1).x,it->at(1).y,it->at(1).z);
        V3D pf1 = Rcw * pt_w1 + Pcw;
        V3D pf2 = Rcw * pt_w2 + Pcw;
        V2D px1 = cam->world2cam(pf1);    // 像素坐标
        V2D px2 = cam->world2cam(pf2);    // 像素坐标
        if(pf1[2] > blind&&pf2[2] > blind){    // 在相机前面,不要太近的点
            auto dis = (px1-px2).norm();
            if(dis<linethre3d_proj) continue; // 不要太短的点


            if(new_frame->cam_->isInFrame(px1.cast<int>(), (patch_size_half+1)*8)
            &&new_frame->cam_->isInFrame(px2.cast<int>(), (patch_size_half+1)*8)){

                cv::Point pt1,pt2;
                pt1.x =  px1[0]; pt1.y = px1[1];
                pt2.x = px2[0]; pt2.y = px2[1];
                // bool depth_continous1 = depthContinue(px1,pf1[2],pt_w1);
                // bool depth_continous2 = depthContinue(px2,pf2[2],pt_w2);
                // if(depth_continous1||depth_continous2) continue;
                //cv::line(img_cp, pt1, pt2, cv::Scalar(255, 255, 0), 1, cv::LINE_4);
                // 将这个3D线段加入lines_3d
                line3d line_3d_temp(pt_w1,pt_w2);
                lines_3d.push_back(line_3d_temp);

            }

        }
    }
}
bool violm::CalculateJLandResL(){
    debug_file<<"第"<<frame_nums<<"帧的第"<<iterate_num_L<<"次线特征迭代"<<std::endl;
    const int H_DIM = 2*matches2d3d.size();  // 观测维度=匹配个数*2
    Rwi = state->rot_end;                //debug_file<<"state->rot_end: "<<state->rot_end<<std::endl;
    Pwi = state->pos_end;               //debug_file<<"state->pos_end: "<<state->pos_end.transpose()<<std::endl;
    Rcw = Rci * Rwi.transpose();    //debug_file<<"state->vel_end: "<<state->vel_end.transpose()<<std::endl;
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;   //debug_file<<"state->bias_a: "<<state->bias_a.transpose()<<std::endl;
    JL.resize(H_DIM,6);  //debug_file<<"state->gravity: "<<state->gravity.transpose()<<std::endl;
    resL.resize(H_DIM,1);
    JL.setZero();
    resL.setZero();
    std::vector<int> index_table;
    //std::cout<<"grid大小："<<length<<std::endl;
    // 遍历grid
    int kk = -2;
    for(auto it = matches2d3d.begin();it!=matches2d3d.end();it++){
        kk = kk+2;
        int num = 0;
        const auto line3d_tmp = it->line3dt;
        const auto line2d_tmp = it->line2dt;

            V3D pt_w1 = line3d_tmp.ptstart; // 空间点坐标
            V3D pf1 = Rcw * pt_w1 + Pcw;  // 相机坐标系坐标
            V2D pc1 = cam->world2cam(pf1);    // 像素坐标
            
            V3D pt_w2 = line3d_tmp.ptend; // 空间点坐标
            V3D pf2 = Rcw * pt_w2 + Pcw;  // 相机坐标系坐标
            V2D pc2 = cam->world2cam(pf2);    // 像素坐标

            double res_i_1 = it->line2dt.A/it->line2dt.A2B2*pc1.x()+it->line2dt.B/it->line2dt.A2B2*pc1.y()+it->line2dt.C/it->line2dt.A2B2;
            double res_i_2 = it->line2dt.A/it->line2dt.A2B2*pc2.x()+it->line2dt.B/it->line2dt.A2B2*pc2.y()+it->line2dt.C/it->line2dt.A2B2;
            int row_index = kk;
            resL(row_index) = res_i_1; 
            resL(row_index+1) = res_i_2;

            MD(1,2) JdDdu1, JdDdu2;
            MD(2,3) Jdpi1, Jdpi2;
            MD(1,3) Jdphi1, Jdp1, JdR1, Jdt1, Jdphi2, Jdp2, JdR2, Jdt2;    
            M3D p_hat1, p_hat2;
            Jdp_dt = Rci * Rwi.transpose(); // 
            dpi(pf1, Jdpi1);// Jdpi就是偏u/偏q，即投影方程关于相机坐标系下三维点的导数
            dpi(pf2, Jdpi2);// Jdpi就是偏u/偏q，即投影方程关于相机坐标系下三维点的导数
            p_hat1 << SKEW_SYM_MATRX(pf1);
            p_hat2 << SKEW_SYM_MATRX(pf2);
            JdDdu1<< it->line2dt.A/it->line2dt.A2B2, it->line2dt.B/it->line2dt.A2B2; // Jimg就是偏I/偏u，也就是灰度的梯度，减去相邻的像素，使用双线性插值
            JdDdu2<< it->line2dt.A/it->line2dt.A2B2, it->line2dt.B/it->line2dt.A2B2; // Jimg就是偏I/偏u，也就是灰度的梯度，减去相邻的像素，使用双线性插值
            // debug_file<<"JdDdu1: "<<JdDdu1<<std::endl;
            // debug_file<<"JdDdu2: "<<JdDdu2<<std::endl;
            // if(res_i_1<0) JdDdu1 = -JdDdu1;
            // if(res_i_2<0) JdDdu2 = -JdDdu2;
            Jdphi1 = JdDdu1 * Jdpi1 * p_hat1;    // 对旋转的雅可比
            Jdphi2 = JdDdu2 * Jdpi2 * p_hat2;    // 对旋转的雅可比
            Jdp1 = -JdDdu1 * Jdpi1;                 // 对平移的雅可比
            Jdp2 = -JdDdu2 * Jdpi2;                 // 对平移的雅可比
            // debug_file<<"Jdp1: "<<Jdp1<<std::endl;
            // debug_file<<"Jdp2: "<<Jdp2<<std::endl;
            JdR1 = Jdphi1 * Jdphi_dR + Jdp1 * Jdp_dR;
            JdR2 = Jdphi2 * Jdphi_dR + Jdp2 * Jdp_dR;
            Jdt1 = Jdp1 * Jdp_dt;
            Jdt2 = Jdp2 * Jdp_dt;
            // debug_file<<"Jdt1: "<<Jdt1<<std::endl;
            // debug_file<<"Jdt2: "<<Jdt2<<std::endl;

            JL.block<1,6>(row_index,0)<< Jdt1, JdR1;
            JL.block<1,6>(row_index+1,0)<< Jdt2, JdR2;
            index_table.push_back(row_index);
            index_table.push_back(row_index+1);
    }

    if(index_table.size()==0) return false;
    JL_sub = MatrixXd::Zero(index_table.size(),6);
    resL_sub = VectorXd::Zero(index_table.size());
    for(int i=0;i<index_table.size();i++){
        JL_sub.block<1,6>(i,0) =  JL.block<1,6>(index_table[i],0);
        resL_sub.block<1,1>(i,0) =  resL.block<1,1>(index_table[i],0);
    }
    return true;
}
}