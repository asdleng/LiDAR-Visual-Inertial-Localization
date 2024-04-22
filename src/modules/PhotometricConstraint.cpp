#include "vio_in_lidar_map.h"
namespace lvo{
/*光度约束相关内容*/
void violm::photometricConstraint(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    setStatePropagate(kf);
    *state = *state_propagat;
    auto origin_cov = state->cov;
    last_error=10000;
    last_pyr_error = 10000;
    memset(outlier_map,0,sizeof(int)*length*grid_pts_num*max_ftr_list_num);
    bool valid_pyr = true;
    for(int j = pyr-1;j>=0;j--){
        
        iterate_num = 0;
        int wrong_time = 0;
        int error_time = 0;
        debug_file<<"第"<<j<<"层金字塔"<<std::endl;
        auto last_pyr_state = *state;
        auto last_pyr_G = G;
        for(int i=0;i<max_iteration;i++){
            auto old_state = *state;
            auto valid = CalculateJandRes(j);
            if(!valid){
                debug_file<<"无参考点"<<std::endl;
                continue;
            }
            //debug_file<<"fuck2"<<std::endl;
            //debug_file<<"J_sub: "<<J_sub<<std::endl;
            const int eff_num = res_sub.size();
            Eigen::MatrixXd H_sub = MatrixXd::Zero(eff_num, DIM_STATE);
            for(int l=0;l<6;l++){
                H_sub.col(l) = J_sub.col(l);
            }
            
            //debug_file<<"H_sub: "<<H_sub<<std::endl; 
            auto H_sub_T = H_sub.transpose();    
            H_T_H = H_sub_T * H_sub;
            
            
            // debug_file<<"IMG_COV: "<<IMG_COV<<std::endl;
            MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state->cov / IMG_COV).inverse()).inverse();
            //debug_file<<"H_T_H: "<<H_T_H<<std::endl;
            //debug_file<<"state->cov: "<<state->cov<<std::endl;
            // debug_file<<"K_1: "<<K_1<<std::endl;
            auto &&HTz = H_sub_T * res_sub;
            // K = K_1.block<DIM_STATE,6>(0,0) * H_sub_T;
            auto vec = (*state_propagat) - (*state);
            G = K_1 * H_T_H;
            auto solution = - K_1 * HTz + vec - G * vec;
            //debug_file<<"K_1: "<<K_1<<std::endl;
            // debug_file<<"HTz: "<<HTz.transpose()<<std::endl;
            // debug_file<<"G: "<<G<<std::endl;
            // debug_file<<"- K_1 * HTz: "<<(- K_1 * HTz).transpose()<<std::endl;
            // debug_file<<"solution: "<<solution.transpose()<<std::endl;
            (*state) += solution;
            auto &&rot_add = solution.block<3,1>(3,0);
            auto &&t_add   = solution.block<3,1>(0,0);
            
            int k = 0;
            double means = 0;
            for(int j=0;j<res_sub.rows();j++){
                k++;
                means+=fabs(res_sub(j));
            }
            means = means/k;
            debug_file<<"第"<<iterate_num<<"次迭代的平均error为："<<means<<std::endl;
            if(means>last_error){
            // 损失增大了，回退
                wrong_time++;
                *state = old_state; 
                G = lastG;
            }
            // else 
            // if(state->pos_end.hasNaN()==true){
            //     *state = old_state;
            // }
            else{
            // 成功了
                last_error = means; 
                lastG = G;
            }   
            // debug_file<<"deltax: "<< rot_add.norm()* 57.3f<<","<<t_add.norm() * 100.0f <<std::endl;
            // if((rot_add.norm() * 57.3f >0.2f) || (t_add.norm() * 100.0f >0.4f)){
            //     error_time++;
            // }
            // if(error_time>2){
            //     debug_file<<"发散，退出"<<std::endl;
            //     *state = old_state; 
            //     break;
            // }
            if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f) || wrong_time >2)
            {
                if(wrong_time>2){
                    debug_file<<"无法降低误差，退出"<<std::endl;
                }
                else{
                    debug_file<<"迭代"<<iterate_num<<"次收敛"<<std::endl;
                }
                break;
            }
            iterate_num++;
        }
        current_pyr_error = last_error;
        if(current_pyr_error>=last_pyr_error){
            debug_file<<"第"<<j<<"层金字塔无法降低误差,退出"<<std::endl;
            *state = last_pyr_state;
            lastG = last_pyr_G;
            break;
        }
        last_pyr_error = current_pyr_error;
    }
    state->cov -= lastG*state->cov;
    // check valid
    auto delta = *state - *state_propagat;
    //debug_file<<"delta: "<<delta.transpose()<<std::endl;
    // if(delta.block<3,1>(0,0).norm()>0.1||delta.block<3,1>(3,0).norm()>0.02){
    //     debug_file<<"Too big delta, the optimization maybe wrong!!!"<<std::endl;
    //     *state = *state_propagat;
    //     state->cov = origin_cov;
    //     updateFrameState();
    //     setKF(kf);
    //     delete new_frame;
    //     return;
    // }
    /*检查旋转正交性*/
    Eigen::Quaterniond rotation(state->rot_end);
    rotation.normalize();
    state->rot_end = rotation.toRotationMatrix();
    setKF(kf);
    updateFrameState();
    //std::cout<<"地图点个数为："<<map.map_points_.size()<<std::endl;
}
bool violm::CalculateJandRes(int level){
    
    //debug_file<<"第"<<iterate_num<<"次迭代"<<std::endl;

    const int H_DIM = length * patch_size_total*grid_pts_num*max_ftr_list_num;  // 有效网格数*每个patch的大小(16)
    Rwi = state->rot_end;                //debug_file<<"state->rot_end: "<<state->rot_end<<std::endl;
    Pwi = state->pos_end;               //debug_file<<"state->pos_end: "<<state->pos_end.transpose()<<std::endl;
    Rcw = Rci * Rwi.transpose();    //debug_file<<"state->vel_end: "<<state->vel_end.transpose()<<std::endl;
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;   //debug_file<<"state->bias_a: "<<state->bias_a.transpose()<<std::endl;
    // 遍历grid，求J和res
    J.resize(H_DIM,6);  //debug_file<<"state->gravity: "<<state->gravity.transpose()<<std::endl;
    res.resize(H_DIM,1);
    J.setZero();
    res.setZero();
    std::vector<int> index_table;
    //std::cout<<"grid大小："<<length<<std::endl;
    // 遍历grid
    int kk = 0;
    // debug_file<<"遍历grid"<<std::endl;
    for(int i=0;i<length;i++){
        if(grid.cells[i]->size()==0){
            continue;
        }
        int num = 0;
        // 遍历grid里面
        
        for(auto it_pv=grid.cells[i]->begin();it_pv!=grid.cells[i]->end();it_pv++,num++){
            V3D pt_w = it_pv->pt->pos_; // 空间点坐标
            V3D pf = Rcw * pt_w + Pcw;  // 相机坐标系坐标
            V2D pc = cam->world2cam(pf);    // 像素坐标
            //std::cout<<pc.transpose()<<"/"<<it_pv->px.transpose()<<std::endl;
            Matrix2d A_cur_ref_zero;
            
            // debug_file<<"第"<<frame_nums<<"帧的第"<<i<<"个网格点被"<<it_pv->pt->n_obs_<<"次观测"<<std::endl;
            // 遍历该点被多个关键帧看到
            // std::vector<FeaturePtr> ftr_list;
            FeaturePtr ref_ftr; // 定义一个特征点，用于返回关键帧
            // 看看这个3D点有没有在与当前帧观测60度以内的帧看到，
            if(it_pv->pt->getCloseViewObs(new_frame->pos(), ref_ftr, pc)){
                int k=0;
                //std::cout<<ftr_list.size()<<std::endl;
                // for(int k=0;k<ftr_list.size();k++){
                // ref_ftr = ftr_list[k];
                // cv::Point gird_point2(ref_ftr->px[0],ref_ftr->px[1]);
                // cv::circle(img_cp,gird_point2,4,cv::Scalar(255, 0, 0), -1, 8);
                patch_wrap = ref_ftr->patch;

                // getWarpMatrixAffine(*cam, ref_ftr->px, ref_ftr->f, (ref_ftr->pos() - pt_w).norm(), 
                //         new_frame->T_f_w_ * ref_ftr->T_f_w_.inverse(), 0, 0, patch_size_half, A_cur_ref_zero);
                // int search_level=0;
                // int pyramid_level = level;
                // debug_file<<"仿射矩阵为："<<A_cur_ref_zero<<std::endl;
                //std::cout<<"参考帧图像为："<<ref_ftr->img<<std::endl;
                //A_cur_ref_zero << 1,0,0,1;
                //warpAffine(A_cur_ref_zero, ref_ftr->frame->img(), ref_ftr->px, ref_ftr->level, search_level, pyramid_level, patch_size_half, patch_wrap);
                getpatch(img, pc, patch_cache, level);
                int pyr_bias = patch_size_total*level;

                // debug_file<<"patch_wrap: "<<setprecision(4)<<std::endl;
                // for(int ii=0;ii<patch_size*patch_size;ii++){
                //     debug_file<<patch_wrap[ii+pyr_bias]<<" ";
                // }
                // debug_file<<std::endl;
                
                // debug_file<<"patch_cache: "<<setprecision(4)<<std::endl;
                // for(int ii=0;ii<patch_size*patch_size;ii++){
                //     debug_file<<patch_cache[ii+pyr_bias]<<" ";
                // }
                // debug_file<<std::endl;
                kk++;
                double ncc;
                // 剔除外点
                if(iterate_num==0){
                    if(ncc_en){
                        // 使用NCC提出外点
                        ncc = NCC(patch_wrap+pyr_bias, patch_cache+pyr_bias, patch_size_total);
                    }
                    else{
                        // 算一下error
                        error = 0;  // 误差清零
                        for (int ind=0; ind<patch_size_total; ind++) 
                        {   // 算一下error
                            error += (patch_wrap[ind+pyr_bias]-patch_cache[ind+pyr_bias]) * (patch_wrap[ind+pyr_bias]-patch_cache[ind+pyr_bias]);
                            //std::cout<<"像素(参考帧/当前帧)为："<<patch_wrap[ind]<<"/"<<patch_cache[ind]<<std::endl;
                        }
                    }
                    if(ncc_en){
                        if(ncc < ncc_thre){
                            //outlier_map[kk] = 1;
                            it_pv->pt->is_outlier =  true;
                            continue;
                        }   // 第一次迭代时，计算出外点
                        // else{
                        //     it_pv->pt->is_outlier = false;
                        // }
                    }
                    else{
                        if(error > outlier_threshold*patch_size_total*(1<<level)){
                            //outlier_map[kk] = 1;
                            it_pv->pt->is_outlier =  true;
                            continue;
                        }   // 第一次迭代时，计算出外点
                        // else{
                        //     it_pv->pt->is_outlier = false;
                        // }
                    }

                    // if(need_keyframe){
                    //     std::tuple<Point*,int,int> frame_pair(it_pv->pt,ref_ftr->frame->id_,new_frame->id_);
                    //     //debug_file<<"构建匹配：第"<<ref_ftr->frame->id_<<"帧和第"<<new_frame->id_<<"帧"<<std::endl;
                    //     covisible_pair.push_back(frame_pair);
                    // }
                    // 第一次迭代时，记录下共视关键帧
                    //debug_file<<"第"<<frame_nums<<"帧的第"<<i<<"个cell的点被第"<<ref_ftr->frame->id_<<"帧看到"<<std::endl;
                    //debug_file<<"该点的error为"<<error<<std::endl;
                    cv::Point gird_point(pc[0],pc[1]);
                    cv::circle(img_cp,gird_point,4,cv::Scalar(0, 0, 255), -1, 8);   // 红色
                }
                if(it_pv->pt->is_outlier ==  true)
                    continue;
                // std::cout<<"当前帧"<<frame_nums<<"上该点的坐标为: "<<pc.transpose()<<std::endl;
                // std::cout<<"参考帧"<<ref_ftr->frame->id_<<"上该点的坐标为："<<ref_ftr->px.transpose()<<std::endl;  
                // std::cout<<"像素(参考帧/当前帧)为：";
                // for (int ind=0; ind<patch_size_total; ind++) 
                // {   
                //    //std::cout <<patch_wrap[ind]<<"/"<<patch_cache[ind]<<" ";
                // }
                //std::cout<<std::endl;
                //std::cout<<"error为："<<error<<std::endl;

                float* P = patch_wrap;

                // for (int ind=0; ind<patch_size_total; ind++) {   
                //     error += (patch_wrap[ind]-patch_cache[ind]) * (patch_wrap[ind]-patch_cache[ind]);
                // }
                // res[i] = error;
            
                // 算一下雅可比
                MD(1,2) Jimg;
                MD(2,3) Jdpi;
                MD(1,3) Jdphi, Jdp, JdR, Jdt;
                
                M3D p_hat;
                const float u_ref = pc[0];
                const float v_ref = pc[1];
                const int scale =  (1<<level);
                const int u_ref_i = floorf(pc[0]/scale)*scale; 
                const int v_ref_i = floorf(pc[1]/scale)*scale;
                const float subpix_u_ref = (u_ref-u_ref_i)/scale;
                const float subpix_v_ref = (v_ref-v_ref_i)/scale;
                const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
                const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
                const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
                const float w_ref_br = subpix_u_ref * subpix_v_ref;
                    
                Jdp_dt = Rci * Rwi.transpose(); // 
                dpi(pf, Jdpi);// Jdpi就是偏u/偏q，即投影方程关于相机坐标系下三维点的导数
                p_hat << SKEW_SYM_MATRX(pf);
                double patch_error = 0.0;
                //std::cout<<"2.1.3"<<std::endl;
                // 遍历patch，scale个像素作为步长
                for (int x=0; x<patch_size; x++) {
                        uint8_t* img_ptr = (uint8_t*) img.data + (v_ref_i+x*scale-patch_size_half*scale)*width + u_ref_i-patch_size_half*scale;
                        for (int y=0; y<patch_size; ++y, img_ptr+=scale) {
                            // if((level==2 && iteration==0) || (level==1 && iteration==0) || level==0)
                            //{
                            float du = 0.5f * ((w_ref_tl*img_ptr[scale] + w_ref_tr*img_ptr[scale*2] + w_ref_bl*img_ptr[scale*width+scale] + w_ref_br*img_ptr[scale*width+scale*2])
                                        -(w_ref_tl*img_ptr[-scale] + w_ref_tr*img_ptr[0] + w_ref_bl*img_ptr[scale*width-scale] + w_ref_br*img_ptr[scale*width]));
                            float dv = 0.5f * ((w_ref_tl*img_ptr[scale*width] + w_ref_tr*img_ptr[scale+scale*width] + w_ref_bl*img_ptr[width*scale*2] + w_ref_br*img_ptr[width*scale*2+scale])
                                        -(w_ref_tl*img_ptr[-scale*width] + w_ref_tr*img_ptr[-scale*width+scale] + w_ref_bl*img_ptr[0] + w_ref_br*img_ptr[scale]));
                            Jimg << du, dv; // Jimg就是偏I/偏u，也就是灰度的梯度，减去相邻的像素，使用双线性插值
                            Jimg = Jimg * (1.0/scale);
                            Jdphi = Jimg * Jdpi * p_hat;    // 对旋转的雅可比
                            Jdp = -Jimg * Jdpi;                 // 对平移的雅可比
                            JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
                            Jdt = Jdp * Jdp_dt;
                            //}
                            // debug_file<<"Jdt: "<<Jdt<<std::endl;
                            double res_i = w_ref_tl*img_ptr[0] + w_ref_tr*img_ptr[scale] + w_ref_bl*img_ptr[scale*width] + w_ref_br*img_ptr[scale*width+scale]  - P[patch_size_total*level + x*patch_size+y];
                            
                            int row_index = i*patch_size_total*grid_pts_num*max_ftr_list_num+
                            num*patch_size_total*max_ftr_list_num+
                            k*patch_size_total+
                            x*patch_size+
                            y;
                            
                            
                            res(row_index) = res_i; // 投影点减去wrap过来的参考关键帧
                            
                            // float weight = 1.0;
                            // if(iteration > 0)
                            //     weight = weight_function_->value(res/weight_scale_); 
                            // R(i*patch_size_total+x*patch_size+y) = weight;       
                            patch_error +=  res_i*res_i;
                            
                            // H.block<1,6>(i*patch_size_total+x*patch_size+y,0) << JdR*weight, Jdt*weight;
                            // if((level==2 && iteration==0) || (level==1 && iteration==0) || level==0)
                            //H_sub.block<1,6>(i*patch_size_total+x*patch_size+y,0) << JdR, Jdt;  // 3列对旋转，3列对平移
                            // debug_file<<"第"<<index_table.size()<<"个点"<<std::endl;
                            // debug_file<<"Jimg: "<<setprecision(8)<<Jimg<<std::endl;
                            // debug_file<<"Jdpi: "<<setprecision(8)<<Jdpi<<std::endl;
                            J.block<1,6>(row_index,0)<< Jdt, JdR;
                            index_table.push_back(row_index);
                            //std::cout<<"res"<<index_table.size()<<": "<<res.row(i*patch_size_total+x*patch_size+y)<<std::endl;
                            //std::cout<<"J "<<index_table.size()<<": "<<J.row(i*patch_size_total+x*patch_size+y)<<std::endl;
                            //std::cout<<fixed<<setprecision(3)<<"J[i]:" <<JdR<< Jdt<<std::endl;
                        }
                    }   // patch
                    //std::cout<<"第"<<frame_nums<<"帧的第"<<i<<"个cell点的res为"<<res(index_table.back())<<std::endl;
                }
                
            }
    }   // 遍历完grid
    //debug_file<<"遍历完grid"<<std::endl;
    if(index_table.size()==0) return false;
    J_sub = MatrixXd::Zero(index_table.size(),6);
    res_sub = VectorXd::Zero(index_table.size());
    for(int i=0;i<index_table.size();i++){
        J_sub.block<1,6>(i,0) =  J.block<1,6>(index_table[i],0);
        //std::cout<<"J "<<i<<": "<<J_sub.block<1,6>(i,0)<<std::endl;
        res_sub.block<1,1>(i,0) =  res.block<1,1>(index_table[i],0);
        //std::cout<<"res "<<i<<": "<<res_sub.block<1,1>(i,0)<<std::endl;
    }
    
    return true;
}
}