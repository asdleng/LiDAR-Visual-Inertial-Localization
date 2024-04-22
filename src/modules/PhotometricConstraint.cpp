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
    double t=0;
    for(int j = pyr-1;j>=0;j--){
        
        iterate_num = 0;
        int wrong_time = 0;
        int error_time = 0;
        debug_file<<"第"<<j<<"层金字塔"<<std::endl;
        auto last_pyr_state = *state;
        auto last_pyr_G = G;
        for(int i=0;i<max_iteration;i++){
            auto old_state = *state;
            auto t1 = omp_get_wtime();
            auto valid = CalculateJandRes(j);
            auto t2 = omp_get_wtime();
            t = t+(t2-t1);
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
    debug_file<<"===计算雅可比耗时"<<t<<std::endl;
}


bool violm::CalculateJandRes(int level){

    const int H_DIM = length * patch_size_total*grid_pts_num*max_ftr_list_num;
    Rwi = state->rot_end;
    Pwi = state->pos_end;
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;
    MatrixXd J;
    VectorXd res;
    J.resize(H_DIM,6);
    res.resize(H_DIM,1);
    J.setZero();
    res.setZero();
    std::vector<int> index_table;
    int kk = 0;

    
    //#pragma omp parallel for
    for(int i=0;i<length;i++){
        if(grid.cells[i]->size()==0){
            continue;
        }
        int num = 0;
        for(auto it_pv=grid.cells[i]->begin();it_pv!=grid.cells[i]->end();it_pv++,num++){
            V3D pt_w = it_pv->pt->pos_;
            V3D pf = Rcw * pt_w + Pcw;
            V2D pc = cam->world2cam(pf);
            Matrix2d A_cur_ref_zero;
            FeaturePtr ref_ftr;
            if(it_pv->pt->getCloseViewObs(new_frame->pos(), ref_ftr, pc)){

                int k=0;

                auto patch_wrap = ref_ftr->patch;
                float* patch_cache = new float[patch_size_total*pyr];
                getpatch(img, pc, patch_cache, level);
                int pyr_bias = patch_size_total*level;
                kk++;
                double ncc;
                if(iterate_num==0){
                    if(ncc_en){
                        ncc = NCC(patch_wrap+pyr_bias, patch_cache+pyr_bias, patch_size_total);
                    }
                    else{
                        error = 0;
                        for (int ind=0; ind<patch_size_total; ind++){
                            error += (patch_wrap[ind+pyr_bias]-patch_cache[ind+pyr_bias]) * (patch_wrap[ind+pyr_bias]-patch_cache[ind+pyr_bias]);
                        }
                    }
                    if(ncc_en){
                        if(ncc < ncc_thre){
                            #pragma omp critical
                            {
                                it_pv->pt->is_outlier =  true;
                            }
                            continue;
                        }
                    }
                    else{
                        if(error > outlier_threshold*patch_size_total*(1<<level)){
                            #pragma omp critical
                            {
                                it_pv->pt->is_outlier =  true;
                            }
                            continue;
                        }
                    }
                    cv::Point gird_point(pc[0],pc[1]);
                    cv::circle(img_cp,gird_point,4,cv::Scalar(0, 0, 255), -1, 8);

                }
                delete[] patch_cache;

                if(it_pv->pt->is_outlier == true)
                    continue;
                
                float* P = patch_wrap;
                MD(1,2) Jimg;
                MD(2,3) Jdpi;
                MD(1,3) Jdphi, Jdp, JdR, Jdt;
                
                M3D p_hat;
                M3D Jdp_dt;
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
                    
                Jdp_dt = Rci * Rwi.transpose();
                dpi(pf, Jdpi);
                p_hat << SKEW_SYM_MATRX(pf);
                double patch_error = 0.0;
                for (int x=0; x<patch_size; x++) {
                    uint8_t* img_ptr = (uint8_t*) img.data + (v_ref_i+x*scale-patch_size_half*scale)*width + u_ref_i-patch_size_half*scale;
                    for (int y=0; y<patch_size; ++y, img_ptr+=scale) {
                        float du = 0.5f * ((w_ref_tl*img_ptr[scale] + w_ref_tr*img_ptr[scale*2] + w_ref_bl*img_ptr[scale*width+scale] + w_ref_br*img_ptr[scale*width+scale*2])
                                    -(w_ref_tl*img_ptr[-scale] + w_ref_tr*img_ptr[0] + w_ref_bl*img_ptr[scale*width-scale] + w_ref_br*img_ptr[scale*width]));
                        float dv = 0.5f * ((w_ref_tl*img_ptr[scale*width] + w_ref_tr*img_ptr[scale+scale*width] + w_ref_bl*img_ptr[width*scale*2] + w_ref_br*img_ptr[width*scale*2+scale])
                                    -(w_ref_tl*img_ptr[-scale*width] + w_ref_tr*img_ptr[-scale*width+scale] + w_ref_bl*img_ptr[0] + w_ref_br*img_ptr[scale]));
                        Jimg << du, dv;
                        Jimg = Jimg * (1.0/scale);
                        Jdphi = Jimg * Jdpi * p_hat;
                        Jdp = -Jimg * Jdpi;
                        JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
                        Jdt = Jdp * Jdp_dt;
                        double res_i = w_ref_tl*img_ptr[0] + w_ref_tr*img_ptr[scale] + w_ref_bl*img_ptr[scale*width] + w_ref_br*img_ptr[scale*width+scale]  - P[patch_size_total*level + x*patch_size+y];
                        int row_index = i*patch_size_total*grid_pts_num*max_ftr_list_num+
                        num*patch_size_total*max_ftr_list_num+
                        k*patch_size_total+
                        x*patch_size+
                        y;
                        patch_error +=  res_i*res_i;
                        #pragma omp critical
                        {
                            res(row_index) = res_i;
                            J.block<1,6>(row_index,0)<< Jdt, JdR;
                            index_table.push_back(row_index);
                        }
                    }
                }
            }
                
        }
    }

    if(index_table.size()==0) return false;
    J_sub = MatrixXd::Zero(index_table.size(),6);
    res_sub = VectorXd::Zero(index_table.size());
    for(int i=0;i<index_table.size();i++){
        J_sub.block<1,6>(i,0) =  J.block<1,6>(index_table[i],0);
        res_sub.block<1,1>(i,0) =  res.block<1,1>(index_table[i],0);
    }
    
    return true;
}

}