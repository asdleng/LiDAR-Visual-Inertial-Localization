#include "lmlvil.h"
namespace lvo{
/*光度约束相关内容*/
void lmlvil::photometricConstraint(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    setStatePropagate(kf);
    *state = *state_propagat;
    auto origin_cov = state->cov;
    last_error = 10000;
    last_pyr_error = 10000;
    memset(outlier_map,0,sizeof(int)*length*grid_pts_num*max_ftr_list_num);
    bool valid_pyr = true;
    double t=0;
    MatrixXd J_sub;
    VectorXd res_sub;
    const int H_DIM = length * patch_size_total*grid_pts_num*max_ftr_list_num;
    for(int j = pyr-1; j >= low_pyr; j--){
        iterate_num = 0;
        int wrong_time = 0;
        int error_time = 0;
        auto last_pyr_state = *state;
        auto last_pyr_G = G;
        for(int i = 0; i < max_iteration; i++){
            auto old_state = *state;
            J_sub.resize(H_DIM,6);
            res_sub.resize(H_DIM,1);
            auto t1 = omp_get_wtime();
            auto valid = CalculateJandRes(j,J_sub,res_sub);
            auto t2 = omp_get_wtime();
            t = t + (t2 - t1);
            if(!valid){
                continue;
            }
            const int eff_num = res_sub.size();
            //debug_file<<"观测维度："<<eff_num<<std::endl;
            Eigen::MatrixXd H_sub = MatrixXd::Zero(eff_num, DIM_STATE);
            for(int l = 0; l < 6; l++){
                H_sub.col(l) = J_sub.col(l);
            }
            auto H_sub_T = H_sub.transpose();
            H_T_H = H_sub_T * H_sub;
            MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state->cov / IMG_COV).inverse()).inverse();
            auto &&HTz = H_sub_T * res_sub;
            auto vec = (*state_propagat) - (*state);
            G.noalias() = K_1 * H_T_H;
            auto solution = - K_1 * HTz + vec - G * vec;
            (*state) += solution;
            auto &&rot_add = solution.block<3,1>(3,0);
            auto &&t_add   = solution.block<3,1>(0,0);
            int k = 0;
            double means = 0;
            for(int j = 0; j < res_sub.rows(); j++){
                k++;
                means += fabs(res_sub(j));
            }
            means = means / k;
            //debug_file<<"误差为："<<means<<std::endl;
            if(means > last_error){
                wrong_time++;
                *state = old_state;
                G = lastG;
            }
            else{
                last_error = means; 
                lastG = G;
            }
            if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f) || wrong_time >2){
                break;
            }
            iterate_num++;
        }
        current_pyr_error = last_error;
        if(current_pyr_error >= last_pyr_error){
            *state = last_pyr_state;
            lastG = last_pyr_G;
            break;
        }
        last_pyr_error = current_pyr_error;
    }
    state->cov -= lastG * state->cov;
    auto delta = *state - *state_propagat;
    Eigen::Quaterniond rotation(state->rot_end);
    rotation.normalize();
    state->rot_end = rotation.toRotationMatrix();
    setKF(kf);
    updateFrameState();
    debug_file << "===计算雅可比耗时" << t << std::endl;
}



bool lmlvil::CalculateJandRes(int level,MatrixXd& J_sub,VectorXd& res_sub){
    Rwi = state->rot_end;
    Pwi = state->pos_end;
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;

    J_sub.setZero();
    res_sub.setZero();
    float* patch_cache = new float[patch_size_total*pyr];
    int current_row = 0;
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
                
                getpatch(img, pc, patch_cache, level);
                int pyr_bias = patch_size_total*level;
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
                            //#pragma omp critical
                            {
                                it_pv->pt->is_outlier =  true;
                            }
                            continue;
                        }
                        else{
                            it_pv->pt->is_outlier =  false;
                        }
                    }
                    else{
                        if(error > outlier_threshold*patch_size_total*(1<<level)){
                            //#pragma omp critical
                            {
                                it_pv->pt->is_outlier =  true;
                            }
                            continue;
                        }
                        else{
                            it_pv->pt->is_outlier =  false;
                        }
                    }
                    cv::Point gird_point(pc[0],pc[1]);
                    //debug_file<<it_pv->pt->is_edge<<std::endl;
                    if(!it_pv->pt->is_edge){
                        cv::circle(img_cp,gird_point,4,cv::Scalar(0, 0, 255), -1, 8);
                    }
                    else{
                        auto p = it_pv->pt;
                        cv::Point start = gird_point - cv::Point(p->dir.y() * 5, p->dir.x() * 5);
                        cv::Point end = gird_point + cv::Point(p->dir.y() * 5, p->dir.x() * 5);
                        cv::line(img_cp, start, end, cv::Scalar(255, 255, 0), 2);
                        continue;
                    }
                    
                }
                

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
                        //#pragma omp critical
                        {   
                            current_row++;
                            res_sub(current_row) = res_i;
                            J_sub.block<1,6>(current_row,0)<< Jdt, JdR;
                        }
                    }
                }
            }
        }
    }
    delete[] patch_cache;
    if(current_row==0) return false;

    J_sub.conservativeResize(current_row, Eigen::NoChange);
    res_sub.conservativeResize(current_row, Eigen::NoChange);
    return true;
}

}