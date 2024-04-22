/*
 * @Author: asdleng lengjianghao2006@163.com
 * @Date: 2023-02-25 11:25:04
 * @LastEditors: asdleng lengjianghao2006@163.com
 * @LastEditTime: 2023-10-02 16:41:52
 * @FilePath: /vio_in_lidar_map/src/vio_in_lidar_map/src/vio_in_lidar_map.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include "vio_in_lidar_map.h"

namespace lvo{
void violm::set_camera2lidar(vector<double>& R,  vector<double>& P )
{
        Rcl << MAT_FROM_ARRAY(R);
        Pcl << VEC_FROM_ARRAY(P);
}
void violm::set_extrinsic(const V3D &transl, const M3D &rot)
{
    Ril = rot;
    Pil = transl;
    Pli = -rot.transpose() * transl;
    Rli = rot.transpose();
}
int violm::obs_points(){
    int k=0;
    for(auto it = map.map_points_.begin();it<map.map_points_.end();it++){
        if(it->get()->n_obs_>0){
            k++;
        }
    }
    return k;
}
void violm::Init(PointCloudXYZI::Ptr ptr){
    res_last = new float[100000]();
    point_selected_surf = new bool[100000]();
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    state = new StatesGroup;
    state_propagat = new StatesGroup;
    debug_file.open(FILE_DIR("debug_J.txt"), ios::out);

    
    if(read_enable) lines_file_i.open(FILE_DIR(lines_file),ios::in);
    else lines_file_w.open(FILE_DIR(lines_file),ios::out);
    
    Rci = Rcl * Rli;
    Pci= Rcl*Pli + Pcl; // 求出camera到imu的外参
    debug_file<<"Ric: "<<Rci.transpose()<<std::endl;
    
    Jdphi_dR = Rci; // imu到cam
    V3D Pic = -Rci.transpose() * Pci;
    debug_file<<"Pic: "<<Pic.transpose()<<std::endl;
    M3D tmp;
    tmp << SKEW_SYM_MATRX(Pic);
    Jdp_dR = -Rci * tmp;
    downSizeFilter.setLeafSize(filter_size, filter_size, filter_size);
    first_frame = true;
    height = cam->height();
    width = cam->width();
    grid_n_width = static_cast<int>(width/grid_size);   // grid的列数
    grid_n_height = static_cast<int>(height/grid_size); // grid的行数
    length = grid_n_width * grid_n_height;  // grid的个数
    depth_img = new float[height*width];
    patch_wrap = new float[patch_size_total*pyr];
    // para_pose = new double*[window_size];
    // for(int i=0;i<window_size;i++){
    //     para_pose[i] = new double[7];
    // }
    adj_pts.resize(width*height);
    grid.cells.resize(length);
    std::for_each(grid.cells.begin(), grid.cells.end(), [&](Cell*& c){ c = new Cell; });
    patch_size_total = patch_size * patch_size;
    patch_size_half = static_cast<int>(patch_size/2);
    patch_cache = new float[patch_size_total*pyr];
    pts_num_in_each_cell = new int[length];
    memset(pts_num_in_each_cell, 0, sizeof(int)*length);
    outlier_map = new int[length*grid_pts_num*max_ftr_list_num];
    fx = cam->errorMultiplier2();
    fy = cam->errorMultiplier() / (4. * fx);
    pin_cam = (vk::PinholeCamera*) cam;
    if(left_coord){
        std::cout<<"左手系，转化地图中"<<std::endl;
        for(auto it=ptr->begin();it!=ptr->end();it++){
            it->y = -it->y;
        }
    }
    if(map_is_based_on_LiDAR){
        std::cout<<"基于LiDAR构建的地图，转化地图中"<<std::endl;
        for(auto it=ptr->begin();it!=ptr->end();it++){
            V3D pos(it->x,it->y,it->z);
            V3D pos_g = Ril*pos + Pil;
            it->x = pos_g.x();
            it->y = pos_g.y();
            it->z = pos_g.z();
        }
    }
    if(need_down_size){
        if(down_sample_manner == 0){
            downSizeFilter.setInputCloud(ptr);
            downSizeFilter.filter(*pcl_down);
        }
        else if(down_sample_manner == 1){
            for(int i=0;i<ptr->size();i = i+down_sample_num){
                pcl_down->push_back(ptr->at(i));
            }
        }
        std::cout<<"从"<<ptr->size()<<"个点降采样到"<<pcl_down->size()<<"个点"<<std::endl;
    }
    else{
        pcl_down = ptr;
    }


    
    std::cout<<"加载地图中"<<std::endl;
    if(cut_points_above_ground){
        auto it = pcl_down->begin();
        while(it!=pcl_down->end()){
            if(it->z<-1){
                it = pcl_down->erase(it);
            }
            else{
                it++;
            }
        }
    }
    ikdtree.Build(pcl_down->points);
    double t00 = omp_get_wtime();
    for(int i=0;i<pcl_down->size();i++){
        V3D pos(pcl_down->at(i).x,pcl_down->at(i).y,pcl_down->at(i).z);
        PointPtr  p(new Point(pos));
        p->id_ = i;
        map.addPoint(p);  
    }   // 放到map里
    double t01 = omp_get_wtime();

    std::cout<<"加载完成，加入map耗时"<<t01-t00<<"s"<<std::endl;
    if(read_enable){
        std::string s;
        std::cout<<"加载3D线段中"<<std::endl;
        while(getline(lines_file_i,s)){
            std::stringstream ss(s);
            std::vector<cv::Point3d> p1_p2;
            cv::Point3d p1,p2;
            ss>>p1.x>>p1.y>>p1.z>>p2.x>>p2.y>>p2.z;
            p1_p2.push_back(p1);
            p1_p2.push_back(p2);
            lines.push_back(p1_p2);
        }
        auto it = lines.begin();
        while(it!=lines.end()){
            auto delta = it->at(0)-it->at(1);
            double dis = delta.x*delta.x+delta.y*delta.y+delta.z*delta.z;
            dis = std::sqrt(dis);
            if(dis<linethre3d)  it = lines.erase(it);
            else  it++;
        }
        std::cout<<"加载完成"<<std::endl;
    }
    else{
        int k = 200;
        std::cout<<"计算3D直线中"<<std::endl;
        PointCloud<double> pointData; 
        std::vector<PLANE> planes;
        std::vector<double> ts;
        for(int i=0;i<ptr->size();i++){
            V3D pos(ptr->at(i).x,ptr->at(i).y,ptr->at(i).z);
            pointData.pts.push_back(PointCloud<double>::PtData(pos.x(),pos.y(),pos.z()));
        }   // 放到pointData里
        
        detector->run(pointData, k, planes, lines, ts);
        std::cout<<"lines number: "<<lines.size()<<std::endl;
        std::cout<<"planes number: "<<planes.size()<<std::endl;
        std::cout<<"计算完成"<<std::endl;
        
        auto it = lines.begin();
        while(it!=lines.end()){
            auto delta = it->at(0)-it->at(1);
            double dis = delta.x*delta.x+delta.y*delta.y+delta.z*delta.z;
            dis = std::sqrt(dis);
            if(dis<linethre3d)  lines.erase(it);
            else  it++;
        }
        for(int i=0;i<lines.size();i++){
            lines_file_w<<lines[i][0].x<<" "<<lines[i][0].y<<" "<<lines[i][0].z<<" "<<
            lines[i][1].x<<" "<<lines[i][1].y<<" "<<lines[i][1].z<<std::endl; 
        }
    }
    detector.reset();
}

void violm::reset_grid()
{
    //std::cout<<"fuck"<<std::endl;
    for(int i=0;i<grid.cells.size();i++){
        //std::cout<<i<<std::endl;
        if(grid.cells[i]->size()>0)
            grid.cells[i]->clear();
    }
    memset(pts_num_in_each_cell, 0, sizeof(int)*length);
}

void violm::dpi(V3D p, MD(2,3)& J) {
    const double x = p[0];
    const double y = p[1];
    const double z_inv = 1./p[2];   // 逆深度
    const double z_inv_2 = z_inv * z_inv;   // 逆深度平方
    J(0,0) = fx * z_inv;
    J(0,1) = 0.0;
    J(0,2) = -fx * x * z_inv_2;
    J(1,0) = 0.0;
    J(1,1) = fy * z_inv;
    J(1,2) = -fy * y * z_inv_2;
}
double violm::NCC(float* ref_patch, float* cur_patch, int patch_size)
{    
    double sum_ref = std::accumulate(ref_patch, ref_patch + patch_size, 0.0);
    double mean_ref =  sum_ref / patch_size;

    double sum_cur = std::accumulate(cur_patch, cur_patch + patch_size, 0.0);
    double mean_curr =  sum_cur / patch_size;

    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < patch_size; i++) 
    {
        double n = (ref_patch[i] - mean_ref) * (cur_patch[i] - mean_curr);
        numerator += n;
        demoniator1 += (ref_patch[i] - mean_ref) * (ref_patch[i] - mean_ref);
        demoniator2 += (cur_patch[i] - mean_curr) * (cur_patch[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);
}
// 获得仿射矩阵，从cur到ref的
void violm::getWarpMatrixAffine(
    const vk::AbstractCamera& cam,  // 相机模型
    const Vector2d& px_ref, // 参考帧的点的像素坐标//std::cout<<"位置为："<<t.transpose()<<std::endl;
        
    const Vector3d& f_ref,  // 参考真的点的归一化相机坐标
    const double depth_ref, // 参考帧的点的深度
    const Sophus::SE3& T_cur_ref,   // 参考帧到当前帧的变换
    const int level_ref,    // the corresponding pyrimid level of px_ref
    const int pyramid_level,
    const int halfpatch_size,
    Matrix2d& A_cur_ref)
{
  // Compute affine warp matrix A_ref_cur
  const Vector3d xyz_ref(f_ref*depth_ref);  // 空间点p
  Vector3d xyz_du_ref(cam.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)*(1<<pyramid_level)));
  Vector3d xyz_dv_ref(cam.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)*(1<<pyramid_level)));
//   Vector3d xyz_du_ref(cam.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)));
//   Vector3d xyz_dv_ref(cam.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)));
  xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];// p1
  xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];// p2
  const Vector2d px_cur(cam.world2cam(T_cur_ref*(xyz_ref)));//p'
  const Vector2d px_du(cam.world2cam(T_cur_ref*(xyz_du_ref)));//p1'
  const Vector2d px_dv(cam.world2cam(T_cur_ref*(xyz_dv_ref)));//p2'
  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
}
void violm::warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int search_level,
    const int pyramid_level,
    const int halfpatch_size,
    float* patch)
{
  const int patch_size = halfpatch_size*2;
  const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  if(isnan(A_ref_cur(0,0)))
  {
    printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
    return;
  }
//   Perform the warp on a larger patch.
//   float* patch_ptr = patch;
//   const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref) / (1<<pyramid_level);
//   const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
  for (int y=0; y<patch_size; ++y)
  {
    for (int x=0; x<patch_size; ++x)//, ++patch_ptr)
    {
      // P[patch_size_total*level + x*patch_size+y]
      Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      px_patch *= (1<<search_level);
      px_patch *= (1<<pyramid_level);
      const Vector2f px(A_ref_cur*px_patch + px_ref.cast<float>());
      if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
        patch[patch_size_total*pyramid_level + y*patch_size+x] = 0;
        // *patch_ptr = 0;
      else
        patch[patch_size_total*pyramid_level + y*patch_size+x] = (float) vk::interpolateMat_8u(img_ref, px[0], px[1]);
        // *patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);
    }
  }
}
void violm::getpatch(cv::Mat img, V2D pc, float* patch_tmp, int level) 
{
    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int scale =  (1<<level);  // 缩小多少倍
    const int u_ref_i = floorf(pc[0]/scale)*scale; 
    const int v_ref_i = floorf(pc[1]/scale)*scale;
    const float subpix_u_ref = (u_ref-u_ref_i)/scale;   // 小数像素
    const float subpix_v_ref = (v_ref-v_ref_i)/scale;
    const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref); // 左上
    const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);   // 右上
    const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;   // 左下
    const float w_ref_br = subpix_u_ref * subpix_v_ref; // 右下
    for (int x=0; x<patch_size; x++) 
    {   // patch 的位置
        uint8_t* img_ptr = (uint8_t*) img.data + (v_ref_i-patch_size_half*scale+x*scale)*width + (u_ref_i-patch_size_half*scale);
        for (int y=0; y<patch_size; y++, img_ptr+=scale)
        {   // patch 赋值
            patch_tmp[patch_size_total*level+x*patch_size+y] = w_ref_tl*img_ptr[0] + w_ref_tr*img_ptr[scale] + w_ref_bl*img_ptr[scale*width] + w_ref_br*img_ptr[scale*width+scale];
        }
    }
}
// 将地图点投影到图像上面
void violm::projection(){
    // if(stage==1){
    //     cv::Sobel(img, gradientX, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    //     cv::Sobel(img, gradientY, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
    //     // Compute the gradient magnitude
    //     cv::magnitude(gradientX, gradientY, gradientMagnitude);
    // }
    observed_points.clear();
    memset(depth_img,0,sizeof(float)*height*width);  // 深度
    
    double t_1 = omp_get_wtime();
    for(auto it = map.map_points_.begin();it!=map.map_points_.end();it++){

        auto it_ptr = (*it);
        auto t1 = omp_get_wtime();
        // if(need_keyframe){
        //     if(it_ptr->n_obs_>0){
        //         auto it_obs=it_ptr->obs_.begin();
        //         while(it_obs!=it_ptr->obs_.end()){
        //             if(remove_frame_num == it_obs->get()->frame->id_){
        //                 it_obs = it_ptr->obs_.erase(it_obs);
        //                 it_ptr->n_obs_--;
        //                 break;
        //             }
        //             else{
        //                 it_obs++;
        //             }
        //         }
        //     }
        // }
        auto t2 = omp_get_wtime();
        V3D pt_w = it_ptr->pos_; // 空间点坐标
        V3D pf = Rcw * pt_w + Pcw;
        V3D pf2 = new_frame->w2f(pt_w);
        V2D px = cam->world2cam(pf);    // 像素坐标
        V2D px2 = new_frame->w2c(pt_w);
        //debug_file<<"1: "<< px.transpose()<<" 2: "<<px2.transpose()<<std::endl;
        if(pf[2] > blind&&pf[2]<max_blind){    // 在相机前面,不要太近的点,也不要太远
            //std::cout<<"第"<<it-map.map_points_.begin()<<"个空间点的像素坐标是："<<px[0]<<","<<px[1]<<std::endl;
            // 在图像内？
            if(new_frame->cam_->isInFrame(px.cast<int>(), (patch_size_half+1)*8))
            {
                observed_points.push_back(it_ptr);
                float depth = pf[2];
                int col = int(px[0]);
                int row = int(px[1]);
                //debug_file<<"第"<<width*row+col<<"个像素点在图像内且被赋予深度值"<<std::endl;
                if(depth_img[width*row+col]!=0){
                    if(depth_img[width*row+col]>depth){
                        depth_img[width*row+col] = depth;
                        adj_pts[width*row+col] = pt_w;
                    }
                    else{

                    }
                }
                else{
                    depth_img[width*row+col] = depth;
                    adj_pts[width*row+col] = pt_w;
                }
                // if(stage==2){
                //     //img_depth.data[width*row+col] = depth*10;
                // }
                // 对某个像素放depth，但可能只有一部分有深度，没深度的地方是0，这个目的是为了去遮挡 
                int index=0;
                int col_grid = static_cast<int>(col/grid_size);
                int row_grid = static_cast<int>(row/grid_size);
                index = row_grid*grid_n_width+col_grid;
                index = std::min(index,(length-1));
                auto score = vk::shiTomasiScore(img, px[0], px[1]);
                //V2D pyr_px = px/(1<<(pyr-1));
                //auto score = vk::shiTomasiScore(pyramid.back(), pyr_px[0], pyr_px[1]);
                if(score<score_threshold) continue;
                //auto edgescore = gradientMagnitude.at<float>(px[1], px[0]);
                // if(edgescore>edgescore_threshold){
                //     //debug_file<<"edgescore: "<<edgescore<<std::endl;
                //     continue;
                // }
                //std::cout<<"该点将要被放到第"<<index<<"个grid里面"<<std::endl;
                
                Candidate c(it_ptr.get(),px);  // 很关键，里面放着该地图点的指针
                c.score = score;
                //std::cout<<"该点的score = "<<c.score<<std::endl;
                auto j = pts_num_in_each_cell[index];
                //std::cout<<"该grid已经有"<<j<<"个候选点"<<std::endl;
                grid.cells[index]->push_back(c);    // 把点放到cell里面    
                pts_num_in_each_cell[index]++;      
            }
        }
    }
    double t_2 = omp_get_wtime();
    debug_file<<"====遍历点云耗时"<<t_2-t_1<<std::endl;
    // 遍历grid，按照评分排序，留下每个grid里面评分前十的有关键帧看到的且深度连续的点
        //debug_file<<"第"<<i<<"个cell里面的第"<<num<<"点的空间坐标为"<<pt_w.transpose()<<std::endl;
    for(int i=0;i<length;i++){
        if(grid.cells[i]->size()==0){
            continue;
        }
        grid.cells[i]->sort([](const Candidate&p1, const Candidate&p2) {return p1.score > p2.score ; }); // 排序按照分数
        std::list<Candidate> *point_list_ptr = new std::list<Candidate>;
        auto it = grid.cells[i]->begin();
        for(;it!=grid.cells[i]->end();it++){
            if(it->pt->n_obs_>0||first_frame){   // 如果被观测到, 进行判断，同一个patch内是否深度连续
                V3D pt_cam(new_frame->w2f(it->pt->pos_));  // 相机坐标系坐标
                if(pt_cam[2]<skip_depth) continue; // 不要太近的点
                bool depth_continous = depthContinue2(it->px,pt_cam[2],it->pt->pos_);
                //cv::circle(img_cp,gird_point,4,cv::Scalar(255, 0, 0), -1, 8);
                if(depth_continous) continue;
                point_list_ptr->push_back(*it);
                cv::Point gird_point(it->px[0],it->px[1]);
                cv::circle(img_cp,gird_point,4,cv::Scalar(255, 0, 0), -1, 8);   // 合格点 蓝色
                PointType p;
                p.x = it->pt->pos_.x();
                p.y = it->pt->pos_.y();
                p.z = it->pt->pos_.z();
                pcl_projected->push_back(p);
                if(point_list_ptr->size()>=grid_pts_num)     break;
            }
        }
        
        grid.cells[i] = point_list_ptr;
        //debug_file<<"第"<<frame_nums<<"帧的第"<<i<<"个grid有"<<point_list_ptr->size()<<"个观测点"<<std::endl;
    }
    double t_3 = omp_get_wtime();
    debug_file<<"====grid排序耗时"<<t_2-t_1<<std::endl;

}
void violm::addObservation(){
    reset_grid();
    memset(depth_img,0,sizeof(float)*height*width);  // 深度
    double t_delete = 0;
    double t_add = 0;
    for(auto it = observed_points.begin();it!=observed_points.end();it++){
        auto it_ptr = (*it);
        
        V3D pt_w = it_ptr->pos_; // 空间点坐标
        V3D pf = Rcw * pt_w + Pcw;
        V3D pf2 = new_frame->w2f(pt_w);
        V2D px = cam->world2cam(pf);    // 像素坐标
        V2D px2 = new_frame->w2c(pt_w);
        //debug_file<<"1: "<< px.transpose()<<" 2: "<<px2.transpose()<<std::endl;
        if(pf[2] > blind&&pf[2]<max_blind){    // 在相机前面,不要太近的点,也不要太远
            //std::cout<<"第"<<it-map.map_points_.begin()<<"个空间点的像素坐标是："<<px[0]<<","<<px[1]<<std::endl;
            // 在图像内？
            if(new_frame->cam_->isInFrame(px.cast<int>(), (patch_size_half+1)*8))
            {
                float depth = pf[2];
                int col = int(px[0]);
                int row = int(px[1]);
                //std::cout<<"第"<<width*row+col<<"个像素点在图像内且被赋予深度值"<<std::endl;
                if(depth_img[width*row+col]!=0)
                    depth_img[width*row+col] = min(depth_img[width*row+col],depth);
                else
                    depth_img[width*row+col] = depth;
                // if(stage==2){
                //     //img_depth.data[width*row+col] = depth*10;
                // }
                // 对某个像素放depth，但可能只有一部分有深度，没深度的地方是0，这个目的是为了去遮挡 

                auto score = vk::shiTomasiScore(img, px[0], px[1]);
                //V2D pyr_px = px/(1<<(pyr-1));
                //auto score = vk::shiTomasiScore(pyramid.back(), pyr_px[0], pyr_px[1]);
                //auto edgescore = gradientMagnitude.at<float>(px[1], px[0]);
                // if(edgescore>edgescore_threshold){
                //     //debug_file<<"edgescore: "<<edgescore<<std::endl;
                //     continue;
                // }
                //std::cout<<"该点将要被放到第"<<index<<"个grid里面"<<std::endl;
                
                // 如果是关键帧，那么就添加这个点关联的帧

                auto t3 = omp_get_wtime();
                bool add_flag = true;
                if(it->get()->is_outlier==true){
                    add_flag = false;
                    it->get()->is_outlier = false;   // 修改外点观测
                } 
                // if(add_flag == true){   // 不是外点，判断一下是否深度连续
                //     bool depth_continous = depthContinues(px,pf[2]);
                // }
                if(need_keyframe&&add_flag){
                    it_ptr->value = score;
                    Vector3d f = cam->cam2world(px);
                    float* patch_temp = new float[patch_size_total*pyr];
                    for(int l = 0;l<pyr;l++){
                        getpatch(img, px, patch_temp, l); // 添加patch 0在前，2在后
                    }
                    FeaturePtr ftr_new(new Feature(patch_temp, px, f, new_frame->T_f_w_, it_ptr->value, 0)); 
                    ftr_new->frame = map.lastKeyframe().get();
                    ftr_new->frame->id_ = new_frame->id_;
                    it_ptr->addFrameRef(ftr_new);    // 给地图点加特征，关键帧
                    //debug_file<<"给点添加关键帧"<<ftr_new->frame->id_<<std::endl;
                }
                auto t4 = omp_get_wtime();
                t_add = t_add+t4-t3;
            }
        }
    }
    debug_file<<"t_add: "<<t_add<<"s"<<std::endl;
}


// 核心
void violm::Process(const cv::Mat &img_, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    auto t1 = omp_get_wtime();
    img = img_;
    
    if(img_scale!=1.0)    cv::resize(img,img,cv::Size(img.cols*img_scale,img.rows*img_scale),0,0,CV_INTER_LINEAR);

    if(img.channels()==3)    {
        img_cp = img.clone();   // imp_cp用于画图
        cv::cvtColor(img,img,CV_BGR2GRAY);  // 到灰度图
    }
    else if(img.channels()==1){
        img_cp = img.clone();
        cvtColor(img, img_cp, CV_GRAY2BGR); // 输入如果是灰度图，imp_cp用于画图
    }
    else{
        std::cout<<"ERROR Channels!!!"<<std::endl;
        return;
    }
    
    if(equalize){   // 太亮或太暗，自适应直方图均衡
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    new_frame = new Frame(cam, img.clone());

    new_frame->id_ = frame_nums;
    frame_nums++;

    Updatekf(kf);
    UpdateCamPose();
    
    double t2 = omp_get_wtime();
    debug_file<<"====拷贝图像等预处理花费"<<t2-t1<<"s"<<std::endl;

    /*** 投影点云到图像上 ***/
    pcl_projected->clear();
    projection();
    double t3 = omp_get_wtime();
    debug_file<<"====点云投影花费"<<t3-t2<<"s"<<std::endl;

    /*** 光度约束 ***/
    if(first_frame){
        //first_frame = false;
    }
    else{
        photometricConstraint(kf);
    }
    double t4 = omp_get_wtime();
    debug_file<<"====光度约束花费"<<t4-t3<<"s"<<std::endl;

    /*** 投影约束 ***/
    if(enable_projection){
        double t5 = omp_get_wtime();
        projectionConstraint(kf);
        double t6 = omp_get_wtime();
        debug_file<<"====投影约束花费"<<t6-t5<<"s"<<std::endl;
    }

    /*** 3D点约束 ***/
    if(enable_triangulate){
        double t5 = omp_get_wtime();
        if(first_frame){}
        else{
            triangulate(kf);
        }
        double t6 = omp_get_wtime();
        debug_file<<"====3D点约束花费"<<t6-t5<<"s"<<std::endl;
    }

    /*** 增加关键帧 ***/
    double t_k1 = omp_get_wtime();
    addKeyFrame();
    double t_k2 = omp_get_wtime();
    debug_file<<"增加关键帧花费："<<t_k2-t_k1<<"s"<< std::endl;

    /*** 线特征约束 ***/
    if(need_keyframe){
        //map.lastKeyframe()->T_f_w_ = new_frame->T_f_w_;
        if(first_frame){
        }
        else{
            if(enable_line){
                double t41 = omp_get_wtime();
                projectionLine();
                double t42 = omp_get_wtime();
                debug_file<<"投影3D直线花费"<<t42-t41<<"s"<<std::endl;
                lineConstraint(kf);
                double t43 = omp_get_wtime();
                debug_file<<"2D-3D匹配优化花费"<<t43-t42<<"s"<<std::endl;
            }
        }
    }
    
    /*** 增加观测点 ***/
    double t7 = omp_get_wtime();
    addObservation();
    double t8 = omp_get_wtime();
    debug_file<<"增加观测点花费"<<t8-t7<<"s"<<std::endl;
    
    debug_file<<"总花费: "<<t8-t1<<"s"<<std::endl;
    delete new_frame;
    if(first_frame){
        first_frame = false;
    }
}
void violm::addKeyFrame(){
    /*关键帧策略*/
    need_keyframe = true;
    update_key_frame_type = 1;
    double min_dis = 100;
    double max_dis = 0;
    int max_id = 0;
    std::list<lvo::FramePtr>::iterator max_it;
    // 第一帧
    if(!first_frame){
        // 若角度变化大或位置变化大，添加关键帧
        V3D Pwc = - Rcw.transpose()*Pcw;
        M3D Rwc = Rcw.transpose();
        euler = RotMtoEuler(Rwc);
        // 遍历关键帧位姿
        for(auto it=map.keyframes_.begin();it!=map.keyframes_.end();it++){
            auto Pwc0 = it->get()->pos();
            //std::cout<<"Pcw0："<<Pcw0.transpose()<<std::endl;
            if((Pwc-Pwc0).norm()<min_dis)
                min_dis = (Pwc-Pwc0).norm();
            if((Pwc-Pwc0).norm()>max_dis){
                max_dis = (Pwc-Pwc0).norm();
                max_id = it->get()->id_;
                max_it = it;
            }
            if( (Pwc-Pwc0).norm()<trans_thre){
                auto Rwc0 = it->get()->T_f_w_.inverse().rotation_matrix();
                auto euler0 = RotMtoEuler(Rwc0);
                auto deuler = euler-euler0;
                for(int i=0;i<3;i++){
                    if(deuler[i]<rot_thre||deuler[i]>(PI_M-rot_thre)){
                        update_key_frame_type = 0;
                        need_keyframe = false;
                        break;
                    }
                }
                //debug_file<<"因角度过大增加关键帧："<<deuler.norm()<<std::endl;
            }
        }
        //debug_file<<"最小位移："<<min_dis<<std::endl;
        //debug_file<<"最大位移："<<max_dis<<std::endl;
    }   
    if(need_keyframe){
        if(map.keyframes_.size()>window_size){
            //remove_frame_num = map.keyframes_.begin()->get()->id_;
            remove_frame_num = max_id;
            debug_file<<"删除关键帧id为："<<remove_frame_num<< std::endl;
            max_it = map.keyframes_.erase(max_it);
        }
        FramePtr cur_frame;
        cur_frame = std::make_shared<Frame>(cam, img.clone());
        cur_frame->T_f_w_ = new_frame->T_f_w_;
        cur_frame->id_ = frame_nums;
        cur_frame->Cov_ = state->cov.block<6,6>(0,0);
        map.addKeyframe(cur_frame);
        debug_file<<"增加关键帧id为："<<frame_nums<< std::endl;
        // auto frame_pos = map.lastKeyframe().get()->T_f_w_.inverse().translation();
        // auto imu_pos = kf.get_x().pos;
        // auto P = (imu_pos-frame_pos).norm();
        // debug_file<<"外参的模长为："<<P<<std::endl;

    }
}
}


