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
#include <vio_in_lidar_map.h>

namespace lvo{
void violm::set_camera2lidar(vector<double>& R,  vector<double>& P ){
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
    exp_file_i.open("/home/i/vio_in_lidar_map/src/vio_in_lidar_map/Log/exp.txt",ios::in);
    res_file_i.open("/home/i/vio_in_lidar_map/src/vio_in_lidar_map/Log/response.txt",ios::in);
    loadExposureTime();
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
        for(int i=0;i<ptr->size();i++){
            V3D pos(ptr->at(i).x,ptr->at(i).y,ptr->at(i).z);
            pointData.pts.push_back(PointCloud<double>::PtData(pos.x(),pos.y(),pos.z()));
        }   // 放到pointData里
        
        detector.run(pointData, k, planes, lines, ts);
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
    double t_delete = 0;
    double t_add = 0;
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
        t_delete = t_delete + t2 - t1;
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
    debug_file<<"t_delete: "<<t_delete<<"s"<<std::endl;
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


// 核心
void violm::Process(const cv::Mat &img_, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    auto t1 = omp_get_wtime();
    img = img_;
    
    if(img_scale!=1.0)    cv::resize(img,img,cv::Size(img.cols*img_scale,img.rows*img_scale),0,0,CV_INTER_LINEAR);
    // pin_cam->undistortImage(img,img);
    //auto img_rgb = img.clone();
    double exp_t;
    
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
    //cv::Sobel(img, gradientX, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    //cv::Sobel(img, gradientY, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);

    // Compute the gradient magnitude
    
    //cv::magnitude(gradientX, gradientY, gradientMagnitude);
    //std::cout<<"2.1.00"<<std::endl;
    //std::cout<<img.size()<<std::endl;
    //std::cout<<cam->height()<<","<<cam->width()<<std::endl;
    new_frame = new Frame(cam, img.clone());
    // pyramid.clear();
    // pyramid.push_back(img);
    // cv::Mat currentImage = img;
    // for (int level = 1; level < pyr; ++level){
    //     cv::Mat downsampledImage;
    //     cv::pyrDown(currentImage, downsampledImage);
        
    //     // Store the downsampled image in the pyramid
    //     pyramid.push_back(downsampledImage);
        
    //     // Set the downsampled image as the current image for the next level
    //     currentImage = downsampledImage;
    // }
    new_frame->id_ = frame_nums;
    frame_nums++;
    // if(frame_nums>990&&frame_nums<1000){
    //     return;
    // }
   //std::cout<<"2.1.0"<<std::endl;
    Updatekf(kf);
    UpdateCamPose();


    // PointVector sub_pcd;
    // BoxPointType box;
    // auto vehicle_frame_pos = Pcw;
    // box.vertex_max[0] = vehicle_frame_pos[0]+30;
    // box.vertex_min[0] = vehicle_frame_pos[0]-30;
    // box.vertex_max[1] = vehicle_frame_pos[1]+30;
    // box.vertex_min[1] = vehicle_frame_pos[1]-30;
    // box.vertex_max[2] = vehicle_frame_pos[2]+30;
    // box.vertex_min[2] = vehicle_frame_pos[2]-30;
    // ikdtree.Box_Search(box,sub_pcd);
    // std::cout<<"当前局部点云有"<<sub_pcd.size()<<"个点"<<std::endl;
    
    //std::cout<<"2.1.1"<<std::endl;

    //std::cout<<"solve time:"<<solve_time<<std::endl;
    
    
    // 测量更新
    double t2 = omp_get_wtime();
    debug_file<<"拷贝图像等预处理花费"<<t2-t1<<"s"<<std::endl;
    //debug_file<<"当前观测到的地图点个数为: "<<obs_points()<<"/"<<map.map_points_.size()<<std::endl;
    pcl_projected->clear();
    projection();
    double t3 = omp_get_wtime();
    debug_file<<"第一次投影花费"<<t3-t2<<"s"<<std::endl;
    if(first_frame){
        //first_frame = false;
    }
    else{
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
    double t4 = omp_get_wtime();
    debug_file<<"迭代花费"<<t4-t3<<"s"<<std::endl;

    /*关键帧策略*/
    double t_k1 = omp_get_wtime();
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
            //map.keyframes_.pop_front();

            // for(auto it = covisible_pair.begin();it!=covisible_pair.end();){
            //     if(std::get<1>(*it)==remove_frame_num||std::get<2>(*it)==remove_frame_num){
            //         it = covisible_pair.erase(it);
            //         //debug_file<<"删除匹配对为："<<std::get<1>(*it)<<"对"<<std::get<2>(*it)<< std::endl;
            //     }
            //     else{
            //         it++;
            //     }
            // }
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
    double t_k2 = omp_get_wtime();
    debug_file<<"增加关键帧花费："<<t_k2-t_k1<<"s"<< std::endl;
    // 线特征
    if(need_keyframe){
        //map.lastKeyframe()->T_f_w_ = new_frame->T_f_w_;
        if(first_frame){
        //first_frame = false;
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
    double t5 = omp_get_wtime();
    // if(enable_triangulate)
    //     geoConstraint(kf);
    if(enable_projection){
        projectionConstraint(kf);
    }
    
    double t6 = omp_get_wtime();
    debug_file<<"投影约束花费"<<t6-t5<<"s"<<std::endl;

    addObservation();
    double t7 = omp_get_wtime();
    debug_file<<"增加观测点花费"<<t7-t6<<"s"<<std::endl;
     double t8 = omp_get_wtime();
    debug_file<<"总花费: "<<t8-t1<<"s"<<std::endl;
    if(first_frame){
        localmapGenerate();
        first_frame = false;
    }
    delete new_frame;
}
void violm::setState(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    state->rot_end = kf.get_x().rot.toRotationMatrix();
    state->pos_end = kf.get_x().pos;
    state->vel_end = kf.get_x().vel;
    state->bias_g = kf.get_x().bg;
    state->bias_a = kf.get_x().ba;
    state->gravity = kf.get_x().grav;
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P;
    state->cov = kf.get_P();

}
void violm::setStatePropagate(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    state_propagat->rot_end = kf.get_x().rot.toRotationMatrix();
    state_propagat->pos_end = kf.get_x().pos;
    state_propagat->vel_end = kf.get_x().vel;
    state_propagat->bias_g = kf.get_x().bg;
    state_propagat->bias_a = kf.get_x().ba;
    state_propagat->gravity = kf.get_x().grav;
    state_propagat->cov = kf.get_P();
}
void violm::setKF(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    state_ikfom state_point;
    state_point.pos = state->pos_end;
    state_point.rot = state->rot_end;
    state_point.vel = state->vel_end;
    state_point.ba = state->bias_a;
    state_point.bg = state->bias_g;
    state_point.grav = state->gravity;
    
    kf.change_x(state_point);
    kf.change_P(state->cov);
    // debug_file<<"setKF state->cov: "<<state->cov<<endl;
    // auto P = kf.get_P();
    // debug_file<<"setKF P: "<<P<<endl;
}
void violm::updateFrameState()
{
    Rwi = state->rot_end;
    //debug_file<<"Rwi合法？"<<Rwi*Rwi.transpose()<<std::endl;
    Pwi = state->pos_end;
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;
    new_frame->T_f_w_ = Sophus::SE3(Rcw, Pcw);
    // auto frame_pos = new_frame->T_f_w_.inverse().translation();
    // auto imu_pos = state->pos_end;
    // auto P = (imu_pos-frame_pos).norm();
    // debug_file<<"在updateFrameState内外参的模长为："<<P<<std::endl;
}
void violm::Updatekf(const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    Rwi = kf.get_x().rot.toRotationMatrix();
    Pwi = kf.get_x().pos;
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;  
}
void violm::UpdateCamPose(){
    new_frame->T_f_w_ = Sophus::SE3(Rcw, Pcw);
}
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

/*投影约束相关内容*/
void violm::projectionConstraint(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
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
        // lightness_factor = 0;
        // if(pts_last.size()>0){
        //     int kkk = 0;
        //     for(int i=0;i<pts_last.size();i++){
        //         const int last_x = floorf(pts_last[i].x);
        //         const int last_y = floorf(pts_last[i].y);
        //         uint8_t* img_ptr1 = (uint8_t*) img_last.data + last_y*width + last_x;
        //         auto last_light = *img_ptr1;
                
        //         const int cur_x = floorf(pts_cur[i].x);
        //         const int cur_y = floorf(pts_cur[i].y);
        //         uint8_t* img_ptr2 = (uint8_t*) img_cur.data + cur_y*width + cur_x;
        //         auto cur_light = *img_ptr2;
        //         auto one_light_factor = cur_light/last_light;
        //         lightness_factor += one_light_factor;
        //         kkk++;
        //     }
        //     lightness_factor = lightness_factor/kkk;
        // }
        // debug_file<<"亮度变化系数："<<lightness_factor<<std::endl;
        img_last = img_cur;
        pts_last = pts_cur;

        debug_file<<"还剩"<<pts_last.size()<<"个点被追踪"<<std::endl;
        // 开始迭代
        iterate_num_P = 0;
        last_error = 10000;
        StatesGroup * state_before = new StatesGroup(*state);
        const int pts_num = pts_last.size();
        track_outlier.resize(pts_num);
        std::fill(track_outlier.begin(),track_outlier.end(),0);
        // for(int iii = 0;iii<track_outlier.size();iii++){
        //     debug_file<<track_outlier[iii]<<",";
        // }
        int wrong_time = 0;
        for(int i=0;i<max_iteration;i++){
            auto old_state = *state;
            auto valid = CalculateJPandResP();
            if(!valid){
                debug_file<<"无跟踪点"<<std::endl;
                // for(int iii = 0;iii<track_outlier.size();iii++){
                //     debug_file<<track_outlier[iii]<<",";
                // }
                // debug_file<<endl;
                continue;
            }
            const int eff_num = resP_sub.size();
            Eigen::MatrixXd HP_sub = MatrixXd::Zero(eff_num, DIM_STATE);
            for(int l=0;l<6;l++){
                HP_sub.col(l) = JP_sub.col(l);
            }
            auto HP_sub_T = HP_sub.transpose();
            HP_T_HP = HP_sub_T * HP_sub;
            

            // debug_file<<"IMG_COV: "<<IMG_COV<<std::endl;
            MD(DIM_STATE, DIM_STATE) &&KP_1 = (HP_T_HP + (state->cov / POINT_COV).inverse()).inverse();
            //debug_file<<"H_T_H: "<<H_T_H<<std::endl;
            //debug_file<<"state->cov: "<<state->cov<<std::endl;
            // debug_file<<"K_1: "<<K_1<<std::endl;
            auto &&HPTz = HP_sub_T * resP_sub;
            // K = K_1.block<DIM_STATE,6>(0,0) * H_sub_T;
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
			debug_file<<"第"<<iterate_num_P<<"次投影约束迭代的平均error为："<<meansP<<std::endl;
            
            if(meansP>last_error){
                debug_file<<"损失增大，回退"<<std::endl;
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
                    debug_file<<"无法降低误差，退出"<<std::endl;
                }
                else{
                    debug_file<<"迭代"<<iterate_num_P<<"次收敛"<<std::endl;
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
        if(1){
            featAdd();
            triangulate(kf);
        }
    }
}
bool violm::featAdd(){
    // pts_last.clear();
    // pts_cur.clear();
    // ptws_last.clear();
    if(pts_last.size()>=500) return true;
    auto mask = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
    
    for(int i=0;i<pts_last.size();i++){
        cv::Point p(pts_last[i].x,pts_last[i].y);
        cv::circle(mask, p, 30, 0, -1);
        //cv::circle(img_cp, p, 30, 0, 2);
    }
    for(int i=0;i<length;i++){
        if(grid.cells[i]->size()==0) continue;
        
        auto it_ptr = grid.cells[i]->begin()->pt;
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
        if(pts_last.size()>=500) break;
        //cv::circle(mask, pp, 30, 0, -1);
    }
    all_tracked_points.push_back(pts_last);
    auto cur_pts_T = std::vector<Sophus::SE3>(pts_last.size(),new_frame->T_f_w_);
    all_pts_T.push_back(cur_pts_T);

    debug_file<<"开始对"<<pts_last.size()<<"个点追踪"<<std::endl;
    return true;
}
template <typename T>
void violm::reduceVector(vector<T> &v, vector<uchar> status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
void violm::reduceVector2(vector<V3D> &v, vector<uchar> status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

bool violm::CalculateJPandResP(){
    const int H_DIM = ptws_last.size();  // 
    Rwi = state->rot_end;                //debug_file<<"state->rot_end: "<<state->rot_end<<std::endl;
    Pwi = state->pos_end;               //debug_file<<"state->pos_end: "<<state->pos_end.transpose()<<std::endl;
    Rcw = Rci * Rwi.transpose();    //debug_file<<"state->vel_end: "<<state->vel_end.transpose()<<std::endl;
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;   //debug_file<<"state->bias_a: "<<state->bias_a.transpose()<<std::endl;
    // 遍历grid，求J和res
    JP.resize(H_DIM*2,6);  //debug_file<<"state->gravity: "<<state->gravity.transpose()<<std::endl;
    resP.resize(H_DIM*2,1);
    JP.setZero();
    resP.setZero();
    std::vector<int> index_table;
    int kk = -2;
    for(int i=0;i<H_DIM;i++){
        V2D pc_ref;
        pc_ref[0] = pts_last[i].x;pc_ref[1] = pts_last[i].y;
        V3D pt_w = ptws_last[i]; // 空间点坐标
        V3D pf = Rcw * pt_w + Pcw;  // 相机坐标系坐标
        V2D pc = cam->world2cam(pf);    // 像素坐标
        kk = kk+2;
        double du = pc[0] - pc_ref[0];
        double dv = pc[1] - pc_ref[1];
                // 剔除外点
        if(iterate_num_P==0){
            // 算一下error
            // debug_file<<"跟踪位置为："<<pc_ref[0]<<","<<pc_ref[1]<<std::endl;
            // debug_file<<"du:"<<du<<std::endl;
            // debug_file<<"dv:"<<dv<<std::endl;
            if((du*du+dv*dv)>400){
                track_outlier[i] = 1;
                //debug_file<<"外点"<<std::endl;
                continue;
            }   // 第一次迭代时，计算出外点
            // else{
            //     it_pv->pt->is_outlier = false;
            // }
            cv::Point gird_point(pc[0],pc[1]);
            cv::circle(img_cp,gird_point,2,cv::Scalar(0, 255, 0), -1, 8);   // 绿色
        }
        if(track_outlier[i]==1) continue;
        // 算一下雅可比
        MD(2,3) Jdpi;
        MD(2,3) Jdphi, Jdp, JdR, Jdt;
        M3D p_hat;
        Jdp_dt = Rci * Rwi.transpose(); // 
        dpi(pf, Jdpi);// Jdpi就是偏u/偏q，即投影方程关于相机坐标系下三维点的导数
        p_hat << SKEW_SYM_MATRX(pf);

        Jdphi = Jdpi * p_hat;    // 对旋转的雅可比
        Jdp = -Jdpi;                 // 对平移的雅可比
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
        //debug_file<<"J "<<j<<": "<<J_sub.block<1,6>(j,0)<<std::endl;
        resP_sub.block<1,1>(j,0) =  resP.block<1,1>(index_table[j],0);
        //debug_file<<"res "<<j<<": "<<res_sub.block<1,1>(j,0)<<std::endl;
    }
    return true;
}



void violm::localmapGenerate(){
    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    local_map->clear();
    local_map->points = ikdtree.PCL_Storage;
}
bool violm::depthContinue(const V2D &px,const double & depth0){
    bool depth_continous = false;
    
    for (int u=-depth_search_radius; u<=depth_search_radius; u++){
                    for (int v=-depth_search_radius; v<=depth_search_radius; v++){
                        if(u==0 && v==0) continue;
                        float depth = depth_img[width*(v+int(px[1]))+u+int(px[0])];
                        if(depth == 0.) continue;
                        double delta = fabs(depth - depth0);
                        if(delta>delta_dist_thre) {
                            depth_continous = true;
                            break;
                        }

                    }
                    if(depth_continous) break;
                }
    return depth_continous;
}
bool violm::depthContinue2(const V2D &px,const double & depth0, const V3D &ptw){
    bool depth_continous = false;
    
    for (int u=-depth_search_radius; u<=depth_search_radius; u++){
                    for (int v=-depth_search_radius; v<=depth_search_radius; v++){
                        if(u==0 && v==0) continue;
                        float depth = depth_img[width*(v+int(px[1]))+u+int(px[0])];
                        if(depth == 0.) continue;
                        double delta = depth - depth0;
                        if(delta>0) continue;
                        V3D v1 = adj_pts[width*(v+int(px[1]))+u+int(px[0])] - ptw;
                        V3D v2 =  new_frame->pos() - ptw;
                        double theta = v1.dot(v2)/v1.norm()/v2.norm();
                        
                        //debug_file<<"theta:"<<theta<<std::endl;
                        if(theta>theta_thre){
                            depth_continous = true;
                            break;
                        }
                    }
                    if(depth_continous) break;
                }
    return depth_continous;
}
float violm::calculateEdgeScore(const cv::Mat& img, int x, int y)
{
    // Calculate the gradient using the Sobel operator


    // Get the edge score at the specified location (x, y)
    float score = gradientMagnitude.at<float>(y, x);

    return score;
}
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
            debug_file<<"该点三角化时总共被"<<tracked_num<<"帧看到"<<std::endl;
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
                debug_file<<"无点面匹配"<<std::endl;
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
			debug_file<<"第"<<iterate_num_T<<"次面约束迭代的平均error为："<<meansT<<std::endl;
            
            if(meansT>last_error){
                debug_file<<"损失增大，回退"<<std::endl;
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
                debug_file<<"迭代"<<iterate_num_T<<"次收敛"<<std::endl;
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
bool violm::CalculateJTandResT(){
    debug_file<<"第"<<frame_nums<<"帧的第"<<iterate_num_T<<"次面特征迭代"<<std::endl;
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
        debug_file<<"新来了"<<triangulate_pts_body->size()<<"个三角化点"<<std::endl;
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
        debug_file<<"面约束有效点个数为："<<effct_feat_num<<std::endl;
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
        // if (extrinsic_est_en)
        // {
        //     V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
        //     ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        // }
        // else
        // {
        JT_sub.block<1, 6>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A);
        // }   // h_x: 残差的雅可比

        /*** Measuremnt: distance to the closest surface/corner ***/
        resT_sub(i) = -norm_p.intensity;    // h: 残差
    }
    //debug_file<<"JP_sub："<<JP_sub<<std::endl;
    //debug_file<<"resP_sub："<<resP_sub.transpose()<<std::endl;
    return valid;
}
void violm::divideGrayMatByScalar(cv::Mat& grayImage, double divisor) {
    // 检查输入图像是否为空
    if (grayImage.empty()) {
        std::cerr << "输入图像为空" << std::endl;
        return;
    }
    debug_file<<"图像大小："<<grayImage.rows<<","<<grayImage.cols<<std::endl;
    // 循环遍历图像的每个像素
    for (int i = 0; i < grayImage.rows; ++i) {
        for (int j = 0; j < grayImage.cols; ++j) {
            // 获取当前像素的值并除以给定数值
            uchar& pixel = grayImage.at<uchar>(i, j);
            double p2 = linearInterpolation(response,index_response,pixel);
            
            p2 /= divisor;
            if(p2>255) p2 = 255;
            pixel = linearInterpolation(index_response,response,p2);
        }
    }
}
void violm::loadExposureTime(){
    std::string s;
    getline(res_file_i,s);
    std::stringstream ss(s);
    for(int i=0;i<256;i++){
        index_response.push_back(i);
        double res;
        ss>>res;
        response.push_back(res);
    }
    double time;
    std::vector<double> exp_10;
    std::vector<double> index_10;
    int index = 0;
    while(exp_file_i>>time){
        index_10.push_back(index);
        index+=10;
        exp_10.push_back(time);
    }
    for(int i=0;i<index;i++){
        double exp_t = linearInterpolation(index_10,exp_10,i);
        exp_time.push_back(exp_t);
    }
}
double violm::linearInterpolation(const std::vector<double>& xValues, const std::vector<double>& yValues, double targetX) {
    if (xValues.size() != yValues.size()) {
        std::cerr << "x坐标和y坐标数组长度不一致" << std::endl;
        return 0.0; // 错误处理，返回默认值
    }

    for (size_t i = 0; i < xValues.size() - 1; ++i) {
        if (xValues[i] <= targetX && targetX <= xValues[i + 1]) {
            double x0 = xValues[i];
            double x1 = xValues[i + 1];
            double y0 = yValues[i];
            double y1 = yValues[i + 1];
            return y0 + (targetX - x0) * (y1 - y0) / (x1 - x0);
        }
    }

    std::cerr << "目标x值不在给定范围内" << std::endl;
    return 0.0; // 错误处理，返回默认值
}
}   //namespace lvo

