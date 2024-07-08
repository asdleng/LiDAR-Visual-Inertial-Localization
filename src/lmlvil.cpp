#include "lmlvil.h"

namespace lvo{
void lmlvil::set_cameraext(vector<double>& R,  vector<double>& P )
{
    Rci << MAT_FROM_ARRAY(R);
    Pci << VEC_FROM_ARRAY(P);
}
void lmlvil::set_extrinsic(const V3D &transl, const M3D &rot)
{
    Ril = rot;
    Pil = transl;
    Pli = -rot.transpose() * transl;
    Rli = rot.transpose();
}
void lmlvil::set_mapext(vector<double>& R,  vector<double>& P )
{
    R_convert << MAT_FROM_ARRAY(R);
    P_convert << VEC_FROM_ARRAY(P);
}
int lmlvil::obs_points(){
    int k=0;
    for(auto it = map.map_points_.begin();it<map.map_points_.end();it++){
        if(it->get()->n_obs_>0){
            k++;
        }
    }
    return k;
}
void lmlvil::Init(PointCloudXYZI::Ptr ptr){
    omp_set_num_threads(32);
    res_last = new float[100000]();
    point_selected_surf = new bool[100000]();
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    state = new StatesGroup;
    state_propagat = new StatesGroup;
    debug_file.open(FILE_DIR("debug_J.txt"), ios::out);
    time_file.open(FILE_DIR("time.txt"), ios::out);
    
    if(read_enable) lines_file_i.open(FILE_DIR(lines_file),ios::in);
    else lines_file_w.open(FILE_DIR(lines_file),ios::out);
    
    
    Jdphi_dR = Rci; 
    V3D Pic = -Rci.transpose() * Pci;

    M3D tmp;
    tmp << SKEW_SYM_MATRX(Pic);
    Jdp_dR = -Rci * tmp;
    downSizeFilter.setLeafSize(filter_size, filter_size, filter_size);
    first_frame = true;
    height = cam->height();
    width = cam->width();
    grid_n_width = static_cast<int>(width/grid_size);   
    grid_n_height = static_cast<int>(height/grid_size); 
    length = grid_n_width * grid_n_height;  
    depth_img = new float[height*width];
    
    adj_pts.resize(width*height);
    grid.cells.resize(length);
    std::for_each(grid.cells.begin(), grid.cells.end(), [&](Cell*& c){ c = new Cell; });
    patch_size_total = patch_size * patch_size;
    patch_size_half = static_cast<int>(patch_size/2);
    
    pts_num_in_each_cell = new int[length];
    memset(pts_num_in_each_cell, 0, sizeof(int)*length);
    outlier_map = new int[length*grid_pts_num*max_ftr_list_num];
    fx = cam->errorMultiplier2();
    fy = cam->errorMultiplier() / (4. * fx);
    pin_cam = (vk::PinholeCamera*) cam;
    if(left_coord){
        for(auto it=ptr->begin();it!=ptr->end();it++){
            it->y = -it->y;
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
    }
    else{
        pcl_down = ptr;
    }
    if(convert_map){
        for(auto it=pcl_down->begin();it!=pcl_down->end();it++){
            V3D pos(it->x,it->y,it->z);
            V3D pos_g = R_convert*pos + P_convert;
            it->x = pos_g.x();
            it->y = pos_g.y();
            it->z = pos_g.z();
        }
    }
    
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

    if(read_enable){
        std::string s;

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
    }
    else{
        int k = 200;
        PointCloud<double> pointData; 
        std::vector<PLANE> planes;
        std::vector<double> ts;
        for(int i=0;i<ptr->size();i++){
            V3D pos(ptr->at(i).x,ptr->at(i).y,ptr->at(i).z);
            pointData.pts.push_back(PointCloud<double>::PtData(pos.x(),pos.y(),pos.z()));
        }   
        
        detector->run(pointData, k, planes, lines, ts);
        
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


void lmlvil::projection(){
    observed_points.clear();
    std::deque<PointPtr>().swap(observed_points);
    memset(depth_img,0,sizeof(float)*height*width); 
    
    double t_1 = omp_get_wtime();
    #pragma omp parallel for
    for(int i=0;i<map.map_points_.size();i++){
        auto it_ptr = map.map_points_[i];
        V3D pt_w = it_ptr->pos_; 
        V3D pf = Rcw * pt_w + Pcw;
        V3D pf2 = new_frame->w2f(pt_w);
        V2D px = pin_cam->world2cam(pf);    
        V2D px2 = new_frame->w2c(pt_w);
        if(pf[2] > blind&&pf[2]<max_blind){    
            if(new_frame->cam_->isInFrame(px.cast<int>(), (patch_size_half+1)*8))
            {   
                float grad,gradx,grady;
                it_ptr->is_edge = isPointEdgelet(grad,gradx,grady,px2[0], px2[1], edge_threshold);
                if(it_ptr->is_edge){
                    it_ptr->grad = grad;
                    Vector2d g = Vector2d(gradx,grady);
                    g.normalize();
                    it_ptr->dir = g;
                }
                #pragma omp critical
                {
                    observed_points.push_back(it_ptr);
                }
                    float depth = pf[2];
                    int col = int(px[0]);
                    int row = int(px[1]);
                
                #pragma omp critical
                {    
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
                }
                    int index=0;
                    int col_grid = static_cast<int>(col/grid_size);
                    int row_grid = static_cast<int>(row/grid_size);
                    index = row_grid*grid_n_width+col_grid;
                    index = std::min(index,(length-1));
                    auto score = vk::shiTomasiScore(img, px[0], px[1]);
                    if(score<score_threshold) continue;
                    
                    Candidate c(it_ptr.get(),px);  
                    c.score = score;
                    auto j = pts_num_in_each_cell[index];
                #pragma omp critical
                {
                    grid.cells[index]->push_back(c);
                    pts_num_in_each_cell[index]++;  
                }      
                  
            }
        }
    }
    double t_2 = omp_get_wtime();
    
    #pragma omp parallel for
    for(int i=0;i<length;i++){
        if(grid.cells[i]->size()==0){
            continue;
        }
        #pragma omp critical
        {    
            grid.cells[i]->sort([](const Candidate&p1, const Candidate&p2) {return p1.score > p2.score ; }); // 排序按照分数
        }    
            std::list<Candidate> *point_list_ptr = new std::list<Candidate>;
            auto it = grid.cells[i]->begin();
            for(;it!=grid.cells[i]->end();it++){
                if(it->pt->n_obs_>0||first_frame){   
                    V3D pt_cam(new_frame->w2f(it->pt->pos_));  
                    if(pt_cam[2]<skip_depth) continue; 
                    bool depth_continous = depthContinue2(it->px,pt_cam[2],it->pt->pos_);
                    if(depth_continous) continue;
                    #pragma omp critical
                    {
                        point_list_ptr->push_back(*it);
                    }
                    cv::Point gird_point(it->px[0],it->px[1]);
                    cv::circle(img_cp,gird_point,4,cv::Scalar(255, 0, 0), -1, 8);   
                    PointType p;
                    p.x = it->pt->pos_.x();
                    p.y = it->pt->pos_.y();
                    p.z = it->pt->pos_.z();
                    #pragma omp critical
                    {
                        pcl_projected->push_back(p);
                    }
                    if(point_list_ptr->size()>=grid_pts_num)     break;
                }
            }
        
        grid.cells[i] = point_list_ptr;
    }
    double t_3 = omp_get_wtime();
    
}
void lmlvil::addObservation(){
    reset_grid();
    memset(depth_img,0,sizeof(float)*height*width);  
    double t_delete = 0;
    double t_add = 0;
    for(auto it = observed_points.begin();it!=observed_points.end();it++){
        auto it_ptr = (*it);
        
        V3D pt_w = it_ptr->pos_; 
        V3D pf = Rcw * pt_w + Pcw;
        V2D px = pin_cam->world2cam(pf);   
        if(pf[2] > blind&&pf[2]<max_blind){
            if(new_frame->cam_->isInFrame(px.cast<int>(), (patch_size_half+1)*8))
            {
                float depth = pf[2];
                int col = int(px[0]);
                int row = int(px[1]);
                if(depth_img[width*row+col]!=0)
                    depth_img[width*row+col] = min(depth_img[width*row+col],depth);
                else
                    depth_img[width*row+col] = depth;

                auto score = vk::shiTomasiScore(img, px[0], px[1]);

                auto t3 = omp_get_wtime();
                bool add_flag = true;
                if(it->get()->is_outlier==true){
                    add_flag = false;
                    it->get()->is_outlier = false;  
                } 
                
                if(need_keyframe&&add_flag){
                    it_ptr->value = score;
                    Vector3d f = pin_cam->cam2world(px);
                    float* patch_temp = new float[patch_size_total*pyr];
                    for(int l = 0;l<pyr;l++){
                        getpatch(img, px, patch_temp, l); 
                    }
                    FeaturePtr ftr_new(new Feature(patch_temp, px, f, new_frame->T_f_w_, it_ptr->value, 0)); 
                    ftr_new->frame = map.lastKeyframe().get();
                    ftr_new->frame->id_ = new_frame->id_;
                    it_ptr->addFrameRef(ftr_new);
                }
                auto t4 = omp_get_wtime();
                t_add = t_add+t4-t3;
            }
        }
    }
}

void lmlvil::Process(const cv::Mat &img_, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    
    auto t1 = omp_get_wtime();
    img = img_;
    
    if(img_scale!=1.0)    cv::resize(img,img,cv::Size(img.cols*img_scale,img.rows*img_scale),0,0,CV_INTER_LINEAR);
    
    if(img.channels()==3)    {
        
        cv::cvtColor(img,img,CV_BGR2GRAY);  
        img_cp = img.clone();   
        cv::cvtColor(img_cp,img_cp,CV_GRAY2BGR);
        
    }
    else if(img.channels()==4)    {
        cv::cvtColor(img,img,CV_BGRA2GRAY);  
        img_cp = img.clone();   
        cv::cvtColor(img_cp,img_cp,CV_GRAY2BGRA);
    }
    else if(img.channels()==1){
        img_cp = img.clone();
        cvtColor(img, img_cp, CV_GRAY2BGR); 
    }
    else{
        std::cout<<"ERROR Channels!!!"<<std::endl;
        return;
    }
    if(equalize){   
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }
    cv::Scharr(img, gradientX, CV_32F, 1, 0, 1/32.0);
    cv::Scharr(img, gradientY, CV_32F, 0, 1, 1/32.0);
    cv::magnitude(gradientX,gradientY,gradientMagnitude);
    cv::Canny(img, canny, 30, 50);
    


    new_frame = new Frame(pin_cam, img.clone());

    new_frame->id_ = frame_nums;
    frame_nums++;

    Updatekf(kf);
    UpdateCamPose();
    
    double t2 = omp_get_wtime();

    pcl_projected->clear();
    projection();
    double t3 = omp_get_wtime();

    if(first_frame){
    }
    else{
        photometricConstraint(kf);
    }
    double t4 = omp_get_wtime();
    time_file<<t4-t3<<" ";
    if(enable_projection){
        double t5 = omp_get_wtime();
        projectionConstraint(kf);
        double t6 = omp_get_wtime();
    }
    
    /*** 3D点约束 ***/
    if(enable_triangulate){
        double t5 = omp_get_wtime();
        if(first_frame){}
        else{
            triangulate(kf);
        }
        double t6 = omp_get_wtime();
    }


    double t_k1 = omp_get_wtime();
    addKeyFrame();
    double t_k2 = omp_get_wtime();


    if(need_keyframe){

        if(first_frame){
        }
        else{
            if(enable_line){
                double t41 = omp_get_wtime();
                projectionLine();
                double t42 = omp_get_wtime();
                lineConstraint(kf);
                double t43 = omp_get_wtime();

            }
        }
    }
    
    double t7 = omp_get_wtime();
    addObservation();
    double t8 = omp_get_wtime();
    delete new_frame;
    if(first_frame){
        first_frame = false;
    }
}
void lmlvil::addKeyFrame(){
    need_keyframe = true;
    update_key_frame_type = 1;
    double min_dis = 100;
    double max_dis = 0;
    int max_id = 0;

    std::list<lvo::FramePtr>::iterator max_it;

    if(!first_frame){

        V3D Pwc = - Rcw.transpose()*Pcw;
        M3D Rwc = Rcw.transpose();
        euler = RotMtoEuler(Rwc);

        for(auto it=map.keyframes_.begin();it!=map.keyframes_.end();it++){
            auto Pwc0 = it->get()->pos();

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
            }
        }

    }   
    if(need_keyframe){
        if(map.keyframes_.size()>window_size){
            remove_frame_num = map.keyframes_.begin()->get()->id_;
            #pragma omp parallel for
            for(auto i = 0;i<map.map_points_.size();i++){
                int index=0;
                FramePtr frame_ptr = map.findFrame(remove_frame_num,index);
                if(map.map_points_[i]->findFrameRef(frame_ptr.get())){
                    #pragma omp critical 
                    {
                        map.map_points_[i]->deleteFrameRef(frame_ptr.get());
                    }
                }
            }
            max_it = map.keyframes_.erase(max_it);
        }

        FramePtr cur_frame;
        cur_frame = std::make_shared<Frame>(pin_cam, img.clone());
        cur_frame->T_f_w_ = new_frame->T_f_w_;
        cur_frame->id_ = frame_nums;
        cur_frame->Cov_ = state->cov.block<6,6>(0,0);
        map.addKeyframe(cur_frame);
    }
}
}


