#include "lmlvil.h"
namespace lvo{
void lmlvil::setState(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    state->rot_end = kf.get_x().rot.toRotationMatrix();
    state->pos_end = kf.get_x().pos;
    state->vel_end = kf.get_x().vel;
    state->bias_g = kf.get_x().bg;
    state->bias_a = kf.get_x().ba;
    state->gravity = kf.get_x().grav;
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P;
    state->cov = kf.get_P();
}
void lmlvil::setStatePropagate(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    state_propagat->rot_end = kf.get_x().rot.toRotationMatrix();
    state_propagat->pos_end = kf.get_x().pos;
    state_propagat->vel_end = kf.get_x().vel;
    state_propagat->bias_g = kf.get_x().bg;
    state_propagat->bias_a = kf.get_x().ba;
    state_propagat->gravity = kf.get_x().grav;
    state_propagat->cov = kf.get_P();
}
void lmlvil::setKF(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    state_ikfom state_point;
    state_point.pos = state->pos_end;
    state_point.rot = state->rot_end;
    state_point.vel = state->vel_end;
    state_point.ba = state->bias_a;
    state_point.bg = state->bias_g;
    state_point.grav = state->gravity;
    
    kf.change_x(state_point);
    kf.change_P(state->cov);
}
void lmlvil::updateFrameState()
{
    Rwi = state->rot_end;
    Pwi = state->pos_end;
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;
    new_frame->T_f_w_ = Sophus::SE3(Rcw, Pcw);
}
void lmlvil::Updatekf(const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf){
    Rwi = kf.get_x().rot.toRotationMatrix();
    Pwi = kf.get_x().pos;
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;  
}
void lmlvil::UpdateCamPose(){
    new_frame->T_f_w_ = Sophus::SE3(Rcw, Pcw);
}

void lmlvil::reduceVector2(vector<V3D> &v, vector<uchar> status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void lmlvil::localmapGenerate(){
    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    local_map->clear();
    local_map->points = ikdtree.PCL_Storage;
}
bool lmlvil::depthContinue(const V2D &px,const double & depth0){
    bool depth_continous = false;
    
    for (int u=-depth_search_radius; u<=depth_search_radius; u++){
        for (int v=-depth_search_radius; v<=depth_search_radius; v++){
            if(u==0 && v==0) continue;
            float depth = depth_img[width*(v+int(px[1]))+u+int(px[0])];
            if(depth == 0.) continue;
            double delta = depth - depth0;
            if(delta>delta_dist_thre) {
                depth_continous = true;
                break;
            }
        }
        if(depth_continous) break;
    }
    return depth_continous;
}
bool lmlvil::depthContinue2(const V2D &px,const double & depth0, const V3D &ptw){
    bool depth_continous = false;
    for (int u=-depth_search_radius; u<=depth_search_radius; u++){
        for (int v=-depth_search_radius; v<=depth_search_radius; v++){
            if(u==0 && v==0) continue;
            float depth = depth_img[width*(v+int(px[1]))+u+int(px[0])];
            if(depth == 0.) continue;
            double delta = depth - depth0;
            if(delta>0) continue;
            int adjusted_v = std::max(0, std::min(height - 1, v + int(px[1])));
            int adjusted_u = std::max(0, std::min(width - 1, u + int(px[0])));
            V3D v1 = adj_pts[width*adjusted_v+adjusted_u] - ptw;
            V3D v2 =  new_frame->pos() - ptw;
            double theta = v1.dot(v2)/v1.norm()/v2.norm();
            if(theta>theta_thre){
                depth_continous = true;
                break;
            }
        }
        if(depth_continous) break;
    }
    return depth_continous;
    bool edge_continous = depthContinue(px,depth0);
    return depth_continous||edge_continous;
}

void lmlvil::reset_grid()
{
    for(int i=0;i<grid.cells.size();i++){
        if(grid.cells[i]->size()>0)
            grid.cells[i]->clear();
    }
    memset(pts_num_in_each_cell, 0, sizeof(int)*length);
}

void lmlvil::dpi(V3D p, MD(2,3)& J) {
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
double lmlvil::NCC(float* ref_patch, float* cur_patch, int patch_size)
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
void lmlvil::getWarpMatrixAffine(
    const vk::AbstractCamera& cam,  
    const Vector2d& px_ref, 
        
    const Vector3d& f_ref,  
    const double depth_ref, 
    const Sophus::SE3& T_cur_ref,  
    const int level_ref,    
    const int pyramid_level,
    const int halfpatch_size,
    Matrix2d& A_cur_ref)
{

  const Vector3d xyz_ref(f_ref*depth_ref);  
  Vector3d xyz_du_ref(cam.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)*(1<<pyramid_level)));
  Vector3d xyz_dv_ref(cam.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)*(1<<pyramid_level)));
  xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];
  xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];
  const Vector2d px_cur(cam.world2cam(T_cur_ref*(xyz_ref)));
  const Vector2d px_du(cam.world2cam(T_cur_ref*(xyz_du_ref)));
  const Vector2d px_dv(cam.world2cam(T_cur_ref*(xyz_dv_ref)));
  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
}
void lmlvil::warpAffine(
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
  for (int y=0; y<patch_size; ++y)
  {
    for (int x=0; x<patch_size; ++x)
    {
      Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      px_patch *= (1<<search_level);
      px_patch *= (1<<pyramid_level);
      const Vector2f px(A_ref_cur*px_patch + px_ref.cast<float>());
      if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
        patch[patch_size_total*pyramid_level + y*patch_size+x] = 0;

      else
        patch[patch_size_total*pyramid_level + y*patch_size+x] = (float) vk::interpolateMat_8u(img_ref, px[0], px[1]);
    }
  }
}
void lmlvil::getpatch(cv::Mat img, V2D pc, float* patch_tmp, int level) 
{
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
    for (int x=0; x<patch_size; x++) 
    {   
        uint8_t* img_ptr = (uint8_t*) img.data + (v_ref_i-patch_size_half*scale+x*scale)*width + (u_ref_i-patch_size_half*scale);
        for (int y=0; y<patch_size; y++, img_ptr+=scale)
        {   
            patch_tmp[patch_size_total*level+x*patch_size+y] = w_ref_tl*img_ptr[0] + w_ref_tr*img_ptr[scale] + w_ref_bl*img_ptr[scale*width] + w_ref_br*img_ptr[scale*width+scale];
        }
    }
}
bool lmlvil::isPointEdgelet(float& grad,float& gradx,float& grady, int x, int y, float edge_threshold) {

    if (x <= 0 || x >= img.cols - 1 || y <= 0 || y >= img.rows - 1) {
        return false;
    }


    grad = gradientMagnitude.at<float>(y, x);
    gradx = gradientX.at<float>(y, x);
    grady = gradientY.at<float>(y, x);

    if (grad < edge_threshold) {
        return false;
    }

    if (canny.at<uchar>(y, x) == 0) {
        return false;
    }
    

    return true;
}


}