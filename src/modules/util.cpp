#include "vio_in_lidar_map.h"
namespace lvo{
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

void violm::reduceVector2(vector<V3D> &v, vector<uchar> status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
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
            int adjusted_v = std::max(0, std::min(height - 1, v + int(px[1])));
            int adjusted_u = std::max(0, std::min(width - 1, u + int(px[0])));
            V3D v1 = adj_pts[width*adjusted_v+adjusted_u] - ptw;
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
}