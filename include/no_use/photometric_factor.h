/*
 * @Author: asdleng lengjianghao2006@163.com
 * @Date: 2023-05-02 14:23:53
 * @LastEditors: asdleng lengjianghao2006@163.com
 * @LastEditTime: 2023-05-16 10:55:48
 * @FilePath: /vio_in_lidar_map/src/vio_in_lidar_map/include/photometric_factor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <common_lib.h>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>
#include <vikit/pinhole_camera.h>
class Photometric_Error
  : public ceres::SizedCostFunction<1 /* number of residuals */,
                             7,7 /* size of parameters */> {
 public:
 Photometric_Error(vk::AbstractCamera* cam_, cv::Mat* img_ptr1_, cv::Mat* img_ptr2_, V3D pt_w_) :
            cam(cam_),img_ptr1(img_ptr1_),img_ptr2(img_ptr2_), pt_w(pt_w_) {
                vk::PinholeCamera* pinhole_ptr = static_cast<vk::PinholeCamera*>(cam);
                fx = pinhole_ptr->fx(); fy = pinhole_ptr->fy();
                cx = pinhole_ptr->cx(); cy = pinhole_ptr->cy();
            };
  virtual ~Photometric_Error() {};

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    double intensity1, intensity2;
    Eigen::Matrix<double, 1, 6> J1, J2;//std::cout<<"1"<<std::endl;
    // 1点
    Eigen::Vector3d p1(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector4d q1(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    calculateJandRes(p1, q1, J1, intensity1,1);//std::cout<<"2"<<std::endl;
    //2 点
    Eigen::Vector3d p2(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector4d q2(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    calculateJandRes(p2, q2, J2, intensity2,2);//std::cout<<"3"<<std::endl;
    //std::cout<<"intensity1: "<<intensity1<<", intensity2"<<intensity2<<std::endl;
    residuals[0] = intensity1 - intensity2;
    if (jacobians != NULL ) {
      if(jacobians[0] != NULL){
        for(int i=0;i<6;i++){
          jacobians[0][i] = J1[i];
        }
        jacobians[0][6] = 0;
      }
      if(jacobians[1] != NULL){
        for(int i=0;i<6;i++){
          jacobians[1][i] = J2[i];
        }
        jacobians[1][6] = 0;
      }
    }
    //std::cout<<"4"<<std::endl;
    return true;
  }
  void dpi(V3D p, MD(2,3)& J) const{
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
void calculateJandRes(const V3D& p, const Eigen::Vector4d& q,
Eigen::Matrix<double, 1 ,6> & J,
double & Intensity,
int img_num
) const
{
      MD(1,2) Jimg;
      MD(2,3) Jdpi;
      MD(1,3) Jdphi, Jdp, JdR, Jdt;
      Eigen::Quaternion<double> q_cw{q[0], q[1], q[2], q[3]};
      Eigen::Matrix<double, 3, 3> R_cw = q_cw.toRotationMatrix();  //ith Rotation
      Eigen::Matrix<double, 3, 1> p_cw{p[0],p[1],p[2]};
      Eigen::Matrix<double, 3, 1> pf = R_cw * pt_w + p_cw;  // 相机坐标系坐标
      Eigen::Matrix<double, 2, 1> pc;
      M3D p_hat;
      const float u_ref = pc[0];  // 投影点
      const float v_ref = pc[1];
      const int u_ref_i = floorf(pc[0]/scale)*scale; 
      const int v_ref_i = floorf(pc[1]/scale)*scale;
      const float subpix_u_ref = (u_ref-u_ref_i)/scale;
      const float subpix_v_ref = (v_ref-v_ref_i)/scale;
      const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
      const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
      const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
      const float w_ref_br = subpix_u_ref * subpix_v_ref;
      //Jdp_dt = Rci * Rwi.transpose(); // 
      dpi(pf, Jdpi);// Jdpi就是偏u/偏q，即投影方程关于相机坐标系下三维点的导数
      p_hat << SKEW_SYM_MATRX(pf);
      int level = 0;
      uint8_t* img_ptr;
      if(img_num==1)
      img_ptr = (uint8_t*) img_ptr1->data + v_ref_i*width + u_ref_i;
      else if(img_num==2)
      img_ptr = (uint8_t*) img_ptr2->data + v_ref_i*width + u_ref_i;
      float du = 0.5f * ((w_ref_tl*img_ptr[scale] + w_ref_tr*img_ptr[scale*2] + w_ref_bl*img_ptr[scale*width+scale] + w_ref_br*img_ptr[scale*width+scale*2])
                                        -(w_ref_tl*img_ptr[-scale] + w_ref_tr*img_ptr[0] + w_ref_bl*img_ptr[scale*width-scale] + w_ref_br*img_ptr[scale*width]));
      float dv = 0.5f * ((w_ref_tl*img_ptr[scale*width] + w_ref_tr*img_ptr[scale+scale*width] + w_ref_bl*img_ptr[width*scale*2] + w_ref_br*img_ptr[width*scale*2+scale])
                                        -(w_ref_tl*img_ptr[-scale*width] + w_ref_tr*img_ptr[-scale*width+scale] + w_ref_bl*img_ptr[0] + w_ref_br*img_ptr[scale]));
      Jimg << du, dv; // Jimg就是偏I/偏u，也就是灰度的梯度，减去相邻的像素，使用双线性插值
      Jimg = Jimg * (1.0/scale);
      Jdphi = Jimg * Jdpi * p_hat;    // 对旋转的雅可比
      Jdp = -Jimg * Jdpi;                 // 对平移的雅可比
      //JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
      //Jdt = Jdp * Jdp_dt;
      Intensity = w_ref_tl*img_ptr[0] + w_ref_tr*img_ptr[scale] + w_ref_bl*img_ptr[scale*width] + w_ref_br*img_ptr[scale*width+scale];
      J << Jdp, Jdphi;
}
    V3D pt_w;
    cv::Mat * img_ptr1;
    cv::Mat * img_ptr2;
    vk::AbstractCamera* cam;
    double fx, fy, cx, cy;
    int scale = 1;
    int patch_size_half = 4;
    int width = 640;
    M3D Jdphi_dR, Jdp_dt, Jdp_dR; 
    M3D Rcl, Rci, Rli, Rcw, Rwi;
    V3D Pcl, Pci, Pli, Pcw; 
};
// struct Photometric_Error {
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     Photometric_Error(vk::AbstractCamera* cam_, cv::Mat* img_ptr1_, cv::Mat* img_ptr2_, V3D pt_w_) :
//             cam(cam_),img_ptr1(img_ptr1_),img_ptr2(img_ptr2_), pt_w(pt_w_) {
//                 vk::PinholeCamera* pinhole_ptr = static_cast<vk::PinholeCamera*>(cam);
//                 fx = pinhole_ptr->fx(); fy = pinhole_ptr->fy();
//                 cx = pinhole_ptr->cx(); cy = pinhole_ptr->cy();
//             }
//     virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const『
//     ;

//     // template<typename T>
//     // bool operator()(double * const p1, double * const q1, double * const p2, double * const q2, T *resi) const {
//     //     int patch_size = 8;
//     //     T patch_size_T = (T) 8;
//     //     int patch_size_half = 4;
//     //     T patch_size_half_T = (T) 4;
//     //     int scale = 1;
//     //     T scale_T = (T) 1;
//     //     int width = 512;
//     //     T width_T = (T) 512;
//     //     Eigen::Quaternion<double> q_cw_1{q1[0], q1[1], q1[2], q1[3]};
//     //     Eigen::Matrix<double, 3, 3> R_cw_1 = q_cw_1.toRotationMatrix();  //ith Rotation
//     //     Eigen::Matrix<double, 3, 1> p_cw_1{p1[0],p1[1],p1[2]};
//     //     Eigen::Quaternion<double> q_cw_2{q2[0], q2[1], q2[2], q2[3]};
//     //     Eigen::Matrix<double, 3, 3> R_cw_2 = q_cw_2.toRotationMatrix();  //jth Rotation
//     //     Eigen::Matrix<double, 3, 1> p_cw_2{p2[0],p2[1],p2[2]};

//     //     //Eigen::Matrix<T, 3, 1> pt_w_T = pt_w.cast<T>();
//     //     // 1点
//     //     Eigen::Matrix<double, 3, 1> pf_1 = R_cw_1 * pt_w + p_cw_1;  // 相机坐标系坐标
//     //     Eigen::Matrix<double, 2, 1> pc_1,uv1;
//     //     uv1[0] = pf_1[0]/ pf_1[2];
//     //     uv1[1] = pf_1[1]/ pf_1[2];
//     //     pc_1[0] = fx*uv1[0] + cx;
//     //     pc_1[1] = fy*uv1[1] + cy;        // 像素坐标
//     //     double u_ref_1 = pc_1[0];  // 投影点
//     //     double v_ref_1 = pc_1[1];
//     //     int u_ref_i_1 = floor(pc_1[0]/scale)*scale; 
//     //     int v_ref_i_1 = floor(pc_1[1]/scale)*scale;
//     //     double subpix_u_ref_1 = (u_ref_1-u_ref_i_1)/scale;
//     //     double subpix_v_ref_1 = (v_ref_1-v_ref_i_1)/scale;
//     //     double w_ref_tl_1 = (1.0-subpix_u_ref_1) * (1.0-subpix_v_ref_1);
//     //     double w_ref_tr_1 = subpix_u_ref_1 * (1.0-subpix_v_ref_1);
//     //     double w_ref_bl_1 = (1.0-subpix_u_ref_1) * subpix_v_ref_1;
//     //     double w_ref_br_1 = subpix_u_ref_1 * subpix_v_ref_1;
//     //     uint8_t* img_ptr_1 = (uint8_t*) img_ptr1->data + (v_ref_i_1-patch_size_half*scale)*width + u_ref_i_1-patch_size_half*scale;
//     //     // // 2点
//     //     // Eigen::Matrix<T, 3, 1> pf_2 = R_cw_2 * pt_w_T + p_cw_2;  // 相机坐标系坐标
//     //     // Eigen::Matrix<double, 2, 1> pc_2,uv2;
//     //     // uv2[0] = pf_2[0]/pf_2[2];
//     //     // uv2[1] = pf_2[1]/pf_2[2];
//     //     // pc_2[0] = fx*uv2[0] + cx;
//     //     // pc_2[1] = fy*uv2[1] + cy;        // 像素坐标
//     //     // double u_ref_2 = pc_2[0];  // 投影点
//     //     // double v_ref_2 = pc_2[1];
//     //     // int u_ref_i_2 = floor(pc_2[0]/scale)*scale; 
//     //     // int v_ref_i_2 = floor(pc_2[1]/scale)*scale;
//     //     // double subpix_u_ref_2 = (u_ref_2-u_ref_i_2)/scale;
//     //     // double subpix_v_ref_2 = (v_ref_2-v_ref_i_2)/scale;
//     //     // T w_ref_tl_2 =  (T)( (1.0-subpix_u_ref_2) * (1.0-subpix_v_ref_2) );
//     //     // T w_ref_tr_2 =  (T)( subpix_u_ref_2 * (1.0-subpix_v_ref_2) );
//     //     // T w_ref_bl_2 =  (T)( (1.0-subpix_u_ref_2) * subpix_v_ref_2 );
//     //     // T w_ref_br_2 =  (T)( subpix_u_ref_2 * subpix_v_ref_2 );
//     //     // uint8_t* img_ptr_2 = (uint8_t*) img_ptr2->data + (v_ref_i_2-patch_size_half*scale)*width + u_ref_i_2-patch_size_half*scale;
//     //     // // 残差
//     //     // T res_i = 
//     //     // w_ref_tl_1*(T)img_ptr_1[0] + w_ref_tr_1*(T)img_ptr_1[scale] + w_ref_bl_1*(T)img_ptr_1[scale*width] + w_ref_br_1*(T)img_ptr_1[scale*width+scale]  
//     //     // - 
//     //     // w_ref_tl_2*(T)img_ptr_2[0] + w_ref_tr_2*(T)img_ptr_2[scale] + w_ref_bl_2*(T)img_ptr_2[scale*width] + w_ref_br_2*(T)img_ptr_2[scale*width+scale];
//     //     // resi[0] = res_i;

//     //     return true;
//     // }

//   bool operator()(const double * x, double* residual) const {
//     const double x1 = (point1_.x);
//     const double y1 = (point1_.y);
//     const double  x2 = (point2_.x);
//     const double  y2 = (point2_.y);
    
//     const double dx = x1 - x2 + (x[0]);
//     const double  dy = y1 - y2 + (x[1]);
    
//     const double distance = ceres::sqrt(dx*dx + dy*dy);
    
//     residual[0] = distance;
    
//     return true;
//   }

//     static ceres::CostFunction *Create(vk::AbstractCamera* cam_,cv::Mat* img_ptr1_,cv::Mat* img_ptr2_, V3D pt_w_) {
//         return (new ceres::AutoDiffCostFunction<Photometric_Error, 1, 2>(
//                 new Photometric_Error(cam_,img_ptr1_,img_ptr2_, pt_w_)));
//     }

//     V3D pt_w;
//     cv::Mat * img_ptr1;
//     cv::Mat * img_ptr2;
//     vk::AbstractCamera* cam;
//     double fx, fy, cx, cy;
//       const cv::Point2i point1_;
//   const cv::Point2i point2_;

// };