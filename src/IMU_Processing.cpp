#include "IMU_Processing.hpp"
M3D Eye3d = M3D::Identity();
M3F Eye3f = M3F::Identity();
V3D Zero3d = V3D::Zero();
V3F Zero3f = V3F::Zero();
ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_acc = V3D(0.1, 0.1, 0.1);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}
void ImuProcess::Reset()
{
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}
void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
  Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}
void ImuProcess::IMU_init(const std::deque<sensor_msgs::Imu::ConstPtr> &imu_buffer, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{


  V3D cur_acc, cur_gyr;

  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = imu_buffer.front()->linear_acceleration;
    const auto &gyr_acc = imu_buffer.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

  }


  for (const auto &imu : imu_buffer)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N); // 线加速度协方差
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N); // 重力加速度协方差

    N++;
  }

  state_ikfom init_state = kf_state.get_x();
  if(!give_init){
    if(normalized)
        init_state.grav = - mean_acc / mean_acc.norm() * G_m_s2;
      else
        init_state.grav = -mean_acc;
        init_state.grav = init_state.rot*init_state.grav;
      ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
                  init_state.grav[0], init_state.grav[1], init_state.grav[2], mean_gyr[0], mean_gyr[1],mean_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
          cov_acc = cov_acc.cwiseProduct(cov_acc_scale);
          cov_gyr = cov_gyr.cwiseProduct(cov_gyr_scale);

      init_state.bg = mean_gyr;
  }
  
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.001; // 旋转外参
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.001; // 平移外参
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.1;  // bg
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.1; // ba
  init_P(21, 21) = init_P(22, 22) = init_P(23, 23) = 0.1;  //g
  kf_state.change_P(init_P);
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr*gyr_Q_scale;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc*acc_Q_scale;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr*gyr_Q_scale;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc*acc_Q_scale;
}

void ImuProcess::intergrate(sensor_msgs::Imu imu_1, sensor_msgs::Imu imu_2,
esekfom::esekf<state_ikfom, 12, input_ikfom> &kf, double img_time){
    double dt;
    if(img_time<imu_2.header.stamp.toSec()){
      dt = img_time-imu_1.header.stamp.toSec();
    }
    else{
      dt = imu_2.header.stamp.toSec()-imu_1.header.stamp.toSec();
    }
     

    input_ikfom in;
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;
    angvel_avr << 0.5 * (imu_1.angular_velocity.x + imu_2.angular_velocity.x),
        0.5 * (imu_1.angular_velocity.y + imu_2.angular_velocity.y),
        0.5 * (imu_1.angular_velocity.z + imu_2.angular_velocity.z);
    acc_avr << 0.5 * (imu_1.linear_acceleration.x + imu_2.linear_acceleration.x),
        0.5 * (imu_1.linear_acceleration.y + imu_2.linear_acceleration.y),
        0.5 * (imu_1.linear_acceleration.z + imu_2.linear_acceleration.z);
    if(normalized)
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    kf.predict(dt, Q, in);
}
void ImuProcess::intergrate2(sensor_msgs::Imu imu_1, sensor_msgs::Imu imu_2,
esekfom::esekf<state_ikfom, 12, input_ikfom> &kf, double img_time){
    double dt;
    if(imu_1.header.stamp.toSec()==0)
      return;
    if(img_time<imu_2.header.stamp.toSec()&&img_time>imu_1.header.stamp.toSec()){
      dt = imu_2.header.stamp.toSec() - img_time;
    }
    else{
      return;
    }
    
    input_ikfom in;
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;
    angvel_avr << 0.5 * (imu_1.angular_velocity.x + imu_2.angular_velocity.x),
        0.5 * (imu_1.angular_velocity.y + imu_2.angular_velocity.y),
        0.5 * (imu_1.angular_velocity.z + imu_2.angular_velocity.z);
    acc_avr << 0.5 * (imu_1.linear_acceleration.x + imu_2.linear_acceleration.x),
        0.5 * (imu_1.linear_acceleration.y + imu_2.linear_acceleration.y),
        0.5 * (imu_1.linear_acceleration.z + imu_2.linear_acceleration.z);
    if(normalized)
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    kf.predict(dt, Q, in);
}
