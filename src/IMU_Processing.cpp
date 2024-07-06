#include "IMU_Processing.hpp"
M3D Eye3d = M3D::Identity();
M3F Eye3f = M3F::Identity();
V3D Zero3d = V3D::Zero();
V3F Zero3f = V3F::Zero();
/*构造函数，设定参数*/
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
/*重置状态到初始状态*/
void ImuProcess::Reset()
{
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  //IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}
/*赋值函数*/
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
/*imu初始化，输入组合测量数据，输出状态、imu个数*/
void ImuProcess::IMU_init(const std::deque<sensor_msgs::Imu::ConstPtr> &imu_buffer, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;
  // 如果是第一帧，就赋值mean_acc、mean_gyr，这里为了算平均加速度和角速度

  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = imu_buffer.front()->linear_acceleration;
    const auto &gyr_acc = imu_buffer.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    // first_lidar_time = meas.lidar_beg_time;
    // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
  }

  // 遍历imu队列，计算平均加速度、角速度
  for (const auto &imu : imu_buffer)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;
    // std::cout<<"imu_acc: "<<imu_acc.x<<","<<imu_acc.y<<","<<imu_acc.z<<std::endl;
    // std::cout<<"mean_acc: "<<mean_acc.transpose()<<std::endl;
    // cwiseProduct：输出相同位置的两个矩阵中各个系数的乘积所组成的矩阵（类似matlab.*）
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
      //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));

      init_state.bg = mean_gyr;
  }
  
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);
  // 协方差 全是自己定的
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
    // std::cout<<"acc:"<<acc_avr.transpose()<<std::endl;
    //std::cout<<"gyro:"<<angvel_avr.transpose()<<std::endl;
    //std::cout<<"dt:"<<dt<<std::endl;
    // std::cout<<"Q:"<<Q<<std::endl;
    kf.predict(dt, Q, in);

    //std::cout<<"kf_pos:"<<kf.get_x().pos<<std::endl;
    //std::cout<<"kf_rot:"<<kf.get_x().rot<<std::endl;
    // std::cout<<"kf_P:"<<kf.get_P()<<std::endl;
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
    //std::cout<<"acc:"<<acc_avr.transpose()<<std::endl;
    //std::cout<<"gyro:"<<angvel_avr.transpose()<<std::endl;
    //std::cout<<"dt:"<<dt<<std::endl;
    // std::cout<<"Q:"<<Q<<std::endl;
    kf.predict(dt, Q, in);

    //std::cout<<"kf_pos:"<<kf.get_x().pos<<std::endl;
    //std::cout<<"kf_rot:"<<kf.get_x().rot<<std::endl;
    // std::cout<<"kf_P:"<<kf.get_P()<<std::endl;
}





// // 点云运动补偿，假设第一个点为基准，使用IMU旋转补偿激光点云
// void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
// {
//   /*
//     IMU预积分
//     把堆积的IMU数据进行预积分处理，推导出每个IMU数据时刻相对于全局坐标系的T，以及对应的方差。
//     每次开始之前，都会把上一次预积分的最后一个IMU加入到本次IMU队列的头部，以及对应的姿态结果作为此次预积分的开始姿态。
//     这种推理会导致IMU推理的轨迹飞的很快，但是不影响我们总的结果，因为我们取的是短时间内的相对运动姿态。
//     这里IMU的预积分使用中值积分。
//   */

//   /* 数据准备 */

//   /*** add the imu of the last frame-tail to the of current frame-head ***/
//   auto v_imu = meas.imu;
//   v_imu.push_front(last_imu_); // 让IMU的时间能包住Lidar的时间
//   const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
//   const double &imu_end_time = v_imu.back()->header.stamp.toSec();
//   const double &pcl_beg_time = meas.lidar_beg_time;
//   const double &pcl_end_time = meas.lidar_end_time;

//   /*** sort point clouds by offset time ***/
//   pcl_out = *(meas.lidar);
//   sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

//   /*** Initialize IMU pose ***/
//   state_ikfom imu_state = kf_state.get_x();
//   IMUpose.clear();
//   IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

//   /* 预积分过程 */

//   /*** forward propagation at each imu point ***/
//   V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
//   M3D R_imu;

//   double dt = 0;

//   input_ikfom in;
//   for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
//   {
//     // 中值积分
//     auto &&head = *(it_imu);
//     auto &&tail = *(it_imu + 1);

//     if (tail->header.stamp.toSec() < last_lidar_end_time_)
//       continue;

//     angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
//         0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
//         0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
//     acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
//         0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
//         0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

//     acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

//     if (head->header.stamp.toSec() < last_lidar_end_time_)
//     {
//       dt = tail->header.stamp.toSec() - last_lidar_end_time_;
//     }
//     else
//     {
//       dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
//     }

//     in.acc = acc_avr;
//     in.gyro = angvel_avr;
//     Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
//     Q.block<3, 3>(3, 3).diagonal() = cov_acc;
//     Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
//     Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
//     // auto a = kf_state.get_x();
//     // cout<<a.ba.transpose()<<endl;
//     kf_state.predict(dt, Q, in);
//     // auto b = kf_state.get_x();
//     // cout<<b.ba.transpose()<<endl;
//     /* save the poses at each IMU measurements */
//     imu_state = kf_state.get_x();
//     angvel_last = angvel_avr - imu_state.bg;
//     acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
//     for (int i = 0; i < 3; i++)
//     {
//       acc_s_last[i] += imu_state.grav[i];
//     }

//     double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
//     //cout<<offs_t<<endl;
//     IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
//   }

//   // PCL的最后一个点的时间和IMU的最后一个时间往往不是精准对齐的，这里根据最后一个IMU的姿态，计算了最后一个点云的姿态，这个是我们其他所有其他时刻的点云要对准的坐标系

//   /*** calculated the pos and attitude prediction at the frame-end ***/
//   double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
//   dt = note * (pcl_end_time - imu_end_time);
//   kf_state.predict(dt, Q, in);
  
//   imu_state = kf_state.get_x();
//   last_imu_ = meas.imu.back();
//   last_lidar_end_time_ = pcl_end_time;

//   /* 点云补偿 */

//   /*** undistort each lidar point (backward propagation) ***/
//   auto it_pcl = pcl_out.points.end() - 1;
//   for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
//   {
//     auto head = it_kp - 1;
//     auto tail = it_kp;
//     R_imu << MAT_FROM_ARRAY(head->rot);
//     vel_imu << VEC_FROM_ARRAY(head->vel);
//     pos_imu << VEC_FROM_ARRAY(head->pos);
//     acc_imu << VEC_FROM_ARRAY(tail->acc);
//     angvel_avr << VEC_FROM_ARRAY(tail->gyr);

//     for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
//     {
//       dt = it_pcl->curvature / double(1000) - head->offset_time;

//       /* Transform to the 'end' frame, using only the rotation
//        * Note: Compensation direction is INVERSE of Frame's moving direction
//        * So if we want to compensate a point at timestamp-i to the frame-e
//        * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
//       M3D R_i(R_imu * Exp(angvel_avr, dt));
//       // T_ei表示i时刻的lidar坐标原点到end时刻的坐标平移量，用的全局坐标系的衡量
//       // T_ei=Ti-Te
//       // 为什么是Ti-Te，在global坐标系下原点为o，Ti是向量oi,Te是向量oe,
//       // Ti-Te就是向量oi-oe，得到的是向量ei,
//       // 即以e指向i的向量，以e为原点的向量
//       // end时刻已经计算出来了就是pos_liD_e
//       // i时刻的坐标计算方式与pos_liD_e一样
//       // pos_liD_i=Pos_i+rot_i*Lidar_offset_to_IMU
//       // 而Pos_i=Pos_head+velocity_head*dt+0.5*acc_head*dt*dt

//       // 本质上是将P_i转换到全局坐标系，再转换到end的坐标系下
//       V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
//       V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
//       V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I); // not accurate!

//       // save Undistorted points and their rotation
//       it_pcl->x = P_compensate(0);
//       it_pcl->y = P_compensate(1);
//       it_pcl->z = P_compensate(2);

//       if (it_pcl == pcl_out.points.begin())
//         break;
//     }
//   }
// }

// void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state)
// {
//   double t1, t2, t3;
//   t1 = omp_get_wtime();

//   if (meas.imu.empty())
//   {
//     return;
//   }

//   ROS_ASSERT(meas.lidar != nullptr);

//   if (imu_need_init_)
//   {
//     // The very first lidar frame
//     IMU_init(meas, kf_state, init_iter_num);

//     imu_need_init_ = true;

//     last_imu_ = meas.imu.back();

//     state_ikfom imu_state = kf_state.get_x();
//     if (init_iter_num > MAX_INI_COUNT)
//     {
//       cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
//       imu_need_init_ = false;

//       cov_acc = cov_acc_scale;
//       cov_gyr = cov_gyr_scale;
//       ROS_INFO("IMU Initial Done");
//       fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
//     }

//     return;
//   }

//   // Undistort points：the first point is assummed as the base frame
//   // Compensate lidar points with IMU rotation (with only rotation now)
//   UndistortPcl(meas, kf_state, *cur_pcl_un_);

//   t2 = omp_get_wtime();
//   t3 = omp_get_wtime();
// }