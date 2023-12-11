/*
 * @Author: asdleng lengjianghao2006@163.com
 * @Date: 2023-02-22 13:19:19
 * @LastEditors: asdleng lengjianghao2006@163.com
 * @LastEditTime: 2023-03-31 12:37:39
 * @FilePath: /vio_in_lidar_map/src/vio_in_lidar_map/include/use-ikfom.hpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2;
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;
// 构建状态(位置、姿态、外参、速度、零偏、重力向量)
MTK_BUILD_MANIFOLD(state_ikfom,
				   ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))((vect3, offset_T_L_I))((vect3, vel))((vect3, bg))((vect3, ba))((vect3, grav)));
// 构建输入(加速度、角速度)
MTK_BUILD_MANIFOLD(input_ikfom,
				   ((vect3, acc))((vect3, gyro)));
// 构建噪声(角速度噪声、加速度噪声、角速度零偏噪声、加速度零偏噪声)
MTK_BUILD_MANIFOLD(process_noise_ikfom,
				   ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

MTK::get_cov<process_noise_ikfom>::type process_noise_cov();

Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in);

Eigen::Matrix<double, 24, 24> df_dx(state_ikfom &s, const input_ikfom &in);

Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in);

vect3 SO3ToEuler(const SO3 &orient);


#endif