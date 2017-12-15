/**
 * @file obstacle_aided_gait2_helical.h
 * @brief 螺旋系の障害物利用推進
 * @author Taichi Akiyama
 * @date 2017/11/23
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_OBSTACL_AIDED_GAIT2_H_
#define SNAKE_CONTROL_SRC_OBSTACL_AIDED_GAIT2_H_

#include <ros/ros.h>
#include <vector>
#include "robot_spec.h"
#include <stdint.h>
#include "snake_feedback.h"


static const double  MIN_INH =/*M_PI/64*/0.0; //補正角度の閾値、これ以下の補正角なら補正しない

class ObstacleAidedGait2 {
 public:

  virtual ~ObstacleAidedGait2(){}

  ObstacleAidedGait2(RobotSpec spec,
		  uint32_t num_ds_link_body,
      double ds
    ) ;

    //関節角度計算
    std::vector<double> CalcJointAngle();

    double CalcKappa(double radius, double pitch);
    double CalcTau(double radius, double pitch);
    void SetDeltaT(double dt);
    void InputOmega(double omega);

    void CalcInh(int i);
    void CalcForceAvereage();
    void LPF4force();

    //形状パラメータ
    double ds_;
    double r_;		//dynamixelの半径(MX_106) */
    double radius_;  //螺旋半径
    double pitch_;   //螺旋ピッチ
    double psi_;   //捻転量
    double dt_;   //微小時間
    double omega_;   //角速度

    int flag_inh;
    double force_average[NUM_COP4SIM]; //2リンク間圧力平均
    double inh_angle[NUM_COP4SIM];  //各関節に与える補正角度
    double k; //補正の比例定数
    double lpf_a; //LPFの係数
    //double pre_psi; //psiの前回値
    int auto_move; //自動推進フラグ

    //CoP情報配列
    double force_scale[NUM_COP4SIM];
    double force_theta[NUM_COP4SIM];
    double force_x[NUM_COP4SIM];

    //過去CoP情報配列
    double pre_force_scale[NUM_COP4SIM];
    double pre_cos_th[NUM_COP4SIM];
    double pre_sin_th[NUM_COP4SIM];

    //三角関数
    double cos_th[NUM_COP4SIM];
    double sin_th[NUM_COP4SIM];



    RobotSpec spec_;

};

#endif
