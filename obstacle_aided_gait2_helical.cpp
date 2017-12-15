/**
 * @file obstacle_aided_gait2_helical.cpp
 * @brief 螺旋系の障害物利用推進
 * @author Taichi Akiyama
 * @date 2017/11/23
 * @detail
 */

#include <ros/ros.h>
#include <cmath>
#include "obstacle_aided_gait2_helical.h"
#include "snake_control.h"
#include "snake_feedback.h"

/** @fn
 * @brief コンストラクタ
 * @param いろいろ
 * @return
 * @detail
 */
ObstacleAidedGait2::ObstacleAidedGait2(RobotSpec spec,
     uint32_t num_ds_link_body,
     double ds
     )
     {

       spec_ = spec;
       ds_ = ds;
       r_ = spec.link_diameter();
       psi_ = 0.0;
       radius_ = 0.055;
       pitch_ = 0.5;

       flag_inh = 0;
       k=0.0;
       lpf_a = 0.9;
       //pre_psi = 0.0;
       auto_move = 0;

 }

 /** @fn
  * @brief 目標関節角を計算する
  * @param なし
  * @return 目標関節角度配列 std::vector<double>
  * @detail
  */
std::vector<double> ObstacleAidedGait2::CalcJointAngle(){

  std::vector<double> target_joint_angle(spec_.num_joint(), 0.0);
  #ifdef USE_SIM_COPX
  SnakeFeedback::ReturnSimCop(force_scale, force_theta, force_x);
  #else
  ROS_INFO("Please check the #deifine USE_SIM_COPX");
  #endif

  std::vector<double> pre_joint_angle(spec_.num_joint(), 0.0);
  pre_joint_angle = SnakeFeedback::ReturnCurrentAngle();
  target_joint_angle = pre_joint_angle;

  //ROS_INFO("radius = %f, pitch = %f", radius_, pitch_);
  //曲率計算
  double b = pitch_/(2*M_PI);
  double kappa = CalcKappa(radius_, b);
  //捩率計算
  double tau = CalcTau(radius_, b);
  double s = 0.0;

  //K自動更新
  //k = SnakeFeedback::ReturnAutoParam();

  ROS_INFO("flag_inh = %d", flag_inh);
  ROS_INFO("k = %f", k);


  //yawにおける前後2関節の圧力平均を算出
  if (flag_inh == 1 ){
    CalcForceAvereage();
  }

  //LPFをかける
  LPF4force();

  for (uint32_t i_joint=0; i_joint<spec_.num_joint(); i_joint++) {

    //曲線長さ
    s = ds_ * (i_joint + 1);

    if ( i_joint%2 == 0 ) { //yaw関節
      //螺旋分
      target_joint_angle[i_joint] = 2 * ds_ * kappa * (-std::sin(tau * s + psi_));
      if (flag_inh == 0){
        //target_joint_angle[i_joint] = 2 * ds_ * kappa * (-std::sin(tau * s + psi_));
        //ROS_INFO("target_joint_angle[%d] = %f", i_joint, target_joint_angle[i_joint]);
      }else if (flag_inh == 1){
        CalcInh(i_joint); //側抑制補正角計算
        if (inh_angle[i_joint] > MIN_INH || inh_angle[i_joint] < - MIN_INH){
          target_joint_angle[i_joint] += inh_angle[i_joint];
        }
      }

    }else{
      target_joint_angle[i_joint] = 2 * ds_ * kappa * (std::cos(tau * s + psi_));
      if (flag_inh == 0){
        //target_joint_angle[i_joint] = 2 * ds_ * kappa * (-std::sin(tau * s + psi_));
      }else if (flag_inh == 1){
        CalcInh(i_joint);
        if (inh_angle[i_joint] > MIN_INH || inh_angle[i_joint] < - MIN_INH){
          target_joint_angle[i_joint] += inh_angle[i_joint];
        }
      }
    }

    //CoPデータ保存
    pre_force_scale[i_joint] = force_scale[i_joint];
    pre_cos_th[i_joint] = cos_th[i_joint];
    pre_sin_th[i_joint] = sin_th[i_joint];

  }

  return target_joint_angle;

}

/** @fn
 * @brief 曲率計算
 * @param 半径、ピッチ
 * @return 曲率
 * @detail
 */
double ObstacleAidedGait2::CalcKappa(double radius, double pitch) {
  double kappa = radius/(pow(radius,2) + pow(pitch,2));
  return kappa;
}

/** @fn
 * @brief 捩率計算
 * @param 半径、ピッチ
 * @return 捩率
 * @detail
 */
double ObstacleAidedGait2::CalcTau(double radius, double pitch) {
  double tau = pitch/(pow(radius,2) + pow(pitch,2));
  return tau;
}

/** @fn
 * @brief サンプリングタイム設定
 * @param サンプリングタイム
 * @return なし
 * @detail
 */
void ObstacleAidedGait2::SetDeltaT(double dt){
  dt_ = dt;
}

/** @fn
 * @brief 角速度入力
 * @param 角速度
 * @return なし
 * @detail psi_を更新
 */
void ObstacleAidedGait2::InputOmega(double omega){
  omega_ = omega;
  psi_ += (omega_ * dt_);
}

/** @fn
 * @brief 反射的な動作分の角度補正を付加する
 * @param 目標関節角度配列、基準ジョイント番号、補正角度
 * @return なし
 * @detail
 */
void ObstacleAidedGait2::CalcInh(int i){
    //頭と尻尾の処理が問題？
    if ( (i - 2) <= 0 ){
      inh_angle[i] = k*(force_average[i] - force_average[i+2]/2)*dt_;
    }else if ( (i + 2) >= LAST_YAW_JOINT) {
      inh_angle[i] = k*(force_average[i] - force_average[i-2]/2)*dt_;
    }else{
      inh_angle[i] = k*(force_average[i] - force_average[i-2]/2 - force_average[i+2]/2)*dt_;
    }
}

/** @fn
 * @brief 2リンク間の圧力平均を算出
 * @param なし
 * @return なし
 * @detail 出力はforce_averageに格納
 */
void ObstacleAidedGait2::CalcForceAvereage(){
  for  (int i=0; i<NUM_COP4SIM; i++){
    if ( i%2 == 0 ) { //yaw関節
      if (i+1 <= LAST_PITCH_JOINT){
        force_average[i] = (force_scale[i]*cos_th[i]+force_scale[i+1]*cos_th[i+1]);
      }
    }else{
      if (i+1 <= LAST_PITCH_JOINT){
        force_average[i] = (force_scale[i]*sin_th[i]+force_scale[i+1]*sin_th[i+1]);
      }else{
        force_average[i] = force_scale[i]*sin_th[i];
      }
    }
  }
}

/** @fn
 * @brief force_scaleとforce_thetaをLPF処理
 * @param なし
 * @return なし
 * @detail センサ値を更新
 */
void ObstacleAidedGait2::LPF4force(){
  for  (int i=0; i<NUM_COP4SIM; i++){
    force_scale[i] = lpf_a*pre_force_scale[i] + (1-lpf_a)*force_scale[i];
    cos_th[i] = lpf_a*pre_cos_th[i] + (1-lpf_a)*std::cos(force_theta[i]);
    sin_th[i] = lpf_a*pre_sin_th[i] + (1-lpf_a)*std::sin(force_theta[i]);
  }
}
