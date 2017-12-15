/**
 * @file snake_feedback.cpp
 * @brief ヘビ型ロボットのセンサ情報などを扱うクラス
 * @author Taichi Akiyama
 * @date 2017/09/27
 * @detail
 */

#include <ros/ros.h>
#include <cmath>
#include "snake_feedback.h"

//Subscriber
ros::Subscriber SnakeFeedback::current_angle_sub;
ros::Subscriber SnakeFeedback::current_sub;
ros::Subscriber SnakeFeedback::force_sub;

ros::Subscriber SnakeFeedback::collision_position_sub;

ros::Subscriber SnakeFeedback::sub_auto_param;

//衝突防止のために名前空間定義
namespace SnakeFeedbsckMembers {

  //現在関節角度関係
  ros::Time current_angle_timestamp[NUM_JOINT];
  double current_angle[NUM_JOINT];

  //電流値関係
  ros::Time current_timestamp[NUM_JOINT];
  double current[NUM_JOINT];

  //実際のロボットのCoP
  ros::Time force_timestamp[NUM_JOINT][COP_ID];
  double force_scale[NUM_JOINT][COP_ID];
  double force_x[NUM_JOINT][COP_ID];
  double force_y[NUM_JOINT][COP_ID];
  double force_z[NUM_JOINT][COP_ID];
  double force_theta[NUM_JOINT][COP_ID];

  //シミュレータ上でのCoPセンサ関係
  ros::Time force_sim_data_timestamp;
  double force_sim_scale[NUM_COP4SIM];
  double force_sim_theta[NUM_COP4SIM];
  double force_sim_x[NUM_COP4SIM];

  //シミュレータ自動化パラメータ
  double auto_k;  //obstacle_aided_gait2_helicalの比例定数K

}

using namespace SnakeFeedbsckMembers ;

/** @fn
 * @brief イニシャライズ
 * @param なし
 * @return なし
 * @detail
 */
void SnakeFeedback::Initialize() {

  ros::NodeHandle node_handle;

  #ifdef CURRENT_ANGLE

    // 配列初期化
    for (int i = 0; i < NUM_JOINT; i++) {
      current_angle_timestamp[i] = ros::Time::now();
      current_angle[i] = 0;
    }
    current_angle_sub = node_handle.subscribe<snake_msgs::SnakeJointData>("joint_position", 100, &SnakeFeedback::CallBackOfJointCurrentAngleData);

  #endif

  #ifdef CURRENT

    // 配列初期化
    for (int i = 0; i < NUM_JOINT; i++) {
      current_timestamp[i] = ros::Time::now();
      current[i] = 0;
    }
    current_sub = node_handle.subscribe<snake_msgs::SnakeJointData>("joint_current", 100, &SnakeFeedback::CallBackOfJointCurrentData);

  #endif

  #ifdef COP

    // 配列初期化
    for (int i = 0; i < NUM_JOINT; i++) {
      for (int j = 0; j < COP_ID; j++){
        force_timestamp[i][j] = ros::Time::now();
        force_scale[i][j] = 0;
        force_x[i][j] = 0;
        force_y[i][j] = 0;
        force_z[i][j] = 0;
        force_theta[i][j] = 0;
      }
    }
    current_sub = node_handle.subscribe<snake_msgs::SnakeJointData>("joint_current", 100, &SnakeFeedback::CallBackOfJointCurrentData);

  #endif

  #ifdef SIM_COP

    // 配列初期化
    force_sim_data_timestamp = ros::Time::now();
    for (int i = 0; i < NUM_COP4SIM; i++) {
      force_sim_scale[i] = 0;
      force_sim_theta[i] = 0;
      force_sim_x[i] = 0;
    }
    #ifdef USE_SIM_COPX
    collision_position_sub = node_handle.subscribe<snake_msgs_abe::FsensorDataPlusx>("collision_position", 100, &SnakeFeedback::CallBackOfSimCopData);
    #else
    collision_position_sub = node_handle.subscribe<snake_msgs_abe::FsensorData>("collision_position", 100, &SnakeFeedback::CallBackOfSimCopData);
    #endif

  #endif

  #ifdef SIM_AUTO_PARAM
    auto_k = 0;
    sub_auto_param = node_handle.subscribe<std_msgs::Float64MultiArray>("auto_change_param", 100, &SnakeFeedback::CallBackOfAutoParam);
  #endif
}


/** 現在関節角度関係 **/

/** @fn
 * @brief 現在関節角度のコールバック関数
 * @param snake_msgs::SnakeJointData
 * @return なし
 * @detail
 */
void SnakeFeedback::CallBackOfJointCurrentAngleData(const snake_msgs::SnakeJointData::ConstPtr& current_angle_datain) {

  int joint_index = current_angle_datain->joint_index;
  if (joint_index > NUM_JOINT-1) return; // メモリ破壊回避

  current_angle_timestamp[joint_index] = ros::Time::now();

  if (current_angle_datain->value < 65000)
    current_angle[joint_index] = (current_angle_datain->value)*2*M_PI/360;
  else current_angle[joint_index] = 0;
}

/** @fn
 * @brief 現在関節角度のリターン
 * @param なし
 * @return std::vector<double>
 * @detail
 */
std::vector<double> SnakeFeedback::ReturnCurrentAngle() {

  std::vector<double> current_angle_vector(NUM_JOINT, 0.0);
  for (int i=0 ; i<NUM_JOINT ; i++){
    current_angle_vector[i] = current_angle[i];
  }
  //ROS_INFO("double = %f, vector=%f", current_angle[5], current_angle_vector[5]);
  return current_angle_vector;
}

/** 電流値関係 **/

/** @fn
 * @brief 電流値のコールバック関数
 * @param snake_msgs::SnakeJointData
 * @return なし
 * @detail
 */
void SnakeFeedback::CallBackOfJointCurrentData(const snake_msgs::SnakeJointData::ConstPtr& current_datain) {

  int joint_index = current_datain->joint_index;
  if (joint_index > NUM_JOINT-1) return; // メモリ破壊回避

  current_timestamp[joint_index] = ros::Time::now();

  if (current_datain->value < 65000) 
    current[joint_index] = fabs(current_datain->value); //絶対値で
  else current[joint_index] = 0;
}

/** @fn
 * @brief 電流値のリターン
 * @param double[]
 * @return なし
 * @detail
 */
void SnakeFeedback::ReturnCurrent(double arg_current[NUM_JOINT]) {

  for (int i = 0; i < NUM_JOINT; i++) {
    arg_current[i] = current[i];
  }
}

/** @fn
 * @brief 実際のロボットのCoPセンサ情報のコールバック
 * @param snake_msgs::SnakeForceData
 * @return なし
 * @detail
 */
void SnakeFeedback::CallBackOfForceData(const snake_msgs::SnakeForceData::ConstPtr& force_datain) {

  int i = force_datain->joint_index;
  int j = force_datain->cop_index;

  force_timestamp[i][j] = ros::Time::now();
  force_scale[i][j] = force_datain->iall;
  force_x[i][j] = force_datain->force_x;
  force_y[i][j] = force_datain->force_y;
  force_z[i][j] = force_datain->force_z;
  force_theta[i][j] = force_datain->theta;

}

/** @fn
 * @brief 実際のロボットのCoPセンサ情報のリータン
 * @param scale (double), theta (double)
 * @return
 * @detail
 */
void SnakeFeedback::ReturnForce(double scale[NUM_JOINT][COP_ID], double theta[NUM_JOINT][COP_ID]){

  for(int i=0; i<NUM_JOINT; i++){
    for(int j=0; j<NUM_JOINT; j++){
      scale[i][j] = force_scale[i][j];
      theta[i][j] = force_theta[i][j];
    }
  }
}

/** 阿部さんのシミュレータ上での力センサ関係 **/
//CoPセンサのx軸方向の力を使うかどうかで分岐（x軸方向を使う場合はV-REPファイルから変更の必要あり）
#ifdef USE_SIM_COPX

/** @fn
 * @brief シミュレータ上でのCoPセンサ情報のコールバック
 * @param snake_msgs_abe::FsensorDataPlusx
 * @return なし
 * @detail
 */
void SnakeFeedback::CallBackOfSimCopData(const snake_msgs_abe::FsensorDataPlusx::ConstPtr& collision_datain) {

  force_sim_data_timestamp = ros::Time::now();

  for (int i = 0; i < NUM_COP4SIM; i++) {
    force_sim_scale[i] = collision_datain->scale[i];
    force_sim_theta[i] = collision_datain->angle[i];
    force_sim_x[i] = collision_datain->forcex[i];
  }
}

/** @fn
 * @brief シミュレータ上でのCoPセンサ情報のリータン
 * @param double[], double[], double[]
 * @return なし
 * @detail
 */
void SnakeFeedback::ReturnSimCop(double scale[NUM_COP4SIM], double theta[NUM_COP4SIM], double x[NUM_COP4SIM]){

  for (int i = 0; i < NUM_COP4SIM; i++) {
    scale[i] = force_sim_scale[i];
    theta[i] = force_sim_theta[i];
    x[i] = force_sim_x[i];
  }
}

#else

/** @fn
 * @brief シミュレータ上でのCoPセンサ情報のコールバック
 * @param snake_msgs_abe::FsensorData
 * @return なし
 * @detail
 */
void SnakeFeedback::CallBackOfSimCopData(const snake_msgs_abe::FsensorData::ConstPtr& collision_datain) {

  force_sim_data_timestamp = ros::Time::now();

  for (int i = 0; i < NUM_COP4SIM; i++) {
    force_sim_scale[i] = collision_datain->scale[i];
    force_sim_theta[i] = collision_datain->angle[i];
  }
}

/** @fn
 * @brief シミュレータ上でのCoPセンサ情報のリータン
 * @param double[], double[]
 * @return なし
 * @detail
 */
void SnakeFeedback::ReturnSimCop(double scale[NUM_COP4SIM], double theta[NUM_COP4SIM]){

  for (int i = 0; i < NUM_COP4SIM; i++) {
    scale[i] = force_sim_scale[i];
    theta[i] = force_sim_theta[i];
  }
}
#endif

/*シミュレータ自動化パラメータ系*/

/** @fn
 * @brief シミュレータ自動化パラメータのコールバック関数
 * @param std_msgs::Float64MultiArray
 * @return なし
 * @detail
 */
 void SnakeFeedback::CallBackOfAutoParam(const std_msgs::Float64MultiArray::ConstPtr& param_datain) {
   auto_k = param_datain->data[0];
 }

 /** @fn
  * @brief シミュレータ自動化パラメータのリターン、ObstacleAidedGait2の比例定数K
  * @param なし
  * @return auto_k
  * @detail
  */
 double SnakeFeedback::ReturnAutoParam(){
   return auto_k;
 }
