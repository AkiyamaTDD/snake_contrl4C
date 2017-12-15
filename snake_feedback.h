/**
 * @file snake_feedback.h
 * @brief ヘビ型ロボットのセンサ情報などを扱うクラス
 * @author Taichi Akiyama
 * @date 2017/09/27
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_SNAKE_FEEDBACK_H_
#define SNAKE_CONTROL_SRC_SNAKE_FEEDBACK_H_

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <joy_handler/JoySelectedData.h>
#include <snake_msgs/SnakeJointData.h>
#include <snake_msgs/SnakeForceData.h>

#include <snake_msgs_abe/FsensorData.h>
#include <snake_msgs_abe/FsensorDataPlusx.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

//defineをいじって使うデータを選択
#define CURRENT_ANGLE //現在関節角度
#define CURRENT //電流値
//#define COP  //実際のロボットのCoP

#define SIM_COP //シミュレータ上でのCoPセンサ情報

#define SIM_AUTO_PARAM //シミュレータ自動化パラメータ

//こっちは定数定義
static const int NUM_COP4SIM = 20;//シミュレータ上でのCoPセンサの数
static const int NUM_JOINT = 20; //関節数
static const int COP_ID = 2; //1リンクあたりのCoPセンサの枚数
static const int LAST_YAW_JOINT = 18; //最後尾yaw関節番号(尻尾抜き)
static const int LAST_PITCH_JOINT = 19; //最後尾pitch関節番号(尻尾抜き)


#define USE_SIM_COPX //シミュレータ上でのCoPセンサのx軸情報使うかどうか

class SnakeFeedback {
  public:

   static void Initialize();

   static std::vector<double> ReturnCurrentAngle();
   static void ReturnCurrent(double arg_current[NUM_JOINT]);
   static void ReturnForce(double scale[NUM_JOINT][COP_ID], double theta[NUM_JOINT][COP_ID]);

   #ifdef USE_SIM_COPX
   static void ReturnSimCop(double scale[NUM_COP4SIM], double theta[NUM_COP4SIM], double x[NUM_COP4SIM]);
   #else
   static void ReturnSimCop(double scale[NUM_COP4SIM], double theta[NUM_COP4SIM]);
   #endif

   static double ReturnAutoParam(); 

 private:
   //subscriber
   static ros::Subscriber current_angle_sub;
   static ros::Subscriber current_sub;
   static ros::Subscriber force_sub;

   static ros::Subscriber collision_position_sub;

   static ros::Subscriber sub_auto_param;

   //Callback
   static void CallBackOfJointCurrentAngleData(const snake_msgs::SnakeJointData::ConstPtr&);
   static void CallBackOfJointCurrentData(const snake_msgs::SnakeJointData::ConstPtr&);
   static void CallBackOfForceData(const snake_msgs::SnakeForceData::ConstPtr&);

   #ifdef USE_SIM_COPX
   static void CallBackOfSimCopData(const snake_msgs_abe::FsensorDataPlusx::ConstPtr&);
   #else
   static void CallBackOfSimCopData(const snake_msgs_abe::FsensorData::ConstPtr&);
   #endif

   static void CallBackOfAutoParam(const std_msgs::Float64MultiArray::ConstPtr&);

} ;






#endif
