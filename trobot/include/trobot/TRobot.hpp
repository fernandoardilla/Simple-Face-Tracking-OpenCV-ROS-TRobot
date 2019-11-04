// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobot.hpp
 * @brief TRobot Component
 *
 */
#ifndef TROBOT_HPP
#define TROBOT_HPP

#include <cmath>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose2D.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>

#include "TRobotSerialIOIface.hpp"
#include "TRobotUtil.hpp"
#include "TRobotAPIMain.hpp"

// サービス
#include "TRobotIOSVC.h"
#include "ros/ros.h"
#include "trobot/SRVCmdDIAGLM.h"
#include "trobot/SRVCmdErrClearLM.h"
#include "trobot/SRVCmdErrStop.h"
#include "trobot/SRVCmdErrStopLM.h"
#include "trobot/SRVCmdGetInf.h"
#include "trobot/SRVCmdGetMode.h"
#include "trobot/SRVCmdRebootLM.h"
#include "trobot/SRVCmdRecovery.h"
#include "trobot/SRVCmdRecoveryLM.h"
#include "trobot/SRVCmdSetDrive.h"
#include "trobot/SRVCmdSetDriveLM.h"
#include "trobot/SRVCmdSetIdle.h"
#include "trobot/SRVCmdSetIdleLM.h"
#include "trobot/SRVCmdSuspendLM.h"
#include "trobot/SRVgetAcc.h"
#include "trobot/SRVgetBatteryStat.h"
#include "trobot/SRVgetBatteryVoltage.h"
#include "trobot/SRVgetBoardTemp.h"
#include "trobot/SRVgetBumper.h"
#include "trobot/SRVgetDaiTemp.h"
#include "trobot/SRVgetDistance.h"
#include "trobot/SRVgetDistance2.h"
#include "trobot/SRVgetErrSTS.h"
#include "trobot/SRVgetForcedVel.h"
#include "trobot/SRVgetGyr.h"
#include "trobot/SRVgetLastFail.h"
#include "trobot/SRVgetMag.h"
#include "trobot/SRVgetMntInf.h"
#include "trobot/SRVgetNoFloor.h"
#include "trobot/SRVgetRobotStat.h"
#include "trobot/SRVgetRobotVersionName.h"
#include "trobot/SRVgetUltraSonic.h"
#include "trobot/SRVgetVelInf.h"
#include "trobot/SRVgetVersionInf.h"
#include "trobot/SRVsetAcc.h"
#include "trobot/SRVsetBatteryStat.h"
#include "trobot/SRVsetBatteryVoltage.h"
#include "trobot/SRVsetBoardTemp.h"
#include "trobot/SRVsetBumper.h"
#include "trobot/SRVsetDaiTemp.h"
#include "trobot/SRVsetDistance.h"
#include "trobot/SRVsetDistance2.h"
#include "trobot/SRVsetErrSTS.h"
#include "trobot/SRVsetForcedVel.h"
#include "trobot/SRVsetGyr.h"
#include "trobot/SRVsetLastFail.h"
#include "trobot/SRVsetMag.h"
#include "trobot/SRVsetMntInf.h"
#include "trobot/SRVsetNoFloor.h"
#include "trobot/SRVsetRobotStat.h"
#include "trobot/SRVsetRobotVersionName.h"
#include "trobot/SRVsetUltraSonic.h"
#include "trobot/SRVsetVelInf.h"
#include "trobot/SRVsetVersionInf.h"



////////////////////////////////////////////////////////////////////////////////
/**
 *  @brief Tロボットベース用ROSノード
 *
 *  @copybrief
 *
 *  シリアルポート経由でロボットベースのメイン基板との通信（制御指令の送信、センサデータの受信）を行う。
 *
 */
class TRobot
{
    public:
        TRobot( int argc, char **argv );
        ~TRobot();
        class TRobotAPI  : public TRobotAPIMain
        {
            public:
                TRobotAPI() {};
                ~TRobotAPI() {};
                void setROSIF(
                        geometry_msgs::Twist     *refVel,
                        geometry_msgs::Pose2D    *odometryPose,
                        SensorDataSVC            *SensorData,
                        bool                     publish_tf,
                        std::string              odom_frame_id,
                        std::string              base_frame,
                        ros::Publisher           odometry_pub,
                        ros::Publisher           odometry2d_pub,
                        tf::TransformBroadcaster *odom_broadcaster,
                        ros::Publisher           sonar_array_pub
                    )
                {
                    m_refVel           = refVel;
                    m_odometryPose     = odometryPose;
                    m_SensorData       = SensorData;
                    m_SensorData->SetserIO( (void *)TRobotAPIMain::m_serIO );    // シリアルIFセーブ
                    m_publish_tf       = publish_tf;
                    m_odom_frame_id    = odom_frame_id;
                    m_base_frame       = base_frame;
                    m_odometry_pub     = odometry_pub;
                    m_odometry2d_pub   = odometry2d_pub;
                    m_odom_broadcaster = odom_broadcaster;
                    m_sonar_array_pub  = sonar_array_pub;
                };
                void serStartInit();
                // オドメトリ更新(本プロセスの出力)
                void odometryOut(
                    double        &odometryPose_x,
                    double        &odometryPose_y,
                    double        &odometryPose_heading
                    );

                //運転状態サービス更新
                void setRobotStat()
                {
                    m_SensorData->setRobotStat( m_robotStat );
                };

                //センサ情報サービス更新(旧ロボットベース)
                void setSensor_EX();

                //センサ情報サービス更新(SIBOT-X)
                void setSensor_SX();

                // センサ情報サービス更新(L-MES)
                void setSensor_LM();

                //メンテナンス情報サービス更新(SIBOT-X)
                void setMent_SX();

                //メンテナンス情報サービス更新(L-MES)
                void setMent_LM();

                //センサ情報サービス更新(CIRCINUS（メカナム）)
                void setSensor_MN();

                //
                // サービスROSラッパー
                //

                // ロボットバージョンの名称
                bool SRVsetRobotVersionName( trobot::SRVsetRobotVersionName::Request &req, trobot::SRVsetRobotVersionName::Response &res )
                {
                    m_SensorData->setRobotVersionName( req.name.c_str() );
                    return true;
                };
                bool SRVgetRobotVersionName( trobot::SRVgetRobotVersionName::Request &req, trobot::SRVgetRobotVersionName::Response &res )
                {
                    const char *name="";
                    m_SensorData->getRobotVersionName( name );
                    res.name = std::string( name );
                    return true;
                };

                // 強制速度更新
                bool SRVsetForcedVel( trobot::SRVsetForcedVel::Request &req, trobot::SRVsetForcedVel::Response &res )
                {
                    m_SensorData->setForcedVel( req.Flg, req.Vx, req.Vy, req.Va );
                    return true;
                };
                bool SRVgetForcedVel( trobot::SRVgetForcedVel::Request &req, trobot::SRVgetForcedVel::Response &res )
                {
                    bool *Flg = (bool *)&res.Flg;
                    m_SensorData->getForcedVel( *Flg, res.Vx, res.Vy, res.Va );
                    return true;
                };

                // バッテリ電圧
                bool SRVsetBatteryVoltage( trobot::SRVsetBatteryVoltage::Request &req, trobot::SRVsetBatteryVoltage::Response &res )
                {
                    m_SensorData->setBatteryVoltage( req.value );
                    return true;
                };
                bool SRVgetBatteryVoltage( trobot::SRVgetBatteryVoltage::Request &req, trobot::SRVgetBatteryVoltage::Response &res )
                {
                    m_SensorData->getBatteryVoltage( res.value );
                    return true;
                };

                // ダイ温度(旧ロボットベース、SCIBOT-Xのみ)
                bool SRVsetDaiTemp( trobot::SRVsetDaiTemp::Request &req, trobot::SRVsetDaiTemp::Response &res )
                {
                    m_SensorData->setDaiTemp( req.value );
                    return true;
                };
                bool SRVgetDaiTemp( trobot::SRVgetDaiTemp::Request &req, trobot::SRVgetDaiTemp::Response &res )
                {
                    m_SensorData->getDaiTemp( res.value );
                    return true;
                };

                // エラーステータス(旧ロボットベースのみ)
                bool SRVsetErrSTS( trobot::SRVsetErrSTS::Request &req, trobot::SRVsetErrSTS::Response &res )
                {
                    m_SensorData->setErrSTS( req.adc, req.com, req.i2c0, req.i2c1 );
                    return true;
                };
                bool SRVgetErrSTS( trobot::SRVgetErrSTS::Request &req, trobot::SRVgetErrSTS::Response &res )
                {
                    m_SensorData->getErrSTS( res.adc, res.com, res.i2c0, res.i2c1 );
                    return true;
                };

                // エラーステータス(SCIBOT-X、L-MESのみ)
                bool SRVsetLastFail( trobot::SRVsetLastFail::Request &req, trobot::SRVsetLastFail::Response &res )
                {
                    m_SensorData->setLastFail( req.LastFailD, req.LastFailC, req.LastFailN, req.LastFailE );
                    return true;
                };
                bool SRVgetLastFail( trobot::SRVgetLastFail::Request &req, trobot::SRVgetLastFail::Response &res )
                {
                    m_SensorData->getLastFail( res.LastFailD, res.LastFailC, res.LastFailN, res.LastFailE );
                    return true;
                };

                // 9D加速度センサ
                bool SRVsetAcc( trobot::SRVsetAcc::Request &req, trobot::SRVsetAcc::Response &res )
                {
                    m_SensorData->setAcc( req.accX, req.accY, req.accZ );
                    return true;
                };
                bool SRVgetAcc( trobot::SRVgetAcc::Request &req, trobot::SRVgetAcc::Response &res )
                {
                    m_SensorData->getAcc( res.accX, res.accY, res.accZ );
                    return true;
                };

                // 9Dジャイロセンサ
                bool SRVsetGyr( trobot::SRVsetGyr::Request &req, trobot::SRVsetGyr::Response &res )
                {
                    m_SensorData->setGyr( req.gyrX, req.gyrY, req.gyrZ );
                    return true;
                };
                bool SRVgetGyr( trobot::SRVgetGyr::Request &req, trobot::SRVgetGyr::Response &res )
                {
                    m_SensorData->getGyr( res.gyrX, res.gyrY, res.gyrZ );
                    return true;
                };

                // 9D磁気センサ
                bool SRVsetMag( trobot::SRVsetMag::Request &req, trobot::SRVsetMag::Response &res )
                {
                    m_SensorData->setMag( req.magX, req.magY, req.magZ );
                    return true;
                };
                bool SRVgetMag( trobot::SRVgetMag::Request &req, trobot::SRVgetMag::Response &res )
                {
                    m_SensorData->getMag( res.magX, res.magY, res.magZ );
                    return true;
                };

                // 床なしセンサ(SCIBOT-Xのみ)
                bool SRVsetNoFloor( trobot::SRVsetNoFloor::Request &req, trobot::SRVsetNoFloor::Response &res )
                {
                    m_SensorData->setNoFloor( req.NoFloor1, req.NoFloor2, req.NoFloor3, req.NoFloor4 );
                    return true;
                };
                bool SRVgetNoFloor( trobot::SRVgetNoFloor::Request &req, trobot::SRVgetNoFloor::Response &res )
                {
                    bool *NoFloor1=(bool*)&res.NoFloor1; bool *NoFloor2=(bool*)&res.NoFloor2;
                    bool *NoFloor3=(bool*)&res.NoFloor3; bool *NoFloor4=(bool*)&res.NoFloor4;
                    m_SensorData->getNoFloor( *NoFloor1, *NoFloor2, *NoFloor3, *NoFloor4 );
                    return true;
                };

                // バンパセンサ(SCIBOT-Xのみ)
                bool SRVsetBumper( trobot::SRVsetBumper::Request &req, trobot::SRVsetBumper::Response &res )
                {
                    m_SensorData->setBumper( req.Bumper1, req.Bumper2, req.Bumper3, req.Bumper4 );
                    return true;
                };
                bool SRVgetBumper( trobot::SRVgetBumper::Request &req, trobot::SRVgetBumper::Response &res )
                {
                    bool *Bumper1=(bool*)&res.Bumper1; bool *Bumper2=(bool*)&res.Bumper2;
                    bool *Bumper3=(bool*)&res.Bumper3; bool *Bumper4=(bool*)&res.Bumper4;
                    m_SensorData->getBumper( *Bumper1, *Bumper2, *Bumper3, *Bumper4 );
                    return true;
                };

                // 測距センサ(SCIBOT-X、L-MESのみ)
                bool SRVsetDistance( trobot::SRVsetDistance::Request &req, trobot::SRVsetDistance::Response &res )
                {
                    m_SensorData->setDistance( req.Distance0, req.Distance1, req.Distance2, req.Distance3 );
                    return true;
                };
                bool SRVgetDistance( trobot::SRVgetDistance::Request &req, trobot::SRVgetDistance::Response &res )
                {
                    m_SensorData->getDistance( res.Distance0, res.Distance1, res.Distance2, res.Distance3 );
                    return true;
                };

                // 測距センサ２(L-MESのみのみ)
                bool SRVsetDistance2( trobot::SRVsetDistance2::Request &req, trobot::SRVsetDistance2::Response &res )
                {
                    m_SensorData->setDistance2( req.Distance4, req.Distance5, req.Distance6, req.Distance7 );
                    return true;
                };
                bool SRVgetDistance2( trobot::SRVgetDistance2::Request &req, trobot::SRVgetDistance2::Response &res )
                {
                    m_SensorData->getDistance2( res.Distance4, res.Distance5, res.Distance6, res.Distance7 );
                    return true;
                };

                // 超音波センサ(L-MESのみのみ)
                bool SRVsetUltraSonic( trobot::SRVsetUltraSonic::Request &req, trobot::SRVsetUltraSonic::Response &res )
                {
                    m_SensorData->setUltraSonic( req.UltraSonic0, req.UltraSonic1 );
                    return true;
                };
                bool SRVgetUltraSonic( trobot::SRVgetUltraSonic::Request &req, trobot::SRVgetUltraSonic::Response &res )
                {
                    m_SensorData->getUltraSonic( res.UltraSonic0, res.UltraSonic1 );
                    return true;
                };

                // 基板温度(L-MESのみのみ)
                bool SRVsetBoardTemp( trobot::SRVsetBoardTemp::Request &req, trobot::SRVsetBoardTemp::Response &res )
                {
                    m_SensorData->setBoardTemp( req.BoardTemp );
                    return true;
                };
                bool SRVgetBoardTemp( trobot::SRVgetBoardTemp::Request &req, trobot::SRVgetBoardTemp::Response &res )
                {
                    m_SensorData->getBoardTemp( res.BoardTemp );
                    return true;
                };

                // バッテリ状態(SCIBOT-X、L-MESのみ)
                // L-MES:BatteryLevel1に使用バッテリー残量、BatteryLevel2に未使用バッテリー残量。あとは未使用。
                bool SRVsetBatteryStat( trobot::SRVsetBatteryStat::Request &req, trobot::SRVsetBatteryStat::Response &res )
                {
                    m_SensorData->setBatteryStat( req.BatteryStat, req.BatteryComStat, req.ErrDetectStat,
                                                  req.BatteryLevel1, req.BatteryLevel2, req.BatteryTemp1, req.BatteryTemp2 );
                    return true;
                };
                bool SRVgetBatteryStat( trobot::SRVgetBatteryStat::Request &req, trobot::SRVgetBatteryStat::Response &res )
                {
                    m_SensorData->getBatteryStat( res.BatteryStat, res.BatteryComStat, res.ErrDetectStat,
                                                  res.BatteryLevel1, res.BatteryLevel2, res.BatteryTemp1, res.BatteryTemp2 );
                    return true;
                };

                // 速度情報(SCIBOT-Xのみ)
                bool SRVsetVelInf( trobot::SRVsetVelInf::Request &req, trobot::SRVsetVelInf::Response &res )
                {
                    m_SensorData->setVelInf( req.rightVelCtl, req.leftVelCtl, req.rightVel, req.leftVel,
                                             req.rightCurrent, req.leftCurrent );
                    return true;
                };
                bool SRVgetVelInf( trobot::SRVgetVelInf::Request &req, trobot::SRVgetVelInf::Response &res )
                {
                    m_SensorData->getVelInf( res.rightVelCtl, res.leftVelCtl, res.rightVel, res.leftVel,
                                             res.rightCurrent, res.leftCurrent );
                    return true;
                };

                // バージョン情報(SCIBOT-X、L-MESのみ)
                bool SRVsetVersionInf( trobot::SRVsetVersionInf::Request &req, trobot::SRVsetVersionInf::Response &res )
                {
                    m_SensorData->setVersionInf( req.ModelRevision, req.HardwareRevision,
                                                 req.FirmwareRevision, req.SerialNumber,
                                                 req.CreationYear, req.CreationMonth, req.CreationDay,
                                                 req.CreationHour, req.CreationMinute );
                    return true;
                };
                bool SRVgetVersionInf( trobot::SRVgetVersionInf::Request &req, trobot::SRVgetVersionInf::Response &res )
                {
                    unsigned long *SerialNumber = (unsigned long*)&res.SerialNumber;
                    m_SensorData->getVersionInf( res.ModelRevision, res.HardwareRevision, res.FirmwareRevision, *SerialNumber,
                                                 res.CreationYear, res.CreationMonth, res.CreationDay,
                                                 res.CreationHour, res.CreationMinute );
                    return true;
                };


                // メンテナンス情報(SCIBOT-X、L-MESのみ)
                // NoFloorSMount:L-MESで未使用 BumperSMount:L-MESで未使用 DistanceSMount2:SCIBOT-Xで未使用
                // UltraSonicSMount:SCIBOT-Xで未使用 ResetCount:SCIBOT-Xで未使用
                // NineDOFSMount:L-MESで未使用 JudgSensor:L-MESで未使用
                bool SRVsetMntInf( trobot::SRVsetMntInf::Request &req, trobot::SRVsetMntInf::Response &res )
                {
                    m_SensorData->setMntInf( req.OperatingTime, req.FailD, req.FailC, req.FailN,
                                             req.FailE, req.FailWDT, req.FailRAMSoft, req.NoFloorSMount, req.BumperSMount,
                                             req.DistanceSMount, req.DistanceSMount2, req.UltraSonicSMount, req.ResetCount,
                                             req.TravelDistance, req.NineDOFSMount, req.JudgSensor );
                    return true;
                };
                bool SRVgetMntInf( trobot::SRVgetMntInf::Request &req, trobot::SRVgetMntInf::Response &res )
                {
                    unsigned long *OperatingTime = (unsigned long*)&res.OperatingTime;
                    unsigned long *TravelDistance = (unsigned long*)&res.TravelDistance;
                    m_SensorData->getMntInf( *OperatingTime, res.FailD, res.FailC, res.FailN,
                                             res.FailE, res.FailWDT, res.FailRAMSoft, res.NoFloorSMount, res.BumperSMount,
                                             res.DistanceSMount, res.DistanceSMount2, res.UltraSonicSMount, res.ResetCount,
                                             *TravelDistance, res.NineDOFSMount, res.JudgSensor );
                    return true;
                };

                // 現在の運転状態
                bool SRVsetRobotStat( trobot::SRVsetRobotStat::Request &req, trobot::SRVsetRobotStat::Response &res )
                {
                    m_SensorData->setRobotStat( req.robotStat );
                    return true;
                };
                bool SRVgetRobotStat( trobot::SRVgetRobotStat::Request &req, trobot::SRVgetRobotStat::Response &res )
                {
                    long *robotStat = (long*)&res.robotStat;
                    m_SensorData->getRobotStat( *robotStat );
                    return true;
                };

                // 運転モードに移行(SCIBOT-Xのみ)
                bool SRVCmdSetDrive( trobot::SRVCmdSetDrive::Request &req, trobot::SRVCmdSetDrive::Response &res )
                {
                    m_SensorData->CmdSetDrive();
                    return true;
                };

                // 待機モードに移行(SCIBOT-Xのみ)
                bool SRVCmdSetIdle( trobot::SRVCmdSetIdle::Request &req, trobot::SRVCmdSetIdle::Response &res )
                {
                    m_SensorData->CmdSetIdle();
                    return true;
                };

                // 緊急停止モードに移行(SCIBOT-Xのみ)
                bool SRVCmdErrStop( trobot::SRVCmdErrStop::Request &req, trobot::SRVCmdErrStop::Response &res )
                {
                    m_SensorData->CmdErrStop();
                    return true;
                };

                // 緊急停止モードから待機モードに移行(SCIBOT-Xのみ)
                bool SRVCmdRecovery( trobot::SRVCmdRecovery::Request &req, trobot::SRVCmdRecovery::Response &res )
                {
                    m_SensorData->CmdRecovery();
                    return true;
                };

                // 現在のモードを取得する(SCIBOT-Xのみ)
                bool SRVCmdGetMode( trobot::SRVCmdGetMode::Request &req, trobot::SRVCmdGetMode::Response &res )
                {
                    m_SensorData->CmdGetMode();
                    return true;
                };

                // 現在の情報を取得する(SCIBOT-Xのみ)
                bool SRVCmdGetInf( trobot::SRVCmdGetInf::Request &req, trobot::SRVCmdGetInf::Response &res )
                {
                    m_SensorData->CmdGetInf();
                    return true;
                };

                // 運転モードに移行(L-MESのみ)
                bool SRVCmdSetDriveLM( trobot::SRVCmdSetDriveLM::Request &req, trobot::SRVCmdSetDriveLM::Response &res )
                {
                    m_SensorData->CmdSetDriveLM();
                    return true;
                };

                // 待機モードに移行(L-MESのみ)
                bool SRVCmdSetIdleLM( trobot::SRVCmdSetIdleLM::Request &req, trobot::SRVCmdSetIdleLM::Response &res )
                {
                    m_SensorData->CmdSetIdleLM();
                    return true;
                };

                // 緊急停止モードに移行(L-MESのみ)
                bool SRVCmdErrStopLM( trobot::SRVCmdErrStopLM::Request &req, trobot::SRVCmdErrStopLM::Response &res )
                {
                    m_SensorData->CmdErrStopLM();
                    return true;
                };

                // 緊急停止システムの自己診断を要求(L-MESのみ)
                bool SRVCmdDIAGLM( trobot::SRVCmdDIAGLM::Request &req, trobot::SRVCmdDIAGLM::Response &res )
                {
                    m_SensorData->CmdDIAGLM();
                    return true;
                };

                // 緊急停止モードから待機モードに移行(L-MESのみ)
                bool SRVCmdRecoveryLM( trobot::SRVCmdRecoveryLM::Request &req, trobot::SRVCmdRecoveryLM::Response &res )
                {
                    m_SensorData->CmdRecoveryLM();
                    return true;
                };

                // ロボットベースを再起動させる(L-MESのみ)
                bool SRVCmdRebootLM( trobot::SRVCmdRebootLM::Request &req, trobot::SRVCmdRebootLM::Response &res )
                {
                    m_SensorData->CmdRebootLM();
                    return true;
                };

                // 省電力モードに移行する(L-MESのみ)
                bool SRVCmdSuspendLM( trobot::SRVCmdSuspendLM::Request &req, trobot::SRVCmdSuspendLM::Response &res )
                {
                    m_SensorData->CmdSuspendLM();
                    return true;
                };

                // エラー、軽故障データクリア(L-MESのみ)
                bool SRVCmdErrClearLM( trobot::SRVCmdErrClearLM::Request &req, trobot::SRVCmdErrClearLM::Response &res )
                {
                    m_SensorData->CmdErrClearLM();
                    return true;
                };


            private:
                geometry_msgs::Twist     *m_refVel;         // 速度目標値
                geometry_msgs::Pose2D    *m_odometryPose;   // オドメトリ位置姿勢データ
                SensorDataSVC            *m_SensorData;
                bool                     m_publish_tf;      // tf使用フラグ
                std::string              m_odom_frame_id;   // オドメトリのフレーム名称
                std::string              m_base_frame;      // ベースフレーム名称
                ros::Publisher           m_odometry_pub;
                ros::Publisher           m_odometry2d_pub;
                tf::TransformBroadcaster *m_odom_broadcaster;
                ros::Publisher           m_sonar_array_pub;
        };


    private:
        std::string m_trobotVersion;             //!< ロボットバージョンの名称（旧伝送フォーマット："OLDrobot",新伝送フォーマット："SCIBOT-X", etc.）
        std::string m_activeConfig;              //!< Configuration SETの名称（pyxis, gemini, chorusline, etc.）
        std::string m_serialPortName;            //!< シリアルポートの名称（Linux: /dev/ttyUSB0, etc.,  Windows: COM3, etc.）
        double      m_wheelRadius;               //!< 駆動輪の半径 [m]
        double      m_axleTrack;                 //!< 輪距（左右の駆動輪中心間の距離） [m]
        double      m_axleTrackFR;               //!< 輪距（前後の駆動輪中心間の距離） [m]（メカナム時）
        double      m_rotEncoderResolution;      //!< 駆動輪モータ軸のエンコーダの分解能 [pulses/rev]
        double      m_wheelGearRatio;            //!< 駆動輪の減速機の減速比 [無単位]
        double      m_coeffRpsToCmd;             //!< 角速度 [rad/s] から回転速度指令値への変換係数
        double      m_coeffVxCmd;                //!< 指令値進行方向速度(vx)にかける係数
        double      m_coeffVyCmd;                //!< 指令値進行方向直角左向き速度(vy)にかける係数
        double      m_coeffVzCmd;                //!< 指令値車体角速度(va)にかける係数
        double      m_coeffXOdo;                 //!< オドメトリX座標増分にかける係数
        double      m_coeffYOdo;                 //!< オドメトリY座標増分にかける係数
        double      m_coeffHeadingOdo;           //!< オドメトリ向き増分にかける係数
        double      m_coeffHeadingOdoAdd;        //!< オドメトリ向き増分に足しこむ係数
        int         m_velAccelMax;               //!< 1指令周期毎の加速度制限
        int         m_velDecelMax;               //!< 1指令周期毎の減速度制限
        std::string m_autoRecovery;              //!< 緊急停止モードで要因が復旧した時に自動で復旧処理を行う（"MANUAL"(default):手動でモード遷移させる、"AUTO"：自動で運転モードに遷移する）
        int         m_FrameRate;                 //!< 1sec毎の処理回数
        bool        m_publish_tf;                //!< tf使用フラグ
        std::string m_odom_frame_id;             //!< オドメトリのフレーム名称
        std::string m_base_frame;                //!< ベースフレーム名称
        bool        m_trace_log;                 //!< トレースログ出力フラグ

        TRobotAPI   *m_APIMain;                  //!< Tロボットベース用APIへのポインタ
        TRobotIF::BaseSensorData *m_sensorData;  //!< ロボットベースのメイン基板に接続されたセンサのデータ（エンコーダ値, etc.）
        TRobotIF::MntInfData     *m_mntInfData;  //!< ロボットベースのメンテナンス情報のデータ

        // メッセージ
        ros::Subscriber          m_cmd_vel_sub;
        ros::Subscriber          m_reset_odom_sub;
        ros::Publisher           m_odometry_pub;
        ros::Publisher           m_odometry2d_pub;
        tf::TransformBroadcaster *m_odom_broadcaster;
        ros::Publisher           m_sonar_array_pub;

        // サービス設定
        void Seting_Service( ros::NodeHandle &n, SensorDataSVC &m_SensorData );

        // ロボットベースへの設定情報取得(CIRCINUS（メカナム）)
        void Geting_CmdConfig_MN( ros::NodeHandle &nh );

        //! スレッドセーフなキューのクラス
        class TRobotFlag
        {
        private:
            std::mutex m_mutex;
            bool m_flag;

        public:
            TRobotFlag()
            {
                this->clear();
            }
            //! on
            void flag_on()
            {
                std::lock_guard<std::mutex> lock(this->m_mutex);
                m_flag = true;
            }
            //! off
            void flag_off()
            {
                std::lock_guard<std::mutex> lock(this->m_mutex);
                m_flag = false;
            }
            //! look
            bool flag_look()
            {
                std::lock_guard<std::mutex> lock(this->m_mutex);
                return this->m_flag;
            }
            //! clear
            void clear()
            {
                std::lock_guard<std::mutex> lock(this->m_mutex);
                this->m_flag = false;
            }

        };

        // 速度制御メッセージ
        geometry_msgs::Twist m_refVel;           //!< 速度目標値
        TRobotFlag m_refVelUpdated;              //!< センサデータのキュー
        bool m_spinOnce_flg;                     //!< 再メッセージチェックフラグ

        // 速度制御メッセージコールバック
        void callback_cmdvel( const geometry_msgs::Twist::ConstPtr& cmd )
        {
            m_refVel = *cmd;
            m_refVelUpdated.flag_on();
            m_spinOnce_flg = true;
        };

        // オドメトリ
        geometry_msgs::Pose2D m_odometryPose;    //!< オドメトリ位置姿勢データ
        geometry_msgs::Pose2D m_odometryPose_internal; //!< オドメトリ位置姿勢データ（内部データ）

        // オドメトリリセット
        void resetOdometry( geometry_msgs::Pose2D& odometryPose )
        {
            odometryPose.x = 0.0;
            odometryPose.y = 0.0;
            odometryPose.theta = 0.0;
        };

        // オドメトリリセットメッセージコールバック
        void callback_rstodom(const std_msgs::Empty::ConstPtr& rst)
        {
            resetOdometry( m_odometryPose );
            resetOdometry( m_odometryPose_internal );
            m_spinOnce_flg = true;
        };

};


#endif // TROBOT_HPP
