// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobot.hpp
 * @brief TRobot Component
 *
 */
#ifndef TROBOT_APIMAIN_HPP
#define TROBOT_APIMAIN_HPP

#include <string> 
#include <stdio.h> 
#include <iostream> 

#include "TRobotSerialIOIface.hpp"
#include "TRobotUtil.hpp"
#include "TRobotSensorData.hpp"


////////////////////////////////////////////////////////////////////////////////
/**
 *  @brief Tロボットベース用API
 *
 *  @copybrief
 *
 *  シリアルポート経由でロボットベースのメイン基板との通信（制御指令の送信、センサデータの受信）を行う。
 *
 */
class TRobotAPIMain
{
    public:
        TRobotAPIMain();
        virtual ~TRobotAPIMain();

        bool onInitializeAPI();
        bool onActivatedAPI(
            std::string &trobotVersion,           // ロボットバージョンの名称（旧伝送フォーマット："OLDrobot",新伝送フォーマット："SCIBOT-X", etc.）
            std::string &activeConfig,            // Configuration SETの名称（pyxis, gemini, chorusline, etc.）
            std::string &serialPortName,          // シリアルポートの名称（Linux: /dev/ttyUSB0, etc.,  Windows: COM3, etc.）
            double      &wheelRadius,             // 駆動輪の半径 [m]
            double      &axleTrack,               // 輪距（左右の駆動輪中心間の距離） [m]
            double      &axleTrackFR,             // 輪距（前後の駆動輪中心間の距離） [m]（メカナム時）
            double      &rotEncoderResolution,    // 駆動輪モータ軸のエンコーダの分解能 [pulses/rev]
            double      &wheelGearRatio,          // 駆動輪の減速機の減速比 [無単位]
            double      &coeffRpsToCmd,           // 角速度 [rad/s] から回転速度指令値への変換係数
            double      &coeffVxCmd,              // 指令値進行方向速度(vx)にかける係数
            double      &coeffVyCmd,              // 指令値進行方向直角左向き速度(vy)にかける係数
            double      &coeffVaCmd,              // 指令値車体角速度(va)にかける係数
            double      &coeffXOdo,               // オドメトリX座標増分にかける係数
            double      &coeffYOdo,               // オドメトリY座標増分にかける係数
            double      &coeffHeadingOdo,         // オドメトリ向き増分に足しこむ係数
            double      &coeffHeadingOdoAdd,      // オドメトリ向き増分に足しこむ係数
            int         &velAccelMax,             // 1指令周期毎の加速度制限
            int         &velDecelMax,             // 1指令周期毎の減速度制限
            std::string &autoRecovery,            // 緊急停止モードで要因が復旧した時に自動で復旧処理を行う（"MANUAL"(default):手動でモード遷移させる、"AUTO"：自動で運転モードに遷移する）
            bool        &trace_log,               // トレースログ出力フラグ
            int         &FrameRate                // 1sec毎の処理回数
        );
        bool onDeactivatedAPI();
        //bool SerStartStop( void (*inqFunc)() );
        //bool SerStartStop( void (TRobotAPIMain::*inqFunc)() );
        bool SerStartStop();

        // 速度指示
        bool sendWheelRotVel( bool updated, double vx, double vy, double va );

        // コマンド
        bool sendCmd( int cmd );

        //
        // プロセス間IFなので上位の派生クラスで宣言する
        //
        virtual void serStartInit() {};   // シリアル接続時の処理(本プロセスの入力の空抜き)
        
        // オドメトリ更新(本プロセスの出力)
        virtual void odometryOut(
             double        &odometryPose_x,
             double        &odometryPose_y,
             double        &odometryPose_heading
             ) {};

        virtual void setRobotStat() {};   // 運転状態サービス更新(本プロセスのサービス)

        // センサ情報サービス更新(本プロセスのサービス)
        virtual void setSensor_EX() {}; virtual void setSensor_SX() {}; virtual void setSensor_LM() {}; virtual void setSensor_MN() {};

        // メンテナンス情報サービス更新(本プロセスのサービス)
        virtual void setMent_SX() {}; virtual void setMent_LM() {};

        bool SensorDataUpdated();

        void getDataAddr( TRobotIF::BaseSensorData *sensorData, TRobotIF::MntInfData *mntInfData )
        {
             sensorData = &m_sensorData;
             mntInfData = &m_mntInfData;
        };

    public:
        std::string  m_trobotVersion;           //!< ロボットバージョンの名称（旧伝送フォーマット："OLDrobot",新伝送フォーマット："SCIBOT-X", etc.）
        std::string  m_activeConfig;            //!< Configuration SETの名称（pyxis, gemini, chorusline, etc.）
        std::string  m_serialPortName;          //!< シリアルポートの名称（Linux: /dev/ttyUSB0, etc.,  Windows: COM3, etc.）
        double       m_wheelRadius;             //!< 駆動輪の半径 [m]
        double       m_axleTrack;               //!< 輪距（左右の駆動輪中心間の距離） [m]
        double       m_axleTrackFR;             //!< 輪距（前後の駆動輪中心間の距離） [m]（メカナム時）
        double       m_rotEncoderResolution;    //!< 駆動輪モータ軸のエンコーダの分解能 [pulses/rev]
        double       m_wheelGearRatio;          //!< 駆動輪の減速機の減速比 [無単位]
        double       m_coeffRpsToCmd;           //!< 角速度 [rad/s] から回転速度指令値への変換係数
        double       m_coeffVxCmd;              //!< 指令値進行方向速度(vx)にかける係数
        double       m_coeffVyCmd;              //!< 指令値進行方向直角左向き速度(vy)にかける係数
        double       m_coeffVaCmd;              //!< 指令値車体角速度(va)にかける係数
        double       m_coeffXOdo;               //!< オドメトリX座標増分にかける係数
        double       m_coeffYOdo;               //!< オドメトリY座標増分にかける係数
        double       m_coeffHeadingOdo;         //!< オドメトリ向き増分にかける係数
        double       m_coeffHeadingOdoAdd;      //!< オドメトリ向き増分に足しこむ係数
        int          m_rot_vel_cmd_min;         //!< 速度指令最小値
        int          m_rot_vel_cmd_max;         //!< 速度指令最大値
        int          m_velAccelMax;             //!< 1指令周期毎の加速度制限
        int          m_velDecelMax;             //!< 1指令周期毎の減速度制限
        std::string  m_autoRecovery;            //!< 緊急停止モードで要因が復旧した時に自動で復旧処理を行う（"MANUAL"(default):手動でモード遷移させる、"AUTO"：自動で運転モードに遷移する）

        bool         m_forcedRotVelFlg;         //!< 強制速度更新フラグ
        double       m_forcedRotVelVx;          //!< 強制進行方向速度(vx)
        double       m_forcedRotVelVy;          //!< 強制進行方向直角左向き速度(vy)
        double       m_forcedRotVelVa;          //!< 強制車体角速度(va)

        bool         m_trace_log;               //!< トレースログ出力フラグ

        int          m_FrameRate;               //!< 1sec毎の処理回数


    public:
        // オドメトリデータを更新する(旧ロボットベース)
        void updateOdometry_EX(
                        TRobotIF::BaseSensorData_EX  &sensorData,
                        double                       &odometryPose_x,
                        double                       &odometryPose_y,
                        double                       &odometryPose_heading );

        // オドメトリデータを更新する(SCIBOT-X)
        void updateOdometry_SX(
                        TRobotIF::BaseSensorData_SX  &sensorData,
                        double                       &odometryPose_x,
                        double                       &odometryPose_y,
                        double                       &odometryPose_heading );

        // オドメトリデータを更新する(L-MES)
        void updateOdometry_LM( 
                        TRobotIF::BaseSensorData_LM  &sensorData,
                        double                       &odometryPose_x,
                        double                       &odometryPose_y,
                        double                       &odometryPose_heading );

        // オドメトリデータを更新する(CIRCINUS（メカナム）)
        void updateOdometry_MN(
                        TRobotIF::BaseSensorData_MN  &sensorData,
                        double                       &odometryPose_x,
                        double                       &odometryPose_y,
                        double                       &odometryPose_heading );

        // ロボットベースの前進速度および旋回速度の組み合わせから左右の車輪の回転速度指令値への変換を行う
        TRobotIF::RetCode convertToWheelRotVelCmds( double baseXVel, double baseYVel, double baseAngVel,
                                                    int&   leftRotVelCmd,   int&   rightRotVelCmd,
                                                    int&   leftRotVelCmd_r, int&   rightRotVelCmd_r );

        // Odometryのクリア
        void resetOdometry()
        {
            m_odometryPose_x = 0.0;
            m_odometryPose_y = 0.0;
            m_odometryPose_heading = 0.0;
        };

        //!< オドメトリ位置姿勢データ
        double      m_odometryPose_x;          //!< 距離(x)
        double      m_odometryPose_y;          //!< 直角左向き距離(y)
        double      m_odometryPose_heading;    //!< 角度

        bool serStart();                       //!< シリアルポートのスタート
        bool serStop();                        //!< シリアルポートのストップ
        

        TRobotIF::SerialIOIface *m_serIO;      //!< シリアル入出力用インスタンスへのポインタ
        TRobotIF::BaseSensorData m_sensorData; //!< ロボットベースのメイン基板に接続されたセンサのデータ（エンコーダ値, etc.）
        TRobotIF::MntInfData     m_mntInfData; //!< ロボットベースのメンテナンス情報のデータ
        int                      m_robotStat;  //!< ロボットベースの運転状態

        // 速度指令値
        int m_leftRotVelCmd;
        int m_rightRotVelCmd;
        int m_leftRotVelCmd_r;
        int m_rightRotVelCmd_r;

        // 目標速度指令値
        int m_leftRotVelCmd_TVal;
        int m_rightRotVelCmd_TVal;
        int m_leftRotVelCmd_TVal_r;
        int m_rightRotVelCmd_TVal_r;

        /*!
         * オドメトリ用変換係数。
         * エンコーダカウント値から，対応する回転により描く弧長の半分の値への変換
         * (その値はコンフィグパラメータに依存るので onActivated() で計算される)
         */
        double m_coeffEncToArcHalf;            //!< エンコーダ変換係数 / 2
        double m_coeffEncToArcQuarter;         //!< エンコーダ変換係数 / 4
        double m_halfAxleTrack;                //!< 輪距（左右の駆動輪中心間の距離）の半分 [m]
        double m_halfAxleTrackFR;              //!< 輪距（前後の駆動輪中心間の距離） [m]（メカナム時）の半分 [m]
        bool   m_travelDirReverse_latch;       //!< 進行方向を逆にする

        bool m_ActivateFLG;                    //!< 処理開始フラグ

};


#endif // TROBOT_APIMAIN_HPP
