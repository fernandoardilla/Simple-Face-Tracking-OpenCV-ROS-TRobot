// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobot.cpp
 *  @brief TRobotの実装
 *
 */


#include "TRobot.hpp"


//
// パラメータデフォルト値
//
#define DEFAULT_TROBOTVERSION            "OLDrobot"       //!< TRobot::m_trobotVersion のデフォルト値
#define DEFAULT_ACTIVECONFIG             "default"        //!< TRobot::m_activeConfig のデフォルト値
#define DEFAULT_SERIAL_PORT_NAME         "/dev/ttyACM0"   //!< TRobot::m_serialPortName のデフォルト値
#define DEFAULT_WHEEL_RADIUS             0.062            //!< TRobot::m_wheelRadius のデフォルト値
#define DEFAULT_AXLE_TRACK               0.400            //!< TRobot::m_axleTrack のデフォルト値 
#define DEFAULT_AXLE_TRACKFR             0.400            //!< TRobot::m_axleTrack のデフォルト値 
#define DEFAULT_ROT_ENCODER_RESOLUTION   2048             //!< TRobot::m_rotEncoderResolution のデフォルト値
#define DEFAULT_WHEEL_GEAR_RATIO         71.1             //!< TRobot::m_wheelGearRatio のデフォルト値
#define DEFAULT_TRAVEL_DIR_REVERSE       0                //!< TRobot::m_travelDirReverse のデフォルト値
#define DEFAULT_COEFFRPSTOCMD            7.1424           //!< TRobot::m_coeffRpsToCmd のデフォルト値
#define DEFAULT_COEFFVXCMD               1.0              //!< TRobot::m_coeffVxCmd のデフォルト値
#define DEFAULT_COEFFVYCMD               1.0              //!< TRobot::m_coeffVyCmd のデフォルト値
#define DEFAULT_COEFFVZCMD               1.0              //!< TRobot::m_coeffVzCmd のデフォルト値
#define DEFAULT_COEFFXODO                1.0              //!< TRobot::m_coeffXOdo のデフォルト値
#define DEFAULT_COEFFYODO                1.0              //!< TRobot::m_coeffYOdo のデフォルト値
#define DEFAULT_COEFFHEADINGODO          1.0              //!< TRobot::m_coeffHeadingOdo のデフォルト値
#define DEFAULT_COEFFHEADINGODOADD       0.0              //!< TRobot::m_coeffHeadingOdoAdd のデフォルト値
#define DEFAULT_VELACCELMAX              100              //!< TRobot::m_velAccelMax のデフォルト値
#define DEFAULT_VELDECELMAX              100              //!< TRobot::m_velDecelMax のデフォルト値
#define DEFAULT_AUTORECOVERY             "MANUAL"         //!< TRobot::m_autoRecovery のデフォルト値
#define DEFAULT_FRAME_RATE               20               //!< TRobot::m_FrameRate のデフォルト値
#define DEFAULT_PUBLISH_TF               false            //!< TRobot::m_publish_tf のデフォルト値
#define DEFAULT_ODOM_ID                  "odom"           //!< TRobot::m_odom_frame_id のデフォルト値
#define DEFAULT_BASE_FRAME               "base_footprint" //!< TRobot::m_base_frame のデフォルト値
#define DEFAULT_TRACE_LOG                false            //!< TRobot::m_trace_log のデフォルト値


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief コンストラクタ
 *
 *  @copybrief
 *
 */
TRobot::TRobot( int argc, char **argv )
{
    using namespace std;
    ros::init(argc, argv, "trobot_node");

    static ros::NodeHandle nh;
    static ros::NodeHandle nh_priv("~");


    LOG_OUT_NOTTIME( << endl );
    LOG_OUT( << "TRobot::onInitialize()" << endl );


    //
    // パラメータ設定
    //
    nh_priv.param<string>( "trobotVersion",        m_trobotVersion,        DEFAULT_TROBOTVERSION );
    nh_priv.param<string>( "activeConfig",         m_activeConfig,         DEFAULT_ACTIVECONFIG );
    nh_priv.param<string>( "serialPortName",       m_serialPortName,       DEFAULT_SERIAL_PORT_NAME );
    nh_priv.param<double>( "wheelRadius",          m_wheelRadius,          DEFAULT_WHEEL_RADIUS );
    nh_priv.param<double>( "axleTrack",            m_axleTrack,            DEFAULT_AXLE_TRACK );
    nh_priv.param<double>( "axleTrackFR",          m_axleTrackFR,          DEFAULT_AXLE_TRACKFR );
    nh_priv.param<double>( "rotEncoderResolution", m_rotEncoderResolution, DEFAULT_ROT_ENCODER_RESOLUTION );
    nh_priv.param<double>( "wheelGearRatio",       m_wheelGearRatio,       DEFAULT_WHEEL_GEAR_RATIO );
    nh_priv.param<double>( "coeffRpsToCmd",        m_coeffRpsToCmd,        DEFAULT_COEFFRPSTOCMD );
    nh_priv.param<double>( "coeffVxCmd",           m_coeffVxCmd,           DEFAULT_COEFFVXCMD );
    nh_priv.param<double>( "coeffVyCmd",           m_coeffVyCmd,           DEFAULT_COEFFVYCMD );
    nh_priv.param<double>( "coeffVaCmd",           m_coeffVzCmd,           DEFAULT_COEFFVZCMD );
    nh_priv.param<double>( "coeffXOdo",            m_coeffXOdo,            DEFAULT_COEFFXODO );
    nh_priv.param<double>( "coeffYOdo",            m_coeffYOdo,            DEFAULT_COEFFYODO );
    nh_priv.param<double>( "coeffHeadingOdo",      m_coeffHeadingOdo,      DEFAULT_COEFFHEADINGODO );
    nh_priv.param<double>( "coeffHeadingOdoAdd",   m_coeffHeadingOdoAdd,   DEFAULT_COEFFHEADINGODOADD );
    nh_priv.param<int>   ( "velAccelMax",          m_velAccelMax,          DEFAULT_VELACCELMAX );
    nh_priv.param<int>   ( "velDecelMax",          m_velDecelMax,          DEFAULT_VELDECELMAX );
    nh_priv.param<string>( "autoRecovery",         m_autoRecovery,         DEFAULT_AUTORECOVERY );
    nh_priv.param<bool>  ( "trace_log",            m_trace_log,            DEFAULT_TRACE_LOG );

    nh_priv.param<int>   ( "FrameRate",            m_FrameRate,            DEFAULT_FRAME_RATE );
    nh_priv.param<bool>  ( "publish_tf",           m_publish_tf,           DEFAULT_PUBLISH_TF );
    nh_priv.param<string>( "odom_frame",           m_odom_frame_id,        DEFAULT_ODOM_ID );
    nh_priv.param<string>( "base_frame",           m_base_frame,           DEFAULT_BASE_FRAME );


    //
    // センサデータサービス クラス作成
    //
    static SensorDataSVC    m_SensorData;


    //
    // TRobotAPI クラス作成
    //
    m_APIMain = new TRobotAPI();
    m_APIMain->getDataAddr( m_sensorData, m_mntInfData );


    //
    // TRobotAPI の onInitialize
    //
    m_APIMain->onInitializeAPI();


    //
    // メッセージ設定
    //
    m_cmd_vel_sub      = nh.subscribe( "cmd_vel", 1, &TRobot::callback_cmdvel, this );
    m_reset_odom_sub   = nh.subscribe( "reset_odometry", 1, &TRobot::callback_rstodom, this );
    m_odometry_pub     = nh.advertise<nav_msgs::Odometry>( "odom", 1 );
    m_sonar_array_pub  = nh.advertise<std_msgs::Int32MultiArray>("sonar_array", 1);
    m_odometry2d_pub   = nh.advertise<geometry_msgs::Pose2D>( "odom2d", 1 );

    static tf::TransformBroadcaster odom_broadcaster;
    m_odom_broadcaster = &odom_broadcaster;


    //
    // サービス設定
    //
    this->Seting_Service( nh_priv, m_SensorData );

    //
    // メインループ周期設定
    //
    ros::Rate loop_rate( m_FrameRate ); 


    //
    // TRobotAPI の onActivated
    //
    m_APIMain->onActivatedAPI(
        m_trobotVersion,           // ロボットバージョンの名称（旧伝送フォーマット："OLDrobot",新伝送フォーマット："SCIBOT-X", etc.）
        m_activeConfig,            // Configuration SETの名称（pyxis, gemini, chorusline, etc.）
        m_serialPortName,          // シリアルポートの名称（Linux: /dev/ttyUSB0, etc.,  Windows: COM3, etc.）
        m_wheelRadius,             // 駆動輪の半径 [m]
        m_axleTrack,               // 輪距（左右の駆動輪中心間の距離） [m]
        m_axleTrackFR,             // 輪距（前後の駆動輪中心間の距離） [m]（メカナム時）
        m_rotEncoderResolution,    // 駆動輪モータ軸のエンコーダの分解能 [pulses/rev]
        m_wheelGearRatio,          // 駆動輪の減速機の減速比 [無単位]
        m_coeffRpsToCmd,           // 角速度 [rad/s] から回転速度指令値への変換係数
        m_coeffVxCmd,              // 指令値進行方向速度(vx)にかける係数
        m_coeffVyCmd,              // 指令値進行方向直角左向き速度(vy)にかける係数
        m_coeffVzCmd,              // 指令値車体角速度(vz)にかける係数
        m_coeffXOdo,               // オドメトリX座標増分にかける係数
        m_coeffYOdo,               // オドメトリY座標増分にかける係数
        m_coeffHeadingOdo,         // オドメトリ向き増分にかける係数
        m_coeffHeadingOdoAdd,      // オドメトリ向き増分に足しこむ係数
        m_velAccelMax,             // 1指令周期毎の加速度制限
        m_velDecelMax,             // 1指令周期毎の減速度制限
        m_autoRecovery,            // 緊急停止モードで要因が復旧した時に自動で復旧処理を行う（"MANUAL"(default):手動でモード遷移させる、"AUTO"：自動で運転モードに遷移する）
        m_trace_log,               // トレースログ出力フラグ
        m_FrameRate                // 1sec毎の処理回数
    );

    m_APIMain->setROSIF( &m_refVel, &m_odometryPose, &m_SensorData,
                         m_publish_tf, m_odom_frame_id, m_base_frame,
                         m_odometry_pub, m_odometry2d_pub, m_odom_broadcaster, m_sonar_array_pub );

    //  ロボットバージョンの名称サービス更新
    const char *trobotVersion = m_trobotVersion.c_str();
    m_SensorData.setRobotVersionName( trobotVersion );


    //
    // 強制速度更新情報の初期化
    //
    m_SensorData.setForcedVel( m_APIMain->m_forcedRotVelFlg,
                               m_APIMain->m_forcedRotVelVx,
                               m_APIMain->m_forcedRotVelVy,
                               m_APIMain->m_forcedRotVelVa );


    bool serIOisRunning = false;

    //
    // 定周期処理
    //
    while( ros::ok() )
    {

        // メッセージチェック
        while( true )
        {
            m_spinOnce_flg = false;
            ros::spinOnce();
            if( m_spinOnce_flg == false )
            {
                break;
            }
        }

        // メンテナンスデータ送信のために
        // 接続処理が動いたかをチェックする為ここで取得
        serIOisRunning = m_APIMain->m_serIO->isRunning();

        //
        // TRobotAPI の シリアルポートがスタートしていなければスタート
        //              停止要求中ならストップ
        //
        bool ret = m_APIMain->SerStartStop();
        if( ret == false )
        {
            loop_rate.sleep();
            continue;
        }

        //
        // ロボットベースへの設定情報書換指令
        //   CIRCINUS（メカナム）
        if( m_APIMain->m_serIO->isCIRCINUS() && !serIOisRunning )
        {
            //
            // ロボットベースへの設定情報取得
            //
            this->Geting_CmdConfig_MN( nh_priv );

            //
            // ロボットベースへの設定情報書換指令
            //
            m_APIMain->sendCmd( (int)TRobotIF::Command_SET_CONFIG_MN );
            loop_rate.sleep();
            continue;
        }

        //
        // 強制速度更新情報の更新
        //
        m_SensorData.getForcedVel( m_APIMain->m_forcedRotVelFlg,
                                   m_APIMain->m_forcedRotVelVx,
                                   m_APIMain->m_forcedRotVelVy,
                                   m_APIMain->m_forcedRotVelVa );


        //
        // ロボットベースへの指令
        //
        bool refVelUpdated = m_refVelUpdated.flag_look();
        m_APIMain->sendWheelRotVel( refVelUpdated, m_refVel.linear.x, m_refVel.linear.y, m_refVel.angular.z );
        m_refVelUpdated.flag_off();


        //
        // ロボットベースからのデータ
        //
        // センサデータがキューにあれば処理する
        m_APIMain->SensorDataUpdated();


        //
        // 次の周期までスリープ
        //
        loop_rate.sleep();
    }

    //
    // TRobotAPI の onDeactivated
    //
    m_APIMain->onDeactivatedAPI();

}

////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief デストラクタ
 *
 *  @copybrief
 *
 */
TRobot::~TRobot()
{
    //
    // TRobotAPI クラス削除
    //
    delete[] m_APIMain;
}

//
// サービス設定
//
void TRobot::Seting_Service( ros::NodeHandle &n, SensorDataSVC &SensorData )
{
    // ロボットバージョンの名称
    static ros::ServiceServer setRobotVersionName_SVC = n.advertiseService( "SRVsetRobotVersionName", &TRobot::TRobotAPI::SRVsetRobotVersionName, m_APIMain );
    static ros::ServiceServer getRobotVersionName_SVC = n.advertiseService( "SRVgetRobotVersionName", &TRobot::TRobotAPI::SRVgetRobotVersionName, m_APIMain );
    // 強制速度更新
    static ros::ServiceServer setForcedVel_SVC = n.advertiseService( "SRVsetForcedVel", &TRobot::TRobotAPI::SRVsetForcedVel, m_APIMain );
    static ros::ServiceServer getForcedVel_SVC = n.advertiseService( "SRVgetForcedVel", &TRobot::TRobotAPI::SRVgetForcedVel, m_APIMain );

    // バッテリ電圧
    static ros::ServiceServer setBatteryVoltage_SVC = n.advertiseService("SRVsetBatteryVoltage", &TRobot::TRobotAPI::SRVsetBatteryVoltage, m_APIMain );
    static ros::ServiceServer getBatteryVoltage_SVC = n.advertiseService("SRVgetBatteryVoltage", &TRobot::TRobotAPI::SRVgetBatteryVoltage, m_APIMain );

    // ダイ温度(旧ロボットベース、SCIBOT-Xのみ)
    static ros::ServiceServer setDaiTemp_SVC = n.advertiseService( "SRVsetDaiTemp", &TRobot::TRobotAPI::SRVsetDaiTemp, m_APIMain );
    static ros::ServiceServer getDaiTemp_SVC = n.advertiseService( "SRVgetDaiTemp", &TRobot::TRobotAPI::SRVgetDaiTemp, m_APIMain );

    // エラーステータス(旧ロボットベースのみ)
    static ros::ServiceServer setErrSTS_SVC = n.advertiseService( "SRVsetErrSTS", &TRobot::TRobotAPI::SRVsetErrSTS, m_APIMain );
    static ros::ServiceServer getErrSTS = n.advertiseService( "SRVgetErrSTS", &TRobot::TRobotAPI::SRVgetErrSTS, m_APIMain );

    // エラーステータス(SCIBOT-X、L-MESのみ)
    static ros::ServiceServer setLastFail_SVC = n.advertiseService( "SRVsetLastFail", &TRobot::TRobotAPI::SRVsetLastFail, m_APIMain );
    static ros::ServiceServer getLastFail_SVC = n.advertiseService( "SRVgetLastFail", &TRobot::TRobotAPI::SRVgetLastFail, m_APIMain );

    // 9D加速度センサ
    static ros::ServiceServer setAcc_SVC = n.advertiseService( "SRVsetAcc", &TRobot::TRobotAPI::SRVsetAcc, m_APIMain );
    static ros::ServiceServer getAcc_SVC = n.advertiseService( "SRVgetAcc", &TRobot::TRobotAPI::SRVgetAcc, m_APIMain );

    // 9Dジャイロセンサ
    static ros::ServiceServer setGyr_SVC = n.advertiseService( "SRVsetGyr", &TRobot::TRobotAPI::SRVsetGyr, m_APIMain );
    static ros::ServiceServer getGyr_SVC = n.advertiseService( "SRVgetGyr", &TRobot::TRobotAPI::SRVgetGyr, m_APIMain );

    // 9D磁気センサ
    static ros::ServiceServer setMag_SVC = n.advertiseService( "SRVsetMag", &TRobot::TRobotAPI::SRVsetMag, m_APIMain );
    static ros::ServiceServer getMag_SVC = n.advertiseService( "SRVgetMag", &TRobot::TRobotAPI::SRVgetMag, m_APIMain );

    // 床なしセンサ(SCIBOT-Xのみ)
    static ros::ServiceServer setNoFloor_SVC = n.advertiseService( "SRVsetNoFloor", &TRobot::TRobotAPI::SRVsetNoFloor, m_APIMain );
    static ros::ServiceServer getNoFloor_SVC = n.advertiseService( "SRVgetNoFloor", &TRobot::TRobotAPI::SRVgetNoFloor, m_APIMain );

    // バンパセンサ(SCIBOT-Xのみ)
    static ros::ServiceServer setBumper_SVC = n.advertiseService( "SRVsetBumper", &TRobot::TRobotAPI::SRVsetBumper, m_APIMain );
    static ros::ServiceServer getBumper_SVC = n.advertiseService( "SRVgetBumper", &TRobot::TRobotAPI::SRVgetBumper, m_APIMain );

    // 測距センサ(SCIBOT-X、L-MESのみ)
    static ros::ServiceServer setDistance_SVC = n.advertiseService( "SRVsetDistance", &TRobot::TRobotAPI::SRVsetDistance, m_APIMain );
    static ros::ServiceServer getDistance_SVC = n.advertiseService( "SRVgetDistance", &TRobot::TRobotAPI::SRVgetDistance, m_APIMain );

    // 測距センサ２(L-MESのみのみ)
    static ros::ServiceServer setDistance2_SVC = n.advertiseService( "SRVsetDistance2", &TRobot::TRobotAPI::SRVsetDistance2, m_APIMain );
    static ros::ServiceServer getDistance2_SVC = n.advertiseService( "SRVgetDistance2", &TRobot::TRobotAPI::SRVgetDistance2, m_APIMain );

    // 超音波センサ(L-MESのみのみ)
    static ros::ServiceServer setUltraSonic_SVC = n.advertiseService( "SRVsetUltraSonic", &TRobot::TRobotAPI::SRVsetUltraSonic, m_APIMain );
    static ros::ServiceServer getUltraSonic_SVC = n.advertiseService( "SRVgetUltraSonic", &TRobot::TRobotAPI::SRVgetUltraSonic, m_APIMain );

    // 基板温度(L-MESのみのみ)
    static ros::ServiceServer setBoardTemp_SVC = n.advertiseService( "SRVsetBoardTemp", &TRobot::TRobotAPI::SRVsetBoardTemp, m_APIMain );
    static ros::ServiceServer getBoardTemp_SVC = n.advertiseService( "SRVgetBoardTemp", &TRobot::TRobotAPI::SRVgetBoardTemp, m_APIMain );

    // バッテリ状態(SCIBOT-X、L-MESのみ)
    // L-MES:BatteryLevel1に使用バッテリー残量、BatteryLevel2に未使用バッテリー残量。あとは未使用。
    static ros::ServiceServer setBatteryStat_SVC = n.advertiseService( "SRVsetBatteryStat", &TRobot::TRobotAPI::SRVsetBatteryStat, m_APIMain );
    static ros::ServiceServer getBatteryStat_SVC = n.advertiseService( "SRVgetBatteryStat", &TRobot::TRobotAPI::SRVgetBatteryStat, m_APIMain );

    // 速度情報(SCIBOT-Xのみ)
    static ros::ServiceServer setVelInf_SVC = n.advertiseService( "SRVsetVelInf", &TRobot::TRobotAPI::SRVsetVelInf, m_APIMain );
    static ros::ServiceServer getVelInf_SVC = n.advertiseService( "SRVgetVelInf", &TRobot::TRobotAPI::SRVgetVelInf, m_APIMain );

    // バージョン情報(SCIBOT-X、L-MESのみ)
    static ros::ServiceServer setVersionInf_SVC = n.advertiseService( "SRVsetVersionInf", &TRobot::TRobotAPI::SRVsetVersionInf, m_APIMain );
    static ros::ServiceServer getVersionInf_SVC = n.advertiseService( "SRVgetVersionInf", &TRobot::TRobotAPI::SRVgetVersionInf, m_APIMain );

    // メンテナンス情報(SCIBOT-X、L-MESのみ)
    // NoFloorSMount:L-MESで未使用 BumperSMount:L-MESで未使用 DistanceSMount2:SCIBOT-Xで未使用
    // UltraSonicSMount:SCIBOT-Xで未使用 ResetCount:SCIBOT-Xで未使用
    // NineDOFSMount:L-MESで未使用 JudgSensor:L-MESで未使用
    static ros::ServiceServer setMntInf_SVC = n.advertiseService( "SRVsetMntInf", &TRobot::TRobotAPI::SRVsetMntInf, m_APIMain );
    static ros::ServiceServer getMntInf_SVC = n.advertiseService( "SRVgetMntInf", &TRobot::TRobotAPI::SRVgetMntInf, m_APIMain );

    // 現在の運転状態
    static ros::ServiceServer setRobotStat_SVC = n.advertiseService( "SRVsetRobotStat", &TRobot::TRobotAPI::SRVsetRobotStat, m_APIMain );
    static ros::ServiceServer getRobotStat_SVC = n.advertiseService( "SRVgetRobotStat", &TRobot::TRobotAPI::SRVgetRobotStat, m_APIMain );

    // 運転モードに移行(SCIBOT-Xのみ)
    static ros::ServiceServer CmdSetDrive_SVC = n.advertiseService( "SRVCmdSetDrive", &TRobot::TRobotAPI::SRVCmdSetDrive, m_APIMain );

    // 待機モードに移行(SCIBOT-Xのみ)
    static ros::ServiceServer CmdSetIdle_SVC = n.advertiseService( "SRVCmdSetIdle", &TRobot::TRobotAPI::SRVCmdSetIdle, m_APIMain );

    // 緊急停止モードに移行(SCIBOT-Xのみ)
    static ros::ServiceServer CmdErrStop_SVC = n.advertiseService( "SRVCmdErrStop", &TRobot::TRobotAPI::SRVCmdErrStop, m_APIMain );

    // 緊急停止モードから待機モードに移行(SCIBOT-Xのみ)
    static ros::ServiceServer CmdRecovery_SVC = n.advertiseService( "SRVCmdRecovery", &TRobot::TRobotAPI::SRVCmdRecovery, m_APIMain );

    // 現在のモードを取得する(SCIBOT-Xのみ)
    static ros::ServiceServer CmdGetMode_SVC = n.advertiseService( "SRVCmdGetMode", &TRobot::TRobotAPI::SRVCmdGetMode, m_APIMain );

    // 現在の情報を取得する(SCIBOT-Xのみ)
    static ros::ServiceServer CmdGetInf_SVC = n.advertiseService( "SRVCmdGetInf", &TRobot::TRobotAPI::SRVCmdGetInf, m_APIMain );

    // 運転モードに移行(L-MESのみ)
    static ros::ServiceServer CmdSetDriveLM_SVC = n.advertiseService( "SRVCmdSetDriveLM", &TRobot::TRobotAPI::SRVCmdSetDriveLM, m_APIMain );

    // 待機モードに移行(L-MESのみ)
    static ros::ServiceServer CmdSetIdleLM_SVC = n.advertiseService( "SRVCmdSetIdleLM", &TRobot::TRobotAPI::SRVCmdSetIdleLM, m_APIMain );

    // 緊急停止モードに移行(L-MESのみ)
    static ros::ServiceServer CmdErrStopLM_SVC = n.advertiseService( "SRVCmdErrStopLM", &TRobot::TRobotAPI::SRVCmdErrStopLM, m_APIMain );

    // 緊急停止システムの自己診断を要求(L-MESのみ)
    static ros::ServiceServer CmdDIAGLM_SVC = n.advertiseService( "SRVCmdDIAGLM", &TRobot::TRobotAPI::SRVCmdDIAGLM, m_APIMain );

    // 緊急停止モードから待機モードに移行(L-MESのみ)
    static ros::ServiceServer CmdRecoveryLM_SVC = n.advertiseService( "SRVCmdRecoveryLM", &TRobot::TRobotAPI::SRVCmdRecoveryLM, m_APIMain );

    // ロボットベースを再起動させる(L-MESのみ)
    static ros::ServiceServer CmdRebootLM_SVC = n.advertiseService( "SRVCmdRebootLM", &TRobot::TRobotAPI::SRVCmdRebootLM, m_APIMain );

    // 省電力モードに移行する(L-MESのみ)
    static ros::ServiceServer CmdSuspendLM_SVC = n.advertiseService( "SRVCmdSuspendLM", &TRobot::TRobotAPI::SRVCmdSuspendLM, m_APIMain );

    // エラー、軽故障データクリア(L-MESのみ)
    static ros::ServiceServer CmdErrClearLM_SVC = n.advertiseService( "SRVCmdErrClearLM", &TRobot::TRobotAPI::SRVCmdErrClearLM, m_APIMain );

}


//
// ロボットベースへの設定情報取得(CIRCINUS（メカナム）)
//
void TRobot::Geting_CmdConfig_MN( ros::NodeHandle &nh )
{
    XmlRpc::XmlRpcValue params;
    nh.getParam("/trobot_node/config", params);
    memset( m_APIMain->m_serIO->CmdConfig_MN, 0x00, TR_ROT_VEL_CMDCONFIG_LEN_MN );
    for(int i = 0; i < params.size(); i++)
    {
       XmlRpc::XmlRpcValue one_param = params[i];
       // printf("name:%s",((std::string)one_param["name"]).c_str());fflush(stdout);
       // printf(" byte_index:%d",(int)one_param["byte_index"] );fflush(stdout);
       // printf(" type:%s",((std::string)one_param["type"]).c_str() );fflush(stdout);
       // printf(" val:%d\n",(int)one_param["val"] );fflush(stdout);

       // インデックス
       int byte_index = (int)one_param["byte_index"];
       if( (byte_index <= 3) || ( byte_index >= TR_ROT_VEL_CMDCONFIG_LEN_MN ) )
       {
           continue;
       }

       // 値
       unsigned int data_val = (int)one_param["val"];
       if( (byte_index <= 3) || ( byte_index >= TR_ROT_VEL_CMDCONFIG_LEN_MN ) )
       {
           continue;
       }


       // データタイプ
       std::string data_type = (std::string)one_param["type"];
       if( data_type == "int8")
       {
           m_APIMain->m_serIO->CmdConfig_MN[ byte_index     ] = (unsigned char)((data_val & 0x000000ff));
       }
       else if( data_type == "int16")
       {
           m_APIMain->m_serIO->CmdConfig_MN[ byte_index     ] = (unsigned char)((data_val & 0x0000ff00)>>8);
           m_APIMain->m_serIO->CmdConfig_MN[ byte_index + 1 ] = (unsigned char)((data_val & 0x000000ff));
       }
       else if( data_type == "uint8")
       {
           m_APIMain->m_serIO->CmdConfig_MN[ byte_index     ] = (unsigned char)((data_val & 0x000000ff));
       }
       else if( data_type == "uint16")
       {
           m_APIMain->m_serIO->CmdConfig_MN[ byte_index     ] = (unsigned char)((data_val & 0x0000ff00)>>8);
           m_APIMain->m_serIO->CmdConfig_MN[ byte_index + 1 ] = (unsigned char)((data_val & 0x000000ff));
       }
       else
       {
           continue;
       }
    }
    //LOG_DBGDUMP(m_APIMain->m_serIO->CmdConfig_MN,TR_ROT_VEL_CMDCONFIG_LEN_MN);
}


//
// シリアル起動時初期化
//
void TRobot::TRobotAPI::serStartInit()
{
    ;
}

void TRobot::TRobotAPI::odometryOut(
             double        &odometry_x,
             double        &odometry_y,
             double        &odometry_heading
             )
{
    double odometryPose_x        = odometry_x;
    double odometryPose_y        = odometry_y;
    double odometryPose_heading  = odometry_heading;

    //
    // 調整
    //
    odometryPose_x       *= m_coeffXOdo;              // 距離(x)
    odometryPose_y       *= m_coeffYOdo;              // 直角左向き距離(y)
    odometryPose_heading *= m_coeffHeadingOdo;        // 角度
    odometryPose_heading += m_coeffHeadingOdoAdd;     // 角度


    // ROS座標系に調整
    m_odometryPose->x     = odometryPose_x;
    m_odometryPose->y     = -1.0 * odometryPose_y;
    m_odometryPose->theta = -1.0 * odometryPose_heading;


    ros::Time current_time;
    current_time = ros::Time::now();


    // 
    // オドメトリ配信
    // 
    // ヨー to クォータニオン
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( m_odometryPose->theta );

    // tf配信有り
    if( m_publish_tf )
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = m_odom_frame_id;
        odom_trans.child_frame_id = m_base_frame;
        odom_trans.transform.translation.x = m_odometryPose->x;
        odom_trans.transform.translation.y = m_odometryPose->y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        m_odom_broadcaster->sendTransform( odom_trans );
    }

    // オドメトリ配信(位置)
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = m_odom_frame_id;
    odom.pose.pose.position.x = m_odometryPose->x;
    odom.pose.pose.position.y = m_odometryPose->y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = m_base_frame;
    m_odometry_pub.publish( odom );

    // オドメトリ配信(2D位置)
    geometry_msgs::Pose2D odom2d;
    odom2d.x = m_odometryPose->x;
    odom2d.y = m_odometryPose->y;
    odom2d.theta = m_odometryPose->theta;
    m_odometry2d_pub.publish( odom2d );

}

//
// サービス更新
//
// センサ情報サービス更新(旧ロボットベース)
void TRobot::TRobotAPI::setSensor_EX()
{
    // バッテリ電圧サービス更新
    m_SensorData->setBatteryVoltage( TRobotAPIMain::m_sensorData.SensorData_EX.m_BatteryVoltage );

    // ダイ温度サービス更新
    m_SensorData->setDaiTemp( m_sensorData.SensorData_EX.m_daiTemp );

    // エラーステータスサービス更新
    m_SensorData->setErrSTS( (unsigned short)m_sensorData.SensorData_EX.m_ADCErrSTS,
                             (unsigned short)m_sensorData.SensorData_EX.m_COMErrSTS,
                             (unsigned short)m_sensorData.SensorData_EX.m_I2CErrSTS0,
                             (unsigned short)m_sensorData.SensorData_EX.m_I2CErrSTS1 );

    // 9D加速度センサ更新
    m_SensorData->setAcc( (double)m_sensorData.SensorData_EX.m_acc[0],
                          (double)m_sensorData.SensorData_EX.m_acc[1],
                          (double)m_sensorData.SensorData_EX.m_acc[2]  );

    // 9Dジャイロセンサセンサ更新
    m_SensorData->setGyr( (double)m_sensorData.SensorData_EX.m_gyr[0],
                          (double)m_sensorData.SensorData_EX.m_gyr[1],
                          (double)m_sensorData.SensorData_EX.m_gyr[2]  );

    // 9D磁気センサ更新
    m_SensorData->setMag( (double)m_sensorData.SensorData_EX.m_mag[0],
                          (double)m_sensorData.SensorData_EX.m_mag[1],
                          (double)m_sensorData.SensorData_EX.m_mag[2]  );
}

// センサ情報サービス更新(CIRCINUS（メカナム）)
void TRobot::TRobotAPI::setSensor_MN()
{
    // バッテリ電圧サービス更新
    m_SensorData->setBatteryVoltage( TRobotAPIMain::m_sensorData.SensorData_MN.m_BatteryVoltage );

    // ダイ温度サービス更新
    m_SensorData->setDaiTemp( m_sensorData.SensorData_MN.m_daiTemp );

    // 9D加速度センサ更新
    m_SensorData->setAcc( (double)m_sensorData.SensorData_MN.m_acc[0],
                          (double)m_sensorData.SensorData_MN.m_acc[1],
                          (double)m_sensorData.SensorData_MN.m_acc[2]  );

    // 9Dジャイロセンサセンサ更新
    m_SensorData->setGyr( (double)m_sensorData.SensorData_MN.m_gyr[0],
                          (double)m_sensorData.SensorData_MN.m_gyr[1],
                          (double)m_sensorData.SensorData_MN.m_gyr[2]  );

    // 9D磁気センサ更新
    m_SensorData->setMag( (double)m_sensorData.SensorData_MN.m_mag[0],
                          (double)m_sensorData.SensorData_MN.m_mag[1],
                          (double)m_sensorData.SensorData_MN.m_mag[2]  );

    // バンパセンササービス更新
    m_SensorData->setBumper( m_sensorData.SensorData_MN.m_BumperSensor,
                             0,
                             0,
                             0 );

    // 測距センササービス更新
    m_SensorData->setDistance(  m_sensorData.SensorData_MN.m_Distance[0],
                                m_sensorData.SensorData_MN.m_Distance[1],
                                m_sensorData.SensorData_MN.m_Distance[2],
                                m_sensorData.SensorData_MN.m_Distance[3] );

    // 測距センササービス更新
    m_SensorData->setDistance2( m_sensorData.SensorData_MN.m_Distance[4],
                                m_sensorData.SensorData_MN.m_Distance[5],
                                m_sensorData.SensorData_MN.m_Distance[6],
                                m_sensorData.SensorData_MN.m_Distance[7] );

    // 測距センサPublisher
    std_msgs::Int32MultiArray array;
    array.data.clear();
    for( unsigned int i = 0; i < TRobotAPIMain::m_sensorData.SensorData_MN.NUM_DISTANCE; i++ )
    {
        int tmp = m_sensorData.SensorData_MN.m_Distance[ i ];
        array.data.push_back( tmp );
    }
    m_sonar_array_pub.publish( array );

}


// センサ情報サービス更新(SIBOT-X)
void TRobot::TRobotAPI::setSensor_SX()
{
    // バッテリ電圧サービス更新
    m_SensorData->setBatteryVoltage( m_sensorData.SensorData_SX.m_BatteryVoltage );

    // ダイ温度サービス更新
    m_SensorData->setDaiTemp( m_sensorData.SensorData_SX.m_daiTemp );

    // エラーステータスサービス更新
    m_SensorData->setLastFail( (unsigned short)m_sensorData.SensorData_SX.m_LastFailD,
                               (unsigned short)m_sensorData.SensorData_SX.m_LastFailC,
                               (unsigned short)m_sensorData.SensorData_SX.m_LastFailN,
                               (unsigned short)m_sensorData.SensorData_SX.m_LastFailE );

    // 9D加速度センササービス更新
    m_SensorData->setAcc( m_sensorData.SensorData_SX.m_acc[0],
                          m_sensorData.SensorData_SX.m_acc[1],
                          m_sensorData.SensorData_SX.m_acc[2] );

    // 9Dジャイロセンサセンササービス更新
    m_SensorData->setGyr( m_sensorData.SensorData_SX.m_gyr[0],
                          m_sensorData.SensorData_SX.m_gyr[1],
                          m_sensorData.SensorData_SX.m_gyr[2] );

    // 9D磁気センササービス更新
    m_SensorData->setMag( m_sensorData.SensorData_SX.m_mag[0],
                          m_sensorData.SensorData_SX.m_mag[1],
                          m_sensorData.SensorData_SX.m_mag[2] );

    // 床なしセンササービス更新
    m_SensorData->setNoFloor( m_sensorData.SensorData_SX.m_NoFloor1,
                              m_sensorData.SensorData_SX.m_NoFloor2,
                              m_sensorData.SensorData_SX.m_NoFloor3,
                              m_sensorData.SensorData_SX.m_NoFloor4 );

    // バンパセンササービス更新
    m_SensorData->setBumper( m_sensorData.SensorData_SX.m_Bumper1,
                             m_sensorData.SensorData_SX.m_Bumper2,
                             m_sensorData.SensorData_SX.m_Bumper3,
                             m_sensorData.SensorData_SX.m_Bumper4 );

    // 測距センササービス更新
    m_SensorData->setDistance( m_sensorData.SensorData_SX.m_Distance1,
                               m_sensorData.SensorData_SX.m_Distance2,
                               m_sensorData.SensorData_SX.m_Distance3,
                               m_sensorData.SensorData_SX.m_Distance4 );

    // バッテリテータスサービス更新
    m_SensorData->setBatteryStat( (unsigned short)m_sensorData.SensorData_SX.m_BatteryStat,
                                  (unsigned short)m_sensorData.SensorData_SX.m_BatteryComStat,
                                  (unsigned short)m_sensorData.SensorData_SX.m_ErrDetectStat,
                                  m_sensorData.SensorData_SX.m_BatteryLevel1,
                                  m_sensorData.SensorData_SX.m_BatteryLevel2,
                                  m_sensorData.SensorData_SX.m_BatteryTemp1,
                                  m_sensorData.SensorData_SX.m_BatteryTemp2 );

    // 速度情報サービス更新
    double DrightVelCtl = 1.0 * m_sensorData.SensorData_SX.m_rightVelCtl / 32.0;          // 右モータの速度制御値
    double DleftVelCtl  = 1.0 * m_sensorData.SensorData_SX.m_rightVelCtl / 32.0;          // 左モータの速度制御値
    double DrightVel    = 1.0 * m_sensorData.SensorData_SX.m_rightVel / 32.0;             // 右モータの速度計測値
    double DleftVel     = 1.0 * m_sensorData.SensorData_SX.m_leftVel / 32.0;              // 左モータの速度計測値
    m_SensorData->setVelInf( DrightVelCtl, DleftVelCtl, DrightVel, DleftVel,
                            m_sensorData.SensorData_SX.m_rightCurrent,
                            m_sensorData.SensorData_SX.m_leftCurrent );
}

// センサ情報サービス更新(L-MES)
void TRobot::TRobotAPI::setSensor_LM()
{
    // 9D加速度センササービス更新
    m_SensorData->setAcc( m_sensorData.SensorData_LM.m_acc[0],
                          m_sensorData.SensorData_LM.m_acc[1],
                          m_sensorData.SensorData_LM.m_acc[2] );

    // 9Dジャイロセンサセンササービス更新
    m_SensorData->setGyr( m_sensorData.SensorData_LM.m_gyr[0],
                          m_sensorData.SensorData_LM.m_gyr[1],
                          m_sensorData.SensorData_LM.m_gyr[2] );

    // 9D磁気センササービス更新
    m_SensorData->setMag( m_sensorData.SensorData_LM.m_mag[0],
                          m_sensorData.SensorData_LM.m_mag[1],
                          m_sensorData.SensorData_LM.m_mag[2] );

    // 測距センササービス更新
    m_SensorData->setDistance( m_sensorData.SensorData_LM.m_Distance0,
                               m_sensorData.SensorData_LM.m_Distance1,
                               m_sensorData.SensorData_LM.m_Distance2,
                               m_sensorData.SensorData_LM.m_Distance3 );

    // 測距センサ2サービス更新
    m_SensorData->setDistance2( m_sensorData.SensorData_LM.m_Distance4,
                                m_sensorData.SensorData_LM.m_Distance5,
                                m_sensorData.SensorData_LM.m_Distance6,
                                m_sensorData.SensorData_LM.m_Distance7 );

    // 超音波センササービス更新
    m_SensorData->setUltraSonic( m_sensorData.SensorData_LM.m_UltraSonic0,
                                 m_sensorData.SensorData_LM.m_UltraSonic1 );

    // バッテリー状態サービス更新する
    m_SensorData->setBatteryStat( 0, 0, 0,
                                  m_sensorData.SensorData_LM.m_BatteryLevel_Use,
                                  m_sensorData.SensorData_LM.m_BatteryLevel_nonUse,
                                  0,
                                  0 );

    // 基板温度サービス更新
    m_SensorData->setBoardTemp( m_sensorData.SensorData_LM.m_BoardTemp );

    // エラーステータスサービス更新
    m_SensorData->setLastFail( (unsigned short)m_sensorData.SensorData_LM.m_LastFailD,
                               (unsigned short)m_sensorData.SensorData_LM.m_LastFailC,
                               (unsigned short)m_sensorData.SensorData_LM.m_LastFailN,
                               (unsigned short)m_sensorData.SensorData_LM.m_LastFailE );

    // 9D加速度センサ更新
    m_SensorData->setAcc( (double)m_sensorData.SensorData_LM.m_acc[0],
                          (double)m_sensorData.SensorData_LM.m_acc[1],
                          (double)m_sensorData.SensorData_LM.m_acc[2]  );

    // 9Dジャイロセンサセンサ更新
    m_SensorData->setGyr( (double)m_sensorData.SensorData_LM.m_gyr[0],
                          (double)m_sensorData.SensorData_LM.m_gyr[1],
                          (double)m_sensorData.SensorData_LM.m_gyr[2]  );

    // 9D磁気センサ更新
    m_SensorData->setMag( (double)m_sensorData.SensorData_LM.m_mag[0],
                          (double)m_sensorData.SensorData_LM.m_mag[1],
                          (double)m_sensorData.SensorData_LM.m_mag[2]  );

    // 速度情報サービス更新
    double DrightVelCtl = 1.0 * m_sensorData.SensorData_LM.m_rightVelCtl / 32.0;          // 右モータの速度制御値
    double DleftVelCtl  = 1.0 * m_sensorData.SensorData_LM.m_rightVelCtl / 32.0;          // 左モータの速度制御値
    m_SensorData->setVelInf( DrightVelCtl, DleftVelCtl, 0, 0,
                             m_sensorData.SensorData_LM.m_rightCurrent,
                             m_sensorData.SensorData_LM.m_leftCurrent );
}

// メンテナンス情報サービス更新(SIBOT-X)
void TRobot::TRobotAPI::setMent_SX()
{
    // バージョン情報サービス更新
    m_SensorData->setVersionInf
    (
        m_mntInfData.MntInfData_SX.m_ModelRevision,      // モデルREV
        m_mntInfData.MntInfData_SX.m_HardwareRevision,   // ハードウエアREV
        m_mntInfData.MntInfData_SX.m_FirmwareRevision,   // ファームウェアREV
        m_mntInfData.MntInfData_SX.m_SerialNumber,       // シリアル番号
        m_mntInfData.MntInfData_SX.m_CreationYear,       // コード作成年
        m_mntInfData.MntInfData_SX.m_CreationMonth,      // コード作成月
        m_mntInfData.MntInfData_SX.m_CreationDay,        // コード作成日
        m_mntInfData.MntInfData_SX.m_CreationHour,       // コード作成時
        m_mntInfData.MntInfData_SX.m_CreationMinute      // コード作成分
    );

    // メンテナンス情報サービス更新
    m_SensorData->setMntInf
    (
        m_mntInfData.MntInfData_SX.m_OperatingTime,      // 稼働時間
        m_mntInfData.MntInfData_SX.m_FailD,              // 危険故障コード
        m_mntInfData.MntInfData_SX.m_FailC,              // 重度故障コード
        m_mntInfData.MntInfData_SX.m_FailN,              // 軽度故障コード
        m_mntInfData.MntInfData_SX.m_FailE,              // エラーコード
        m_mntInfData.MntInfData_SX.m_FailWDT,            // WDTエラー発生回数
        m_mntInfData.MntInfData_SX.m_FailRAMSoft,        // RAMソフトエラー
        m_mntInfData.MntInfData_SX.m_NoFloorSMount,      // 床なしセンサ実装
        m_mntInfData.MntInfData_SX.m_BumperSMount,       // バンパセンサ実装
        m_mntInfData.MntInfData_SX.m_DistanceSMount,     // 測距センサ実装
        0,                                               // 測距センサ実装(L-MESのみ:4-7)
        0,                                               // 超音波センサ実装(L-MESのみ:0)
        0,                                               // リセット(未定義命令)エラー発生回数
        m_mntInfData.MntInfData_SX.m_TravelDistance,     // 走行距離
        m_mntInfData.MntInfData_SX.m_9DOFSMount,         // 9軸センサ実装
        m_mntInfData.MntInfData_SX.m_JudgSensor          // センサ判定
    );
}

// メンテナンス情報サービス更新(L-MES)
void TRobot::TRobotAPI::setMent_LM()
{
    // バージョン情報サービス更新
    m_SensorData->setVersionInf
    (
        m_mntInfData.MntInfData_LM.m_ModelRevision,      // モデルREV
        m_mntInfData.MntInfData_LM.m_HardwareRevision,   // ハードウエアREV
        m_mntInfData.MntInfData_LM.m_FirmwareRevision,   // ファームウェアRE
        m_mntInfData.MntInfData_LM.m_SerialNumber,       // シリアル番号
        m_mntInfData.MntInfData_LM.m_CreationYear,       // コード作成年
        m_mntInfData.MntInfData_LM.m_CreationMonth,      // コード作成月
        m_mntInfData.MntInfData_LM.m_CreationDay,        // コード作成日
        m_mntInfData.MntInfData_LM.m_CreationHour,       // コード作成時
        m_mntInfData.MntInfData_LM.m_CreationMinute      // コード作成分
    );

    // メンテナンス情報サービス更新
    m_SensorData->setMntInf
    (
        m_mntInfData.MntInfData_LM.m_OperatingTime,      // 稼働時間
        m_mntInfData.MntInfData_LM.m_FailD,              // 危険故障コード
        m_mntInfData.MntInfData_LM.m_FailC,              // 重度故障コード
        m_mntInfData.MntInfData_LM.m_FailN,              // 軽度故障コード
        m_mntInfData.MntInfData_LM.m_FailE,              // エラーコード
        m_mntInfData.MntInfData_LM.m_FailWDT,            // WDTエラー発生回数
        m_mntInfData.MntInfData_LM.m_FailRAMSoft,        // RAMソフトエラー
        0,                                               // 床なしセンサ実装
        0,                                               // バンパセンサ実装
        m_mntInfData.MntInfData_LM.m_DistanceSMount0_3,  // 測距センサ実装
        m_mntInfData.MntInfData_LM.m_DistanceSMount4_7,  // 測距センサ実装(L-MESのみ:4-7)
        m_mntInfData.MntInfData_LM.m_UltraSonicSMount0_1,// 超音波センサ実装(L-MESのみ:0)
        m_mntInfData.MntInfData_LM.m_ResetCount,         // リセット(未定義命令)エラー発生回数
        m_mntInfData.MntInfData_LM.m_TravelDistance,     // 走行距離
        0,                                               // 9軸センサ実装
        0                                                // センサ判定
    );
}


