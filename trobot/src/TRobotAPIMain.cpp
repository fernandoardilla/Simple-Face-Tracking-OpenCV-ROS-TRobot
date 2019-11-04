// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotAPIMain.cpp
 *  @brief TRobotAPIMainの実装
 *
 */

#include <cstring>
#include <cmath> 

#include "TRobotAPIMain.hpp"


////////////////////////////////////////////////////////////////////////////////
/*!
 * TRobotAPIMainコンストラクタ
 *
 */
TRobotAPIMain::TRobotAPIMain()
    : m_robotStat( 1 )
{
    resetOdometry();
    m_ActivateFLG = false;
    m_serIO = new TRobotIF::SerialIOIface();

    TRobotIF::SetserIO( m_serIO );
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief デストラクタ
 *
 *  @copybrief
 *
 */
TRobotAPIMain::~TRobotAPIMain()
{
    delete m_serIO;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief onInitialize 起動時の初期化処理
 *
 *  @copybrief
 */
bool TRobotAPIMain::onInitializeAPI()
{
    m_ActivateFLG = false;
    m_forcedRotVelFlg = false;
    m_forcedRotVelVx  = 0.0;
    m_forcedRotVelVy  = 0.0;
    m_forcedRotVelVa  = 0.0;
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief onActivated アクティベートの処理
 *
 *  @copybrief
 */
bool TRobotAPIMain::onActivatedAPI( 
        std::string &trobotVersion,                   // ロボットバージョンの名称（旧伝送フォーマット："OLDrobot",新伝送フォーマット："SCIBOT-X", etc.）
        std::string &activeConfig,                    // Configuration SETの名称（pyxis, gemini, chorusline, etc.）
        std::string &serialPortName,                  // シリアルポートの名称（Linux: /dev/ttyUSB0, etc.,  Windows: COM3, etc.）
        double      &wheelRadius,                     // 駆動輪の半径 [m]
        double      &axleTrack,                       // 輪距（左右の駆動輪中心間の距離） [m]
        double      &axleTrackFR,                     // 輪距（前後の駆動輪中心間の距離） [m]（メカナム時）
        double      &rotEncoderResolution,            // 駆動輪モータ軸のエンコーダの分解能 [pulses/rev]
        double      &wheelGearRatio,                  // 駆動輪の減速機の減速比 [無単位]
        double      &coeffRpsToCmd,                   // 角速度 [rad/s] から回転速度指令値への変換係数
        double      &coeffVxCmd,                      // 指令値進行方向速度(vx)にかける係数
        double      &coeffVyCmd,                      // 指令値進行方向直角左向き速度(vy)にかける係数
        double      &coeffVaCmd,                      // 指令値車体角速度(va)にかける係数
        double      &coeffXOdo,                       // オドメトリX座標増分にかける係数
        double      &coeffYOdo,                       // オドメトリY座標増分にかける係数
        double      &coeffHeadingOdo,                 // オドメトリ向き増分にかける係数
        double      &coeffHeadingOdoAdd,              // オドメトリ向き増分に足しこむ係数
        int         &velAccelMax,                     // 1指令周期毎の加速度制限
        int         &velDecelMax,                     // 1指令周期毎の減速度制限
        std::string &autoRecovery,                    // 緊急停止モードで要因が復旧した時に自動で復旧処理を行う（"MANUAL"(default):手動でモード遷移させる、"AUTO"：自動で運転モードに遷移する）
        bool        &trace_log,                       // トレースログ出力フラグ
        int         &FrameRate                        // 1sec毎の処理回数
 )
{
    using namespace std;


    //
    // コンフィグパラメータ設定
    //
    m_trobotVersion        = trobotVersion;           // ロボットバージョンの名称（旧伝送フォーマット："OLDrobot",新伝送フォーマット："SCIBOT-X", etc.）
    m_activeConfig         = activeConfig;            // Configuration SETの名称（pyxis, gemini, chorusline, etc.）
    m_serialPortName       = serialPortName;          // シリアルポートの名称（Linux: /dev/ttyUSB0, etc.,  Windows: COM3, etc.）
    m_wheelRadius          = wheelRadius;             // 駆動輪の半径 [m]
    m_axleTrack            = axleTrack;               // 輪距（左右の駆動輪中心間の距離） [m]
    m_axleTrackFR          = axleTrackFR;             // 輪距（前後の駆動輪中心間の距離） [m]（メカナム時）
    m_rotEncoderResolution = rotEncoderResolution;    // 駆動輪モータ軸のエンコーダの分解能 [pulses/rev]
    m_wheelGearRatio       = wheelGearRatio;          // 駆動輪の減速機の減速比 [無単位]
    m_coeffRpsToCmd        = coeffRpsToCmd;           // 角速度 [rad/s] から回転速度指令値への変換係数
    m_coeffVxCmd           = coeffVxCmd;              // 指令値進行方向速度(vx)にかける係数
    m_coeffVyCmd           = coeffVyCmd;              // 指令値進行方向直角左向き速度(vy)にかける係数
    m_coeffVaCmd           = coeffVaCmd;              // 指令値車体角速度(va)にかける係数
    m_coeffXOdo            = coeffXOdo;               // オドメトリX座標増分にかける係数
    m_coeffYOdo            = coeffYOdo;               // オドメトリY座標増分にかける係数
    m_coeffHeadingOdo      = coeffHeadingOdo;         // オドメトリ向き増分に足しこむ係数
    m_coeffHeadingOdoAdd   = coeffHeadingOdoAdd;      // オドメトリ向き増分に足しこむ係数
    m_velAccelMax          = velAccelMax;             // 1指令周期毎の加速度制限
    m_velDecelMax          = velDecelMax;             // 1指令周期毎の減速度制限
    m_autoRecovery         = autoRecovery;            // 緊急停止モードで要因が復旧した時に自動で復旧処理を行う（"MANUAL"(default):手動でモード遷移させる、"AUTO"：自動で運転モードに遷移する）
    m_trace_log            = trace_log;               // トレースログ出力フラグ
    m_FrameRate            = FrameRate;               // 1sec毎の処理回数


    LOG_OUT_NOTTIME( << endl );
    LOG_OUT( << "Configuration Variable"                           <<  endl );
    LOG_OUT( << "trobotVersion       : " << m_trobotVersion        <<  endl );
    LOG_OUT( << "activeConfig        : " << m_activeConfig         <<  endl );
    LOG_OUT( << "serialPortName      : " << m_serialPortName       <<  endl );
    LOG_OUT( << "wheelRadius         : " << m_wheelRadius          <<  endl );
    LOG_OUT( << "axleTrack           : " << m_axleTrack            <<  endl );
    LOG_OUT( << "axleTrackFR         : " << m_axleTrackFR          <<  endl );
    LOG_OUT( << "rotEncoderResolution: " << m_rotEncoderResolution <<  endl );
    LOG_OUT( << "wheelGearRatio      : " << m_wheelGearRatio       <<  endl );
    LOG_OUT( << "coeffRpsToCmd       : " << m_coeffRpsToCmd        <<  endl );
    LOG_OUT( << "coeffVxCmd          : " << m_coeffVxCmd           <<  endl );
    LOG_OUT( << "coeffVyCmd          : " << m_coeffVyCmd           <<  endl );
    LOG_OUT( << "coeffVaCmd          : " << m_coeffVaCmd           <<  endl );
    LOG_OUT( << "coeffXOdo           : " << m_coeffXOdo            <<  endl );
    LOG_OUT( << "coeffXOdo           : " << m_coeffYOdo            <<  endl );
    LOG_OUT( << "coeffHeadingOdo     : " << m_coeffHeadingOdo      <<  endl );
    LOG_OUT( << "coeffHeadingOdoAdd  : " << m_coeffHeadingOdoAdd   <<  endl );
    LOG_OUT( << "velAccelMax         : " << velAccelMax            <<  endl );
    LOG_OUT( << "velDecelMax         : " << velDecelMax            <<  endl );
    LOG_OUT( << "autoRecovery        : " << m_autoRecovery         <<  endl );
    LOG_OUT( << "trace_log           : " << trace_log              <<  endl );
    LOG_OUT( << "FrameRate           : " << FrameRate      << endl <<  endl );


    // 使用するロボットベースの種類
    m_serIO->setScibotXFlag( (m_trobotVersion=="SCIBOT-X") ? true: false );
    m_serIO->setLMESFlag( (m_trobotVersion=="L-MES") ? true: false );
    m_serIO->setCIRCINUSFlag( (m_trobotVersion=="CIRCINUS") ? true: false );

    // 緊急停止モードからの自動復旧
    m_serIO->setAutoRecovery( (m_autoRecovery=="AUTO") ? true: false );

    // 繰り返し計算しないようにするため，ここで計算する
    m_halfAxleTrack        = m_axleTrack * 0.5;                                   // 左右輪距 / 2
    m_halfAxleTrackFR      = m_axleTrackFR * 0.5;                                 // 前後輪距 / 2
    m_coeffEncToArcHalf    = m_wheelRadius * M_PI
                             / (m_rotEncoderResolution * m_wheelGearRatio);       // エンコーダ変換係数 / 2
    m_coeffEncToArcQuarter = m_wheelRadius * M_PI
                             / (m_rotEncoderResolution * m_wheelGearRatio) * 0.5; // エンコーダ変換係数 / 4

    m_serIO->setTraceLogFlag( (m_trace_log==true) ? true: false );


    // SCIBOT-Xフォーマット
    if( m_serIO->isScibotX() )
    {
        // 起動・診断モードに初期化
        TRobotIF::SetMode( (int)TRobotIF::RobotMode_INIT );

        // サンプリングレートを設定
        m_serIO->setCtrlRate( TR_CTRL_RATE_SX );

        // 送受信間隔を設定
        m_serIO->setWaitDuration( TR_CTRL_PERIOD_SX );

        // 速度指令最小値を設定
        m_rot_vel_cmd_min = TR_ROT_VEL_CMD_MIN_SX;

        // 速度指令最大値を設定
        m_rot_vel_cmd_max =  TR_ROT_VEL_CMD_MAX_SX;

        // 送信データ長を設定
        m_serIO->setSndLength( TR_CMD_LENGTH_SX );

        // 受信データ長を設定
        m_serIO->setRcvLength( TR_RCV_LENGTH_SX );
    }
    // L-MESフォーマット
    else if( m_serIO->isLMES() )
    {
        // 起動・診断モードに初期化
        TRobotIF::SetMode( (int)TRobotIF::RobotMode_INIT );

        // サンプリングレートを設定
        m_serIO->setCtrlRate( TR_CTRL_RATE_LM );

        // 送受信間隔を設定
        m_serIO->setWaitDuration( TR_CTRL_PERIOD_LM );

        // 速度指令最小値を設定
        m_rot_vel_cmd_min = TR_ROT_VEL_CMD_MIN_LM;

        // 速度指令最大値を設定
        m_rot_vel_cmd_max =  TR_ROT_VEL_CMD_MAX_LM;

        // 送信データ長を設定
        m_serIO->setSndLength( TR_CMD_LENGTH_LM );

        // 受信データ長を設定
        m_serIO->setRcvLength( TR_RCV_LENGTH_LM );
    }
    // CIRCINUS（メカナム）フォーマット
    else if( m_serIO->isCIRCINUS() )
    {
        // 運転モードに初期化
        TRobotIF::SetMode( (int)TRobotIF::RobotMode_DRIVE );

        // サンプリングレートを設定
        m_serIO->setCtrlRate( TR_CTRL_RATE_MN );

        // 送受信間隔を設定
        m_serIO->setWaitDuration( TR_CTRL_PERIOD_MN );

        // 速度指令最小値を設定
        m_rot_vel_cmd_min = TR_ROT_VEL_CMD_MIN_MN;

        // 速度指令最大値を設定
        m_rot_vel_cmd_max =  TR_ROT_VEL_CMD_MAX_MN;

        // 送信データ長を設定
        m_serIO->setSndLength( TR_ROT_VEL_CMD_LEN_MN );

        // 受信データ長を設定
        m_serIO->setRcvLength( TR_BASE_SENSOR_DATA_LEN_MN );
    }
    // 旧ロボットベースフォーマット
    else
    {
        // 運転モードに初期化
        TRobotIF::SetMode( (int)TRobotIF::RobotMode_DRIVE );

        // サンプリングレートを設定
        m_serIO->setCtrlRate( TR_CTRL_RATE );

        // 送受信間隔を設定
        m_serIO->setWaitDuration( TR_CTRL_PERIOD );

        // 速度指令最小値を設定
        m_rot_vel_cmd_min = TR_ROT_VEL_CMD_MIN;

        // 速度指令最大値を設定
        m_rot_vel_cmd_max =  TR_ROT_VEL_CMD_MAX;

        // 送信データ長を設定
        m_serIO->setSndLength( TR_ROT_VEL_CMD_LEN );

        // 受信データ長を設定
        m_serIO->setRcvLength( TR_BASE_SENSOR_DATA_LEN );
    }

    m_ActivateFLG = true;

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief onDeactivated ディアクティベートの処理
 *
 *  @copybrief
 */
bool TRobotAPIMain::onDeactivatedAPI()
{
    using namespace std;

    m_ActivateFLG = false;

    TRobotIF::RetCode retVal;
    m_serIO->sendWheelRotVel( 0, 0, 0, 0 ); // 安全対策として停止させる
    while( m_serIO->receiveSensorData( m_sensorData ) == TRobotIF::SUCCESS )
    {
        ;// 受信済みのセンサデータが尽きるまで読み捨てる
    }
    retVal = m_serIO->stop();
    if( retVal != TRobotIF::SUCCESS )
    {
        LOG_ERROUT( << "Failed to stop communication" << endl );
    }

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief SerStartStop シリアルポートのスタート・ストップ
 *         運転状態を更新。
 *         シリアルポートがスタートしていなければスタート、
 *         シリアルポートが停止要求中ならストップ。
 *
 *  @copybrief
 */
bool TRobotAPIMain::SerStartStop()
{
    using namespace std;
    static unsigned int ser_restart_cnt = 0;


    if( m_ActivateFLG == false )
    {
        return false;
    }


    //
    // 運転状態がキューにあれば処理する
    //
    bool robotStatUpdated = false;
    while( m_serIO->receiveRobotStatData( m_robotStat ) == TRobotIF::SUCCESS )
    {
        robotStatUpdated = true;
    }

    if( robotStatUpdated )
    {
        // 運転状態サービス更新
        setRobotStat();
    }


    //
    // シリアルポートがスタートしていなければスタート
    //
    if( !m_serIO->isRunning() )
    {
        ser_restart_cnt ++;
        if( ser_restart_cnt % (m_serIO->getCtrlRate() * 3) != 1 )
        {
            // 3秒おき
            return false;
        }

        serStartInit();

        bool ret = serStart();
        if( !ret )
        {
            // つながらなければ次回の周期でリトライ
            return false;
        }
        ser_restart_cnt = 0;
    }


    //
    // シリアルポートが停止要求中ならストップ
    //
    if( m_serIO->stopRequested() )
    {
        serStop();
        return false;
    }


    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ロボットベースへの速度指令
 *
 *  @copybrief
 */
bool TRobotAPIMain::sendWheelRotVel(
        bool        updated,
        double      vx,             //!< 指令値進行方向速度(vx)
        double      vy,             //!< 指令値進行方向直角左向き速度(vy)
        double      va              //!< 指令値角度(va)
        )
{
    using namespace std;

    //
    // 強制速度更新時
    if( m_forcedRotVelFlg )
    {
        TRobotIF::RetCode retVal = convertToWheelRotVelCmds(
                                           m_forcedRotVelVx * m_coeffVxCmd, m_forcedRotVelVy * m_coeffVyCmd, m_forcedRotVelVa * m_coeffVaCmd,
                                           m_leftRotVelCmd_TVal,   m_rightRotVelCmd_TVal,
                                           m_leftRotVelCmd_TVal_r, m_rightRotVelCmd_TVal_r );
        if( retVal != TRobotIF::SUCCESS )
        {
            LOG_ERROUT( << "Error on Vel2D to rot vel conversion" << endl );
            return false;
        }

        m_leftRotVelCmd  = m_leftRotVelCmd_TVal;
        m_rightRotVelCmd = m_rightRotVelCmd_TVal;
        m_leftRotVelCmd_r  = m_leftRotVelCmd_TVal_r;
        m_rightRotVelCmd_r = m_rightRotVelCmd_TVal_r;

        //
        // 速度指令
        m_serIO->sendWheelRotVel( m_leftRotVelCmd, m_rightRotVelCmd, m_leftRotVelCmd_r, m_rightRotVelCmd_r );

        return true;
    }


    //
    // 新しい指示あり目標速度更新
    if( updated )
    {
        TRobotIF::RetCode retVal = convertToWheelRotVelCmds( 
                                           vx * m_coeffVxCmd, vy * m_coeffVyCmd, va * m_coeffVaCmd,
                                           m_leftRotVelCmd_TVal,   m_rightRotVelCmd_TVal,
                                           m_leftRotVelCmd_TVal_r, m_rightRotVelCmd_TVal_r );
        if( retVal != TRobotIF::SUCCESS )
        {
            LOG_ERROUT( << "Error on Vel2D to rot vel conversion" << endl );
            return false;
        }
    }

    //
    // 加速度制限を加味して目標速度から速度を決定する
#if 0
    // 左
    m_leftRotVelCmd  = TRobotIF::AccellimitRange( m_leftRotVelCmd_TVal, m_leftRotVelCmd,
                                                  m_leftRotVelCmd_TVal_r, m_leftRotVelCmd_r,
                                                  m_velAccelMax, m_velDecelMax );
    // 右
    m_rightRotVelCmd  = TRobotIF::AccellimitRange( m_rightRotVelCmd_TVal, m_rightRotVelCmd,
                                                   m_rightRotVelCmd_TVal_r, m_rightRotVelCmd_r,
                                                   m_velAccelMax, m_velDecelMax );
#else
    TRobotIF::AccellimitRange_Smooth_MN(
                     ( m_serIO->isCIRCINUS() ) ? 4 : 2,                // 駆動輪の数
                     m_leftRotVelCmd_TVal,   m_rightRotVelCmd_TVal,    // 目標速度 前(左,右)
                     m_leftRotVelCmd_TVal_r, m_rightRotVelCmd_TVal_r,  // 目標速度 後(左,右)
                     m_leftRotVelCmd,        m_rightRotVelCmd,         // 現在速度 前(左,右)
                     m_leftRotVelCmd_r,      m_rightRotVelCmd_r,       // 現在速度 後(左,右)
                     m_velAccelMax,          m_velDecelMax,            // 加速MAX値, 減速MAX値
                     m_leftRotVelCmd,        m_rightRotVelCmd,         // 結果速度 前(左,右)
                     m_leftRotVelCmd_r,      m_rightRotVelCmd_r );     // 結果速度 後(左,右)
#endif

    //
    // 速度指令
    m_serIO->sendWheelRotVel( m_leftRotVelCmd, m_rightRotVelCmd,  m_leftRotVelCmd_r, m_rightRotVelCmd_r );

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ロボットベースへのコマンドを送信する
 *
 *  @copybrief
 *
 *  @param  cmd    [in]  コマンド(TRobotIF::CommandNum)
 *
 *  @return true
 */
bool TRobotAPIMain::sendCmd( int cmd )
{
    using namespace std;

    //
    // 速度指令
    m_serIO->sendCmd( cmd );

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ロボットベースへの速度指令
 *
 *  @copybrief
 */
bool TRobotAPIMain::SensorDataUpdated()
{
    using namespace std;


    //
    // ロボットベースからのデータ
    //
    // センサデータがキューにあれば処理する
    bool refOdoUpdated = false;
    while( m_serIO->receiveSensorData( m_sensorData ) == TRobotIF::SUCCESS )
    {
        // SCIBOT-Xフォーマット
        refOdoUpdated = true;

        // オドメトリ計算
        // SCIBOT-X以外フォーマット
        if( !m_serIO->isScibotX() )
        {
            // L-MESフォーマット
            if( m_serIO->isLMES() )
            {
                updateOdometry_LM( m_sensorData.SensorData_LM, m_odometryPose_x, m_odometryPose_y, m_odometryPose_heading );
            }
            // CIRCINUS（メカナム）
            else if( m_serIO->isCIRCINUS() )
            {
                updateOdometry_MN( m_sensorData.SensorData_MN, m_odometryPose_x, m_odometryPose_y, m_odometryPose_heading );
            }
            // 旧ロボットベース
            else
            {
                updateOdometry_EX( m_sensorData.SensorData_EX, m_odometryPose_x, m_odometryPose_y, m_odometryPose_heading );
            }
        }
    }

    //
    // SCIBOT-Xは最後の電文のみ処理をする
    // (応答と定周期データの区別が付かない為、
    //  また、エンコーダ値でなく速度の為、
    //  所定の周期で積分する)
    //
    if( refOdoUpdated && m_serIO->isScibotX() )
    {
        // SCIBOT-Xフォーマット
        updateOdometry_SX( m_sensorData.SensorData_SX, m_odometryPose_x, m_odometryPose_y, m_odometryPose_heading );
    }

    //LOG_DBGOUT( << "X=" << m_odometryPose_x << " Y=" << m_odometryPose_y << " H=" << m_odometryPose_heading << endl );


    // センサデータ更新
    if( refOdoUpdated )
    {
        // SCIBOT-X
        if( m_serIO->isScibotX() )
        {
            setSensor_SX();
        }
        // L-MES
        else if( m_serIO->isLMES() )
        {
            setSensor_LM();
        }
        // CIRCINUS（メカナム）
        else if( m_serIO->isCIRCINUS() )
        {
            setSensor_MN();
        }
        // 旧ロボットベース
        else
        {
            setSensor_EX();
        }
    }

    // オドメトリ ポート出力
    odometryOut( m_odometryPose_x, m_odometryPose_y, m_odometryPose_heading );


    //
    // メンテナンス情報がキューにあれば処理する
    //
    bool refMntInfUpdated = false;
    if( m_serIO->isScibotX() || m_serIO->isLMES() )
    {
        while( m_serIO->receiveMntInfData( m_mntInfData ) == TRobotIF::SUCCESS )
        {
            refMntInfUpdated = true;
        }
    }

    if( refMntInfUpdated )
    {
        // SIBOT-X
        if( m_serIO->isScibotX() )
        {
            // メンテナンス情報サービス更新
            setMent_SX();
        }
        // L-MES
        else
        {
            // メンテナンス情報サービス更新
            setMent_LM();
        }
    }


    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief オドメトリデータを更新する(旧ロボットベース)
 *
 *  @copybrief
 *
 *  @param sensorData           [in]    最新のセンサデータ
 *  @param odometryPose_x       [out]   現在のオドメトリデータ 距離(x)
 *  @param odometryPose_y       [out]   現在のオドメトリデータ 直角左向き距離(y)
 *  @param odometryPose_heading [out]   現在のオドメトリデータ 角度
 */
void TRobotAPIMain::updateOdometry_EX(
       TRobotIF::BaseSensorData_EX  &sensorData,
       double                       &odometryPose_x,             //!< 距離(x)
       double                       &odometryPose_y,             //!< 直角左向き距離(y)
       double                       &odometryPose_heading        //!< 角度
       )
{
    //
    // NUM_ENC_COUNTS分積分
    //
    for(size_t i = 0; i < sensorData.NUM_ENC_COUNTS; ++i)
    {
        int    renc   = sensorData.m_rightEnc[i];
        int    lenc   = sensorData.m_leftEnc[i];
        int    sumEnc = renc + lenc;

        double h      = odometryPose_heading;                   // 方向角の一時保持用

        //
        // 旋回している場合（旋回半径が大きいとしても）
        //
        int diffEnc = renc - lenc;
        const int DIFF_ENC_EPS = 5;
        if( abs(diffEnc) > DIFF_ENC_EPS )
        {
            double rho = (m_halfAxleTrack * sumEnc) / diffEnc; // 符号付き瞬時旋回半径 [m]

            odometryPose_heading    += m_coeffEncToArcHalf * diffEnc / m_halfAxleTrack;
            odometryPose_x += rho * (-sin(h) + sin(odometryPose_heading));
            odometryPose_y += rho * (+cos(h) - cos(odometryPose_heading));
        }
        //
        // 直進している場合
        //
        else
        {
            double dfwd = m_coeffEncToArcHalf * sumEnc;        // 直進距離(or 後退距離) [m]

            odometryPose_x += dfwd * cos( h );
            odometryPose_y += dfwd * sin( h );
        }
    }

}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief オドメトリデータを更新する(CIRCINUS（メカナム）)
 *
 *  @copybrief
 *
 *  @param sensorData           [in]    最新のセンサデータ
 *  @param odometryPose_x       [out]   現在のオドメトリデータ 距離(x)
 *  @param odometryPose_y       [out]   現在のオドメトリデータ 直角左向き距離(y)
 *  @param odometryPose_heading [out]   現在のオドメトリデータ 角度
 */
void TRobotAPIMain::updateOdometry_MN(
       TRobotIF::BaseSensorData_MN &sensorData,
       double                      &odometryPose_x,             //!< 距離(x)
       double                      &odometryPose_y,             //!< 直角左向き距離(y)
       double                      &odometryPose_heading        //!< 角度
       )
{


    //
    // NUM_ENC_COUNTS分積分
    //
    for(size_t i = 0; i < sensorData.NUM_ENC_COUNTS; ++i)
    {
        // 方向角の一時保持用
        double h      = odometryPose_heading;

        //
        // 車輪の回転からみた、
        // みかけの車輪直進距離(or 後退距離) [m]
        // の4分の1
        double Frd = m_coeffEncToArcQuarter * sensorData.m_rightEnc[i];
        double Fld = m_coeffEncToArcQuarter * sensorData.m_leftEnc[i];
        double Rrd = m_coeffEncToArcQuarter * sensorData.m_rightEnc_r[i];
        double Rld = m_coeffEncToArcQuarter * sensorData.m_leftEnc_r[i];
        
        // ロボットベースの瞬時距離
        double Vxdt = (Frd + Fld + Rrd + Rld);
        double Vydt = (Frd - Fld - Rrd + Rld);
        double Vhdt = (Frd - Fld + Rrd - Rld) / (m_halfAxleTrack + m_halfAxleTrackFR);

        // 結果を積算
        odometryPose_x       += (Vxdt *  cos(h)) - (Vydt * sin(h));
        odometryPose_y       += (Vxdt *  sin(h)) + (Vydt * cos(h));
        odometryPose_heading += Vhdt;
    }
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief オドメトリデータを更新する(SCIBOT-X)
 *
 *  @copybrief
 *
 *  @param sensorData           [in]    最新のセンサデータ
 *  @param odometryPose_x       [out]   現在のオドメトリデータ 距離(x)
 *  @param odometryPose_y       [out]   現在のオドメトリデータ 直角左向き距離(y)
 *  @param odometryPose_heading [out]   現在のオドメトリデータ 角度
 */
void TRobotAPIMain::updateOdometry_SX(
       TRobotIF::BaseSensorData_SX &sensorData,
       double                      &odometryPose_x,             //!< 距離(x)
       double                      &odometryPose_y,             //!< 直角左向き距離(y)
       double                      &odometryPose_heading        //!< 角度
       )
{
    static const double COEFF_32kmphTOmp50ms = 1000.0/32.0/60.0/60.0/TR_CTRL_RATE_SX; // [km/32h] → [m/50ms] 係数

    int    rvel   = sensorData.m_rightVel;    // 瞬間速度 [km/32h]
    int    lvel   = sensorData.m_leftVel;     // 瞬間速度 [km/32h]
    int    sumVel = rvel + lvel;

    double h      = odometryPose_heading;     // 方向角の一時保持用

    //
    // 旋回している場合（旋回半径が大きいとしても）
    //
    int diffVel = rvel - lvel;
    const int DIFF_VEL_EPS = 5;
    if( abs(diffVel) > DIFF_VEL_EPS )
    {
        double rho = (m_halfAxleTrack * sumVel) / diffVel;             // 符号付き瞬時旋回半径 [m]

        odometryPose_heading += diffVel / m_halfAxleTrack * 0.5 * COEFF_32kmphTOmp50ms;
        odometryPose_x       += rho * (-sin(h) + sin(odometryPose_heading));
        odometryPose_y       += rho * (+cos(h) - cos(odometryPose_heading));
    }
    //
    // 直進している場合
    //
    else
    {
        double dfwd = 0.5 * sumVel * COEFF_32kmphTOmp50ms; // 直進距離(or 後退距離) [m]

        odometryPose_x += dfwd * cos(h);
        odometryPose_y += dfwd * sin(h);
    }


}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief オドメトリデータを更新する(L-MES)
 *
 *  @copybrief
 *
 *  @param sensorData           [in]    最新のセンサデータ
 *  @param odometryPose_x       [out]   現在のオドメトリデータ 距離(x)
 *  @param odometryPose_y       [out]   現在のオドメトリデータ 直角左向き距離(y)
 *  @param odometryPose_heading [out]   現在のオドメトリデータ 角度
 */
void TRobotAPIMain::updateOdometry_LM(
       TRobotIF::BaseSensorData_LM &sensorData,
       double                      &odometryPose_x,             //!< 距離(x)
       double                      &odometryPose_y,             //!< 直角左向き距離(y)
       double                      &odometryPose_heading        //!< 角度
       )
{
    int    renc   = sensorData.m_leftEncInc;
    int    lenc   = sensorData.m_rightEncInc;
    int    sumEnc = renc + lenc;

    double h      = odometryPose_heading;                   // 方向角の一時保持用

    //
    // 旋回している場合（旋回半径が大きいとしても）
    //
    int diffEnc = renc - lenc;
    const int DIFF_ENC_EPS = 5;
    if( abs(diffEnc) > DIFF_ENC_EPS )
    {
        double rho = (m_halfAxleTrack * sumEnc) / diffEnc; // 符号付き瞬時旋回半径 [m]

        odometryPose_heading    += m_coeffEncToArcHalf * diffEnc / m_halfAxleTrack;
        odometryPose_x += rho * (-sin(h) + sin(odometryPose_heading));
        odometryPose_y += rho * (+cos(h) - cos(odometryPose_heading));
    }
    //
    // 直進している場合
    //
    else
    {
        double dfwd = m_coeffEncToArcHalf * sumEnc;        // 直進距離(or 後退距離) [m]

        odometryPose_x += dfwd * cos( h );
        odometryPose_y += dfwd * sin( h );
    }

}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief シリアルポートのスタート
 *
 *  @copybrief
 */
bool TRobotAPIMain::serStart()
{
    using namespace std;

    TRobotIF::RetCode retVal;

    // オドメトリデータのリセット
    resetOdometry();

    m_leftRotVelCmd         = 0;
    m_rightRotVelCmd        = 0;
    m_leftRotVelCmd_r       = 0;
    m_rightRotVelCmd_r      = 0;
    m_leftRotVelCmd_TVal    = 0;
    m_rightRotVelCmd_TVal   = 0;
    m_leftRotVelCmd_TVal_r  = 0;
    m_rightRotVelCmd_TVal_r = 0;

    // 緊急停止モードからの自動復旧?
    if( m_serIO->isAutoRecovery() )
    {
        // 運転状態へ自動遷移させる
        TRobotIF::SetInitFlag();
    }

    // シリアルIOスタート
    retVal = m_serIO->start( m_serialPortName.c_str() );
    if( retVal != TRobotIF::SUCCESS )
    {
        LOG_ERROUT( << "Failed to start communication on " << m_serialPortName << endl );
        return false;
    }

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief シリアルポートのストップ
 *
 *  @copybrief
 */
bool TRobotAPIMain::serStop()
{
    using namespace std;

    TRobotIF::RetCode retVal;

    m_serIO->sendWheelRotVel( 0, 0, 0, 0 ); // 安全対策として停止させる
    while( m_serIO->receiveSensorData( m_sensorData ) == TRobotIF::SUCCESS )
    {
        // 受信済みのセンサデータが尽きるまで読み捨てる
    }
    retVal = m_serIO->stop();
    if( retVal != TRobotIF::SUCCESS )
    {
        LOG_ERROUT( << "Failed to stop communication" << endl );
        return false;
    }

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ロボットベースの前進速度および旋回速度の組み合わせから
 *         左右の車輪の回転速度指令値への変換を行う
 *
 *  @copybrief
 *
 *  @param baseXVel         [in]  ロボットベースの前進速度
 *  @param baseYVel         [in]  ロボットベースのY方向速度
 *  @param baseAngVel       [in]  ロボットベースの旋回速度
 *  @param leftRotVelCmd    [out] 前左車輪の回転速度指令値
 *  @param rightRotVelCmd   [out] 前右車輪の回転速度指令値
 *  @param leftRotVelCmd_r  [out] 後左車輪の回転速度指令値
 *  @param rightRotVelCmd_r [out] 後右車輪の回転速度指令値
 */
TRobotIF::RetCode TRobotAPIMain::convertToWheelRotVelCmds(
                                                    double baseXVel,
                                                    double baseYVel,
                                                    double baseAngVel,
                                                    int& leftRotVelCmd,
                                                    int& rightRotVelCmd,
                                                    int& leftRotVelCmd_r,
                                                    int& rightRotVelCmd_r )
{
    // CIRCINUS（メカナム）
    if( m_serIO->isCIRCINUS() )
    {
        // 前提:
        //  * 上から見て反時計回りが正方向の旋回
        //  * スリップなし
        double Fwl = (0.5/m_wheelRadius) * (baseXVel - baseYVel - (m_halfAxleTrack+m_halfAxleTrackFR) * baseAngVel );
        double Fwr = (0.5/m_wheelRadius) * (baseXVel + baseYVel + (m_halfAxleTrack+m_halfAxleTrackFR) * baseAngVel );
        double Rwl = (0.5/m_wheelRadius) * (baseXVel + baseYVel - (m_halfAxleTrack+m_halfAxleTrackFR) * baseAngVel );
        double Rwr = (0.5/m_wheelRadius) * (baseXVel - baseYVel + (m_halfAxleTrack+m_halfAxleTrackFR) * baseAngVel );

        leftRotVelCmd    = TRobotIF::limitRange( (int)(m_coeffRpsToCmd * Fwl), (int)m_rot_vel_cmd_min, (int)m_rot_vel_cmd_max );
        rightRotVelCmd   = TRobotIF::limitRange( (int)(m_coeffRpsToCmd * Fwr), (int)m_rot_vel_cmd_min, (int)m_rot_vel_cmd_max );
        leftRotVelCmd_r  = TRobotIF::limitRange( (int)(m_coeffRpsToCmd * Rwl), (int)m_rot_vel_cmd_min, (int)m_rot_vel_cmd_max );
        rightRotVelCmd_r = TRobotIF::limitRange( (int)(m_coeffRpsToCmd * Rwr), (int)m_rot_vel_cmd_min, (int)m_rot_vel_cmd_max );
    }
    else
    {
        // 前提:
        //  * 上から見て反時計回りが正方向の旋回
        //  * スリップなし
        double halfATAngVel = m_halfAxleTrack * baseAngVel;
        double wl = (baseXVel - halfATAngVel) / m_wheelRadius;
        double wr = (baseXVel + halfATAngVel) / m_wheelRadius;

        leftRotVelCmd  = TRobotIF::limitRange( (int)(m_coeffRpsToCmd * wl), (int)m_rot_vel_cmd_min, (int)m_rot_vel_cmd_max );
        rightRotVelCmd = TRobotIF::limitRange( (int)(m_coeffRpsToCmd * wr), (int)m_rot_vel_cmd_min, (int)m_rot_vel_cmd_max );
    }

    return TRobotIF::SUCCESS;
}
