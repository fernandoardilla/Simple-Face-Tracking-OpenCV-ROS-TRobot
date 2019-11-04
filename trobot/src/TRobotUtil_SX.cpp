// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotUtil_SX.cpp
 *  @brief ユーティリティ(SCIBOT-X)の実装
 *
 */

#include <cmath>
#include <algorithm>
#include <string.h>

#include "TRobotSerialIOIface.hpp"
#include "TRobotUtil.hpp"



////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文を解析処理する(SCIBOT-X)
 *
 *  @param  buf          [in]        受信電文
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::AnalysisMessage_SX( const unsigned char * buf )
{
    int event = 0;

    switch( buf[ResponsePos_SX] )
    {
    // 運転モードに移行する。
    case Response_SET_DRIVE_SX:
        LOG_PRINTF( "Recve Response_SET_DRIVE_SX mode=%02x\n", buf[ ResModePos ] );
        event = Event_SET_DRIVE_SX;
        break;

    // 待機モードに移行する。
    case Response_SET_IDLE_SX:
        LOG_PRINTF( "Recve Response_SET_IDLE_SX mode=%02x\n", buf[ ResModePos ] );
        event = Event_SET_IDLE_SX;
        break;

    // 緊急停止モードに移行する。
    case Response_ERROR_STOP_SX:
        LOG_PRINTF( "Recve Response_ERROR_STOP_SX mode=%02x\n", buf[ ResModePos ] );
        event = Event_ERROR_STOP_SX;
        break;

    // 緊急停止モードから待機モードに戻る。
    case Response_RECOVERY_SX:
        LOG_PRINTF( "Recve Response_RECOVERY_SX mode=%02x\n", buf[ ResModePos ] );
        event = Event_RECOVERY_SX;
        break;

    // 速度制御値を指定する。
    case Response_SPEED_CONTROL_SX:
        //LOG_PRINTF( "Recve Response_SPEED_CONTROL_SX mode=%02x\n", buf[ ResModePos ] );
        event = Event_SPEED_CONTROL_SX;
        break;

    // 現在のモードを取得する。
    case Response_GET_MODE_SX:
        //LOG_PRINTF( "Recve Response_GET_MODE_SX mode=%02x\n", buf[ ResModePos ] );
        event = Event_GET_MODE_SX;
        break;

    // 現在の情報を取得する。
    case Response_GET_INF_SX:
        LOG_PRINTF( "Recve Response_GET_INF_SX mode=%02x\n", buf[ ResModePos ] );
        event = Event_GET_INF_SX;
        break;

    // タイムアウト。
    case Response_TIMEOUT_SX:
        //LOG_PRINTF( "Recve Response_TIMEOUT_SX mode=%02x\n", GetMode() );
        event = Event_TIMEOUT_SX;
        break;

    default:
        LOG_PRINTF( "Recve Unknown Response mode=%02x\n", buf[ ResModePos ] );
        event = Event_MAX_SX;
        return TRobotIF::SUCCESS;
    }

    // 状態、イベントで分岐
    GoEventFunc_SX( event, buf );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  受信電文の先頭を検索する(SCIBOT-X)
 *
 *  @param  bufFirst   [in]        検索開始ポインタ
 *  @param  bufLast    [in]        検索終了ポインタ
 *
 *  @return 指定されたバッファ中のヘッダの先頭へのポインタ。
 *          ヘッダが見つからなければバッファの末尾(有効部分の終端+1)へのポインタ
 */
const unsigned char *TRobotIF::searchForValidHeader_SX( const unsigned char *bufFirst,
                                                        const unsigned char *bufLast )
{
    // 指定されたバッファ中の先頭から末尾までループ TR_START_SX を探す
    for( unsigned char *buf=(unsigned char *)bufFirst; buf<=(unsigned char *)bufLast; buf++ )
    {
        if( *buf != TR_START_SX )
        {
            continue;
        }
        return buf;
    }

    return (unsigned char *)NULL;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文をデコードする(SCIBOT-X)
 *
 *  @param  buf          [in]    受信電文
 *  @param  sensorData   [out]   センサーデータ
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::decodeBaseSensorData_SX( const unsigned char * const buf,
                                                     BaseSensorData_SX *sensorData )
{
    short tmpshort;

    //
    // モータの速度制御値
    //
    // 右モータの速度制御値
    tmpshort =  buf[ ResRightMSpeedCtlPos_SX     ] << 8;
    tmpshort |= buf[ ResRightMSpeedCtlPos_SX + 1 ];
    sensorData->m_rightVelCtl = tmpshort;

    // 左モータの速度制御値
    tmpshort =  buf[ ResLeftMSpeedCtlPos_SX     ] << 8;
    tmpshort |= buf[ ResLeftMSpeedCtlPos_SX + 1 ];
    sensorData->m_leftVelCtl = tmpshort;


    //
    // モータの速度計測値
    //
    // 右モータの速度計測値
    tmpshort =  buf[ ResRightMSpeedPos_SX     ] << 8;
    tmpshort |= buf[ ResRightMSpeedPos_SX + 1 ];
    sensorData->m_rightVel = tmpshort;

    // 左モータの速度計測値
    tmpshort =  buf[ ResLeftMSpeedPos_SX     ] << 8;
    tmpshort |= buf[ ResLeftMSpeedPos_SX + 1 ];
    sensorData->m_leftVel = tmpshort;


    //
    // 駆動電流計測値
    //
    // 右側駆動電流計測値(mA)
    tmpshort =  buf[ ResRightMCurrentPos_SX     ] << 8;
    tmpshort |= buf[ ResRightMCurrentPos_SX + 1 ];
    sensorData->m_rightCurrent =  tmpshort;

    // 左側駆動電流計測値(mA)
    tmpshort =  buf[ ResLeftMCurrentPos_SX     ] << 8;
    tmpshort |= buf[ ResLeftMCurrentPos_SX + 1 ];
    sensorData->m_leftCurrent =  tmpshort;


    //
    // 床なしセンサ値
    //
    // ch1
    sensorData->m_NoFloor1 = ( ( buf[ TR_NoFloorPos_SX ] & TR_NoFloorCH1MSK_SX ) == TR_NoFloorCH1MSK_SX );

    // ch2
    sensorData->m_NoFloor2 = ( ( buf[ TR_NoFloorPos_SX ] & TR_NoFloorCH2MSK_SX ) == TR_NoFloorCH2MSK_SX );

    // ch3
    sensorData->m_NoFloor3 = ( ( buf[ TR_NoFloorPos_SX ] & TR_NoFloorCH3MSK_SX ) == TR_NoFloorCH3MSK_SX );

    // ch4
    sensorData->m_NoFloor4 = ( ( buf[ TR_NoFloorPos_SX ] & TR_NoFloorCH4MSK_SX ) == TR_NoFloorCH4MSK_SX );


    //
    // バンパセンサ値
    //
    // ch1
    sensorData->m_Bumper1 = ( ( buf[ TR_BumperSPos_SX ] & TR_BumperSCH1MSK_SX ) == TR_BumperSCH1MSK_SX );

    // ch2
    sensorData->m_Bumper2 = ( ( buf[ TR_BumperSPos_SX ] & TR_BumperSCH2MSK_SX ) == TR_BumperSCH2MSK_SX );

    // ch3
    sensorData->m_Bumper3 = ( ( buf[ TR_BumperSPos_SX ] & TR_BumperSCH3MSK_SX ) == TR_BumperSCH3MSK_SX );

    // ch4
    sensorData->m_Bumper4 = ( ( buf[ TR_BumperSPos_SX ] & TR_BumperSCH4MSK_SX ) == TR_BumperSCH4MSK_SX );


    //
    // 測距センサ値(cm)
    //
    // ch1
    tmpshort =  buf[ TR_Distance1Pos_SX     ] << 8;
    tmpshort |= buf[ TR_Distance1Pos_SX + 1 ];
    sensorData->m_Distance1 =  tmpshort;

    // ch2
    tmpshort =  buf[ TR_Distance2Pos_SX     ] << 8;
    tmpshort |= buf[ TR_Distance2Pos_SX + 1 ];
    sensorData->m_Distance2 =  tmpshort;

    // ch3
    tmpshort =  buf[ TR_Distance3Pos_SX     ] << 8;
    tmpshort |= buf[ TR_Distance3Pos_SX + 1 ];
    sensorData->m_Distance3 =  tmpshort;

    // ch4
    tmpshort =  buf[ TR_Distance4Pos_SX     ] << 8;
    tmpshort |= buf[ TR_Distance4Pos_SX + 1 ];
    sensorData->m_Distance4 =  tmpshort;

    // ch5
    tmpshort =  buf[ TR_Distance5Pos_SX     ] << 8;
    tmpshort |= buf[ TR_Distance5Pos_SX + 1 ];
    sensorData->m_Distance5 =  tmpshort;


    //
    // 9Dセンサ値
    //
    // 加速度センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
    for( int i=0; i < (int)sensorData->NUM_XYZ; i++ )
    {
        tmpshort =  buf[ TR_AccXPos_SX + (i * 2)     ] << 8;
        tmpshort |= buf[ TR_AccXPos_SX + (i * 2) + 1 ];
        sensorData->m_acc[i] = tmpshort;
    }

    // ジャイロのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
    for( int i=0; i < (int)sensorData->NUM_XYZ; i++ )
    {
        tmpshort =  buf[ TR_GyrXPos_SX + (i * 2)     ] << 8;
        tmpshort |= buf[ TR_GyrXPos_SX + (i * 2) + 1 ];
        sensorData->m_gyr[i] = tmpshort;
    }

    // 磁気センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
    for( int i=0; i < (int)sensorData->NUM_XYZ; i++ )
    {
        tmpshort =  buf[ TR_MagXPos_SX + (i * 2)     ] << 8;
        tmpshort |= buf[ TR_MagXPos_SX + (i * 2) + 1 ];
        sensorData->m_mag[i] =  tmpshort;
    }


    //
    // バッテリ電圧
    //
    sensorData->m_BatteryVoltage =  buf[ TR_BatteryVoltagePos_SX ];


    //
    // モータドライバIC周辺温度計測値
    //
    // 右モータドライバIC周辺温度計測値(℃)
    sensorData->m_rightIC =  buf[ TR_RightICPos_SX ];

    // 左モータドライバIC周辺温度計測値(℃)
    sensorData->m_leftIC =  buf[ TR_LeftICPos_SX ];


    //
    // ダイ温度(℃)
    //
    sensorData->m_daiTemp =  buf[ TR_DaiTempPos_SX ];


    //
    // エラーコード
    //
    // 最後に発生した危険故障コード
    sensorData->m_LastFailD =  buf[ TR_LastFailDPos_SX ];

    // 最後に発生した重度故障コード
    sensorData->m_LastFailC =  buf[ TR_LastFailCPos_SX ];

    // 最後に発生した軽度故障コード
    sensorData->m_LastFailN =  buf[ TR_LastFailNPos_SX ];

    // 最後に発生したエラーコード
    sensorData->m_LastFailE =  buf[ TR_LastFailEPos_SX ];


    //
    // バッテリー状態
    //
    sensorData->m_BatteryStat =  buf[ TR_BatteryConnectPos_SX ];


    //
    // バッテリー通信状態
    //
    sensorData->m_BatteryComStat =  buf[ TR_BatteryComPos_SX ];


    //
    // エラー検出状態
    //
    sensorData->m_ErrDetectStat =  buf[ TR_ErrDetectStatPos_SX ];


    //
    // バッテリ残量
    //
    // バッテリ残量1
    sensorData->m_BatteryLevel1 =  buf[ TR_BatteryLevel1Pos_SX ];

    // バッテリ残量2
    sensorData->m_BatteryLevel2 =  buf[ TR_BatteryLevel2Pos_SX ];


    //
    // バッテリ温度
    //
    // バッテリ温度1
    sensorData->m_BatteryTemp1 =  buf[ TR_BatteryTemp1Pos_SX ];

    // バッテリ温度2
    sensorData->m_BatteryTemp2 =  buf[ TR_BatteryTemp2Pos_SX ];


    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文をデコードする(SCIBOT-X)
 *
 *  @param  buf          [in]    受信電文
 *  @param  sensorData   [out]   センサーデータ
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::decodeMntInfData_SX( const unsigned char * const buf,
                                                 MntInfData_SX *mntInfData )
{
    //
    // バージョン情報
    //
    // モデルREV
    mntInfData->m_ModelRevision = buf[ TR_ModelRevisionPos_SX ];

    // ハードウエアREV
    mntInfData->m_HardwareRevision = buf[ TR_HardwareRevisionPos_SX ];

    // ファームウェアREV
    mntInfData->m_FirmwareRevision = buf[ TR_FirmwareRevisionPos_SX ];

    // シリアル番号
    mntInfData->m_SerialNumber = buf[ TR_SerialNumberPos_SX ];


    //
    // 稼働時間（リトルエンディアン？？）
    //
    //mntInfData->m_OperatingTime =  buf[ TR_OperatingTimePos_SX + 0 ] << 24;
    //mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_SX + 1 ] << 16;
    //mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_SX + 2 ] <<  8;
    //mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_SX + 3 ] <<  0;
    mntInfData->m_OperatingTime =  buf[ TR_OperatingTimePos_SX + 0 ] <<  0;
    mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_SX + 1 ] <<  8;
    mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_SX + 2 ] << 16;
    mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_SX + 3 ] << 24;


    //
    // エラー情報
    //
    // 危険故障コード
    mntInfData->m_FailD = buf[ TR_FailDPos_SX ];

    // 重度故障コード
    mntInfData->m_FailC = buf[ TR_FailCPos_SX ];

    // 軽度故障コード
    mntInfData->m_FailN = buf[ TR_FailNPos_SX ];

    // エラーコード
    mntInfData->m_FailE = buf[ TR_FailEPos_SX ];

    // WDTエラー発生回数
    mntInfData->m_FailWDT = buf[ TR_FailWDTPos_SX ];

    // RAMソフトエラー
    mntInfData->m_FailRAMSoft = buf[ TR_FailRAMSoftPos_SX ];


    //
    // センサ実装
    //
    // 床なしセンサ実装
    mntInfData->m_NoFloorSMount = buf[ TR_NoFloorSMountPos_SX ];

    // バンパセンサ実装
    mntInfData->m_BumperSMount = buf[ TR_BumperSMountPos_SX ];

    // 測距センサ実装
    mntInfData->m_DistanceSMount = buf[ TR_DistanceSMountPos_SX ];


    //
    // 走行距離（リトルエンディアン？？）
    //
    mntInfData->m_TravelDistance =  buf[ TR_TravelDistancePos_SX + 0 ] <<  0;
    mntInfData->m_TravelDistance |= buf[ TR_TravelDistancePos_SX + 1 ] <<  8;
    mntInfData->m_TravelDistance |= buf[ TR_TravelDistancePos_SX + 2 ] << 16;
    mntInfData->m_TravelDistance |= buf[ TR_TravelDistancePos_SX + 3 ] << 24;


    //
    // 9軸センサ実装
    //
    mntInfData->m_9DOFSMount = buf[ TR_9DOFSMountPos_SX ];


    //
    // センサ判定
    //
    mntInfData->m_JudgSensor = buf[ TR_JudgSensorPos_SX ];


    //
    // 開発用(コード作成 年月日時分)
    //
    // コード作成年
    mntInfData->m_CreationYear = buf[ TR_CreationYearPos_SX ];

    // コード作成月
    mntInfData->m_CreationMonth = buf[ TR_CreationMonthPos_SX ];

    // コード作成日
    mntInfData->m_CreationDay = buf[ TR_CreationDayPos_SX ];

    // コード作成時
    mntInfData->m_CreationHour = buf[ TR_CreationHourPos_SX ];

    // コード作成分
    mntInfData->m_CreationMinute = buf[ TR_CreationMinutePos_SX ];


    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用指令シーケンスを組み立てる(SCIBOT-X)
 *
 *  @param  cmd          [in]  コマンド(TRobotIF::CommandNum)
 *  @param  leftRotVel   [in]  左車輪の回転速度指令値
 *                           （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  rightRotVel  [in]  右車輪の回転速度指令値
 *                           （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSet_SX( int cmd, int leftRotVel, int rightRotVel,
                                               unsigned char *cmdbuf )
{
    TRobotIF::RetCode retCode;

    switch(cmd)
    {
    case Command_SET_DRIVE_SX:          // 運転モードに移行する。
        LOG_PRINTF( "send Command_SET_DRIVE_SX\n" );
        retCode = buildCmdSeqSetDrive_SX( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_SET_IDLE_SX:           // 待機モードに移行する。
        LOG_PRINTF( "send Command_SET_IDLE_SX\n" );
        retCode = buildCmdSeqSetIdle_SX( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_ERROR_STOP_SX:         // 緊急停止モードに移行する。
        LOG_PRINTF( "send Command_ERROR_STOP_SX\n" );
        retCode = buildCmdSeqSetErrStop_SX( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_RECOVERY_SX:           // 緊急停止モードから待機モードに戻る。
        LOG_PRINTF( "send Command_RECOVERY_SX\n" );
        retCode = buildCmdSeqSetRecovery_SX( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_SPEED_CONTROL_SX:      // 速度制御値を指定する。
        //LOG_PRINTF( "send Command_SPEED_CONTROL_SX\n" );
        // 状態不一致
        if( GetMode() != RobotMode_DRIVE )
        {
            //LOG_PRINTF( "ERROR_ON_STATE\n" );
            return TRobotIF::ERROR_ON_STATE;
        }

        retCode = buildCmdSeqSetRotVel_SX( leftRotVel, rightRotVel, cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_GET_MODE_SX:           // 現在のモードを取得する。
        //LOG_PRINTF( "send Command_GET_MODE_SX\n" );
        retCode = buildCmdSeqSetGetMode_SX( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_GET_INF_SX:            // 現在の情報を取得する。
        LOG_PRINTF( "send Command_GET_INF_SX\n" );
        retCode = buildCmdSeqSetGetInf_SX( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_SPEED_CONTROL_INF_SX:  // 現在の情報を取得する。
        LOG_PRINTF( "send Command_SPEED_CONTROL_INF_SX\n" );
        // 状態不一致
        if( (GetMode() != RobotMode_ERROR_STOP) &&
            (GetMode() != RobotMode_FAIL) )
        {
            LOG_PRINTF( "ERROR_ON_STATE\n" );
            //LOG_PRINTF( "ERROR_ON_STATE\n" );
            return TRobotIF::ERROR_ON_STATE;
        }
        retCode = buildCmdSeqSetRotVel_SX( leftRotVel, rightRotVel, cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    default:
        break;
    }

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用運転モードコマンドを組み立てる(SCIBOT-X)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetDrive_SX( unsigned char *cmdbuf )
{
    // 運転or待機モード以外は送信しない
    if( (GetMode() != RobotMode_DRIVE) && (GetMode() != RobotMode_IDLE) )
    {
       LOG_PRINTF( "ERROR_ON_STATE\n" );
       return TRobotIF::ERROR_ON_STATE;     // 状態不一致
    }

    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_SX;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SET_DRIVE_SX;   // 運転モードに移行する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用待機コマンドを組み立てる(SCIBOT-X)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetIdle_SX( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_SX;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SET_IDLE_SX;   // 待機モードに移行する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用緊急停止コマンドを組み立てる(SCIBOT-X)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetErrStop_SX( unsigned char *cmdbuf )
{
    // モードを強制的セットして速度指令指令を出さないようにする。
    //SetMode( (int)RobotMode_ERROR_STOP );

    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_SX;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_ERROR_STOP_SX;   // 緊急停止モードに移行する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用リカバリコマンドを組み立てる(SCIBOT-X)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetRecovery_SX( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_SX;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_RECOVERY_SX;   // 緊急停止モードから待機モードに戻る。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  速度制御コマンドシーケンスを組み立てる(SCIBOT-X)
 *
 *  @param  leftRotVel   [in]  左車輪の回転速度指令値
 *                           （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  rightRotVel  [in]  右車輪の回転速度指令値
 *                           （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetRotVel_SX( int leftRotVel, int rightRotVel,
                                                     unsigned char *cmdbuf )
{
    if( (leftRotVel  < TR_ROT_VEL_CMD_MIN_SX || TR_ROT_VEL_CMD_MAX_SX < leftRotVel)  ||
        (rightRotVel < TR_ROT_VEL_CMD_MIN_SX || TR_ROT_VEL_CMD_MAX_SX < rightRotVel) )
    {
        return TRobotIF::ERROR_OUT_OF_RANGE;
    }

    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_SX;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SPEED_CONTROL_SX;   // 速度制御値を指定する。

    // 右車輪の回転速度指令値
    cmdbuf[ 4 ] = (unsigned char)(( ((short)rightRotVel) & 0xff00) >> 8 );
    cmdbuf[ 5 ] = (unsigned char) ( ((short)rightRotVel) & 0x00ff);

    // 左車輪の回転速度指令値
    cmdbuf[ 6 ] = (unsigned char)(( ((short)leftRotVel)  & 0xff00) >> 8 );
    cmdbuf[ 7 ] = (unsigned char) ( ((short)leftRotVel)  & 0x00ff);

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用モード取得コマンドを組み立てる(SCIBOT-X)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetGetMode_SX( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_SX;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_GET_MODE_SX;   // 現在のモードを取得する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( cmdbuf, &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用情報取得コマンドを組み立てる(SCIBOT-X)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetGetInf_SX( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_SX;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_GET_INF_SX;   // 現在の情報を取得する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}
