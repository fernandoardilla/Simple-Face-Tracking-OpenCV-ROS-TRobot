// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotUtil_LM.cpp
 *  @brief ユーティリティ(L-MES)の実装
 *
 */

#include <cmath>
#include <algorithm>
#include <string.h>

#include "TRobotSerialIOIface.hpp"
#include "TRobotUtil.hpp"



////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文を解析処理する(L-MES)
 *
 *  @param  buf          [in]        受信電文
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::AnalysisMessage_LM( const unsigned char * buf )
{
    int event = 0;

    switch( buf[ResponsePos_LM] )
    {
    // 運転モードに移行する。
    case Response_SET_DRIVE_LM:
        LOG_PRINTF( "Recve Response_SET_DRIVE_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_SET_DRIVE_LM;
        break;

    // 待機モードに移行する。
    case Response_SET_IDLE_LM:
        LOG_PRINTF( "Recve Response_SET_IDLE_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_SET_IDLE_LM;
        break;

    // 緊急停止システムの自己診断を要求する。
    case Response_DIAG_LM:
        LOG_PRINTF( "Recve Response_DIAG_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_DIAG_LM;
        break;

    // 緊急停止モードから待機モードに戻る。
    case Response_RECOVERY_LM:
        LOG_PRINTF( "Recve Response_RECOVERY_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_RECOVERY_LM;
        break;

    // ロボットベースを再起動させる（MPUのソフトリセット）。
    case Response_REBOOT_LM:
        LOG_PRINTF( "Recve Response_REBOOT_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_REBOOT_LM;
        break;

    // 省電力モードに移行させる。
    case Response_SUSPEND_LM:
        LOG_PRINTF( "Recve Response_SUSPEND_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_SUSPEND_LM;
        break;

    // 速度制御値を指定する。
    case Response_SPEED_CONTROL_LM:
        //LOG_PRINTF( "Recve Response_SPEED_CONTROL_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_SPEED_CONTROL_LM;
        break;

    // ロボットベースのモード、センサ情報を取得する。
    case Response_SCAN_CONDITION_LM:
        LOG_PRINTF( "Recve Response_SCAN_CONDITION_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_SCAN_CONDITION_LM;
        break;

    // 設定情報を書き換える。
    case Response_SET_CONFIG_LM:
        LOG_PRINTF( "Recve Response_SET_CONFIG_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_SET_CONFIG_LM;
        break;

    // 設定情報、レビジョン、シリアル番号を取得する。
    case Response_GET_CONFIG_LM:
        LOG_PRINTF( "Recve Response_GET_CONFIG_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_GET_CONFIG_LM;
        break;

    // I2C/SPIインタフェース・デバイスへのライトアクセス。
    case Response_PORT_WRITE_LM:
        LOG_PRINTF( "Recve Response_PORT_WRITE_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_PORT_WRITE_LM;
        break;

    // I2C/SPIインタフェース・デバイスへのリードアクセス。
    case Response_PORT_READ_LM:
        LOG_PRINTF( "Recve Response_PORT_READ_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_PORT_READ_LM;
        break;

    // エラー、軽故障データクリア
    case Response_ERROR_CLEAR_LM:
        LOG_PRINTF( "Recve Response_ERROR_CLEAR_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_ERROR_CLEAR_LM;
        break;

    // パラメータ1を設定する。
    case Response_SET_PARA1_LM:
        LOG_PRINTF( "Recve Response_SET_PARA1_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_SET_PARA1_LM;
        break;

    // パラメータ2を設定する。
    case Response_SET_PARA2_LM:
        LOG_PRINTF( "Recve Response_SET_PARA2_LM mode=%02x\n", buf[ ResModePos ] );
        event = Event_SET_PARA2_LM;
        break;

    // タイムアウト。
    case Response_TIMEOUT_LM:
        //LOG_PRINTF( "Recve Response_TIMEOUT_LM mode=%02x\n", GetMode() );
        event = Event_TIMEOUT_LM;
        break;

    default:
        LOG_PRINTF( "Recve Unknown Response mode=%02x\n", buf[ ResModePos ] );
        event = Event_MAX_LM;
        return TRobotIF::SUCCESS;
    }

    // 状態、イベントで分岐
    GoEventFunc_LM( event, buf );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  運転モード応答の処理(L-MES)
 *  stat_func_SX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */


////////////////////////////////////////////////////////////////////////////////
/**
 *  受信電文の先頭を検索する(L-MES)
 *
 *  @param  bufFirst   [in]        検索開始ポインタ
 *  @param  bufLast    [in]        検索終了ポインタ
 *
 *  @return 指定されたバッファ中のヘッダの先頭へのポインタ。
 *          ヘッダが見つからなければバッファの末尾(有効部分の終端+1)へのポインタ
 */
const unsigned char *TRobotIF::searchForValidHeader_LM( const unsigned char *bufFirst,
                                                        const unsigned char *bufLast )
{
    // 指定されたバッファ中の先頭から末尾までループ TR_START_SX を探す
    for( unsigned char *buf=(unsigned char *)bufFirst; buf<=(unsigned char *)bufLast; buf++ )
    {
        if( *buf != TR_START_LM )
        {
            continue;
        }
        return buf;
    }

    return (unsigned char *)NULL;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文をデコードする(L-MES)
 *
 *  @param  buf          [in]    受信電文
 *  @param  sensorData   [out]   センサーデータ
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::decodeBaseSensorData_LM( const unsigned char * const buf,
                                                     BaseSensorData_LM *sensorData )
{
    short tmpshort;

    //
    // モータの速度制御値
    //
    // 右モータの速度制御値
    tmpshort =  buf[ ResRightMSpeedCtlPos_LM     ] << 8;
    tmpshort |= buf[ ResRightMSpeedCtlPos_LM + 1 ];
    sensorData->m_rightVelCtl = tmpshort;

    // 左モータの速度制御値
    tmpshort =  buf[ ResLeftMSpeedCtlPos_LM     ] << 8;
    tmpshort |= buf[ ResLeftMSpeedCtlPos_LM + 1 ];
    sensorData->m_leftVelCtl = tmpshort;


    //
    // モータのエンコーダ値差分
    //
    // 右モータの速度計測値
    tmpshort =  buf[ ResRightEncPos_LM     ] << 8;
    tmpshort |= buf[ ResRightEncPos_LM + 1 ];
    sensorData->m_rightEncInc = tmpshort;

    // 左モータのエンコーダ値差分
    tmpshort =  buf[ ResLeftEncPos_LM     ] << 8;
    tmpshort |= buf[ ResLeftEncPos_LM + 1 ];
    sensorData->m_leftEncInc = tmpshort;


    //
    // 駆動電流計測値
    //
    // 右側駆動電流計測値(mA)
    tmpshort =  buf[ ResRightMCurrentPos_LM     ] << 8;
    tmpshort |= buf[ ResRightMCurrentPos_LM + 1 ];
    sensorData->m_rightCurrent =  tmpshort;

    // 左側駆動電流計測値(mA)
    tmpshort =  buf[ ResRightMCurrentPos_LM     ] << 8;
    tmpshort |= buf[ ResRightMCurrentPos_LM + 1 ];
    sensorData->m_leftCurrent =  tmpshort;


    //
    // 測距センサ値(cm)
    //
    // ch0
    sensorData->m_Distance0 = buf[ TR_Distance0Pos_LM ];

    // ch1
    sensorData->m_Distance1 = buf[ TR_Distance1Pos_LM ];

    // ch2
    sensorData->m_Distance2 = buf[ TR_Distance2Pos_LM ];

    // ch3
    sensorData->m_Distance3 = buf[ TR_Distance3Pos_LM ];

    // ch4
    sensorData->m_Distance4 = buf[ TR_Distance4Pos_LM ];

    // ch5
    sensorData->m_Distance5 = buf[ TR_Distance5Pos_LM ];

    // ch6
    sensorData->m_Distance6 = buf[ TR_Distance6Pos_LM ];

    // ch7
    sensorData->m_Distance7 = buf[ TR_Distance7Pos_LM ];


    //
    // 超音波センサ値(cm)
    //
    // ch0
    sensorData->m_UltraSonic0 = buf[ TR_UltraSonic0Pos_LM ];

    // ch1
    sensorData->m_UltraSonic1 = buf[ TR_UltraSonic1Pos_LM ];


    //
    // 9Dセンサ値
    //
    // 加速度センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
    for( int i=0; i < (int)sensorData->NUM_XYZ; i++ )
    {
        tmpshort =  buf[ TR_AccXPos_LM + (i * 2)     ] << 8;
        tmpshort |= buf[ TR_AccXPos_LM + (i * 2) + 1 ];
        sensorData->m_acc[i] = tmpshort;
    }

    // ジャイロのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
    for( int i=0; i < (int)sensorData->NUM_XYZ; i++ )
    {
        tmpshort =  buf[ TR_GyrXPos_LM + (i * 2)     ] << 8;
        tmpshort |= buf[ TR_GyrXPos_LM + (i * 2) + 1 ];
        sensorData->m_gyr[i] = tmpshort;
    }

    // 磁気センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
    for( int i=0; i < (int)sensorData->NUM_XYZ; i++ )
    {
        tmpshort =  buf[ TR_MagXPos_LM + (i * 2)     ] << 8;
        tmpshort |= buf[ TR_MagXPos_LM + (i * 2) + 1 ];
        sensorData->m_mag[i] =  tmpshort;
    }


    //
    // バッテリ
    //
    // 使用中バッテリ残量(%)
    sensorData->m_BatteryLevel_Use =  buf[ TR_BatteryLevelUsePos_LM ];

    // 未使用中バッテリ残量(%)
    sensorData->m_BatteryLevel_nonUse =  buf[ TR_BatteryLevelNonUsePos_LM ];


    //
    // 基板温度(℃)
    //
    sensorData->m_BoardTemp =  buf[ TR_BoardTempPos_LM ];


    //
    // エラーコード
    //
    // 最後に発生した危険故障コード
    sensorData->m_LastFailD =  buf[ TR_LastFailDPos_LM ];

    // 最後に発生した重度故障コード
    sensorData->m_LastFailC =  buf[ TR_LastFailCPos_LM ];

    // 最後に発生した軽度故障コード
    sensorData->m_LastFailN =  buf[ TR_LastFailNPos_LM ];

    // 最後に発生したエラーコード
    sensorData->m_LastFailE =  buf[ TR_LastFailEPos_LM ];


    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文をデコードする(L-MES)
 *
 *  @param  buf          [in]    受信電文
 *  @param  sensorData   [out]   センサーデータ
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::decodeMntInfData_LM( const unsigned char * const buf,
                                                 MntInfData_LM *mntInfData )
{
    //
    // バージョン情報
    //
    // モデルREV
    mntInfData->m_ModelRevision = buf[ TR_ModelRevisionPos_LM ];

    // ハードウエアREV
    mntInfData->m_HardwareRevision = buf[ TR_HardwareRevisionPos_LM ];

    // ファームウェアREV
    mntInfData->m_FirmwareRevision = buf[ TR_FirmwareRevisionPos_LM ];

    // シリアル番号
    mntInfData->m_SerialNumber = buf[ TR_SerialNumberPos_LM ];


    //
    // 稼働時間
    //
    mntInfData->m_OperatingTime =  buf[ TR_OperatingTimePos_LM + 0 ] << 24;
    mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_LM + 1 ] << 16;
    mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_LM + 2 ] <<  8;
    mntInfData->m_OperatingTime |= buf[ TR_OperatingTimePos_LM + 3 ] <<  0;


    //
    // エラー情報
    //
    // 危険故障コード
    mntInfData->m_FailD = buf[ TR_FailDPos_LM ];

    // 重度故障コード
    mntInfData->m_FailC = buf[ TR_FailCPos_LM ];

    // 軽度故障コード
    mntInfData->m_FailN = buf[ TR_FailNPos_LM ];

    // エラーコード
    mntInfData->m_FailE = buf[ TR_FailEPos_LM ];

    // WDTエラー発生回数
    mntInfData->m_FailWDT = buf[ TR_FailWDTPos_LM ];

    // RAMソフトエラー
    mntInfData->m_FailRAMSoft = buf[ TR_FailRAMSoftPos_LM ];


    //
    // センサ実装
    //
    // 測距センサ0-3実装
    mntInfData->m_DistanceSMount0_3 = buf[ TR_DistanceSMountPos_LM ];

    // 測距センサ4-7実装
    mntInfData->m_DistanceSMount4_7 = buf[ TR_DistanceSMountPos2_LM ];

    // 超音波センサ0-1実装
    mntInfData->m_UltraSonicSMount0_1 = buf[ TR_UltraSonicSMountPos_LM ];


    //
    // リセット(未定義命令)エラー発生回数
    //
    mntInfData->m_ResetCount = buf[ TR_ResetNumPos_LM ];


    //
    // 走行距離
    //
    mntInfData->m_TravelDistance =  buf[ TR_TravelDistancePos_LM + 0 ] << 24;
    mntInfData->m_TravelDistance |= buf[ TR_TravelDistancePos_LM + 1 ] << 16;
    mntInfData->m_TravelDistance |= buf[ TR_TravelDistancePos_LM + 2 ] <<  8;
    mntInfData->m_TravelDistance |= buf[ TR_TravelDistancePos_LM + 3 ] <<  0;


    //
    // 開発用(コード作成 年月日時分)
    //
    // コード作成年
    mntInfData->m_CreationYear = buf[ TR_CreationYearPos_LM ];

    // コード作成月
    mntInfData->m_CreationMonth = buf[ TR_CreationMonthPos_LM ];

    // コード作成日
    mntInfData->m_CreationDay = buf[ TR_CreationDayPos_LM ];

    // コード作成時
    mntInfData->m_CreationHour = buf[ TR_CreationHourPos_LM ];

    // コード作成分
    mntInfData->m_CreationMinute = buf[ TR_CreationMinutePos_LM ];


    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用指令シーケンスを組み立てる(L-MES)
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
TRobotIF::RetCode TRobotIF::buildCmdSeqSet_LM( int cmd, int leftRotVel, int rightRotVel,
                                               unsigned char *cmdbuf )
{
    TRobotIF::RetCode retCode;

    switch(cmd)
    {
    case Command_SET_DRIVE_LM:          // 運転モードに移行する。
        LOG_PRINTF( "send Command_SET_DRIVE_LM\n" );
        retCode = buildCmdSeqSetDrive_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_SET_IDLE_LM:           // 待機モードに移行する。
        LOG_PRINTF( "send Command_SET_IDLE_LM\n" );
        retCode = buildCmdSeqSetIdle_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_ERROR_STOP_LM:         // 緊急停止モードに移行する。
        LOG_PRINTF( "send Command_ERROR_STOP_LM\n" );
        retCode = buildCmdSeqSetErrStop_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_DIAG_LM:               // 緊急停止システムの自己診断を要求する。
        LOG_PRINTF( "send Command_DIAG_LM\n" );
        retCode = buildCmdSeqSetDIAG_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_RECOVERY_LM:           // 緊急停止モードから待機モードに戻る。
        LOG_PRINTF( "send Command_RECOVERY_LM\n" );
        retCode = buildCmdSeqSetRecovery_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_REBOOT_LM:             // ロボットベースを再起動させる（MPUのソフトリセット）。
        LOG_PRINTF( "send Command_REBOOT_LM\n" );
        retCode = buildCmdSeqSetReboot_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_SUSPEND_LM:           // 省電力モードに移行する。
        LOG_PRINTF( "send Command_SUSPEND_LM\n" );
        retCode = buildCmdSeqSetSuspend_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_SPEED_CONTROL_LM:      // 速度制御値を指定する。
        //LOG_PRINTF( "send Command_SPEED_CONTROL_LM\n" );
        // 状態不一致
        if( GetMode() != RobotMode_DRIVE )
        {
            //LOG_PRINTF( "ERROR_ON_STATE\n" );
            return TRobotIF::ERROR_ON_STATE;
        }

        retCode = buildCmdSeqSetRotVel_LM( leftRotVel, rightRotVel, cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_SCAN_CONDITION_LM:  // 現在の情報を取得する。
        LOG_PRINTF( "send Command_SCAN_CONDITION_LM\n" );
        // 状態不一致
        if( (GetMode() != RobotMode_ERROR_STOP) &&
            (GetMode() != RobotMode_FAIL) )
        {
            LOG_PRINTF( "ERROR_ON_STATE\n" );
            //LOG_PRINTF( "ERROR_ON_STATE\n" );
            return TRobotIF::ERROR_ON_STATE;
        }
        retCode = buildCmdSeqSetScanCondition_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_SET_CONFIG_LM:           // 設定情報を書き換える。
        LOG_PRINTF( "send Command_SET_CONFIG_LM\n" );
        retCode = buildCmdSeqSetSetConfig_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_GET_CONFIG_LM:           // 設定情報、レビジョン、シリアル番号を取得する。
        LOG_PRINTF( "send Command_GET_CONFIG_LM\n" );
        retCode = buildCmdSeqSetGetConfig_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;

    case Command_PORT_WRITE_LM:           // I2C/SPIインタフェース・デバイスへのライトアクセス。
        LOG_PRINTF( "send Command_PORT_WRITE_LM\n" );
        retCode = buildCmdSeqSetPortWrite_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_PORT_READ_LM:           // I2C/SPIインタフェース・デバイスへのリードアクセス。
        LOG_PRINTF( "send Command_PORT_READ_LM\n" );
        retCode = buildCmdSeqSetPortRead_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_ERROR_CLEAR_LM:           // エラー、軽故障データクリア
        LOG_PRINTF( "send Command_ERROR_CLEAR_LM\n" );
        retCode = buildCmdSeqSetErrorClear_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_SET_PARA1_LM:           // パラメータ1を設定する。
        LOG_PRINTF( "send Command_SET_PARA1_LM\n" );
        retCode = buildCmdSeqSetSetPara1_LM( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_SET_PARA2_LM:           // パラメータ2を設定する。
        LOG_PRINTF( "send Command_SET_PARA2_LM\n" );
        retCode = buildCmdSeqSetSetPara2_LM( cmdbuf );
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
 *  ファームウェア用運転モードコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetDrive_LM( unsigned char *cmdbuf )
{
    // 運転or待機モード以外は送信しない
    if( (GetMode() != RobotMode_DRIVE) && (GetMode() != RobotMode_IDLE) )
    {
       LOG_PRINTF( "ERROR_ON_STATE\n" );
       return TRobotIF::ERROR_ON_STATE;     // 状態不一致
    }

    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SET_DRIVE_LM;   // 運転モードに移行する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用待機コマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetIdle_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SET_IDLE_LM;   // 待機モードに移行する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用待機コマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetErrStop_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_ERROR_STOP_LM;   // 緊急停止モードに移行する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用緊急停止システムの自己診断を要求するコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetDIAG_LM( unsigned char *cmdbuf )
{
    // モードを強制的セットして速度指令指令を出さないようにする。
    //SetMode( (int)RobotMode_ERROR_STOP );

    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_DIAG_LM;   // 緊急停止システムの自己診断を要求する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用リカバリコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetRecovery_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_RECOVERY_LM;   // 緊急停止モードから待機モードに戻る。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用ロボットベースを再起動させるコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetReboot_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_REBOOT_LM;   // 緊急停止モードから待機モードに戻る。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用省電力モードに移行するコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetSuspend_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SUSPEND_LM;   // 省電力モードに移行する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  速度制御コマンドシーケンスを組み立てる(L-MES)
 *
 *  @param  leftRotVel   [in]  左車輪の回転速度指令値
 *                           （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  rightRotVel  [in]  右車輪の回転速度指令値
 *                           （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetRotVel_LM( int leftRotVel, int rightRotVel,
                                                     unsigned char *cmdbuf )
{
    if( (leftRotVel  < TR_ROT_VEL_CMD_MIN_SX || TR_ROT_VEL_CMD_MAX_SX < leftRotVel)  ||
        (rightRotVel < TR_ROT_VEL_CMD_MIN_SX || TR_ROT_VEL_CMD_MAX_SX < rightRotVel) )
    {
        return TRobotIF::ERROR_OUT_OF_RANGE;
    }

    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SPEED_CONTROL_LM;   // 速度制御値を指定する。

    // 右車輪の回転速度指令値
    cmdbuf[ 4 ] = (unsigned char)(( ((short)rightRotVel) & 0xff00) >> 8 );
    cmdbuf[ 5 ] = (unsigned char) ( ((short)rightRotVel) & 0x00ff);

    // 左車輪の回転速度指令値
    cmdbuf[ 6 ] = (unsigned char)(( ((short)leftRotVel)  & 0xff00) >> 8 );
    cmdbuf[ 7 ] = (unsigned char) ( ((short)leftRotVel)  & 0x00ff);

    // 保持時間
    cmdbuf[ 8 ] = 0x01;

    // １周期足並みがそろわなくても動作するよう繰り返す
    // 右車輪の回転速度指令値
    cmdbuf[ 9 ] = (unsigned char)(( ((short)rightRotVel) & 0xff00) >> 8 );
    cmdbuf[10 ] = (unsigned char) ( ((short)rightRotVel) & 0x00ff);

    // 左車輪の回転速度指令値
    cmdbuf[11 ] = (unsigned char)(( ((short)leftRotVel)  & 0xff00) >> 8 );
    cmdbuf[12 ] = (unsigned char) ( ((short)leftRotVel)  & 0x00ff);

    // 保持時間
    cmdbuf[13 ] = 0x01;

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用ロボットベースのモード、センサ情報を取得するコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetScanCondition_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SCAN_CONDITION_LM;   // ロボットベースのモード、センサ情報を取得する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用設定情報を書き換えるコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetSetConfig_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SET_CONFIG_LM;   // 設定情報を書き換える。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用設定情報、レビジョン、シリアル番号を取得するコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetGetConfig_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_GET_CONFIG_LM;   // 設定情報、レビジョン、シリアル番号を取得する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用I2C/SPIインタフェース・デバイスへのライトアクセスコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetPortWrite_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_PORT_WRITE_LM;   // I2C/SPIインタフェース・デバイスへのライトアクセス。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用I2C/SPIインタフェース・デバイスへのリードアクセスコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetPortRead_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_PORT_READ_LM;   // I2C/SPIインタフェース・デバイスへのリードアクセス。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用エラー、軽故障データクリアコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetErrorClear_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_ERROR_CLEAR_LM;   // エラー、軽故障データクリア。
    cmdbuf[ 4 ] = 1;                        // クリア。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用パラメータ1を設定するコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetSetPara1_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SET_PARA1_LM;   // パラメータ1を設定する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用パラメータ2を設定するコマンドを組み立てる(L-MES)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetSetPara2_LM( unsigned char *cmdbuf )
{
    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_LM;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SET_PARA2_LM;   // パラメータ2を設定する。

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}
