// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotSensorData.cpp
 *  @brief TRobotSensorDataの実装
 *
 */

#include "TRobotUtil.hpp"
#include "TRobotSensorData.hpp"
#include <cstring>


////////////////////////////////////////////////////////////////////////////////
/*!
 * BaseSensorData_EXコンストラクタ
 * メイン基板に接続されたセンサのデータを集約する構造体(旧ロボットベース)
 */
TRobotIF::BaseSensorData_EX::BaseSensorData_EX() 
{
    //LOG_PRINTF("BaseSensorData\n");

    memset(m_leftEnc,     DEFAULT_VAL, NUM_ENC_COUNTS); // 左モータのエンコーダ値
    memset(m_rightEnc,    DEFAULT_VAL, NUM_ENC_COUNTS); // 右モータのエンコーダ値
    memset(m_acc,         DEFAULT_VAL, NUM_XYZ);        // 加速度センサのデータ
    memset(m_gyr,         DEFAULT_VAL, NUM_XYZ);        // ジャイロのデータ
    memset(m_mag,         DEFAULT_VAL, NUM_XYZ);        // 磁気センサのデータ
    m_BatteryVoltage    = DEFAULT_DVAL;                 // バッテリ電圧
    m_daiTemp           = DEFAULT_DVAL;                 // ダイ温度
    m_InfraredSensor0   = DEFAULT_VAL;                  // 赤外線センサー0
    m_InfraredSensor1   = DEFAULT_VAL;                  // 赤外線センサー1
    m_InfraredSensor2   = DEFAULT_VAL;                  // 赤外線センサー2
    m_InfraredSensor3   = DEFAULT_VAL;                  // 赤外線センサー3
    m_ADCErrSTS         = DEFAULT_VAL;                  // 内蔵ADC エラーステータス
    m_COMErrSTS         = DEFAULT_VAL;                  // 通信エラーステータス
    m_I2CErrSTS0        = DEFAULT_VAL;                  // I2C 通信 エラーステータス0
    m_I2CErrSTS1        = DEFAULT_VAL;                  // I2C 通信 エラーステータス1
}


////////////////////////////////////////////////////////////////////////////////
/*!
 * BaseSensorData_SXコンストラクタ
 * メイン基板に接続されたセンサのデータを集約する構造体(SCIBOT-X)
 */
TRobotIF::BaseSensorData_SX::BaseSensorData_SX() 
{
    //LOG_PRINTF("BaseSensorData_SX\n");

    m_rightVelCtl      = DEFAULT_VAL;                   // 右モータの速度制御値
    m_leftVelCtl       = DEFAULT_VAL;                   // 左モータの速度制御値
    m_rightVel         = DEFAULT_VAL;                   // 右モータの速度計測値
    m_leftVel          = DEFAULT_VAL;                   // 左モータの速度計測値
    m_rightCurrent     = DEFAULT_VAL;                   // 右側駆動電流計測値(mA)
    m_leftCurrent      = DEFAULT_VAL;                   // 左側駆動電流計測値(mA)
    m_NoFloor1         = DEFAULT_BVAL;                  // 床なしセンサ値ch1
    m_NoFloor2         = DEFAULT_BVAL;                  // 床なしセンサ値ch2
    m_NoFloor3         = DEFAULT_BVAL;                  // 床なしセンサ値ch3
    m_NoFloor4         = DEFAULT_BVAL;                  // 床なしセンサ値ch4
    m_Bumper1          = DEFAULT_BVAL;                  // バンパセンサ値ch1
    m_Bumper2          = DEFAULT_BVAL;                  // バンパセンサ値ch2
    m_Bumper3          = DEFAULT_BVAL;                  // バンパセンサ値ch3
    m_Bumper4          = DEFAULT_BVAL;                  // バンパセンサ値ch4
    m_Distance1        = DEFAULT_DVAL;                  // 測距センサ値ch1(cm)
    m_Distance2        = DEFAULT_DVAL;                  // 測距センサ値ch2(cm)
    m_Distance3        = DEFAULT_DVAL;                  // 測距センサ値ch3(cm)
    m_Distance4        = DEFAULT_DVAL;                  // 測距センサ値ch4(cm)
    m_Distance5        = DEFAULT_DVAL;                  // 測距センサ値ch5(cm)

    for(int i=0;i<(int)NUM_XYZ;i++)
    {
        m_acc[i]       = DEFAULT_DVAL;                  // 加速度センサのデータ
        m_gyr[i]       = DEFAULT_DVAL;                  // ジャイロのデータ
        m_mag[i]       = DEFAULT_DVAL;                  // 磁気センサのデータ
    }

    m_BatteryVoltage   = DEFAULT_DVAL;                  // バッテリ電圧
    m_rightIC          = DEFAULT_DVAL;                  // 右モータドライバIC周辺温度計測値(℃)
    m_leftIC           = DEFAULT_DVAL;                  // 左モータドライバIC周辺温度計測値(℃)
    m_daiTemp          = DEFAULT_DVAL;                  // ダイ温度(℃)
    m_LastFailD        = DEFAULT_VAL;                   // 最後に発生した危険故障コード
    m_LastFailC        = DEFAULT_VAL;                   // 最後に発生した重度故障コード
    m_LastFailN        = DEFAULT_VAL;                   // 最後に発生した軽度故障コード
    m_LastFailE        = DEFAULT_VAL;                   // 最後に発生したエラーコード
    m_BatteryStat      = DEFAULT_VAL;                   // バッテリー状態
    m_BatteryComStat   = DEFAULT_VAL;                   // バッテリー通信状態
    m_ErrDetectStat    = DEFAULT_VAL;                   // エラー検出状態
    m_BatteryLevel1    = DEFAULT_DVAL;                  // バッテリ残量1
    m_BatteryLevel2    = DEFAULT_DVAL;                  // バッテリ残量2
    m_BatteryTemp1     = DEFAULT_DVAL;                  // バッテリ温度1
    m_BatteryTemp2     = DEFAULT_DVAL;                  // バッテリ温度2

}


////////////////////////////////////////////////////////////////////////////////
/*!
 * BaseSensorData_LMコンストラクタ
 * メイン基板に接続されたセンサのデータを集約する構造体(L-MES)
 */
TRobotIF::BaseSensorData_LM::BaseSensorData_LM() 
{
    //LOG_PRINTF("BaseSensorData_LM\n");

    m_rightVelCtl         = DEFAULT_VAL;                // 右モータの速度制御値
    m_leftVelCtl          = DEFAULT_VAL;                // 左モータの速度制御値
    m_leftEncInc          = DEFAULT_VAL;                // 左モータのエンコーダ値差分
    m_rightEncInc         = DEFAULT_VAL;                // 右モータのエンコーダ値差分
    m_rightCurrent        = DEFAULT_VAL;                // 右側駆動電流計測値(mA)
    m_leftCurrent         = DEFAULT_VAL;                // 左側駆動電流計測値(mA)
    m_Distance0           = DEFAULT_DVAL;               // 測距センサ値ch0(cm)
    m_Distance1           = DEFAULT_DVAL;               // 測距センサ値ch1(cm)
    m_Distance2           = DEFAULT_DVAL;               // 測距センサ値ch2(cm)
    m_Distance3           = DEFAULT_DVAL;               // 測距センサ値ch3(cm)
    m_Distance4           = DEFAULT_DVAL;               // 測距センサ値ch4(cm)
    m_Distance5           = DEFAULT_DVAL;               // 測距センサ値ch5(cm)
    m_Distance6           = DEFAULT_DVAL;               // 測距センサ値ch6(cm)
    m_Distance7           = DEFAULT_DVAL;               // 測距センサ値ch7(cm)
    m_UltraSonic0         = DEFAULT_DVAL;               // 超音波センサ値ch0(cm)
    m_UltraSonic1         = DEFAULT_DVAL;               // 超音波センサ値ch1(cm)

    for(int i=0;i<(int)NUM_XYZ;i++)
    {
        m_acc[i]          = DEFAULT_DVAL;               // 加速度センサのデータ
        m_gyr[i]          = DEFAULT_DVAL;               // ジャイロのデータ
        m_mag[i]          = DEFAULT_DVAL;               // 磁気センサのデータ
    }

    m_BatteryLevel_Use    = DEFAULT_DVAL;               //!< 使用中バッテリ残量(%)
    m_BoardTemp           = DEFAULT_DVAL;               //!< 基板温度(℃)
    m_LastFailD           = DEFAULT_VAL;                //!< 最後に発生した危険故障コード
    m_LastFailC           = DEFAULT_VAL;                //!< 最後に発生した重度故障コード
    m_LastFailN           = DEFAULT_VAL;                //!< 最後に発生した軽度故障コード
    m_LastFailE           = DEFAULT_VAL;                //!< 最後に発生したエラーコード
    m_BatteryLevel_nonUse = DEFAULT_DVAL;               //!< 未使用バッテリ残量(%)

}


////////////////////////////////////////////////////////////////////////////////
/*!
 * MaintenanceInformationData_SXコンストラクタ
 * メンテナンス情報のデータを集約する構造体(SCIBOT-X)
 */
TRobotIF::MntInfData_SX::MntInfData_SX() 
{
    //LOG_PRINTF("MaintenanceInformationData_SX\n");

    m_ModelRevision    = DEFAULT_VAL;                   // モデルREV
    m_HardwareRevision = DEFAULT_VAL;                   // ハードウエアREV
    m_FirmwareRevision = DEFAULT_VAL;                   // ファームウェアREV
    m_SerialNumber     = DEFAULT_VAL;                   // シリアル番号
    m_OperatingTime    = DEFAULT_VAL;                   // 稼働時間
    m_FailD            = DEFAULT_VAL;                   // 危険故障コード
    m_FailC            = DEFAULT_VAL;                   // 重度故障コード
    m_FailN            = DEFAULT_VAL;                   // 軽度故障コード
    m_FailE            = DEFAULT_VAL;                   // エラーコード
    m_FailWDT          = DEFAULT_VAL;                   // WDTエラー発生回数
    m_FailRAMSoft      = DEFAULT_VAL;                   // RAMソフトエラー
    m_NoFloorSMount    = DEFAULT_VAL;                   // 床なしセンサ実装
    m_BumperSMount     = DEFAULT_VAL;                   // バンパセンサ実装
    m_DistanceSMount   = DEFAULT_VAL;                   // 測距センサ実装
    m_TravelDistance   = DEFAULT_VAL;                   // 走行距離
    m_9DOFSMount       = DEFAULT_VAL;                   // 9軸センサ実装
    m_JudgSensor       = DEFAULT_VAL;                   // センサ判定
    m_CreationYear     = DEFAULT_VAL;                   // コード作成年
    m_CreationMonth    = DEFAULT_VAL;                   // コード作成月
    m_CreationDay      = DEFAULT_VAL;                   // コード作成日
    m_CreationHour     = DEFAULT_VAL;                   // コード作成時
    m_CreationMinute   = DEFAULT_VAL;                   // コード作成分
}


////////////////////////////////////////////////////////////////////////////////
/*!
 * MaintenanceInformationData_LMコンストラクタ
 * メンテナンス情報のデータを集約する構造体(L-MES)
 */
TRobotIF::MntInfData_LM::MntInfData_LM() 
{
    //LOG_PRINTF("MaintenanceInformationData_LM\n");

    m_ModelRevision       = DEFAULT_VAL;                   // モデルREV
    m_HardwareRevision    = DEFAULT_VAL;                   // ハードウエアREV
    m_FirmwareRevision    = DEFAULT_VAL;                   // ファームウェアREV
    m_SerialNumber        = DEFAULT_VAL;                   // シリアル番号
    m_OperatingTime       = DEFAULT_VAL;                   // 稼働時間
    m_FailD               = DEFAULT_VAL;                   // 危険故障コード
    m_FailC               = DEFAULT_VAL;                   // 重度故障コード
    m_FailN               = DEFAULT_VAL;                   // 軽度故障コード
    m_FailE               = DEFAULT_VAL;                   // エラーコード
    m_FailWDT             = DEFAULT_VAL;                   // WDTエラー発生回数
    m_FailRAMSoft         = DEFAULT_VAL;                   // RAMソフトエラー
    m_DistanceSMount0_3   = DEFAULT_VAL;                   // 測距センサ0-3実装
    m_DistanceSMount4_7   = DEFAULT_VAL;                   // 測距センサ4-7実装
    m_UltraSonicSMount0_1 = DEFAULT_VAL;                   // 超音波センサ0-1実装
    m_ResetCount          = DEFAULT_VAL;                   // リセット(未定義命令)エラー発生回数
    m_TravelDistance      = DEFAULT_VAL;                   // 走行距離
    m_CreationYear        = DEFAULT_VAL;                   // コード作成年
    m_CreationMonth       = DEFAULT_VAL;                   // コード作成月
    m_CreationDay         = DEFAULT_VAL;                   // コード作成日
    m_CreationHour        = DEFAULT_VAL;                   // コード作成時
    m_CreationMinute      = DEFAULT_VAL;                   // コード作成分
}


////////////////////////////////////////////////////////////////////////////////
/*!
 * BaseSensorData_MNコンストラクタ
 * メイン基板に接続されたセンサのデータを集約する構造体(CIRCINUS（メカナム）)
 */
TRobotIF::BaseSensorData_MN::BaseSensorData_MN() 
{
    //LOG_PRINTF("BaseSensorData\n");

    memset(m_leftEnc,     DEFAULT_VAL, NUM_ENC_COUNTS);    // 左モータのエンコーダ値
    memset(m_rightEnc,    DEFAULT_VAL, NUM_ENC_COUNTS);    // 右モータのエンコーダ値
    memset(m_leftEnc_r,   DEFAULT_VAL, NUM_ENC_COUNTS);    // 左モータのエンコーダ値
    memset(m_rightEnc_r,  DEFAULT_VAL, NUM_ENC_COUNTS);    // 右モータのエンコーダ値
    memset(m_acc,         DEFAULT_VAL, NUM_XYZ);           // 加速度センサのデータ
    memset(m_gyr,         DEFAULT_VAL, NUM_XYZ);           // ジャイロのデータ
    memset(m_mag,         DEFAULT_VAL, NUM_XYZ);           // 磁気センサのデータ
    m_BatteryVoltage    = DEFAULT_DVAL;                    // バッテリ電圧
    m_daiTemp           = DEFAULT_DVAL;                    // ダイ温度
    memset( m_Distance,  DEFAULT_VAL, NUM_DISTANCE);       // 超音波センサのデータ
    m_BumperSensor      = DEFAULT_BVAL;                    // バンパセンサ
}
