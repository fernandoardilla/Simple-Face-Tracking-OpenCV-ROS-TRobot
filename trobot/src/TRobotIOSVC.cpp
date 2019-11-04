// -*-C++-*-
/*!
 * @file  TRobotIOSVC.cpp
 * @brief Service implementation code
 *
 */

#include <string>
#include <stdio.h>
#include <iostream>

#include "TRobotIOSVC.h"
#include "TRobotUtil.hpp"


/////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief TRobot IDL interface コンストラクタ
 *
 *  @copybrief
 *
 */
SensorDataSVC::SensorDataSVC()
{
#define DEFAULT_IVAL (0);                   // デフォルト値 (通信失敗の場合などに利用)
#define DEFAULT_DVAL (0.0);                 // デフォルト値 (double)
#define DEFAULT_BVAL (false);               // デフォルト値 (double)

    // ロボットベースの運転状態
    m_robotStat        = 1;

    //
    // センサデータの初期化
    //
    m_batteryvoltage   = DEFAULT_DVAL;      // バッテリ電圧
    m_daiTemp          = DEFAULT_DVAL;      // ダイ温度
    m_ADCErrSTS        = DEFAULT_IVAL;      // 内蔵ADC エラーステータス(旧ロボットベースのみ)
    m_COMErrSTS        = DEFAULT_IVAL;      // 通信エラーステータス(旧ロボットベースのみ)
    m_I2CErrSTS0       = DEFAULT_IVAL;      // I2C 通信 エラーステータス0(旧ロボットベースのみ)
    m_I2CErrSTS1       = DEFAULT_IVAL;      // I2C 通信 エラーステータス1(旧ロボットベースのみ)
    m_LastFailD        = DEFAULT_IVAL;      // 最後に発生した危険故障コード(SCIBOT-Xのみ)
    m_LastFailC        = DEFAULT_IVAL;      // 最後に発生した重度故障コード(SCIBOT-Xのみ)
    m_LastFailN        = DEFAULT_IVAL;      // 最後に発生した軽度故障コード(SCIBOT-Xのみ)
    m_LastFailE        = DEFAULT_IVAL;      // 最後に発生したエラーコード(SCIBOT-Xのみ)
    m_accX             = DEFAULT_DVAL;      // 9D加速度センサX
    m_accY             = DEFAULT_DVAL;      // 9D加速度センサY
    m_accZ             = DEFAULT_DVAL;      // 9D加速度センサZ
    m_gyrX             = DEFAULT_DVAL;      // 9D加ジャイロセンサX
    m_gyrY             = DEFAULT_DVAL;      // 9D加ジャイロセンサY
    m_gyrZ             = DEFAULT_DVAL;      // 9D加ジャイロセンサZ
    m_magX             = DEFAULT_DVAL;      // 9D磁気センサX
    m_magY             = DEFAULT_DVAL;      // 9D磁気センサY
    m_magZ             = DEFAULT_DVAL;      // 9D磁気センサZ
    m_NoFloor1         = DEFAULT_BVAL;      // 床なしセンサ値ch1(SCIBOT-Xのみ)
    m_NoFloor2         = DEFAULT_BVAL;      // 床なしセンサ値ch2(SCIBOT-Xのみ)
    m_NoFloor3         = DEFAULT_BVAL;      // 床なしセンサ値ch3(SCIBOT-Xのみ)
    m_NoFloor4         = DEFAULT_BVAL;      // 床なしセンサ値ch4(SCIBOT-Xのみ)
    m_Bumper1          = DEFAULT_BVAL;      // バンパセンサ値ch1(SCIBOT-Xのみ)
    m_Bumper2          = DEFAULT_BVAL;      // バンパセンサ値ch2(SCIBOT-Xのみ)
    m_Bumper3          = DEFAULT_BVAL;      // バンパセンサ値ch3(SCIBOT-Xのみ)
    m_Bumper4          = DEFAULT_BVAL;      // バンパセンサ値ch4(SCIBOT-Xのみ)
    m_Distance0        = DEFAULT_DVAL;      // 測距センサ値ch1(cm)(SCIBOT-X,L-MESのみ)
    m_Distance1        = DEFAULT_DVAL;      // 測距センサ値ch2(cm)(SCIBOT-X,L-MESのみ)
    m_Distance2        = DEFAULT_DVAL;      // 測距センサ値ch3(cm)(SCIBOT-X,L-MESのみ)
    m_Distance3        = DEFAULT_DVAL;      // 測距センサ値ch4(cm)(SCIBOT-X,L-MESのみ)
    m_Distance4        = DEFAULT_DVAL;      // 測距センサ値ch5(cm)(L-MESのみ)
    m_Distance5        = DEFAULT_DVAL;      // 測距センサ値ch6(cm)(L-MESのみ)
    m_Distance6        = DEFAULT_DVAL;      // 測距センサ値ch7(cm)(L-MESのみ)
    m_Distance7        = DEFAULT_DVAL;      // 測距センサ値ch8(cm)(L-MESのみ)
    m_UltraSonic0      = DEFAULT_DVAL;      // 超音波センサ値ch0(L-MESのみ)
    m_UltraSonic1      = DEFAULT_DVAL;      // 超音波センサ値ch1(L-MESのみ)
    m_BatteryStat      = DEFAULT_IVAL;      // バッテリー状態
    m_BatteryComStat   = DEFAULT_IVAL;      // バッテリー通信状態
    m_ErrDetectStat    = DEFAULT_IVAL;      // エラー検出状態
    m_BatteryLevel1    = DEFAULT_DVAL;      // バッテリ残量1
    m_BatteryLevel2    = DEFAULT_DVAL;      // バッテリ残量2
    m_BatteryTemp1     = DEFAULT_DVAL;      // バッテリ温度1
    m_BatteryTemp2     = DEFAULT_DVAL;      // バッテリ温度2
    m_BoardTemp        = DEFAULT_DVAL;      // 基板温度(℃)
    m_rightVelCtl      = DEFAULT_DVAL;      // 右モータの速度制御値
    m_leftVelCtl       = DEFAULT_DVAL;      // 左モータの速度制御値
    m_rightVel         = DEFAULT_DVAL;      // 右モータの速度計測値
    m_leftVel          = DEFAULT_DVAL;      // 左モータの速度計測値
    m_rightCurrent     = DEFAULT_DVAL;      // 右側駆動電流計測値(mA)
    m_leftCurrent      = DEFAULT_DVAL;      // 左側駆動電流計測値(mA)


    //
    // メンテナンス情報の初期化
    //
    m_ModelRevision    = DEFAULT_IVAL;      // モデルREV
    m_HardwareRevision = DEFAULT_IVAL;      // ハードウエアREV
    m_FirmwareRevision = DEFAULT_IVAL;      // ファームウェアREV
    m_SerialNumber     = DEFAULT_IVAL;      // シリアル番号
    m_OperatingTime    = DEFAULT_IVAL;      // 稼働時間
    m_FailD            = DEFAULT_IVAL;      // 危険故障コード
    m_FailC            = DEFAULT_IVAL;      // 重度故障コード
    m_FailN            = DEFAULT_IVAL;      // 軽度故障コード
    m_FailE            = DEFAULT_IVAL;      // エラーコード
    m_FailWDT          = DEFAULT_IVAL;      // WDTエラー発生回数
    m_FailRAMSoft      = DEFAULT_IVAL;      // RAMソフトエラー
    m_NoFloorSMount    = DEFAULT_IVAL;      // 床なしセンサ実装
    m_BumperSMount     = DEFAULT_IVAL;      // バンパセンサ実装
    m_DistanceSMount   = DEFAULT_IVAL;      // 測距センサ実装
    m_DistanceSMount2  = DEFAULT_IVAL;      // 測距センサ実装2
    m_UltraSonicSMount = DEFAULT_IVAL;      // 超音波センサ実装
    m_ResetCount       = DEFAULT_IVAL;      // リセット(未定義命令)エラー発生回数
    m_TravelDistance   = DEFAULT_IVAL;      // 走行距離
    m_9DOFSMount       = DEFAULT_IVAL;      // 9軸センサ実装
    m_JudgSensor       = DEFAULT_IVAL;      // センサ判定
    m_CreationYear     = DEFAULT_IVAL;      // コード作成年
    m_CreationMonth    = DEFAULT_IVAL;      // コード作成月
    m_CreationDay      = DEFAULT_IVAL;      // コード作成日
    m_CreationHour     = DEFAULT_IVAL;      // コード作成時
    m_CreationMinute   = DEFAULT_IVAL;      // コード作成分
}


/////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief TRobot IDL interface デストラクタ
 *
 *  @copybrief
 *
 */
SensorDataSVC::~SensorDataSVC()
{
    // Please add extra destructor code here.
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  シリアル入出力用インスタンスへのポインタを登録
 *
 *  @param  serIO   [in]   シリアル入出力用インスタンスへのポインタ
 *
 */
bool SensorDataSVC::SetserIO( void *serIO )
{
    m_serIO = serIO;
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ロボットバージョンの名称を更新する
 *
 *  @copybrief
 *
 *  @param value   [in]    ロボットバージョンの名称
 */
bool SensorDataSVC::setRobotVersionName( const char* name )
{
    m_trobotVersion =  std::string( name );            // ロボットバージョンの名称

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ロボットバージョンの名称を取得する
 *
 *  @copybrief
 *
 *  @param value   [out]    ロボットバージョンの名称
 */
bool SensorDataSVC::getRobotVersionName( const char *name )
{
    name = m_trobotVersion.c_str();             // ロボットバージョンの名称

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 強制速度更新を更新する
 *
 *  @copybrief
 *
 *  @param Flg   [in]    強制速度更新フラグ
 *  @param Vx    [in]    強制進行方向速度(vx)
 *  @param Vy    [in]    強制進行方向直角左向き速度(vy)
 *  @param Va    [in]    強制車体角速度(va)
 */
bool SensorDataSVC::setForcedVel( bool Flg, double Vx, double Vy, double Va )
{
    m_forcedRotVelFlg = Flg;         // 強制速度更新フラグ
    m_forcedRotVelVx  = Vx;          // 強制進行方向速度(vx)
    m_forcedRotVelVy  = Vy;          // 強制進行方向直角左向き速度(vy)
    m_forcedRotVelVa  = Va;          // 強制車体角速度(va)

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 強制速度更新を取得する
 *
 *  @copybrief
 *
 *  @param Flg   [out]    強制速度更新フラグ
 *  @param Vx    [out]    強制進行方向速度(vx)
 *  @param Vy    [out]    強制進行方向直角左向き速度(vy)
 *  @param Va    [out]    強制車体角速度(va)
 */
bool SensorDataSVC::getForcedVel( bool & Flg, double & Vx, double & Vy, double & Va )
{
    Flg = m_forcedRotVelFlg;         // 強制速度更新フラグ
    Vx  = m_forcedRotVelVx;          // 強制進行方向速度(vx)
    Vy  = m_forcedRotVelVy;          // 強制進行方向直角左向き速度(vy)
    Va  = m_forcedRotVelVa;          // 強制車体角速度(va)

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief バッテリ電圧を更新する(旧ロボットベース、SCIBOT-X、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param value   [in]    バッテリ電圧
 */
bool SensorDataSVC::setBatteryVoltage( double value )
{
    m_batteryvoltage = value;               // バッテリ電圧

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief バッテリ電圧を取得する(旧ロボットベース、SCIBOT-X、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param value   [out]    バッテリ電圧
 */
bool SensorDataSVC::getBatteryVoltage( double & value )
{
    value = m_batteryvoltage;               // バッテリ電圧

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ダイ温度を更新する(旧ロボットベース、SCIBOT-X、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param value   [in]    ダイ温度
 */
bool SensorDataSVC::setDaiTemp( double value )
{
    m_daiTemp = value;                      // ダイ温度

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ダイ温度を取得する(旧ロボットベース、SCIBOT-X、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param value   [out]    ダイ温度
 */
bool SensorDataSVC::getDaiTemp( double & value )
{
    value = m_daiTemp;                      // ダイ温度

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief エラーステータスを更新する(旧ロボットベースのみ)
 *
 *  @copybrief
 *
 *  @param adc   [in]    内蔵ADC エラーステータス
 *  @param com   [in]    通信エラーステータス
 *  @param i2c0  [in]    I2C 通信 エラーステータス0
 *  @param i2c1  [in]    I2C 通信 エラーステータス1
 */
bool SensorDataSVC::setErrSTS( unsigned char adc,
                                    unsigned char com,
                                    unsigned char i2c0,
                                    unsigned char i2c1 )
{
    m_ADCErrSTS  = adc;                     // 内蔵ADC エラーステータス
    m_COMErrSTS  = com;                     // 通信エラーステータス
    m_I2CErrSTS0 = i2c0;                    // I2C 通信 エラーステータス0
    m_I2CErrSTS1 = i2c1;                    // I2C 通信 エラーステータス1

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief エラーステータスを取得する(旧ロボットベースのみ)
 *
 *  @copybrief
 *
 *  @param adc   [out]    内蔵ADC エラーステータス
 *  @param com   [out]    通信エラーステータス
 *  @param i2c0  [out]    I2C 通信 エラーステータス0
 *  @param i2c1  [out]    I2C 通信 エラーステータス1
 */
bool SensorDataSVC::getErrSTS( unsigned char &adc,
                                    unsigned char &com,
                                    unsigned char &i2c0,
                                    unsigned char &i2c1  )
{
    adc  = m_ADCErrSTS;                     // 内蔵ADC エラーステータス
    com  = m_COMErrSTS;                     // 通信エラーステータス
    i2c0 = m_I2CErrSTS0;                    // I2C 通信 エラーステータス0
    i2c1 = m_I2CErrSTS1;                    // I2C 通信 エラーステータス1

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief エラーステータスを更新する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param LastFailD  [in]    最後に発生した危険故障コード
 *  @param LastFailC  [in]    最後に発生した重度故障コード
 *  @param LastFailN  [in]    最後に発生した軽度故障コード
 *  @param LastFailE  [in]    最後に発生したエラーコード
 */
bool SensorDataSVC::setLastFail( unsigned char LastFailD,
                                      unsigned char LastFailC,
                                      unsigned char LastFailN,
                                      unsigned char LastFailE )
{
    m_LastFailD = LastFailD;                // 最後に発生した危険故障コード
    m_LastFailC = LastFailC;                // 最後に発生した重度故障コード
    m_LastFailN = LastFailN;                // 最後に発生した軽度故障コード
    m_LastFailE = LastFailE;                // 最後に発生したエラーコード

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief エラーステータスを取得する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param LastFailD  [out]    最後に発生した危険故障コード
 *  @param LastFailC  [out]    最後に発生した重度故障コード
 *  @param LastFailN  [out]    最後に発生した軽度故障コード
 *  @param LastFailE  [out]    最後に発生したエラーコード
 */
bool SensorDataSVC::getLastFail( unsigned char &LastFailD,
                                      unsigned char &LastFailC,
                                      unsigned char &LastFailN,
                                      unsigned char &LastFailE  )
{
    LastFailD = m_LastFailD;                // 最後に発生した危険故障コード
    LastFailC = m_LastFailC;                // 最後に発生した重度故障コード
    LastFailN = m_LastFailN;                // 最後に発生した軽度故障コード
    LastFailE = m_LastFailE;                // 最後に発生したエラーコード

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 9D加速度センサを更新する
 *
 *  @copybrief
 *
 *  @param accX  [in]    9D加速度センサX
 *  @param accY  [in]    9D加速度センサY
 *  @param accZ  [in]    9D加速度センサZ
 */
bool SensorDataSVC::setAcc( double accX,
                                 double accY,
                                 double accZ )
{
    m_accX  = accX;                         // 9D加速度センサX
    m_accY  = accY;                         // 9D加速度センサY
    m_accZ  = accZ;                         // 9D加速度センサZ

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 9D加速度センサを取得する
 *
 *  @copybrief
 *
 *  @param accX  [out]    9D加速度センサX
 *  @param accY  [out]    9D加速度センサY
 *  @param accZ  [out]    9D加速度センサZ
 */
bool SensorDataSVC::getAcc( double &accX,
                                 double &accY,
                                 double &accZ )
{
    accX = m_accX;                          // 9D加速度センサX
    accY = m_accY;                          // 9D加速度センサY
    accZ = m_accZ;                          // 9D加速度センサZ

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 9Dジャイロセンサを更新する
 *
 *  @copybrief
 *
 *  @param gyrX  [in]    9DジャイロセンサX
 *  @param gyrY  [in]    9DジャイロセンサY
 *  @param gyrZ  [in]    9DジャイロセンサZ
 */
bool SensorDataSVC::setGyr( double gyrX,
                                 double gyrY,
                                 double gyrZ )
{
    m_gyrX  = gyrX;                         // 9DジャイロセンサX
    m_gyrY  = gyrY;                         // 9DジャイロセンサY
    m_gyrZ  = gyrZ;                         // 9DジャイロセンサZ

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 9Dジャイロセンサを取得する
 *
 *  @copybrief
 *
 *  @param gyrX  [out]    9DジャイロセンサX
 *  @param gyrY  [out]    9DジャイロセンサY
 *  @param gyrZ  [out]    9DジャイロセンサZ
 */
bool SensorDataSVC::getGyr( double &gyrX,
                                 double &gyrY,
                                 double &gyrZ )
{
    gyrX = m_gyrX;                          // 9DジャイロセンサX
    gyrY = m_gyrY;                          // 9DジャイロセンサY
    gyrZ = m_gyrZ;                          // 9DジャイロセンサZ

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 9D磁気センサを更新する
 *
 *  @copybrief
 *
 *  @param magX  [in]    9D磁気センサX
 *  @param magY  [in]    9D磁気センサY
 *  @param magZ  [in]    9D磁気センサZ
 */
bool SensorDataSVC::setMag( double magX,
                                 double magY,
                                 double magZ )
{
    m_magX  = magX;                         // 9D磁気センサX
    m_magY  = magY;                         // 9D磁気センサY
    m_magZ  = magZ;                         // 9D磁気センサZ

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 9D磁気センサを取得する
 *
 *  @copybrief
 *
 *  @param magX  [out]    9D磁気センサX
 *  @param magY  [out]    9D磁気センサY
 *  @param magZ  [out]    9D磁気センサZ
 */
bool SensorDataSVC::getMag( double &magX,
                                 double &magY,
                                 double &magZ )
{
    magX = m_magX;                          // 9D磁気センサX
    magY = m_magY;                          // 9D磁気センサY
    magZ = m_magZ;                          // 9D磁気センサZ

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 床なしセンサを更新する(SCIBOT-Xのみ)
 *
 *  @copybrief
 *
 *  @param NoFloor1  [in]    床なしセンサ値ch1
 *  @param NoFloor2  [in]    床なしセンサ値ch2
 *  @param NoFloor3  [in]    床なしセンサ値ch3
 *  @param NoFloor4  [in]    床なしセンサ値ch4
 */
bool SensorDataSVC::setNoFloor( bool NoFloor1, bool NoFloor2,
                                     bool NoFloor3, bool NoFloor4 )
{
    m_NoFloor1  = NoFloor1;                 // 床なしセンサ値ch1
    m_NoFloor2  = NoFloor2;                 // 床なしセンサ値ch2
    m_NoFloor3  = NoFloor3;                 // 床なしセンサ値ch3
    m_NoFloor4  = NoFloor4;                 // 床なしセンサ値ch4

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 床なしセンサを取得する(SCIBOT-Xのみ)
 *
 *  @copybrief
 *
 *  @param NoFloor1  [out]    床なしセンサ値ch1
 *  @param NoFloor2  [out]    床なしセンサ値ch2
 *  @param NoFloor3  [out]    床なしセンサ値ch3
 *  @param NoFloor4  [out]    床なしセンサ値ch4
 */
bool SensorDataSVC::getNoFloor( bool &NoFloor1, bool &NoFloor2,
                                     bool &NoFloor3, bool &NoFloor4 )
{
    NoFloor1 = m_NoFloor1;                  // 床なしセンサ値ch1
    NoFloor2 = m_NoFloor2;                  // 床なしセンサ値ch2
    NoFloor3 = m_NoFloor3;                  // 床なしセンサ値ch3
    NoFloor4 = m_NoFloor4;                  // 床なしセンサ値ch4

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief バンパセンサを更新する(SCIBOT-X、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param Bumper1  [in]    バンパセンサ値ch1
 *  @param Bumper2  [in]    バンパセンサ値ch2
 *  @param Bumper3  [in]    バンパセンサ値ch3
 *  @param Bumper4  [in]    バンパセンサ値ch4
 */
bool SensorDataSVC::setBumper( bool Bumper1, bool Bumper2,
                                    bool Bumper3, bool Bumper4 )
{
    m_Bumper1  = Bumper1;                   // バンパセンサ値ch1
    m_Bumper2  = Bumper2;                   // バンパセンサ値ch2
    m_Bumper3  = Bumper3;                   // バンパセンサ値ch3
    m_Bumper4  = Bumper4;                   // バンパセンサ値ch4

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief バンパセンサを取得する(SCIBOT-X、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param Bumper1  [out]    バンパセンサ値ch1
 *  @param Bumper2  [out]    バンパセンサ値ch2
 *  @param Bumper3  [out]    バンパセンサ値ch3
 *  @param Bumper4  [out]    バンパセンサ値ch4
 */
bool SensorDataSVC::getBumper( bool &Bumper1, bool &Bumper2,
                                    bool &Bumper3, bool &Bumper4 )
{
    Bumper1 = m_Bumper1;                    // バンパセンサ値ch1
    Bumper2 = m_Bumper2;                    // バンパセンサ値ch2
    Bumper3 = m_Bumper3;                    // バンパセンサ値ch3
    Bumper4 = m_Bumper4;                    // バンパセンサ値ch4

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 測距センサを更新する(SCIBOT-X、L-MES、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param Distance0  [in]    測距センサ値ch1
 *  @param Distance1  [in]    測距センサ値ch2
 *  @param Distance2  [in]    測距センサ値ch3
 *  @param Distance3  [in]    測距センサ値ch4
 */
bool SensorDataSVC::setDistance( double Distance0, double Distance1,
                                      double Distance2, double Distance3 )
{
    m_Distance0  = Distance0;               // 測距センサ値ch1
    m_Distance1  = Distance1;               // 測距センサ値ch2
    m_Distance2  = Distance2;               // 測距センサ値ch3
    m_Distance3  = Distance3;               // 測距センサ値ch4

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 測距センサを取得する(SCIBOT-X、L-MES、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param Distance0  [out]    測距センサ値ch1
 *  @param Distance1  [out]    測距センサ値ch2
 *  @param Distance2  [out]    測距センサ値ch3
 *  @param Distance3  [out]    測距センサ値ch4
 */
bool SensorDataSVC::getDistance( double &Distance0, double &Distance1,
                                      double &Distance2, double &Distance3 )
{
    Distance0 = m_Distance0;                // 測距センサ値ch1
    Distance1 = m_Distance1;                // 測距センサ値ch2
    Distance2 = m_Distance2;                // 測距センサ値ch3
    Distance3 = m_Distance3;                // 測距センサ値ch4

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 測距センサ2を更新する(L-MES、CIRCINUS（メカナム）のみ)
 *
 *  @copybrief
 *
 *  @param Distance4  [in]    測距センサ値ch5
 *  @param Distance5  [in]    測距センサ値ch6
 *  @param Distance6  [in]    測距センサ値ch7
 *  @param Distance7  [in]    測距センサ値ch8
 */
bool SensorDataSVC::setDistance2( double Distance4, double Distance5,
                                       double Distance6, double Distance7 )
{
    m_Distance4  = Distance4;               // 測距センサ値ch4
    m_Distance5  = Distance5;               // 測距センサ値ch5
    m_Distance6  = Distance6;               // 測距センサ値ch6
    m_Distance7  = Distance7;               // 測距センサ値ch7

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 測距センサ2を取得する(L-MES、CIRCINUS（メカナム）のみのみ)
 *
 *  @copybrief
 *
 *  @param Distance4  [out]    測距センサ値ch5
 *  @param Distance5  [out]    測距センサ値ch6
 *  @param Distance6  [out]    測距センサ値ch7
 *  @param Distance7  [out]    測距センサ値ch8
 */
bool SensorDataSVC::getDistance2( double &Distance4, double &Distance5,
                                       double &Distance6, double &Distance7 )
{
    Distance4 = m_Distance4;                // 測距センサ値ch4
    Distance5 = m_Distance5;                // 測距センサ値ch5
    Distance6 = m_Distance6;                // 測距センサ値ch6
    Distance7 = m_Distance7;                // 測距センサ値ch7

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 超音波センサを更新する(L-MESのみ)
 *
 *  @copybrief
 *
 *  @param UltraSonic0  [in]    超音波センサ値ch0
 *  @param UltraSonic1  [in]    超音波センサ値ch1
 */
bool SensorDataSVC::setUltraSonic( double UltraSonic0, double UltraSonic1 )
{
    m_UltraSonic0  = UltraSonic0;           // 超音波センサ値ch0
    m_UltraSonic1  = UltraSonic1;           // 超音波センサ値ch1

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 超音波センサを取得する(L-MESのみのみ)
 *
 *  @copybrief
 *
 *  @param UltraSonic0  [out]    超音波センサ値ch0
 *  @param UltraSonic1  [out]    超音波センサ値ch1
 */
bool SensorDataSVC::getUltraSonic( double &UltraSonic0, double &UltraSonic1 )
{
    UltraSonic0 = m_UltraSonic0;         // 超音波センサ値ch0
    UltraSonic1 = m_UltraSonic1;         // 超音波センサ値ch1

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 基板温度を更新する(L-MESのみ)
 *
 *  @copybrief
 *
 *  @param BoardTemp  [in]    基板温度(℃)
 */
bool SensorDataSVC::setBoardTemp( double BoardTemp )
{
    m_BoardTemp  = BoardTemp;           // 基板温度

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 基板温度を取得する(L-MESのみのみ)
 *
 *  @copybrief
 *
 *  @param BoardTemp  [out]    基板温度(℃)
 */
bool SensorDataSVC::getBoardTemp( double &BoardTemp )
{
    BoardTemp = m_BoardTemp;              // 基板温度

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief バッテリー状態を更新する(SCIBOT-Xのみ、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param BatteryStat    [in]    バッテリー状態(L-MES:未使用)
 *  @param BatteryComStat [in]    バッテリー通信状態(L-MES:未使用)
 *  @param ErrDetectStat  [in]    エラー検出状態(L-MES:未使用)
 *  @param BatteryLevel1  [in]    バッテリ残量1(L-MES:使用バッテリー残量)
 *  @param BatteryLevel2  [in]    バッテリ残量2(L-MES:未使用バッテリー残量)
 *  @param BatteryTemp1   [in]    バッテリ温度1(L-MES:バッテリ温度)
 *  @param BatteryTemp2   [in]    バッテリ温度2(L-MES:未使用)
 */
bool SensorDataSVC::setBatteryStat(
                                     unsigned char BatteryStat,
                                     unsigned char BatteryComStat,
                                     unsigned char ErrDetectStat,
                                     double BatteryLevel1, double BatteryLevel2,
                                     double BatteryTemp1,  double BatteryTemp2 )
{
    m_BatteryStat    = BatteryStat;         // バッテリー状態
    m_BatteryComStat = BatteryComStat;      // バッテリー通信状態
    m_ErrDetectStat  = ErrDetectStat;       // エラー検出状態
    m_BatteryLevel1  = BatteryLevel1;       // バッテリ残量1
    m_BatteryLevel2  = BatteryLevel2;       // バッテリ残量2
    m_BatteryTemp1   = BatteryTemp1;        // バッテリ温度1
    m_BatteryTemp2   = BatteryTemp2;        // バッテリ温度2

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief バッテリー状態を取得する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param BatteryStat    [out]    バッテリー状態(L-MES:未使用)
 *  @param BatteryComStat [out]    バッテリー通信状態(L-MES:未使用)
 *  @param ErrDetectStat  [out]    エラー検出状態(L-MES:未使用)
 *  @param BatteryLevel1  [out]    バッテリ残量1(L-MES:使用バッテリー残量)
 *  @param BatteryLevel2  [out]    バッテリ残量2(L-MES:未使用バッテリー残量)
 *  @param BatteryTemp1   [out]    バッテリ温度1(L-MES:未使用)
 *  @param BatteryTemp2   [out]    バッテリ温度2(L-MES:未使用)
 */
bool SensorDataSVC::getBatteryStat(
                                     unsigned char &BatteryStat,
                                     unsigned char &BatteryComStat,
                                     unsigned char &ErrDetectStat,
                                     double &BatteryLevel1, double &BatteryLevel2,
                                     double &BatteryTemp1,  double &BatteryTemp2 )
{
    BatteryStat    = m_BatteryStat;         // バッテリー状態
    BatteryComStat = m_BatteryComStat;      // バッテリー通信状態
    ErrDetectStat  = m_ErrDetectStat;       // エラー検出状態
    BatteryLevel1  = m_BatteryLevel1;       // バッテリ残量1
    BatteryLevel2  = m_BatteryLevel2;       // バッテリ残量2
    BatteryTemp1   = m_BatteryTemp1;        // バッテリ温度1
    BatteryTemp2   = m_BatteryTemp2;        // バッテリ温度2

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 速度情報を更新する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param rightVelCtl  [in]    右モータの速度制御値(km/h)
 *  @param leftVelCtl   [in]    左モータの速度制御値(km/h)
 *  @param rightVel     [in]    右モータの速度計測値(km/h)(L-MES:未使用)
 *  @param leftVel      [in]    左モータの速度計測値(km/h)(L-MES:未使用)
 *  @param rightCurrent [in]    右側駆動電流計測値(mA)
 *  @param leftCurrent  [in]    左側駆動電流計測値(mA)
 */
bool SensorDataSVC::setVelInf(
                                     double rightVelCtl,  double leftVelCtl,
                                     double rightVel,     double leftVel,
                                     double rightCurrent, double leftCurrent )
{
    m_rightVelCtl  = rightVelCtl;           // 右モータの速度制御値(km/h)
    m_leftVelCtl   = leftVelCtl;            // 左モータの速度制御値(km/h)
    m_rightVel     = rightVel;              // 右モータの速度計測値(km/h)
    m_leftVel      = leftVel;               // 左モータの速度計測値(km/h)
    m_rightCurrent = rightCurrent;          // 右側駆動電流計測値(mA)
    m_leftCurrent  = leftCurrent;           // 左側駆動電流計測値(mA)

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  速度情報を取得する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param rightVelCtl  [out]    右モータの速度制御値(km/h)
 *  @param leftVelCtl   [out]    左モータの速度制御値(km/h)
 *  @param rightVel     [out]    右モータの速度計測値(km/h)(L-MES:未使用)
 *  @param leftVel      [out]    左モータの速度計測値(km/h)(L-MES:未使用)
 *  @param rightCurrent [out]    右側駆動電流計測値(mA)
 *  @param leftCurrent  [out]    左側駆動電流計測値(mA)
 */
bool SensorDataSVC::getVelInf(
                                     double &rightVelCtl,  double &leftVelCtl,
                                     double &rightVel,     double &leftVel,
                                     double &rightCurrent, double &leftCurrent )
{
    rightVelCtl  = m_rightVelCtl;           // 右モータの速度制御値(km/h)
    leftVelCtl   = m_leftVelCtl;            // 左モータの速度制御値(km/h)
    rightVel     = m_rightVel;              // 右モータの速度計測値(km/h)
    leftVel      = m_leftVel;               // 左モータの速度計測値(km/h)
    rightCurrent = m_rightCurrent;          // 右側駆動電流計測値(mA)
    leftCurrent  = m_leftCurrent;           // 左側駆動電流計測値(mA)

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief バージョン情報を更新する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param ModelRevision      [in]    モデルREV
 *  @param HardwareRevision   [in]    ハードウエアREV
 *  @param FirmwareRevision   [in]    ファームウェアREV
 *  @param SerialNumber       [in]    シリアル番号
 *  @param CreationYear       [in]    コード作成年
 *  @param CreationMonth      [in]    コード作成月
 *  @param CreationDay        [in]    コード作成日
 *  @param CreationHour       [in]    コード作成時
 *  @param CreationMinute     [in]    コード作成分
 */
bool SensorDataSVC::setVersionInf(
                                     unsigned char ModelRevision,
                                     unsigned char HardwareRevision,
                                     unsigned char FirmwareRevision,
                                     unsigned long SerialNumber,
                                     unsigned char CreationYear,
                                     unsigned char CreationMonth,
                                     unsigned char CreationDay,
                                     unsigned char CreationHour,
                                     unsigned char CreationMinute )
{
    m_ModelRevision     = ModelRevision;    // モデルREV
    m_HardwareRevision  = HardwareRevision; // ハードウエアREV
    m_FirmwareRevision  = FirmwareRevision; // ファームウェアREV
    m_SerialNumber      = SerialNumber;     // シリアル番号
    m_CreationYear      = CreationYear;     // コード作成年
    m_CreationMonth     = CreationMonth;    // コード作成月
    m_CreationDay       = CreationDay;      // コード作成日
    m_CreationHour      = CreationHour;     // コード作成時
    m_CreationMinute    = CreationMinute;   // コード作成分

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief バージョン情報を取得する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param ModelRevision      [out]    モデルREV
 *  @param HardwareRevision   [out]    ハードウエアREV
 *  @param FirmwareRevision   [out]    ファームウェアREV
 *  @param SerialNumber       [out]    シリアル番号
 *  @param CreationYear       [out]    コード作成年
 *  @param CreationMonth      [out]    コード作成月
 *  @param CreationDay        [out]    コード作成日
 *  @param CreationHour       [out]    コード作成時
 *  @param CreationMinute     [out]    コード作成分
 */
bool SensorDataSVC::getVersionInf(
                                     unsigned char &ModelRevision,
                                     unsigned char &HardwareRevision,
                                     unsigned char &FirmwareRevision,
                                     unsigned long &SerialNumber,
                                     unsigned char &CreationYear,
                                     unsigned char &CreationMonth,
                                     unsigned char &CreationDay,
                                     unsigned char &CreationHour,
                                     unsigned char &CreationMinute )
{
    ModelRevision     = m_ModelRevision;    // モデルREV
    HardwareRevision  = m_HardwareRevision; // ハードウエアREV
    FirmwareRevision  = m_FirmwareRevision; // ファームウェアREV
    SerialNumber      = m_SerialNumber;     // シリアル番号
    CreationYear      = m_CreationYear;     // コード作成年
    CreationMonth     = m_CreationMonth;    // コード作成月
    CreationDay       = m_CreationDay;      // コード作成日
    CreationHour      = m_CreationHour;     // コード作成時
    CreationMinute    = m_CreationMinute;   // コード作成分

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief メンテナンス情報を更新する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param OperatingTime      [in]    稼働時間
 *  @param FailD              [in]    危険故障コード
 *  @param FailC              [in]    重度故障コード
 *  @param FailN              [in]    軽度故障コード
 *  @param FailE              [in]    エラーコード
 *  @param FailWDT            [in]    WDTエラー発生回数
 *  @param FailRAMSoft        [in]    RAMソフトエラー
 *  @param NoFloorSMount      [in]    床なしセンサ実装
 *  @param BumperSMount       [in]    バンパセンサ実装
 *  @param DistanceSMount     [in]    測距センサ実装
 *  @param DistanceSMount2    [in]    測距センサ実装(L-MES:4-7)
 *  @param UltraSonicSMount   [in]    超音波センサ実装(L-MESのみ:0-1)
 *  @param ResetCount         [in]    リセット(未定義命令)エラー発生回数
 *  @param TravelDistance     [in]    走行距離
 *  @param 9DOFSMount         [in]    9軸センサ実装
 *  @param JudgSensor         [in]    センサ判定
 */
bool SensorDataSVC::setMntInf(
                                     unsigned long OperatingTime,
                                     unsigned char FailD,
                                     unsigned char FailC,
                                     unsigned char FailN,
                                     unsigned char FailE,
                                     unsigned char FailWDT,
                                     unsigned char FailRAMSoft,
                                     unsigned char NoFloorSMount,
                                     unsigned char BumperSMount,
                                     unsigned char DistanceSMount,
                                     unsigned char DistanceSMount2,
                                     unsigned char UltraSonicSMount,
                                     unsigned char ResetCount,
                                     unsigned long TravelDistance,
                                     unsigned char NineDOFSMount,
                                     unsigned char JudgSensor )
{
    m_OperatingTime     = OperatingTime;      // 稼働時間
    m_FailD             = FailD;              // 危険故障コード
    m_FailC             = FailC;              // 重度故障コード
    m_FailN             = FailN;              // 軽度故障コード
    m_FailE             = FailE;              // エラーコード
    m_FailWDT           = FailWDT;            // WDTエラー発生回数
    m_FailRAMSoft       = FailRAMSoft;        // RAMソフトエラー
    m_NoFloorSMount     = NoFloorSMount;      // 床なしセンサ実装
    m_BumperSMount      = BumperSMount;       // バンパセンサ実装
    m_DistanceSMount    = DistanceSMount;     // 測距センサ実装
    m_DistanceSMount2   = DistanceSMount2;    // 測距センサ実装
    m_UltraSonicSMount  = UltraSonicSMount;   // 超音波センサ実装
    m_ResetCount        = ResetCount;         // リセット(未定義命令)エラー発生回数
    m_TravelDistance    = TravelDistance;     // 走行距離
    m_9DOFSMount        = NineDOFSMount;      // 9軸センサ実装
    m_JudgSensor        = JudgSensor;         // センサ判定

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief メンテナンス情報を取得する(SCIBOT-X、L-MESのみ)
 *
 *  @copybrief
 *
 *  @param OperatingTime      [out]   稼働時間
 *  @param FailD              [out]   危険故障コード
 *  @param FailC              [out]   重度故障コード
 *  @param FailN              [out]   軽度故障コード
 *  @param FailE              [out]   エラーコード
 *  @param FailWDT            [out]   WDTエラー発生回数
 *  @param FailRAMSoft        [out]   RAMソフトエラー
 *  @param NoFloorSMount      [out]   床なしセンサ実装
 *  @param BumperSMount       [out]   バンパセンサ実装
 *  @param DistanceSMount     [out]   測距センサ実装
 *  @param DistanceSMount2    [out]   測距センサ実装(L-MESのみ:4-7)
 *  @param UltraSonicSMount   [out]   超音波センサ実装(L-MESのみ:0-1)
 *  @param ResetCount         [out]   リセット(未定義命令)エラー発生回数
 *  @param TravelDistance     [out]   走行距離
 *  @param 9DOFSMount         [out]   9軸センサ実装
 *  @param JudgSensor         [out]   センサ判定
 */
bool SensorDataSVC::getMntInf(
                                     unsigned long &OperatingTime,
                                     unsigned char &FailD,
                                     unsigned char &FailC,
                                     unsigned char &FailN,
                                     unsigned char &FailE,
                                     unsigned char &FailWDT,
                                     unsigned char &FailRAMSoft,
                                     unsigned char &NoFloorSMount,
                                     unsigned char &BumperSMount,
                                     unsigned char &DistanceSMount,
                                     unsigned char &DistanceSMount2,
                                     unsigned char &UltraSonicSMount,
                                     unsigned char &ResetCount,
                                     unsigned long &TravelDistance,
                                     unsigned char &NineDOFSMount,
                                     unsigned char &JudgSensor )
{
    OperatingTime     = m_OperatingTime;    // 稼働時間
    FailD             = m_FailD;            // 危険故障コード
    FailC             = m_FailC;            // 重度故障コード
    FailN             = m_FailN;            // 軽度故障コード
    FailE             = m_FailE;            // エラーコード
    FailWDT           = m_FailWDT;          // WDTエラー発生回数
    FailRAMSoft       = m_FailRAMSoft;      // RAMソフトエラー
    NoFloorSMount     = m_NoFloorSMount;    // 床なしセンサ実装
    BumperSMount      = m_BumperSMount;     // バンパセンサ実装
    DistanceSMount    = m_DistanceSMount;   // 測距センサ実装
    DistanceSMount2   = m_DistanceSMount2;  // 測距センサ実装
    UltraSonicSMount  = m_UltraSonicSMount; // 超音波センサ実装
    ResetCount        = m_ResetCount;       // リセット(未定義命令)エラー発生回数
    TravelDistance    = m_TravelDistance;   // 走行距離
    NineDOFSMount     = m_9DOFSMount;       // 9軸センサ実装
    JudgSensor        = m_JudgSensor;       // センサ判定

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 運転状態を更新する
 *
 *  @copybrief
 *
 *  @param robotStat          [in]    運転状態
 */
bool SensorDataSVC::setRobotStat( long robotStat )
{
    m_robotStat     = robotStat;             // 運転状態

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 運転状態を取得する
 *
 *  @copybrief
 *
 *  @param robotStat          [out]   運転状態
 */
bool SensorDataSVC::getRobotStat( long &robotStat )
{
    robotStat     = m_robotStat;             // 運転状態

    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  運転モードに移行(SCIBOT-Xのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdSetDrive()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SET_DRIVE_SX );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  待機モードに移行(SCIBOT-Xのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdSetIdle()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SET_IDLE_SX );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  緊急停止モードに移行(SCIBOT-Xのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdErrStop()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_ERROR_STOP_SX );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  緊急停止モードから待機モードに移行(SCIBOT-Xのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdRecovery()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_RECOVERY_SX );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  現在のモードを取得する(SCIBOT-Xのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdGetMode()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_GET_MODE_SX );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  現在の情報を取得する(SCIBOT-Xのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdGetInf()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_GET_INF_SX );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  運転モードに移行(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdSetDriveLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SET_DRIVE_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  待機モードに移行(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdSetIdleLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SET_IDLE_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  緊急停止システムの自己診断を要求(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdDIAGLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_DIAG_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  緊急停止モードに移行(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdErrStopLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_ERROR_STOP_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  緊急停止モードから待機モードに移行(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdRecoveryLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_RECOVERY_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  ロボットベースを再起動させる(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdRebootLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_REBOOT_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  省電力モードに移行する(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdSuspendLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SUSPEND_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  エラー、軽故障データクリア(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdErrClearLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_ERROR_CLEAR_LM );
    return true;
}


#if 0
////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  ロボットベースのモード、センサ情報を取得する(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdScanConditionLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SCAN_CONDITION_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  設定情報を書き換える(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdSetConfigLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SET_CONFIG_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  設定情報、レビジョン、シリアル番号を取得する(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdGetConfigLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_GET_CONFIG_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  I2C/SPIインタフェース・デバイスへのライトアクセス(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdPortWriteLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_PORT_WRITE_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  I2C/SPIインタフェース・デバイスへのリードアクセス(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdPortReadLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_PORT_READ_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  エラー、軽故障データクリア(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdErrClearLM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_ERROR_CLEAR_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  パラメータを設定する1(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdSetPara1LM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SET_PARA1_LM );
    return true;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  パラメータを設定する2(L-MESのみ)
 *
 *  @copybrief
 *
 */
bool SensorDataSVC::CmdSetPara2LM()
{
    ((TRobotIF::SerialIOIface *)m_serIO)->sendCmd( (int)TRobotIF::Command_SET_PARA2_LM );
    return true;
}
#endif

