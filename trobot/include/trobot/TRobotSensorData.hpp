// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotSensorData.hpp
 * @brief TRobotSensorDataヘッダ
 *
 */

#ifndef TROBOT_SENSOR_DATA_HPP
#define TROBOT_SENSOR_DATA_HPP

#include <vector> 
#include <cstddef>
#include <time.h>

namespace TRobotIF
{
    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief メイン基板に接続されたセンサのデータを集約する構造体(旧ロボットベース)
     *
     *  @copybrief
     */
    struct BaseSensorData_EX
    {
        BaseSensorData_EX();
        static const size_t NUM_ENC_COUNTS = 16;    //!< データチャンク1個あたりのエンコーダ値のサンプルの個数
        static const size_t NUM_XYZ        = 3;     //!< XYZ軸の軸の数

        int           m_leftEnc[NUM_ENC_COUNTS];    //!< 左モータのエンコーダ値
        int           m_rightEnc[NUM_ENC_COUNTS];   //!< 右モータのエンコーダ値
        int           m_cycleCount[NUM_ENC_COUNTS]; //!< サイクルカウント
        int           m_acc[NUM_XYZ];               //!< 加速度センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        int           m_gyr[NUM_XYZ];               //!< ジャイロのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        int           m_mag[NUM_XYZ];               //!< 磁気センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        double        m_BatteryVoltage;             //!< バッテリ電圧
        double        m_daiTemp;                    //!< ダイ温度
        unsigned char m_InfraredSensor0;            //!< 赤外線センサー0
        unsigned char m_InfraredSensor1;            //!< 赤外線センサー1
        unsigned char m_InfraredSensor2;            //!< 赤外線センサー2
        unsigned char m_InfraredSensor3;            //!< 赤外線センサー3
        unsigned char m_ADCErrSTS;                  //!< 内蔵ADC エラーステータス
        unsigned char m_COMErrSTS;                  //!< 通信エラーステータス
        unsigned char m_I2CErrSTS0;                 //!< I2C 通信 エラーステータス0
        unsigned char m_I2CErrSTS1;                 //!< I2C 通信 エラーステータス1

        static int    const DEFAULT_VAL  = 0;       //!< センサデータのデフォルト値 (通信失敗の場合などに利用)
        constexpr static double const DEFAULT_DVAL = 0.0;     //!< センサデータのデフォルト値 (double)
    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief メイン基板に接続されたセンサのデータを集約する構造体(SCIBOT-X)
     *
     *  @copybrief
     */
    struct BaseSensorData_SX
    {
        BaseSensorData_SX();
        static const size_t NUM_XYZ = 3;            //!< XYZ軸の軸の数

        int           m_rightVelCtl;                //!< 右モータの速度制御値
        int           m_leftVelCtl;                 //!< 左モータの速度制御値
        int           m_rightVel;                   //!< 右モータの速度計測値
        int           m_leftVel;                    //!< 左モータの速度計測値
        int           m_rightCurrent;               //!< 右側駆動電流計測値(mA)
        int           m_leftCurrent;                //!< 左側駆動電流計測値(mA)
        bool          m_NoFloor1;                   //!< 床なしセンサ値ch1
        bool          m_NoFloor2;                   //!< 床なしセンサ値ch2
        bool          m_NoFloor3;                   //!< 床なしセンサ値ch3
        bool          m_NoFloor4;                   //!< 床なしセンサ値ch4
        bool          m_Bumper1;                    //!< バンパセンサ値ch1
        bool          m_Bumper2;                    //!< バンパセンサ値ch2
        bool          m_Bumper3;                    //!< バンパセンサ値ch3
        bool          m_Bumper4;                    //!< バンパセンサ値ch4
        double        m_Distance1;                  //!< 測距センサ値ch1(cm)
        double        m_Distance2;                  //!< 測距センサ値ch2(cm)
        double        m_Distance3;                  //!< 測距センサ値ch3(cm)
        double        m_Distance4;                  //!< 測距センサ値ch4(cm)
        double        m_Distance5;                  //!< 測距センサ値ch5(cm)
        double        m_acc[NUM_XYZ];               //!< 加速度  センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        double        m_gyr[NUM_XYZ];               //!< ジャイロセンサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        double        m_mag[NUM_XYZ];               //!< 磁気    センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        double        m_BatteryVoltage;             //!< バッテリ電圧
        double        m_rightIC;                    //!< 右モータドライバIC周辺温度計測値(℃)
        double        m_leftIC;                     //!< 左モータドライバIC周辺温度計測値(℃)
        double        m_daiTemp;                    //!< ダイ温度(℃)
        unsigned char m_LastFailD;                  //!< 最後に発生した危険故障コード
        unsigned char m_LastFailC;                  //!< 最後に発生した重度故障コード
        unsigned char m_LastFailN;                  //!< 最後に発生した軽度故障コード
        unsigned char m_LastFailE;                  //!< 最後に発生したエラーコード
        unsigned char m_BatteryStat;                //!< バッテリー状態
        unsigned char m_BatteryComStat;             //!< バッテリー通信状態
        unsigned char m_ErrDetectStat;              //!< エラー検出状態
        double        m_BatteryLevel1;              //!< バッテリ残量1
        double        m_BatteryLevel2;              //!< バッテリ残量2
        double        m_BatteryTemp1;               //!< バッテリ温度1
        double        m_BatteryTemp2;               //!< バッテリ温度2

        static int    const DEFAULT_VAL  = 0;       //!< センサデータのデフォルト値 (通信失敗の場合などに利用)
        constexpr static double const DEFAULT_DVAL = 0.0;     //!< センサデータのデフォルト値 (double)
        static bool   const DEFAULT_BVAL = false;   //!< センサデータのデフォルト値 (bool)
    };




    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief メイン基板に接続されたセンサのデータを集約する構造体(L-MES)
     *
     *  @copybrief
     */
    struct BaseSensorData_LM
    {
        BaseSensorData_LM();
        static const size_t NUM_XYZ = 3;            //!< XYZ軸の軸の数

        int           m_rightVelCtl;                //!< 右モータの速度制御値
        int           m_leftVelCtl;                 //!< 左モータの速度制御値
        int           m_leftEncInc;                 //!< 左モータのエンコーダ値差分
        int           m_rightEncInc;                //!< 右モータのエンコーダ値差分
        int           m_rightCurrent;               //!< 右側駆動電流計測値(mA)
        int           m_leftCurrent;                //!< 左側駆動電流計測値(mA)
        double        m_Distance0;                  //!< 測距センサ値ch0(cm)
        double        m_Distance1;                  //!< 測距センサ値ch1(cm)
        double        m_Distance2;                  //!< 測距センサ値ch2(cm)
        double        m_Distance3;                  //!< 測距センサ値ch3(cm)
        double        m_Distance4;                  //!< 測距センサ値ch4(cm)
        double        m_Distance5;                  //!< 測距センサ値ch5(cm)
        double        m_Distance6;                  //!< 測距センサ値ch6(cm)
        double        m_Distance7;                  //!< 測距センサ値ch7(cm)
        double        m_UltraSonic0;                //!< 超音波センサ値ch0(cm)
        double        m_UltraSonic1;                //!< 超音波センサ値ch1(cm)
        double        m_acc[NUM_XYZ];               //!< 加速度  センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        double        m_gyr[NUM_XYZ];               //!< ジャイロセンサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        double        m_mag[NUM_XYZ];               //!< 磁気    センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        double        m_BatteryLevel_Use;           //!< 使用中バッテリ残量(%)
        double        m_BoardTemp;                  //!< 基板温度(℃)
        unsigned char m_LastFailD;                  //!< 最後に発生した危険故障コード
        unsigned char m_LastFailC;                  //!< 最後に発生した重度故障コード
        unsigned char m_LastFailN;                  //!< 最後に発生した軽度故障コード
        unsigned char m_LastFailE;                  //!< 最後に発生したエラーコード
        double        m_BatteryLevel_nonUse;        //!< 未使用バッテリ残量(%)

        static int    const DEFAULT_VAL  = 0;       //!< センサデータのデフォルト値 (通信失敗の場合などに利用)
        constexpr static double const DEFAULT_DVAL = 0.0;     //!< センサデータのデフォルト値 (double)
        static bool   const DEFAULT_BVAL = false;   //!< センサデータのデフォルト値 (bool)
    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief メンテナンス情報のデータを集約する構造体(SCIBOT-X)
     *
     *  @copybrief
     */
    struct MntInfData_SX
    {
        MntInfData_SX();

        unsigned char m_ModelRevision;              //!< モデルREV
        unsigned char m_HardwareRevision;           //!< ハードウエアREV
        unsigned char m_FirmwareRevision;           //!< ファームウェアREV
        unsigned int  m_SerialNumber;               //!< シリアル番号
        time_t        m_OperatingTime;              //!< 稼働時間
        unsigned char m_FailD;                      //!< 危険故障コード
        unsigned char m_FailC;                      //!< 重度故障コード
        unsigned char m_FailN;                      //!< 軽度故障コード
        unsigned char m_FailE;                      //!< エラーコード
        unsigned char m_FailWDT;                    //!< WDTエラー発生回数
        unsigned char m_FailRAMSoft;                //!< RAMソフトエラー
        unsigned char m_NoFloorSMount;              //!< 床なしセンサ実装
        unsigned char m_BumperSMount;               //!< バンパセンサ実装
        unsigned char m_DistanceSMount;             //!< 測距センサ実装
        unsigned int  m_TravelDistance;             //!< 走行距離
        unsigned char m_9DOFSMount;                 //!< 9軸センサ実装
        unsigned char m_JudgSensor;                 //!< センサ判定
        unsigned char m_CreationYear;               //!< コード作成年
        unsigned char m_CreationMonth;              //!< コード作成月
        unsigned char m_CreationDay;                //!< コード作成日
        unsigned char m_CreationHour;               //!< コード作成時
        unsigned char m_CreationMinute;             //!< コード作成分

        static int    const DEFAULT_VAL  = 0;       //!< メンテナンス情報のデータのデフォルト値 (通信失敗の場合などに利用)
    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief メンテナンス情報のデータを集約する構造体(L-MES)
     *
     *  @copybrief
     */
    struct MntInfData_LM
    {
        MntInfData_LM();

        unsigned char m_ModelRevision;              //!< モデルREV
        unsigned char m_HardwareRevision;           //!< ハードウエアREV
        unsigned char m_FirmwareRevision;           //!< ファームウェアREV
        unsigned int  m_SerialNumber;               //!< シリアル番号
        time_t        m_OperatingTime;              //!< 稼働時間
        unsigned char m_FailD;                      //!< 危険故障コード
        unsigned char m_FailC;                      //!< 重度故障コード
        unsigned char m_FailN;                      //!< 軽度故障コード
        unsigned char m_FailE;                      //!< エラーコード
        unsigned char m_FailWDT;                    //!< WDTエラー発生回数
        unsigned char m_FailRAMSoft;                //!< RAMソフトエラー
        unsigned char m_DistanceSMount0_3;          //!< 測距センサ0-3実装
        unsigned char m_DistanceSMount4_7;          //!< 測距センサ4-7実装
        unsigned char m_UltraSonicSMount0_1;        //!< 超音波センサ0-1実装
        unsigned int  m_ResetCount;                 //!< リセット(未定義命令)エラー発生回数
        unsigned int  m_TravelDistance;             //!< 走行距離
        unsigned char m_CreationYear;               //!< コード作成年
        unsigned char m_CreationMonth;              //!< コード作成月
        unsigned char m_CreationDay;                //!< コード作成日
        unsigned char m_CreationHour;               //!< コード作成時
        unsigned char m_CreationMinute;             //!< コード作成分

        static int    const DEFAULT_VAL  = 0;       //!< メンテナンス情報のデータのデフォルト値 (通信失敗の場合などに利用)
    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief メンテナンス情報のデータを集約する構造体
     *
     *  @copybrief
     */
    struct MntInfData
    {
        struct MntInfData_SX MntInfData_SX;      // SCIBOT-X
        struct MntInfData_LM MntInfData_LM;      // L-MES
    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief メイン基板に接続されたセンサのデータを集約する構造体(CIRCINUS（メカナム）)
     *
     *  @copybrief
     */
    struct BaseSensorData_MN
    {
        BaseSensorData_MN();
        static const size_t NUM_ENC_COUNTS = 8;     //!< データチャンク1個あたりのエンコーダ値のサンプルの個数
        static const size_t NUM_XYZ        = 3;     //!< XYZ軸の軸の数
        static const size_t NUM_DISTANCE   = 8;     //!< 超音波センサのデータの数

        int           m_leftEnc[NUM_ENC_COUNTS];    //!< 前左モータのエンコーダ値
        int           m_rightEnc[NUM_ENC_COUNTS];   //!< 前右モータのエンコーダ値
        int           m_leftEnc_r[NUM_ENC_COUNTS];  //!< 後左モータのエンコーダ値
        int           m_rightEnc_r[NUM_ENC_COUNTS]; //!< 後右モータのエンコーダ値
        int           m_cycleCount[NUM_ENC_COUNTS]; //!< サイクルカウント
        int           m_acc[NUM_XYZ];               //!< 加速度センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        int           m_gyr[NUM_XYZ];               //!< ジャイロのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        int           m_mag[NUM_XYZ];               //!< 磁気センサのデータ (インデックスと各軸の対応: 0:X, 1:Y, 2:Z)
        double        m_BatteryVoltage;             //!< バッテリ電圧
        double        m_daiTemp;                    //!< ダイ温度
        int           m_Distance[NUM_DISTANCE];     //!< 超音波センサのデータ
        bool          m_BumperSensor;               //!< バンパセンサ値

        static int    const DEFAULT_VAL  = 0;       //!< センサデータのデフォルト値 (通信失敗の場合などに利用)
        constexpr static double const DEFAULT_DVAL = 0.0;     //!< センサデータのデフォルト値 (double)
        static bool   const DEFAULT_BVAL = false;   //!< センサデータのデフォルト値 (bool)
    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief メイン基板に接続されたセンサのデータを集約する構造体
     *
     *  @copybrief
     */
    struct BaseSensorData
    {
        struct BaseSensorData_EX SensorData_EX;      // 旧ロボットベース
        struct BaseSensorData_SX SensorData_SX;      // SCIBOT-X
        struct BaseSensorData_LM SensorData_LM;      // L-MES
        struct BaseSensorData_MN SensorData_MN;      // CIRCINUS（メカナム）
    };
} // namespace TRobotIF

#endif // TROBOT_SENSOR_DATA_HPP

