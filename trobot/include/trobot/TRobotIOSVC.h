// -*-C++-*-
/*!
 * @file  TRobotIOSVC.h
 * @brief Service implementation header
 *
 */

#include <string> 
#include <stdio.h> 
#include <iostream> 

#ifndef TROBOTIOSVC_H
#define TROBOTIOSVC_H

class SensorDataSVC
{
 public:
   // standard constructor
   SensorDataSVC();
   ~SensorDataSVC();

   // ロボットバージョンの名称
   bool setRobotVersionName( const char* name );
   bool getRobotVersionName( const char* name );

   // 強制速度更新
   bool setForcedVel( bool Flg, double Vx, double Vy, double Va );

   bool getForcedVel( bool & Flg, double & Vx, double & Vy, double & Va );

   // バッテリ電圧
   bool setBatteryVoltage( double value );
   bool getBatteryVoltage( double & value );

   // ダイ温度(旧ロボットベース、SCIBOT-Xのみ)
   bool setDaiTemp( double value );
   bool getDaiTemp( double & value );

   // エラーステータス(旧ロボットベースのみ)
   bool setErrSTS( unsigned char adc,
                   unsigned char com,
                   unsigned char i2c0,
                   unsigned char i2c1 );
   bool getErrSTS( unsigned char & adc,
                   unsigned char & com,
                   unsigned char & i2c0,
                   unsigned char & i2c1  );

   // エラーステータス(SCIBOT-X、L-MESのみ)
   bool setLastFail( unsigned char LastFailD,
                     unsigned char LastFailC,
                     unsigned char LastFailN,
                     unsigned char LastFailE );
   bool getLastFail( unsigned char & LastFailD,
                     unsigned char & LastFailC,
                     unsigned char & LastFailN,
                     unsigned char & LastFailE  );

   // 9D加速度センサ
   bool setAcc( double accX,  double accY,  double accZ );
   bool getAcc( double &accX, double &accY, double &accZ );

   // 9Dジャイロセンサ
   bool setGyr( double gyrX,  double gyrY,  double gyrZ );
   bool getGyr( double &gyrX, double &gyrY, double &gyrZ );

   // 9D磁気センサ
   bool setMag( double magX,  double magY,  double magZ );
   bool getMag( double &magX, double &magY, double &magZ );

   // 床なしセンサ(SCIBOT-Xのみ)
   bool setNoFloor( bool NoFloor1,
                    bool NoFloor2,
                    bool NoFloor3,
                    bool NoFloor4 );
   bool getNoFloor( bool &NoFloor1,
                    bool &NoFloor2,
                    bool &NoFloor3,
                    bool &NoFloor4 );

   // バンパセンサ(SCIBOT-Xのみ)
   bool setBumper( bool Bumper1,
                   bool Bumper2,
                   bool Bumper3,
                   bool Bumper4 );
   bool getBumper( bool &Bumper1,
                   bool &Bumper2,
                   bool &Bumper3,
                   bool &Bumper4 );

   // 測距センサ(SCIBOT-X、L-MESのみ)
   bool setDistance( double Distance0,
                     double Distance1,
                     double Distance2,
                     double Distance3 );
   bool getDistance( double &Distance0,
                     double &Distance1,
                     double &Distance2,
                     double &Distance3 );

   // 測距センサ２(L-MESのみのみ)
   bool setDistance2( double Distance4,
                      double Distance5,
                      double Distance6,
                      double Distance7 );
   bool getDistance2( double &Distance4,
                      double &Distance5,
                      double &Distance6,
                      double &Distance7 );

   // 超音波センサ(L-MESのみのみ)
   bool setUltraSonic( double UltraSonic0,
                       double UltraSonic1 );
   bool getUltraSonic( double &UltraSonic0,
                       double &UltraSonic1 );

   // 基板温度(L-MESのみのみ)
   bool setBoardTemp( double BoardTemp );
   bool getBoardTemp( double &BoardTemp );

   // バッテリ状態(SCIBOT-X、L-MESのみ)
   // L-MES:BatteryLevel1に使用バッテリー残量、BatteryLevel2に未使用バッテリー残量。あとは未使用。
   bool setBatteryStat( unsigned char  BatteryStat,
                        unsigned char  BatteryComStat,
                        unsigned char  ErrDetectStat,
                        double BatteryLevel1,  double BatteryLevel2,
                        double BatteryTemp1,   double BatteryTemp2 );
   bool getBatteryStat( unsigned char  &BatteryStat,
                        unsigned char  &BatteryComStat,
                        unsigned char  &ErrDetectStat,
                        double &BatteryLevel1, double &BatteryLevel2,
                        double &BatteryTemp1,  double &BatteryTemp2 );

   // 速度情報(SCIBOT-Xのみ)
   bool setVelInf( double rightVelCtl,   double leftVelCtl,
                   double rightVel,      double leftVel,
                   double rightCurrent,  double leftCurrent );
   bool getVelInf( double &rightVelCtl,  double &leftVelCtl,
                   double &rightVel,     double &leftVel,
                   double &rightCurrent, double &leftCurrent );

   // バージョン情報(SCIBOT-X、L-MESのみ)
   bool setVersionInf( unsigned char ModelRevision,
                       unsigned char HardwareRevision,
                       unsigned char FirmwareRevision,
                       unsigned long SerialNumber,
                       unsigned char CreationYear,
                       unsigned char CreationMonth,
                       unsigned char CreationDay,
                       unsigned char CreationHour,
                       unsigned char CreationMinute );
   bool getVersionInf( unsigned char &ModelRevision,
                       unsigned char &HardwareRevision,
                       unsigned char &FirmwareRevision,
                       unsigned long &SerialNumber,
                       unsigned char &CreationYear,
                       unsigned char &CreationMonth,
                       unsigned char &CreationDay,
                       unsigned char &CreationHour,
                       unsigned char &CreationMinute );

   // メンテナンス情報(SCIBOT-X、L-MESのみ)
   // NoFloorSMount:L-MESで未使用 BumperSMount:L-MESで未使用 DistanceSMount2:SCIBOT-Xで未使用
   // UltraSonicSMount:SCIBOT-Xで未使用 ResetCount:SCIBOT-Xで未使用
   // NineDOFSMount:L-MESで未使用 JudgSensor:L-MESで未使用
   bool setMntInf( unsigned long OperatingTime,
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
                   unsigned char JudgSensor );
   bool getMntInf( unsigned long &OperatingTime,
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
                   unsigned char &JudgSensor );

   // 現在の運転状態
   bool  setRobotStat( long robotStat );
   bool  getRobotStat( long &robotStat );



   // シリアル入出力用インスタンスへのポインタを登録(TRobotのみ使用)
   bool  SetserIO( void *serIO );

   // 運転モードに移行(SCIBOT-Xのみ)
   bool  CmdSetDrive();

   // 待機モードに移行(SCIBOT-Xのみ)
   bool  CmdSetIdle();

   // 緊急停止モードに移行(SCIBOT-Xのみ)
   bool  CmdErrStop();

   // 緊急停止モードから待機モードに移行(SCIBOT-Xのみ)
   bool  CmdRecovery();

   // 現在のモードを取得する(SCIBOT-Xのみ)
   bool  CmdGetMode();

   // 現在の情報を取得する(SCIBOT-Xのみ)
   bool  CmdGetInf();

   // 運転モードに移行(L-MESのみ)
   bool CmdSetDriveLM();

   // 待機モードに移行(L-MESのみ)
   bool CmdSetIdleLM();

   // 緊急停止モードに移行(L-MESのみ)
   bool  CmdErrStopLM();

   // 緊急停止システムの自己診断を要求(L-MESのみ)
   bool CmdDIAGLM();

   // 緊急停止モードから待機モードに移行(L-MESのみ)
   bool CmdRecoveryLM();

   // ロボットベースを再起動させる(L-MESのみ)
   bool CmdRebootLM();

   // 省電力モードに移行する(L-MESのみ)
   bool CmdSuspendLM();

   // エラー、軽故障データクリア(L-MESのみ)
   bool CmdErrClearLM();

#if 0
   // ロボットベースのモード、センサ情報を取得する(L-MESのみ)
   bool SensorDataSVC_impl::CmdScanConditionLM();

   // 設定情報を書き換える(L-MESのみ)
   bool SensorDataSVC_impl::CmdSetConfigLM();

   // 設定情報、レビジョン、シリアル番号を取得する(L-MESのみ)
   bool SensorDataSVC_impl::CmdGetConfigLM();

   // I2C/SPIインタフェース・デバイスへのライトアクセス(L-MESのみ)
   bool SensorDataSVC_impl::CmdPortWriteLM();

   // I2C/SPIインタフェース・デバイスへのリードアクセス(L-MESのみ)
   bool SensorDataSVC_impl::CmdPortReadLM();

   // エラー、軽故障データクリア(L-MESのみ)
   bool SensorDataSVC_impl::CmdErrClearLM();

   // パラメータを設定する1(L-MESのみ)
   bool SensorDataSVC_impl::CmdSetPara1LM();

   // パラメータを設定する2(L-MESのみ)
   bool SensorDataSVC_impl::CmdSetPara2LM();
#endif


private:
    // シリアル入出力用インスタンスへのポインタ
    void *m_serIO;

    // ロボットベースの運転状態
    int m_robotStat;

    // ロボットバージョンの名称
    std::string        m_trobotVersion;

    bool          m_forcedRotVelFlg;         // 強制速度更新フラグ
    double        m_forcedRotVelVx;          // 強制進行方向速度(vx)
    double        m_forcedRotVelVy;          // 強制進行方向直角左向き速度(vy)
    double        m_forcedRotVelVa;          // 強制車体角速度(va)

    // センサデータプール
    double        m_batteryvoltage;          // バッテリ電圧
    double        m_daiTemp;                 // ダイ温度
    unsigned char m_ADCErrSTS;               // 内蔵ADC エラーステータス(旧ロボットベースのみ)
    unsigned char m_COMErrSTS;               // 通信エラーステータス(旧ロボットベースのみ)
    unsigned char m_I2CErrSTS0;              // I2C 通信 エラーステータス0(旧ロボットベースのみ)
    unsigned char m_I2CErrSTS1;              // I2C 通信 エラーステータス1(旧ロボットベースのみ)
    unsigned char m_LastFailD;               // 最後に発生した危険故障コード(SCIBOT-Xのみ)
    unsigned char m_LastFailC;               // 最後に発生した重度故障コード(SCIBOT-Xのみ)
    unsigned char m_LastFailN;               // 最後に発生した軽度故障コード(SCIBOT-Xのみ)
    unsigned char m_LastFailE;               // 最後に発生したエラーコード(SCIBOT-Xのみ)
    double        m_accX;                    // 9D加速度センサX
    double        m_accY;                    // 9D加速度センサY
    double        m_accZ;                    // 9D加速度センサZ
    double        m_gyrX;                    // 9D加ジャイロセンサX
    double        m_gyrY;                    // 9D加ジャイロセンサY
    double        m_gyrZ;                    // 9D加ジャイロセンサZ
    double        m_magX;                    // 9D磁気センサX
    double        m_magY;                    // 9D磁気センサY
    double        m_magZ;                    // 9D磁気センサZ
    bool          m_NoFloor1;                // 床なしセンサ値ch1(SCIBOT-Xのみ)
    bool          m_NoFloor2;                // 床なしセンサ値ch2(SCIBOT-Xのみ)
    bool          m_NoFloor3;                // 床なしセンサ値ch3(SCIBOT-Xのみ)
    bool          m_NoFloor4;                // 床なしセンサ値ch4(SCIBOT-Xのみ)
    bool          m_Bumper1;                 // バンパセンサ値ch1(SCIBOT-Xのみ)
    bool          m_Bumper2;                 // バンパセンサ値ch2(SCIBOT-Xのみ)
    bool          m_Bumper3;                 // バンパセンサ値ch3(SCIBOT-Xのみ)
    bool          m_Bumper4;                 // バンパセンサ値ch4(SCIBOT-Xのみ)
    double        m_Distance0;               // 測距センサ値ch1(cm)(SCIBOT-X,L-MESのみ)
    double        m_Distance1;               // 測距センサ値ch2(cm)(SCIBOT-X,L-MESのみ)
    double        m_Distance2;               // 測距センサ値ch3(cm)(SCIBOT-X,L-MESのみ)
    double        m_Distance3;               // 測距センサ値ch4(cm)(SCIBOT-X,L-MESのみ)
    double        m_Distance4;               // 測距センサ値ch5(cm)(L-MESのみ)
    double        m_Distance5;               // 測距センサ値ch6(cm)(L-MESのみ)
    double        m_Distance6;               // 測距センサ値ch7(cm)(L-MESのみ)
    double        m_Distance7;               // 測距センサ値ch8(cm)(L-MESのみ)
    double        m_UltraSonic0;             // 超音波センサ値ch0(L-MESのみ)
    double        m_UltraSonic1;             // 超音波センサ値ch1(L-MESのみ)
    unsigned char m_BatteryStat;             // バッテリー状態
    unsigned char m_BatteryComStat;          // バッテリー通信状態
    unsigned char m_ErrDetectStat;           // エラー検出状態
    double        m_BatteryLevel1;           // バッテリ残量1
    double        m_BatteryLevel2;           // バッテリ残量2
    double        m_BatteryTemp1;            // バッテリ温度1
    double        m_BatteryTemp2;            // バッテリ温度2
    double        m_BoardTemp;               // 基板温度(℃)
    double        m_rightVelCtl;             // 右モータの速度制御値
    double        m_leftVelCtl;              // 左モータの速度制御値
    double        m_rightVel;                // 右モータの速度計測値
    double        m_leftVel;                 // 左モータの速度計測値
    double        m_rightCurrent;            // 右側駆動電流計測値(mA)
    double        m_leftCurrent;             // 左側駆動電流計測値(mA)


    // メンテナンス情報プール
    unsigned char m_ModelRevision;             // モデルREV
    unsigned char m_HardwareRevision;          // ハードウエアREV
    unsigned char m_FirmwareRevision;          // ファームウェアREV
    unsigned long m_SerialNumber;              // シリアル番号
    unsigned long m_OperatingTime;             // 稼働時間
    unsigned char m_FailD;                     // 危険故障コード
    unsigned char m_FailC;                     // 重度故障コード
    unsigned char m_FailN;                     // 軽度故障コード
    unsigned char m_FailE;                     // エラーコード
    unsigned char m_FailWDT;                   // WDTエラー発生回数
    unsigned char m_FailRAMSoft;               // RAMソフトエラー
    unsigned char m_NoFloorSMount;             // 床なしセンサ実装
    unsigned char m_BumperSMount;              // バンパセンサ実装
    unsigned char m_DistanceSMount;            // 測距センサ実装
    unsigned char m_DistanceSMount2;           // 測距センサ2実装
    unsigned char m_UltraSonicSMount;          // 超音波センサ実装
    unsigned char m_ResetCount;                // リセット(未定義命令)エラー発生回数
    unsigned long m_TravelDistance;            // 走行距離
    unsigned char m_9DOFSMount;                // 9軸センサ実装
    unsigned char m_JudgSensor;                // センサ判定
    unsigned char m_CreationYear;              // コード作成年
    unsigned char m_CreationMonth;             // コード作成月
    unsigned char m_CreationDay;               // コード作成日
    unsigned char m_CreationHour;              // コード作成時
    unsigned char m_CreationMinute;            // コード作成分
};



#endif // TROBOTIOSVC_H

