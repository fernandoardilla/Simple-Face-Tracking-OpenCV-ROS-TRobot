// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotConstants.hpp
 * @brief TRobotConstantsヘッダ
 *
 */

#ifndef TROBOT_CONSTANTS_SX_HPP
#define TROBOT_CONSTANTS_SX_HPP



namespace TRobotIF
{


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief 故障モードエラーステータス定義
     *
     *  @copybrief
     */
    enum FailureStatus_SX
    {
        Fail_Nothing_SX                   = 0x00,   //!< エラー無し

        // モータ電源開閉
        FailD_InabilityOpen_SX            = 0x01,   //!< 開不能故障(開指示でのモータ電圧)
        FailC_InabilityClose_SX           = 0x02,   //!< 閉不能故障(閉指示でのモータ電圧)

        // バッテリ
        FailD_BatteryOverVoltage_SX       = 0x0f,   //!< 過電圧（□V以上）
        FailC_BatteryHighVoltage_SX       = 0x10,   //!< 高電圧（□V以上）
        FailC_BatteryLowVoltage_SX        = 0x11,   //!< 低電圧（□V以下）

        // エンコーダ
        FailC_EncoderRight_SX             = 0x12,   //!< 右側エンコーダ故障
        FailC_EncoderLeft_SX              = 0x13,   //!< 左側エンコーダ故障

        // PRADA(バッテリ)
        FailC_PRADABattery1Temp_SX        = 0x1a,   //!< バッテリ1温度異常
        FailC_PRADABattery2Temp_SX        = 0x1b,   //!< バッテリ2温度異常

        // PRADA(エンコーダ)
        FailC_PRADAEncoderOverSpeedR_SX   = 0x1c,   //!< 右側エンコーダ過速度
        FailC_PRADAEncoderOverSpeedL_SX   = 0x1d,   //!< 左側エンコーダ過速度

        // USB通信系
        FailS_USBUnreceivable_SX          = 0x1e,   //!< 受信不能
        FailE_USBFormat_SX                = 0x21,   //!< フォーマット・エラー
        FailE_USBDataLength_SX            = 0x22,   //!< データ長エラー
        FailE_USBSUMValue_SX              = 0x23,   //!< SUMエラー
        FailE_USBRecvOverflow_SX          = 0x24,   //!< 受信オーバーフロー
        FailE_USBCommandValue_SX          = 0x25,   //!< 不正指令値

        // 電流検出器
        FailN_CurrentDetectorRight_SX     = 0x28,   //!< 右側電流検出器故障
        FailN_CurrentDetectorLeft_SX      = 0x29,   //!< 左側電流検出器故障

        // ADC(10bit)
        FailN_ADC_SX                      = 0x2a,   //!< ADC故障

        // 電源制御基板通信
        FailD_PowerControlRecvErr_SX      = 0x2b,   //!< 電源制御基板受信エラー

        // ADC(12bit)
        FailN_ADCCurrenSRight_SX          = 0x2c,   //!< 右側電流センサADC故障
        FailN_ADCCurrenSLeft_SX           = 0x2d,   //!< 左側電流センサADC故障

        // 床なしセンサ
        FailC_NoFloorSNoDetection_SX      = 0x32,   //!< 床なしセンサ無検知(初期値0)

        // 測距センサ
        FailC_DistanceSNoDetection_SX     = 0x34,   //!< 測距センサ無検知故障

        // バンパセンサ
        FailC_BumperSNoDetection_SX       = 0x36,   //!< 無検知故障

        // 加速度センサ
        FailC_AccelerometerOutput_SX      = 0x38,   //!< 出力故障
        FailC_AccelerometerRegister_SX    = 0x39,   //!< レジスタ故障（データ）

        // ジャイロセンサ
        FailC_GyroSOutput_SX              = 0x3a,   //!< 出力故障
        FailC_GyroSRegister_SX            = 0x3b,   //!< レジスタ故障（データ）

        //!< 地磁気センサ
        FailC_GeomagneticSOutput_SX       = 0x3c,   //!< 出力故障
        FailC_GeomagneticSRegister_SX     = 0x3d,   //!< レジスタ故障（データ）

        // バンパセンサ
        FailS_BumperSErr_SX               = 0x46,   //!< バンパセンサエラー

        // 床なしセンサ
        FailS_NoFloorSErr_SX              = 0x4c,   //!< 床なしセンサエラー

        // バッテリ不足信号
        FailC_LowBatterySignalErr_SX      = 0x4f    //!< バッテリ不足信号エラー

    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief 上位ソフトウェアがロボットベースに対して発行するコマンドの定義
     *
     *  @copybrief
     */
    enum CommandNum_SX
    {
        Command_SET_DRIVE_SX              = 0x01,   //!< 運転モードに移行する。
        Command_SET_IDLE_SX               = 0x02,   //!< 待機モードに移行する。
        Command_ERROR_STOP_SX             = 0x43,   //!< 緊急停止モードに移行する。
        Command_RECOVERY_SX               = 0x04,   //!< 緊急停止モードから待機モードに戻る。
        Command_SPEED_CONTROL_SX          = 0x10,   //!< 速度制御値を指定する。
        Command_GET_MODE_SX               = 0x40,   //!< 現在のモードを取得する。
        Command_GET_INF_SX                = 0x42,   //!< 現在の情報を取得する。
        Command_SPEED_CONTROL_INF_SX      = 0x99    //!< 速度制御値を指定する。(エラー情報GET用)
    };
#define CommandPos_SX                 (3)


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief ロボットベースの上位ソフトウェアコマンドに対する応答の定義
     *
     *  @copybrief
     */
    enum ResponseNum_SX
    {
        Response_SET_DRIVE_SX             = 0x01,   //!< 運転モードに移行する。
        Response_SET_IDLE_SX              = 0x02,   //!< 待機モードに移行する。
        Response_ERROR_STOP_SX            = 0x43,   //!< 緊急停止モードに移行する。
        Response_RECOVERY_SX              = 0x04,   //!< 緊急停止モードから待機モードに戻る。
        Response_SPEED_CONTROL_SX         = 0x10,   //!< 速度制御値を指定する。
        Response_GET_MODE_SX              = 0x40,   //!< 現在のモードを取得する。
        Response_GET_INF_SX               = 0x42,   //!< 現在の情報を取得する。
        Response_TIMEOUT_SX               = 0x99    //!< タイムアウト。(本プログラム内で生成)
    };
#define ResponsePos_SX                 (3)


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief 受信イベントの定義
     *
     *  @copybrief
     */
    enum EventNum_SX
    {
        Event_SET_DRIVE_SX                = 1,      //!< 運転モードに移行する応答受信。
        Event_SET_IDLE_SX                 = 2,      //!< 待機モードに移行する応答受信。
        Event_ERROR_STOP_SX               = 3,      //!< 緊急停止モードに移行する応答受信。
        Event_RECOVERY_SX                 = 4,      //!< 緊急停止モードから待機モードに戻る応答受信。
        Event_SPEED_CONTROL_SX            = 5,      //!< 速度制御値を指定する応答受信。
        Event_GET_MODE_SX                 = 6,      //!< 現在のモードを取得する応答受信。
        Event_GET_INF_SX                  = 7,      //!< 現在の情報を取得する応答受信。
        Event_TIMEOUT_SX                  = 8,      //!< タイムアウト。(本プログラム内で生成)
        Event_MAX_SX                      = 9       //!< 受信イベント最大
    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief エラー検出状態の定義
     *         速度制御コマンド応答で使用
     *  @copybrief
     */
    enum ErrStatNum
    {
        ErrStat_NORMAL_SX                = 0,       //!< 正常エラー検出状態
        ErrStat_BatteryAbnTemp1Ini_SX    = 1,       //!< バッテリー1温度エラー(初期化時)
        ErrStat_BatteryAbnTemp2Ini_SX    = 2,       //!< バッテリー2温度エラー(初期化時)
        ErrStat_BatteryAbnTemp1Nom_SX    = 3,       //!< バッテリー1温度エラー(通常時)
        ErrStat_BatteryAbnTemp2Nom_SX    = 4,       //!< バッテリー2温度エラー(通常時)
        ErrStat_OverSpeed1Ini_SX         = 5,       //!< モーター1過速度エラー(初期化時)
        ErrStat_OverSpeed2Ini_SX         = 6,       //!< モーター2過速度エラー(初期化時)
        ErrStat_OverSpeed1Nom_SX         = 7,       //!< モーター1過速度エラー(通常時)
        ErrStat_OverSpeed2Nom_SX         = 8        //!< モーター2過速度エラー(通常時)
    };

} // namespace TRobotIF



////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 電文コードデファイン
 *
 *  @copybrief
 */
#define TR_CMD_LENGTH_SX               (64)                                     //!< コマンド長

#define TR_RCV_LENGTH_SX               TR_CMD_LENGTH_SX                         //!< 受信電文長

#define TR_START_SX                    (0x54)                                   //!< コマンド先頭ーク

#define TR_ROT_VEL_CMD_MIN_SX          (-192)                                   //!< 速度指令最小値
#define TR_ROT_VEL_CMD_MAX_SX          (+192)                                   //!< 速度指令最大値

#define TR_CTRL_PERIOD_SX              (50000)                                  //!< 周期[usec]
#define TR_CTRL_RATE_SX                (TR_SEC_TO_USEC / TR_CTRL_PERIOD_SX)     //!< サンプリング


//
// 速度制御値
// 速度制御コマンド応答で使用
//
#define ResRightMSpeedCtlPos_SX        (6)                                       //!< 右速度制御値バイト位置
#define ResLeftMSpeedCtlPos_SX         (8)                                       //!< 左速度制御値バイト位置


//
// 速度計測値
// 速度制御コマンド応答で使用
//
#define ResRightMSpeedPos_SX           (10)                                     //!< 右速度計測値バイト位置
#define ResLeftMSpeedPos_SX            (12)                                     //!< 左速度計測値バイト位置


//
// 駆動電流計測値
// 速度制御コマンド応答で使用
//
#define ResRightMCurrentPos_SX         (14)                                     //!< 右駆動電流計測値バイト位置
#define ResLeftMCurrentPos_SX          (16)                                     //!< 左駆動電流計測値バイト位置


//
// 床なしセンサ
// bit OFF:床あり、ON:床なし
// 速度制御コマンド応答で使用
//
#define TR_NoFloorPos_SX               (18)                                     //!< 床なしセンサバイト位置

#define TR_NoFloorCH1BIT_SX            (0)                                      //!< CH1床なしセンサビット位置
#define TR_NoFloorCH1MSK_SX            (TR_BitON<<TR_NoFloorCH1BIT_SX)          //!< CH1床なしセンサマスク

#define TR_NoFloorCH2BIT_SX            (2)                                      //!< CH2床なしセンサビット位置
#define TR_NoFloorCH2MSK_SX            (TR_BitON<<TR_NoFloorCH2BIT_SX)          //!< CH2床なしセンサマスク

#define TR_NoFloorCH3BIT_SX            (4)                                      //!< CH3床なしセンサビット位置
#define TR_NoFloorCH3MSK_SX            (TR_BitON<<TR_NoFloorCH3BIT_SX)          //!< CH3床なしセンサマスク

#define TR_NoFloorCH4BIT_SX            (6)                                      //!< CH4床なしセンサビット位置
#define TR_NoFloorCH4MSK_SX            (TR_BitON<<TR_NoFloorCH4BIT_SX)          //!< CH4床なしセンサマスク


//
// バンパセンサ
// bit OFF:非接触、ON:接触
// 速度制御コマンド応答で使用
//
#define TR_BumperSPos_SX               (19)                                     //!< バンパセンサバイト位置

#define TR_BumperSCH1BIT_SX            (0)                                      //!< CH1バンパセンサビット位置
#define TR_BumperSCH1MSK_SX            (TR_BitON<<TR_BumperSCH1BIT_SX)          //!< CH1バンパセンサマスク

#define TR_BumperSCH2BIT_SX            (2)                                      //!< CH2バンパセンサビット位置
#define TR_BumperSCH2MSK_SX            (TR_BitON<<TR_BumperSCH2BIT_SX)          //!< CH2バンパセンサマスク

#define TR_BumperSCH3BIT_SX            (4)                                      //!< CH3バンパセンサビット位置
#define TR_BumperSCH3MSK_SX            (TR_BitON<<TR_BumperSCH3BIT_SX)          //!< CH3バンパセンサマスク

#define TR_BumperSCH4BIT_SX            (6)                                      //!< CH4バンパセンサビット位置
#define TR_BumperSCH4MSK_SX            (TR_BitON<<TR_BumperSCH4BIT_SX)          //!< CH4バンパセンサマスク


//
// 測距センサ値
// 速度制御コマンド応答で使用
//
#define TR_Distance1Pos_SX             (20)                                     //!< CH1測距センサバイト位置
#define TR_Distance2Pos_SX             (22)                                     //!< CH2測距センサバイト位置
#define TR_Distance3Pos_SX             (24)                                     //!< CH3測距センサバイト位置
#define TR_Distance4Pos_SX             (26)                                     //!< CH4測距センサバイト位置
#define TR_Distance5Pos_SX             (28)                                     //!< CH5測距センサバイト位置


//
// 9軸センサ値
// 速度制御コマンド応答で使用
//
#define TR_AccXPos_SX                  (30)                                     //!< X 加速度センサバイト位置
#define TR_AccYPos_SX                  (32)                                     //!< Y 加速度センサバイト位置
#define TR_AccZPos_SX                  (34)                                     //!< Z 加速度センサバイト位置

#define TR_GyrXPos_SX                  (36)                                     //!< X ジャイロセンサバイト位置
#define TR_GyrYPos_SX                  (38)                                     //!< Y ジャイロセンサバイト位置
#define TR_GyrZPos_SX                  (40)                                     //!< Z ジャイロセンサバイト位置

#define TR_MagXPos_SX                  (42)                                     //!< X コンパスセンサバイト位置
#define TR_MagYPos_SX                  (44)                                     //!< Y コンパスセンサバイト位置
#define TR_MagZPos_SX                  (46)                                     //!< Z コンパスセンサバイト位置


//
// バッテリ電圧
// 速度制御コマンド応答で使用
//
#define TR_BatteryVoltagePos_SX        (48)                                     //!< バッテリ電圧計測値バイト位置


//
// モータドライバIC周辺温度計測値
// 速度制御コマンド応答で使用
//
#define TR_RightICPos_SX               (49)                                     //!< 右モータドライバIC周辺温度計測値バイト位置
#define TR_LeftICPos_SX                (50)                                     //!< 左モータドライバIC周辺温度計測値バイト位置


//
// ダイ温度計測値
// 速度制御コマンド応答で使用
//
#define TR_DaiTempPos_SX               (51)                                     //!< ダイ温度計測値バイト位置


//
// エラーコード
// 速度制御コマンド応答で使用
//
#define TR_LastFailDPos_SX             (52)                                     //!< 最後に発生した危険故障コードバイト位置
#define TR_LastFailCPos_SX             (53)                                     //!< 最後に発生した重度故障コード
#define TR_LastFailNPos_SX             (54)                                     //!< 最後に発生した軽度故障コード
#define TR_LastFailEPos_SX             (55)                                     //!< 最後に発生したエラーコード


//
// バッテリ接続状態
// bit OFF:未接続、ON:接続
// 速度制御コマンド応答で使用
//
#define TR_BatteryConnectPos_SX        (56)                                     //!< バッテリ接触バイト位置

#define TR_BatteryConnect1BIT_SX       (0)                                      //!< バッテリ1接触ビット位置
#define TR_BatteryConnect1MSK_SX       (TR_BitON<<TR_BatteryConnect1BIT_SX)     //!< バッテリ1接触マスク

#define TR_BatteryConnect2BIT_SX       (1)                                      //!< バッテリ2接触ビット位置
#define TR_BatteryConnect2MSK_SX       (TR_BitON<<TR_BatteryConnect2BIT_SX)     //!< バッテリ2接触マスク


//
// バッテリ電源供給
// bit OFF:未接続、ON:接続
// 速度制御コマンド応答で使用
//
#define TR_BatteryPowerSupplyPos_SX    (56)                                     //!< バッテリ電源供給バイト位置

#define TR_BatteryPowerSupply1BIT_SX   (2)                                      //!< バッテリ1電源供給ビット位置
#define TR_BatteryPowerSupply1MSK_SX   (TR_BitON<<TR_BatteryPowerSupply1BIT_SX) //!< バッテリ1電源供給マスク

#define TR_BatteryPowerSupply2BIT_SX   (3)                                      //!< バッテリ2電源供給ビット位置
#define TR_BatteryPowerSupply2MSK_SX   (TR_BitON<<TR_BatteryPowerSupply2BIT_SX) //!< バッテリ2電源供給マスク


//
// バッテリ温度異常
// bit OFF:正常、ON:異常
// 速度制御コマンド応答で使用
//
#define TR_BatteryAbnTempPos_SX        (56)                                     //!< バッテリ温度異常バイト位置

#define TR_BatteryAbnTemp1BIT_SX       (4)                                      //!< バッテリ1温度異常ビット位置
#define TR_BatteryAbnTemp1MSK_SX       (TR_BitON<<TR_BatteryAbnTemp1BIT_SX)     //!< バッテリ1温度異常マスク

#define TR_BatteryAbnTemp2BIT_SX       (5)                                      //!< バッテリ2温度異常ビット位置
#define TR_BatteryAbnTemp2MSK_SX       (TR_BitON<<TR_BatteryAbnTemp1BIT_SX)     //!< バッテリ2温度異常マスク


//
// バッテリ電力不足
// bit OFF:正常、ON:不足
// 速度制御コマンド応答で使用
//
#define TR_BatteryShortagePos_SX       (56)                                     //!< バッテリ電力不足バイト位置

#define TR_BatteryShortage1BIT_SX      (6)                                      //!< バッテリ1電力不足ビット位置
#define TR_BatteryShortage1MSK_SX      (TR_BitON<<TR_BatteryShortage1BIT_SX)    //!< バッテリ1電力不足マスク


//
// モーター電源供給
// bit OFF:供給なし、ON:不供給
// 速度制御コマンド応答で使用
//
#define TR_MotorPowerSupplyPos_SX      (56)                                     //!< バッテリ電力不足バイト位置

#define TR_MotorPowerSupplyBIT_SX      (7)                                      //!< モーター電源供給ビット位置
#define TR_MotorPowerSupplyMSK_SX      (TR_BitON<<TR_MotorPowerSupplyBIT_SX)    //!< モーター電源供給マスク


//
// バッテリ通信状態
// 速度制御コマンド応答で使用
//
#define TR_BatteryComPos_SX            (57)                                     //!< バッテリ通信状態バイト位置

#define TR_BatteryICComErrBIT_SX       (0)                                      //!< ICデバイス通信エラービット位置
#define TR_BatteryICComErrMSK_SX       (TR_BitON<<TR_BatteryICComErrBIT_SX)     //!< ICデバイス通信エラーマスク

#define TR_Battery1ComErrBIT_SX        (1)                                      //!< バッテリ1通信エラービット位置
#define TR_Battery1ComErrMSK_SX        (TR_BitON<<TR_Battery1ComErrBIT_SX)      //!< バッテリ1通信エラーマスク

#define TR_Battery2ComErrBIT_SX        (2)                                      //!< バッテリ2通信エラービット位置
#define TR_Battery2ComErrMSK_SX        (TR_BitON<<TR_Battery2ComErrBIT_SX)      //!< バッテリ2通信エラーマスク

#define TR_BatteryUsingACBIT_SX        (3)                                      //!< ACアダプタ入力状態ビット位置
#define TR_BatteryUsingACMSK_SX        (TR_BitON<<TR_BatteryUsingACBIT_SX)      //!< ACアダプタ入力状態マスク

#define TR_MotorPowerSupply1BIT_SX     (4)                                      //!< モーター電源供給なし(モーター1過速度検出)
#define TR_MotorPowerSupply1MSK_SX     (TR_BitON<<TR_MotorPowerSupply1BIT_SX)   //!< モーター電源供給なし(モーター1過速度検出)

#define TR_MotorPowerSupply2BIT_SX     (5)                                      //!< モーター電源供給なし(モーター2過速度検出)
#define TR_MotorPowerSupply2MSK_SX     (TR_BitON<<TR_MotorPowerSupply2BIT_SX)   //!< モーター電源供給なし(モーター2過速度検出)

#define TR_MotorPowerSupplyCBIT_SX     (6)                                      //!< モーター電源供給なし(制御ボード（１）通信エラー)
#define TR_MotorPowerSupplyCMSK_SX     (TR_BitON<<TR_MotorPowerSupplyCBIT_SX)   //!< モーター電源供給なし(制御ボード（１）通信エラー)


//
// エラー検出状態
// 速度制御コマンド応答で使用
//
#define TR_ErrDetectStatPos_SX         (58)                                     //!< エラー検出状態位置


//
// バッテリ残量
// 速度制御コマンド応答で使用
//
#define TR_BatteryLevel1Pos_SX         (59)                                     //!< バッテリ残量1バイト位置
#define TR_BatteryLevel2Pos_SX         (60)                                     //!< バッテリ残量2バイト位置


//
// バッテリ温度
// 速度制御コマンド応答で使用
//
#define TR_BatteryTemp1Pos_SX          (61)                                     //!< バッテリ温度1バイト位置
#define TR_BatteryTemp2Pos_SX          (62)                                     //!< バッテリ温度2バイト位置


//
// バージョン情報
// 情報取得コマンド応答で使用
//
#define TR_ModelRevisionPos_SX         (5)                                      //!< モデルREVバイト位置
#define TR_HardwareRevisionPos_SX      (6)                                      //!< ハードウエアREVバイト位置
#define TR_FirmwareRevisionPos_SX      (7)                                      //!< ファームウェアREVバイト位置
#define TR_SerialNumberPos_SX          (8)                                      //!< シリアル番号バイト位置


//
// 稼働時間(秒) (ビッグエンディア形式)
// 情報取得コマンド応答で使用
//
#define TR_OperatingTimePos_SX        (10)                                      //!< 稼働時間バイト位置


//
// エラー情報
// 情報取得コマンド応答で使用
//
#define TR_FailDPos_SX                (14)                                      //!< 危険故障コードバイト位置
#define TR_FailCPos_SX                (15)                                      //!< 重度故障コードバイト位置
#define TR_FailNPos_SX                (16)                                      //!< 軽度故障コードバイト位置
#define TR_FailEPos_SX                (17)                                      //!< エラーコードバイト位置
#define TR_FailWDTPos_SX              (18)                                      //!< WDTエラー発生回数バイト位置
#define TR_FailRAMSoftPos_SX          (19)                                      //!< RAMソフトエラーバイト位置


//
// 床なしセンサ実装
// bit OFF:実装なし、ON:実装
// 情報取得コマンド応答で使用
//
#define TR_NoFloorSMountPos_SX         (20)                                     //!< 床なしセンサ実装バイト位置

#define TR_NoFloorSMountCH1BIT_SX      (0)                                      //!< CH1床なしセンサ実装ビット位置
#define TR_NoFloorSMountCH1MSK_SX      (TR_BitON<<TR_NoFloorSMountCH1BIT_SX)    //!< CH1床なしセンサ実装マスク

#define TR_NoFloorSMountCH2BIT_SX      (1)                                      //!< CH2床なしセンサ実装ビット位置
#define TR_NoFloorSMountCH2MSK_SX      (TR_BitON<<TR_NoFloorSMountCH2BIT_SX)    //!< CH2床なしセンサ実装マスク

#define TR_NoFloorSMountCH3BIT_SX      (2)                                      //!< CH3床なしセンサ実装ビット位置
#define TR_NoFloorSMountCH3MSK_SX      (TR_BitON<<TR_NoFloorSMountCH3BIT_SX)    //!< CH3床なしセンサ実装マスク

#define TR_NoFloorSMountCH4BIT_SX      (3)                                      //!< CH4床なしセンサ実装ビット位置
#define TR_NoFloorSMountCH4MSK_SX      (TR_BitON<<TR_NoFloorSMountCH4BIT_SX)    //!< CH4床なしセンサ実装マスク


//
// バンパセンサ実装
// bit OFF:実装なし、ON:実装
// 情報取得コマンド応答で使用
//
#define TR_BumperSMountPos_SX          (21)                                     //!< バンパセンサ実装バイト位置

#define TR_BumperSMountCH1BIT_SX       (0)                                      //!< CH1バンパセンサ実装ビット位置
#define TR_BumperSMountCH1MSK_SX       (TR_BitON<<TR_BumperSMountCH1BIT_SX)     //!< CH1バンパセンサ実装マスク

#define TR_BumperSMountCH2BIT_SX       (1)                                      //!< CH2バンパセンサ実装ビット位置
#define TR_BumperSMountCH2MSK_SX       (TR_BitON<<TR_BumperSMountCH2BIT_SX)     //!< CH2バンパセンサ実装マスク

#define TR_BumperSMountCH3BIT_SX       (2)                                      //!< CH3バンパセンサ実装ビット位置
#define TR_BumperSMountCH3MSK_SX       (TR_BitON<<TR_BumperSMountCH3BIT_SX)     //!< CH3バンパセンサ実装マスク

#define TR_BumperSMountCH4BIT_SX       (3)                                      //!< CH4バンパセンサ実装ビット位置
#define TR_BumperSMountCH4MSK_SX       (TR_BitON<<TR_BumperSMountCH4BIT_SX)     //!< CH4バンパセンサ実装マスク


//
// 測距センサ実装
// bit OFF:実装なし、ON:実装
// 情報取得コマンド応答で使用
//
#define TR_DistanceSMountPos_SX        (22)                                     //!< 測距センサ実装バイト位置

#define TR_DistanceSMountCH1BIT_SX     (0)                                      //!< CH1測距センサ実装ビット位置
#define TR_DistanceSMountCH1MSK_SX     (TR_BitON<<TR_DistanceSMountCH1BIT_SX)   //!< CH1測距センサ実装マスク

#define TR_DistanceSMountCH2BIT_SX     (1)                                      //!< CH2測距センサ実装ビット位置
#define TR_DistanceSMountCH2MSK_SX     (TR_BitON<<TR_DistanceSMountCH2BIT_SX)   //!< CH2測距センサ実装マスク

#define TR_DistanceSMountCH3BIT_SX     (2)                                      //!< CH3測距センサ実装ビット位置
#define TR_DistanceSMountCH3MSK_SX     (TR_BitON<<TR_DistanceSMountCH3BIT_SX)   //!< CH3測距センサ実装マスク

#define TR_DistanceSMountCH4BIT_SX     (3)                                      //!< CH4測距センサ実装ビット位置
#define TR_DistanceSMountCH4MSK_SX     (TR_BitON<<TR_DistanceSMountCH4BIT_SX)   //!< CH4測距センサ実装マスク

#define TR_DistanceSMountCH5BIT_SX     (5)                                      //!< CH5測距センサ実装ビット位置
#define TR_DistanceSMountCH5MSK_SX     (TR_BitON<<TR_DistanceSMountCH5BIT_SX)   //!< CH5測距センサ実装マスク


//
// 走行距離(m) (ビッグエンディア形式)
// 情報取得コマンド応答で使用
//
#define TR_TravelDistancePos_SX        (23)                                     //!< 走行距離バイト位置


//
// 9軸センサ実装
// bit OFF:実装なし、ON:実装
// 情報取得コマンド応答で使用
//
#define TR_9DOFSMountPos_SX            (27)                                     //!< 9軸センサ実装バイト位置

#define TR_9DOFSMountBIT_SX            (0)                                      //!< 9軸センサビット位置
#define TR_9DOFSMountCH1MSK_SX         (TR_BitON<<TR_9DOFSMountBIT_SX)          //!< 9軸センサ実装マスク


//
// センサ判定
// bit OFF:判定なし、ON:判定あり
// 情報取得コマンド応答で使用
//
#define TR_JudgSensorPos_SX            (28)                                     //!< センサ判定バイト位置
#define TR_JudgDistanceSBIT_SX         (0)                                      //!< 測距センサ判定ビット位置
#define TR_JudgDistanceSMSK_SX         (TR_BitON<<TR_JudgDistanceSBIT_SX)       //!< 測距センサ判定マスク

#define TR_JudgBumperSBIT_SX           (1)                                      //!< バンパセンサ判定ビット位置
#define TR_JudgBumperSMSK_SX           (TR_BitON<<TR_JudgBumperSBIT_SX)         //!< バンパセンサ判定マスク

#define TR_JudgNoFloorSBIT_SX          (2)                                      //!< 床なしセンサ判定ビット位置
#define TR_JudgNoFloorSMSK_SX          (TR_BitON<<TR_JudgNoFloorSBIT_SX)        //!< 床なしセンサ判定マスク

#define TR_Judg9DOFSBIT_SX             (3)                                      //!< 9軸センサ判定ビット位置
#define TR_Judg9DOFSMSK_SX             (TR_BitON<<TR_Judg9DOFSBIT_SX)           //!< 9軸センサ判定マスク


//
// 開発用(コード作成 年月日時分)
// bit OFF:判定なし、ON:判定あり
// 情報取得コマンド応答で使用
//
#define TR_CreationYearPos_SX          (58)                                     //!< コード作成年バイト位置
#define TR_CreationMonthPos_SX         (59)                                     //!< コード作成月バイト位置
#define TR_CreationDayPos_SX           (60)                                     //!< コード作成日バイト位置
#define TR_CreationHourPos_SX          (61)                                     //!< コード作成時バイト位置
#define TR_CreationMinutePos_SX        (62)                                     //!< コード作成分バイト位置


#endif // TROBOT_CONSTANTS_SX_HPP

