// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotConstants_LM.hpp
 * @brief TRobotConstants_LMヘッダ
 *
 */

#ifndef TROBOT_CONSTANTS_LM_HPP
#define TROBOT_CONSTANTS_LM_HPP



namespace TRobotIF
{


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief 故障モードエラーステータス定義
     *
     *  @copybrief
     */
    enum FailureStatus_LM
    {
        Fail_Nothing_LM                    = 0x00,   //!< エラー無し

        // モータ電源開閉
        FailD_InabilityOpen_LM             = 0x01,   //!< 開不能故障(開指示でのモータ電圧)
        FailC_InabilityClose_LM            = 0x02,   //!< 閉不能故障(閉指示でのモータ電圧)

        // MPU
        FailC_AccelerometerRegisterFail_LM = 0x03,   //!< 演算器故障（定周期演算チェック）
        FailC_RegisterFail_LM              = 0x04,   //!< レジスタ故障（定周期R/Wチェック）
        FailD_ROMFail_LM                   = 0x05,   //!< ROM故障（定周期SUMチェック）
        FailD_RAMFail_LM                   = 0x06,   //!< RAM故障（定周期R/Wチェック）
        FailN_RAMOneShotErr_LM             = 0x07,   //!< 単発RAMソフトエラー
        FailD_RAMContinuationErr_LM        = 0x08,   //!< 連続RAMソフトエラー　注２
        FailN_WDTOneShotErr_LM             = 0x09,   //!< 単発WDTエラー
        FailD_WDTContinuationErr_LM        = 0x0a,   //!< 連続WDTエラー　注２
        FailN_ResetOneShotErr_LM           = 0x0b,   //!< 単発リセット要因（未定義命令等）
        FailD_ResetContinuationErr_LM      = 0x0c,   //!< 連続リセット要因　注２

        // WDT
        FailC_InsideWDTFail_LM             = 0x0d,   //!< 内部WDT故障
        FailC_OutsideWDTFail_LM            = 0x0e,   //!< 外部WDT故障

        // バッテリ
        FailD_BatteryOverVoltage_LM        = 0x0f,   //!< 過電圧（□V以上）
        FailC_BatteryHighVoltage_LM        = 0x10,   //!< 高電圧（□V以上）
        FailC_BatteryLowVoltage_LM         = 0x11,   //!< 低電圧（□V以下）

        // 電源
        FailC_RegulatorOverVoltage_LM      = 0x12,   //!< レギュレータの過電圧出力故障
        FailC_RegulatorLowVoltage_LM       = 0x13,   //!< レギュレータの低電圧出力故障

        // モータ駆動回路
        FailC_RightCircuitFail_LM          = 0x14,   //!< 右側モータ駆動回路故障
        FailC_LeftCircuitFail_LM           = 0x15,   //!< 左側モータ駆動回路故障
        FailC_RightCircuitOverCurrent_LM   = 0x16,   //!< 右側過電流検出
        FailC_LeftCircuitOverCurrent_LM    = 0x17,   //!< 左側過電流検出

        // エンコーダ
        FailC_EncoderRight_LM              = 0x18,   //!< 右側エンコーダ故障
        FailC_EncoderLeft_LM               = 0x19,   //!< 左側エンコーダ故障

        // PRADA(バッテリ)
        FailC_PRADABattery1Temp_LM         = 0x1a,   //!< バッテリ1温度異常
        FailC_PRADABattery2Temp_LM         = 0x1b,   //!< バッテリ2温度異常

        // PRADA(エンコーダ)
        FailC_PRADAEncoderOverSpeedR_LM    = 0x1c,   //!< 右側エンコーダ過速度
        FailC_PRADAEncoderOverSpeedL_LM    = 0x1d,   //!< 左側エンコーダ過速度

        // USB通信系
        FailE_USBUnreceivable_LM           = 0x1e,   //!< 受信不能
        FailE_USBSeqNumMismatch_LM         = 0x20,   //!< 受信シーケンス番号不一致
        FailE_USBFormat_LM                 = 0x21,   //!< フォーマット・エラー
        FailE_USBDataLength_LM             = 0x22,   //!< データ長エラー
        FailE_USBSUMValue_LM               = 0x23,   //!< SUMエラー
        FailE_USBRecvOverflow_LM           = 0x24,   //!< 受信オーバーフロー
        FailC_USBCommandValue_LM           = 0x25,   //!< 不正指令値

        // 電流検出器
        FailC_CurrentDetectorRight_LM      = 0x28,   //!< 右側電流検出器故障
        FailC_CurrentDetectorLeft_LM       = 0x29,   //!< 左側電流検出器故障

        // ADC
        FailC_ADC_LM                       = 0x2a,   //!< ADC故障

        // PRADA
        FailC_PRADARecv_LM                 = 0x2b,   //!< 電源速度監視受信エラー

        // ADC
        FailC_RightCurrentSensorADC_LM     = 0x2c,   //!< 右側電流センサADC故障
        FailC_LeftCurrentSensorADC_LM      = 0x2d,   //!< 左側電流センサADC故障


        // 電源制御基板通信
        FailD_PowerControlRecvErr_LM       = 0x2b,   //!< 電源制御基板受信エラー

        // ADC(12bit)
        FailN_ADCCurrentSRight_LM          = 0x2c,   //!< 右側電流センサADC故障
        FailN_ADCCurrentSLeft_LM           = 0x2d,   //!< 左側電流センサADC故障

        // 測距センサ
        FailDN_DistanceS0NoDetection_LM    = 0x32,   //!< 測距センサ0無検知故障
        FailC_DistanceS0False_LM           = 0x33,   //!< 測距センサ0誤検知故障

        FailDN_DistanceS1NoDetection_LM    = 0x33,   //!< 測距センサ1無検知故障
        FailC_DistanceS1False_LM           = 0x33,   //!< 測距センサ1誤検知故障

        FailDN_DistanceS2NoDetection_LM    = 0x33,   //!< 測距センサ2無検知故障
        FailC_DistanceS2False_LM           = 0x33,   //!< 測距センサ2誤検知故障

        FailDN_DistanceS3NoDetection_LM    = 0x33,   //!< 測距センサ3無検知故障
        FailC_DistanceS3False_LM           = 0x33,   //!< 測距センサ3誤検知故障

        FailDN_DistanceS4NoDetection_LM    = 0x33,   //!< 測距センサ4無検知故障
        FailC_DistanceS4False_LM           = 0x33,   //!< 測距センサ4誤検知故障

        FailDN_DistanceS5NoDetection_LM    = 0x33,   //!< 測距センサ5無検知故障
        FailC_DistanceS5False_LM           = 0x33,   //!< 測距センサ5誤検知故障

        FailDN_DistanceS6NoDetection_LM    = 0x33,   //!< 測距センサ6無検知故障
        FailC_DistanceS6False_LM           = 0x33,   //!< 測距センサ6誤検知故障

        FailDN_DistanceS7NoDetection_LM    = 0x33,   //!< 測距センサ7無検知故障
        FailC_DistanceS7False_LM           = 0x33,   //!< 測距センサ7誤検知故障

        FailDN_UltraSonicS0NoDetection_LM  = 0x33,   //!< 超音波センサ0無検知故障
        FailC_UltraSonicS0False_LM         = 0x33,   //!< 超音波センサ0誤検知故障

        FailDN_UltraSonicS1NoDetection_LM  = 0x33,   //!< 超音波センサ1無検知故障
        FailC_UltraSonicS1False_LM         = 0x33,   //!< 超音波センサ1誤検知故障

        // バンパセンサ
        FailC_BumperSNoDetection_LM        = 0x36,   //!< バンパセンサ無検知故障
        FailC_BumperSFalse_LM              = 0x37,   //!< バンパセンサ誤検知故障

        // 加速度センサ
        FailCN_AccelerometerOutput_LM      = 0x38,   //!< 加速度センサ出力故障
        FailCN_AccelerometerRegister_LM    = 0x39,   //!< 加速度センサレジスタ故障（定周期リード／ライト）

        // ジャイロセンサ
        FailCN_GyroSOutput_LM              = 0x3a,   //!< ジャイロセンサ出力故障
        FailCN_GyroSRegister_LM            = 0x3b,   //!< ジャイロセンサレジスタ故障（定周期リード／ライト）

        // 地磁気センサ
        FailCN_GeomagneticSOutput_LM       = 0x3c,   //!< 地磁気センサ出力故障
        FailCN_GeomagneticSRegister_LM     = 0x3d,   //!< 地磁気センサレジスタ故障（定周期リード／ライト）

        // エラー
        FailC_BumperSErr_LM                = 0x46,   //!< バンパセンサエラー
        FailC_APIOverCurrentErr_LM         = 0x47,   //!< アプリモータ過電流
        FailC_DistanceSErr_LM              = 0x48,   //!< 測距センサエラー
        FailC_AccelerometerErr_LM          = 0x49,   //!< 加速度センサエラー
        FailC_GyroSErr_LM                  = 0x4a,   //!< ジャイロセンサエラー
        FailC_GeomagneticSErr_LM           = 0x4b,   //!< 地磁気センサエラー
        FailC_NoFloorSErr_LM               = 0x4c,   //!< 床なしセンサエラー
        FailE_TempS1Err_LM                 = 0x4d,   //!< 温度1センサエラー
        FailE_TempS2Err_LM                 = 0x4e,   //!< 温度2センサエラー
        FailC_LowBatterySignalErr_LM       = 0x4f,   //!< バッテリ不足信号エラー
        FailC_RightOverCurrentErr_LM       = 0x50,   //!< 右側ハード過電流エラー
        FailC_LeftOverCurrentErr_LM        = 0x51,   //!< 左側ハード過電流エラー
    };


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief 上位ソフトウェアがロボットベースに対して発行するコマンドの定義
     *
     *  @copybrief
     */
    enum CommandNum_LM
    {
        Command_SET_DRIVE_LM              = 0x01,   //!< 運転モードに移行する。
        Command_SET_IDLE_LM               = 0x02,   //!< 待機モードに移行する。
        Command_DIAG_LM                   = 0x03,   //!< 緊急停止システムの自己診断を要求する。
        Command_RECOVERY_LM               = 0x04,   //!< 緊急停止モードから待機モードに戻る。
        Command_REBOOT_LM                 = 0x05,   //!< ロボットベースを再起動させる（MPUのソフトリセット）。
        Command_SUSPEND_LM                = 0x06,   //!< 省電力モードに移行する。
        Command_ERROR_STOP_LM             = 0x07,   //!< 緊急停止モードに移行する。

        Command_SPEED_CONTROL_LM          = 0x10,   //!< 速度制御値を指定する。

        Command_SCAN_CONDITION_LM         = 0x40,   //!< ロボットベースのモード、センサ情報を取得する。
        Command_SET_CONFIG_LM             = 0x41,   //!< 設定情報を書き換える。
        Command_GET_CONFIG_LM             = 0x42,   //!< 設定情報、レビジョン、シリアル番号を取得する。
        Command_PORT_WRITE_LM             = 0x43,   //!< I2C/SPIインタフェース・デバイスへのライトアクセス。
        Command_PORT_READ_LM              = 0x44,   //!< I2C/SPIインタフェース・デバイスへのリードアクセス。
        Command_ERROR_CLEAR_LM            = 0x45,   //!< エラー、軽故障データクリア

        Command_SET_PARA1_LM              = 0x50,   //!< パラメータ1を設定する。
        Command_SET_PARA2_LM              = 0x51,   //!< パラメータ2を設定する。

    };
#define CommandPos_LM                 (3)


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief ロボットベースの上位ソフトウェアコマンドに対する応答の定義
     *
     *  @copybrief
     */
    enum ResponseNum_LM
    {
        Response_SET_DRIVE_LM              = 0x01,   //!< 運転モードに移行する。
        Response_SET_IDLE_LM               = 0x02,   //!< 待機モードに移行する。
        Response_DIAG_LM                   = 0x03,   //!< 緊急停止システムの自己診断を要求する。
        Response_RECOVERY_LM               = 0x04,   //!< 緊急停止モードから待機モードに戻る。
        Response_REBOOT_LM                 = 0x05,   //!< ロボットベースを再起動させる（MPUのソフトリセット）。
        Response_SUSPEND_LM                = 0x06,   //!< 省電力モードに移行させる。

        Response_SPEED_CONTROL_LM          = 0x10,   //!< 速度制御値を指定する。

        Response_SCAN_CONDITION_LM         = 0x40,   //!< ロボットベースのモード、センサ情報を取得する。
        Response_SET_CONFIG_LM             = 0x41,   //!< 設定情報を書き換える。
        Response_GET_CONFIG_LM             = 0x42,   //!< 設定情報、レビジョン、シリアル番号を取得する。
        Response_PORT_WRITE_LM             = 0x43,   //!< I2C/SPIインタフェース・デバイスへのライトアクセス。
        Response_PORT_READ_LM              = 0x44,   //!< I2C/SPIインタフェース・デバイスへのリードアクセス。
        Response_ERROR_CLEAR_LM            = 0x45,   //!< エラー、軽故障データクリア

        Response_SET_PARA1_LM              = 0x50,   //!< パラメータ1を設定する。
        Response_SET_PARA2_LM              = 0x51,   //!< パラメータ2を設定する。

        Response_TIMEOUT_LM                = 0x99    //!< タイムアウト。(本プログラム内で生成)
    };
#define ResponsePos_LM                 (3)


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief 受信イベントの定義
     *
     *  @copybrief
     */
    enum EventNum_LM
    {
        Event_SET_DRIVE_LM                = 1,   //!< 運転モードに移行する。
        Event_SET_IDLE_LM                 = 2,   //!< 待機モードに移行する。
        Event_DIAG_LM                     = 3,   //!< 緊急停止システムの自己診断を要求する。
        Event_RECOVERY_LM                 = 4,   //!< 緊急停止モードから待機モードに戻る。
        Event_REBOOT_LM                   = 5,   //!< ロボットベースを再起動させる（MPUのソフトリセット）。
        Event_SUSPEND_LM                  = 6,   //!< 省電力モードに移行する。

        Event_SPEED_CONTROL_LM            = 7,   //!< 速度制御値を指定する。

        Event_SCAN_CONDITION_LM           = 8,   //!< ロボットベースのモード、センサ情報を取得する。
        Event_SET_CONFIG_LM               = 9,   //!< 設定情報を書き換える。
        Event_GET_CONFIG_LM               = 10,  //!< 設定情報、レビジョン、シリアル番号を取得する。
        Event_PORT_WRITE_LM               = 11,  //!< I2C/SPIインタフェース・デバイスへのライトアクセス。
        Event_PORT_READ_LM                = 12,  //!< I2C/SPIインタフェース・デバイスへのリードアクセス。
        Event_ERROR_CLEAR_LM              = 13,  //!< エラー、軽故障データクリア

        Event_SET_PARA1_LM                = 14,  //!< パラメータを設定する。
        Event_SET_PARA2_LM                = 15,  //!< パラメータを設定する。

        Event_TIMEOUT_LM                  = 16,  //!< タイムアウト。(本プログラム内で生成)
        Event_MAX_LM                      = 17   //!< 受信イベント最大
    };

} // namespace TRobotIF



////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 電文コードデファイン
 *
 *  @copybrief
 */
#define TR_CMD_LENGTH_LM             (64)                                      //!< コマンド長

#define TR_RCV_LENGTH_LM             TR_CMD_LENGTH_LM                          //!< 受信電文長

#define TR_START_LM                  (0x54)                                    //!< コマンド先頭ーク

#define TR_ROT_VEL_CMD_MIN_LM        (-160)                                    //!< 速度指令最小値
#define TR_ROT_VEL_CMD_MAX_LM        (+160)                                    //!< 速度指令最大値

#define TR_CTRL_PERIOD_LM            (62500)                                   //!< 周期[usec]
#define TR_CTRL_RATE_LM              (TR_SEC_TO_USEC / TR_CTRL_PERIOD_LM)      //!< サンプリング


//
// 速度制御値
// 速度制御コマンド応答で使用
//
#define ResRightMSpeedCtlPos_LM      (6)                                       //!< 右速度制御値バイト位置
#define ResLeftMSpeedCtlPos_LM       (8)                                       //!< 左速度制御値バイト位置


//
// 速度計測値
// 速度制御コマンド応答で使用
//
#define ResRightEncPos_LM            (10)                                      //!< 右速度計測値バイト位置
#define ResLeftEncPos_LM             (12)                                      //!< 左速度計測値バイト位置


//
// 駆動電流計測値
// 速度制御コマンド応答で使用
//
#define ResRightMCurrentPos_LM       (14)                                      //!< 右駆動電流計測値バイト位置
#define ResLeftMCurrentPos_LM        (16)                                      //!< 左駆動電流計測値バイト位置


//
// 測距センサ値
// 速度制御コマンド応答で使用
//
#define TR_Distance0Pos_LM           (18)                                      //!< CH0測距センサバイト位置
#define TR_Distance1Pos_LM           (19)                                      //!< CH1測距センサバイト位置
#define TR_Distance2Pos_LM           (20)                                      //!< CH2測距センサバイト位置
#define TR_Distance3Pos_LM           (21)                                      //!< CH3測距センサバイト位置
#define TR_Distance4Pos_LM           (22)                                      //!< CH4測距センサバイト位置
#define TR_Distance5Pos_LM           (23)                                      //!< CH5測距センサバイト位置
#define TR_Distance6Pos_LM           (24)                                      //!< CH6測距センサバイト位置
#define TR_Distance7Pos_LM           (25)                                      //!< CH7測距センサバイト位置


//
// 超音波センサ値
// 速度制御コマンド応答で使用
//
#define TR_UltraSonic0Pos_LM         (26)                                      //!< CH0超音波センサバイト位置
#define TR_UltraSonic1Pos_LM         (27)                                      //!< CH1超音波センサバイト位置


//
// 9軸センサ値
// 速度制御コマンド応答で使用
//
#define TR_AccXPos_LM                (28)                                      //!< X 加速度センサバイト位置
#define TR_AccYPos_LM                (30)                                      //!< Y 加速度センサバイト位置
#define TR_AccZPos_LM                (32)                                      //!< Z 加速度センサバイト位置

#define TR_GyrXPos_LM                (34)                                      //!< X ジャイロセンサバイト位置
#define TR_GyrYPos_LM                (36)                                      //!< Y ジャイロセンサバイト位置
#define TR_GyrZPos_LM                (38)                                      //!< Z ジャイロセンサバイト位置

#define TR_MagXPos_LM                (40)                                      //!< X コンパスセンサバイト位置
#define TR_MagYPos_LM                (42)                                      //!< Y コンパスセンサバイト位置
#define TR_MagZPos_LM                (44)                                      //!< Z コンパスセンサバイト位置


//
// バッテリ残量
// 速度制御コマンド応答で使用
//
#define TR_BatteryLevelUsePos_LM     (46)                                      //!< 使用中バッテリ残量バイト位置
#define TR_BatteryLevelNonUsePos_LM  (52)                                      //!< 未使用バッテリ残量バイト位置


//
// 基板温度
// 速度制御コマンド応答で使用
//
#define TR_BoardTempPos_LM           (47)                                      //!< 基板温度計測値バイト位置


//
// エラーコード
// 速度制御コマンド応答で使用
//
#define TR_LastFailDPos_LM           (48)                                      //!< 最後に発生した危険故障コードバイト位置
#define TR_LastFailCPos_LM           (49)                                      //!< 最後に発生した重度故障コード
#define TR_LastFailNPos_LM           (50)                                      //!< 最後に発生した軽度故障コード
#define TR_LastFailEPos_LM           (51)                                      //!< 最後に発生したエラーコード



//
// バージョン情報
// GET_CONFIGコマンド応答で使用
//
#define TR_ModelRevisionPos_LM       (5)                                       //!< モデルREVバイト位置
#define TR_HardwareRevisionPos_LM    (6)                                       //!< ハードウエアREVバイト位置
#define TR_FirmwareRevisionPos_LM    (7)                                       //!< ファームウェアREVバイト位置
#define TR_SerialNumberPos_LM        (8)                                       //!< シリアル番号バイト位置


//
// 稼働時間(秒) (ビッグエンディア形式)
// GET_CONFIGコマンド応答で使用
//
#define TR_OperatingTimePos_LM       (10)                                      //!< 稼働時間バイト位置


//
// エラー情報
// 情報取得コマンド応答で使用
//
#define TR_FailDPos_LM               (14)                                      //!< 危険故障コードバイト位置
#define TR_FailCPos_LM               (15)                                      //!< 重度故障コードバイト位置
#define TR_FailNPos_LM               (16)                                      //!< 軽度故障コードバイト位置
#define TR_FailEPos_LM               (17)                                      //!< エラーコードバイト位置
#define TR_FailWDTPos_LM             (18)                                      //!< WDTエラー発生回数バイト位置
#define TR_FailRAMSoftPos_LM         (19)                                      //!< RAMソフトエラーバイト位置


//
// 測距センサ実装(0-3)
// bit OFF:実装なし、ON:実装
// GET_CONFIGコマンド応答で使用
//
#define TR_DistanceSMountPos_LM      (20)                                      //!< 測距センサ実装バイト位置

#define TR_DistanceSMountCH0BIT_LM   (0)                                       //!< CH0測距センサ実装ビット位置
#define TR_DistanceSMountCH0MSK_LM   (TR_BitON<<TR_DistanceSMountCH0BIT_LM)  //!< CH0測距センサ実装マスク

#define TR_DistanceSMountCH1BIT_LM   (1)                                       //!< CH1測距センサ実装ビット位置
#define TR_DistanceSMountCH1MSK_LM   (TR_BitON<<TR_DistanceSMountCH1BIT_LM)  //!< CH1測距センサ実装マスク

#define TR_DistanceSMountCH2BIT_LM   (2)                                       //!< CH2測距センサ実装ビット位置
#define TR_DistanceSMountCH2MSK_LM   (TR_BitON<<TR_DistanceSMountCH2BIT_LM)  //!< CH2測距センサ実装マスク

#define TR_DistanceSMountCH3BIT_LM   (3)                                       //!< CH3測距センサ実装ビット位置
#define TR_DistanceSMountCH3MSK_LM   (TR_BitON<<TR_DistanceSMountCH3BIT_LM)  //!< CH3測距センサ実装マスク


//
// 測距センサ実装(4-7)
// bit OFF:実装なし、ON:実装
// GET_CONFIGコマンド応答で使用
//
#define TR_DistanceSMountPos2_LM     (21)                                      //!< 測距センサ実装バイト位置

#define TR_DistanceSMountCH4BIT_LM   (0)                                       //!< CH4測距センサ実装ビット位置
#define TR_DistanceSMountCH4MSK_LM   (TR_BitON<<TR_DistanceSMountCH4BIT_LM)    //!< CH4測距センサ実装マスク

#define TR_DistanceSMountCH5BIT_LM   (1)                                       //!< CH5測距センサ実装ビット位置
#define TR_DistanceSMountCH5MSK_LM   (TR_BitON<<TR_DistanceSMountCH5BIT_LM)    //!< CH5測距センサ実装マスク

#define TR_DistanceSMountCH6BIT_LM   (2)                                       //!< CH6測距センサ実装ビット位置
#define TR_DistanceSMountCH6MSK_LM   (TR_BitON<<TR_DistanceSMountCH6BIT_LM)    //!< CH6測距センサ実装マスク

#define TR_DistanceSMountCH7BIT_LM   (3)                                       //!< CH7測距センサ実装ビット位置
#define TR_DistanceSMountCH7MSK_LM   (TR_BitON<<TR_DistanceSMountCH7BIT_LM)    //!< CH7測距センサ実装マスク


//
// 超音波センサ実装(4-7)
// bit OFF:実装なし、ON:実装
// GET_CONFIGコマンド応答で使用
//
#define TR_UltraSonicSMountPos_LM    (22)                                      //!< 超音波センサ実装バイト位置

#define TR_UltraSonicSMountCH0BIT_LM (0)                                       //!< CH0超音波センサ実装ビット位置
#define TR_UltraSonicSMountCH0MSK_LM (TR_BitON<<TR_UltraSonicSMountCH0BIT_LM)  //!< CH0超音波センサ実装マスク

#define TR_UltraSonicSMountCH1BIT_LM (1)                                       //!< CH1超音波センサ実装ビット位置
#define TR_UltraSonicSMountCH1MSK_LM (TR_BitON<<TR_UltraSonicSMountCH1BIT_LM)  //!< CH1超音波センサ実装マスク


//
// 走行距離(m) (下位バイトから上位バイトの順)
// GET_CONFIGコマンド応答で使用
//
#define TR_TravelDistancePos_LM      (23)                                      //!< 走行距離バイト位置


//
// リセット(未定義命令)エラー発生回数
// GET_CONFIGコマンド応答で使用
//
#define TR_ResetNumPos_LM            (27)                                      //!< 走行距離バイト位置


//
// 開発用(コード作成 年月日時分)
// bit OFF:判定なし、ON:判定あり
// 情報取得コマンド応答で使用
//
#define TR_CreationYearPos_LM        (58)                                      //!< コード作成年バイト位置
#define TR_CreationMonthPos_LM       (59)                                      //!< コード作成月バイト位置
#define TR_CreationDayPos_LM         (60)                                      //!< コード作成日バイト位置
#define TR_CreationHourPos_LM        (61)                                      //!< コード作成時バイト位置
#define TR_CreationMinutePos_LM      (62)                                      //!< コード作成分バイト位置


#endif // TROBOT_CONSTANTS_LM_HPP

