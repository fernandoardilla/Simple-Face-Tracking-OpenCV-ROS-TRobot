// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotConstants_MN.hpp
 * @brief TRobotConstants_MNヘッダ
 *
 */

#ifndef TROBOT_CONSTANTS_MN_HPP
#define TROBOT_CONSTANTS_MN_HPP


//
// CIRCINUS（メカナム）電文
//

namespace TRobotIF
{
    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief 上位ソフトウェアがロボットベースに対して発行するコマンドの定義
     *
     *  @copybrief
     */
    enum CommandNum_MN
    {
        Command_SPEED_CONTROL_MN          = 0x05,   //!< 速度制御値を指定する。
        Command_SET_CONFIG_MN             = 0x16    //!< 設定情報を書き換える。
    };
} // namespace TRobotIF


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 電文コードデファイン
 *
 *  @copybrief
 */
#define TR_START_MN                    (0x02)                                //!< コマンド先頭ーク
#define TR_START_MN_CMDCONFIG          (0x54)                                //!< コマンド先頭ーク(設定情報書換コマンド)
#define TR_END_MN                      (0x03)                                //!< 終端
#define TR_RESERVED_MN                 (0x00)                                //!< リザーブ領域クリア値

#define TR_ROT_VEL_CMDSPEED_LEN_MN     (14)                                  //!< 速度指令コマンド送信長[bytes]
#define TR_ROT_VEL_CMDCONFIG_LEN_MN    (64)                                  //!< 設定情報書換コマンド送信長[bytes]
#define TR_ROT_VEL_CMD_LEN_MN          (TR_ROT_VEL_CMDCONFIG_LEN_MN)         //!< コマンド送信バッファ長[bytes]
#define TR_BASE_SENSOR_DATA_LEN_MN     (145)                                 //!< センサデータ電文長[bytes]
#define TR_DATA_CHUNK_LEN_MN           TR_BASE_SENSOR_DATA_LEN_MN            //!< 受信電文長[bytes]

#define TR_CTRL_RATE_MN                (16)                                  //!< サンプリング
#define TR_CTRL_PERIOD_MN              (TR_SEC_TO_USEC / TR_CTRL_RATE_MN)    //!< 周期[usec]

#define TR_ROT_VEL_CMD_MIN_MN          (-192)                                //!< 速度指令最小値
#define TR_ROT_VEL_CMD_MAX_MN          (+192)                                //!< 速度指令最大値

#define TR_PREV_DATA_RECV_SUCCESS_MN   (0x55)                                //!< 受信電文ステータス：上位コントローラから受信有
#define TR_PREV_DATA_RECV_FAILURE_MN   (0x66)                                //!< 受信電文ステータス：上位コントローラから受信無

#define TR_R_CTL_LSB_OFFSET_MN         (0x03)                                //!< モータ制御値データ先頭までのインデックス
#define TR_L_ENC_LSB_OFFSET_MN         (0x06)                                //!< 左車輪エンコーダ値データ先頭までのインデックス
#define TR_R_ENC_LSB_OFFSET_MN         (0x07)                                //!< 右車輪エンコーダ値データ先頭までのインデックス
#define TR_RL_ENC_MSB_OFFSET_MN        (0x08)                                //!< 左右車輪上位エンコーダ値データ先頭までのインデックス
#define TR_CYCLE_CNT_OFFSET_MN         (0x09)                                //!< サイクルカウント先頭までのインデックス


#endif // TROBOT_CONSTANTS_MN_HPP

