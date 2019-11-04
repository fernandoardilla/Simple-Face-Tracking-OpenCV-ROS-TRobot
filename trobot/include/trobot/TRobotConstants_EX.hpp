// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotConstants_EX.hpp
 * @brief TRobotConstants_EXヘッダ
 *
 */

#ifndef TROBOT_CONSTANTS_EX_HPP
#define TROBOT_CONSTANTS_EX_HPP


//
// 旧電文
//

namespace TRobotIF
{

} // namespace TRobotIF



////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 電文コードデファイン
 *
 *  @copybrief
 */
#define TR_START                    (0x02)                                //!< コマンド先頭ーク
#define TR_END                      (0x03)                                //!< 終端
#define TR_ROT_VEL_CMD_ID           (0x02)                                //!< 速度指令コマンドID
#define TR_RESERVED                 (0x00)                                //!< リザーブ領域クリア値

#define TR_ROT_VEL_CMD_LEN          (8)                                   //!< 速度指令コマンド送信長[bytes]
#define TR_BASE_SENSOR_DATA_LEN     (145)                                 //!< センサデータ電文長[bytes]
#define TR_DATA_CHUNK_LEN           TR_BASE_SENSOR_DATA_LEN               //!< 受信電文長[bytes]

#define TR_CTRL_RATE                (16)                                  //!< サンプリング
#define TR_CTRL_PERIOD              (TR_SEC_TO_USEC / TR_CTRL_RATE)       //!< 周期[usec]

#define TR_ROT_VEL_CMD_MIN          (-100)                                //!< 速度指令最小値
#define TR_ROT_VEL_CMD_MAX          (+100)                                //!< 速度指令最大値

#define TR_PREV_DATA_RECV_SUCCESS   (0x99)                                //!< 受信電文ステータス：上位コントローラから受信有
#define TR_PREV_DATA_RECV_FAILURE   (0x66)                                //!< 受信電文ステータス：上位コントローラから受信無

#define TR_R_CTL_LSB_OFFSET         (0x03)                                //!< モータ制御値データ先頭までのインデックス
#define TR_R_ENC_LSB_OFFSET         (0x06)                                //!< 右車輪エンコーダ値データ先頭までのインデックス
#define TR_L_ENC_LSB_OFFSET         (0x07)                                //!< 左車輪エンコーダ値データ先頭までのインデックス
#define TR_RL_ENC_MSB_OFFSET        (0x08)                                //!< 左右車輪上位エンコーダ値データ先頭までのインデックス
#define TR_CYCLE_CNT_OFFSET         (0x09)                                //!< サイクルカウント先頭までのインデックス


#endif // TROBOT_CONSTANTS_EX_HPP

