// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotConstants_COM.hpp
 * @brief TRobotConstants_COMヘッダ
 *
 */

#ifndef TROBOT_CONSTANTS_COM_HPP
#define TROBOT_CONSTANTS_COM_HPP



namespace TRobotIF
{

    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief ロボットベースモードの定義
     *
     *  @copybrief
     */
    enum RobotModeNum
    {
        RobotMode_INIT                = 1,     //!< 起動・診断モード
        RobotMode_IDLE                = 2,     //!< 待機モード
        RobotMode_DRIVE               = 3,     //!< 運転モード
        RobotMode_ERROR_STOP          = 4,     //!< 緊急停止モード
        RobotMode_FAIL                = 5,     //!< 故障モード
        RobotMode_POWER_SAVING        = 6,     //!< 省電力モード
        RobotMode_REBOOT              = 7,     //!< リブートモード
        RobotMode_MAX                 = 8      //!< ロボットベースモード最大
    };
#define ResModePos                  (4)


    ////////////////////////////////////////////////////////////////////////////////
    /*!
     *  @brief 内部リターンコードの定義
     *
     *  @copybrief
     */
    enum RetCode
    {
        SUCCESS                       = 0,     //!< 正常
        ERROR_QUEUE_EMPTY,                     //!< ERROR_NO_DATA は winerror.h で定義されているので使用不可 
        ERROR_OPEN_PORT,                       //!< ポートオープン失敗
        ERROR_CREATE_RECV_THREAD,              //!< レシーブスレッド作成失敗
        ERROR_CREATE_SEND_THREAD,              //!< センドスレッド作成失敗
        ERROR_OUT_OF_RANGE,                    //!< 範囲外エラー
        ERROR_ON_SEND,                         //!< 送信時エラー
        ERROR_ON_RECEIVE,                      //!< 受信時エラー
        ERROR_ON_STATE,                        //!< 状態不一致
        ERROR_ON_TIMEOUT,                      //!< タイムアウト
        ERROR_ON_READERR,                      //!< リードエラー
        ERROR_ON_WRITEERR,                     //!< ライトエラー
        STOPREQUESTED,                         //!< ストップリクエスト検知
        NUM_RET_CODES                          //!< リターンコード最大
    };


} // namespace TRobotIF



////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 電文コードデファイン
 *
 *  @copybrief
 */
#define TR_SEC_TO_USEC              (1000000)                             //!< μ秒単位の１秒

#define TR_BitON                    (0x01)                                //!< bitON

#endif // TROBOT_CONSTANTS_COM_HPP

