// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotUtil.cpp
 *  @brief ユーティリティ(ステートマシーン)の実装
 *
 */

#include <cmath>
#include <algorithm>
#include <string.h>

#include "TRobotSerialIOIface.hpp"
#include "TRobotUtil.hpp"
#include "TRobotSerialSM.hpp"
TRobotIF::TRobotStateM SM;



////////////////////////////////////////////////////////////////////////////////
/*!
 *  シリアル入出力用インスタンスへのポインタをステートマシーンクラスに登録
 *
 *  @param  serIO   [in]   シリアル入出力用インスタンスへのポインタ
 *
 */
void TRobotIF::SetserIO( SerialIOIface *serIO )
{
    SM.SetserIO( serIO );
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  シリアル入出力用インスタンスへのポインタをステートマシーンクラスに登録
 *
 *  @param  serIO   [in]   シリアル入出力用インスタンスへのポインタ
 *
 */
TRobotIF::SerialIOIface *TRobotIF::GetserIO()
{
    return SM.GetserIO();
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  モードをステートマシーンクラスに登録
 *
 *  @param  serIO   [in]   シリアル入出力用インスタンスへのポインタ
 *
 */
void TRobotIF::SetMode( int num )
{
    SM.SetMode( num );
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  初回運転状態へ自動遷移させる
 *
 */
void TRobotIF::SetInitFlag()
{
    SM.SetInitFlag();
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  状態、イベントで分岐(旧ロボットベース)
 *
 *  @param  num   [in]   イベント
 *          buf   [in]   受信電文
 *
 */
void TRobotIF::GoEventFunc_EX( int num, const unsigned char * buf )
{
    // 状態、イベントで分岐
    SM.GoEventFunc_EX( num, buf );
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  状態、イベントで分岐(SCIBOT-X)
 *
 *  @param  num   [in]   イベント
 *          buf   [in]   受信電文
 *
 */
void TRobotIF::GoEventFunc_SX( int num, const unsigned char * buf )
{
    // 状態、イベントで分岐
    SM.GoEventFunc_SX( num, buf );
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  状態、イベントで分岐(L-MES)
 *
 *  @param  num   [in]   イベント
 *          buf   [in]   受信電文
 *
 */
void TRobotIF::GoEventFunc_LM( int num, const unsigned char * buf )
{
    // 状態、イベントで分岐
    SM.GoEventFunc_LM( num, buf );
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  状態、イベントで分岐(CIRCINUS（メカナム）)
 *
 *  @param  num   [in]   イベント
 *          buf   [in]   受信電文
 *
 */
void TRobotIF::GoEventFunc_MN( int num, const unsigned char * buf )
{
    // 状態、イベントで分岐
    SM.GoEventFunc_MN( num, buf );
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief モード移行をする
 *
 *  @copybrief
 *
 *  @param  num        [in]     モード
 */
// モード移行
void TRobotIF::TRobotStateM::SetMode( int num )
{
    switch( num )
    {
    // 運転モード
    case RobotMode_DRIVE:
        break;

    // 起動・診断モード
    case RobotMode_INIT:
        break;

    // 待機モード
    case RobotMode_IDLE:
        if( this->_stat != RobotMode_IDLE )
        {
            ResGetInf_f  =  false;     // 現在情報取得応答未受信
            ResGetMode_f =  false;     // 現在モード取得応未答受信
        }
        break;

    // 緊急停止モード
    case RobotMode_ERROR_STOP:
        break;

    // 故障モード
    case RobotMode_FAIL:
        break;

    // 省電力モード
    case RobotMode_POWER_SAVING:
        break;

    // リブートモード
    case RobotMode_REBOOT:
        break;

    default:
        // 移行しない
        return;
    }

    if( this->_stat != (RobotModeNum)num )
    {
        this->_stat = (RobotModeNum)num;

        // 運転状態をキューに入れる
        this->m_serIO->sendRobotStatData( num );

        if( ((RobotModeNum)num == RobotMode_ERROR_STOP)   ||
            ((RobotModeNum)num == RobotMode_FAIL)         ||
            ((RobotModeNum)num == RobotMode_POWER_SAVING) ||
            ((RobotModeNum)num == RobotMode_REBOOT) )
        {
            LOG_PRINTF( "Mode ERROR_STOP or RobotMode_FAIL or RobotMode_POWER_SAVING or RobotMode_REBOOT mode=%02x\n", num );

            if( m_serIO->isScibotX() )
            {
                // エラー情報取得の為、速度０で速度制御をする(緊急停止、故障モードでは定周期で情報取得できる)
                this->m_serIO->sendCmd( (int)Command_SPEED_CONTROL_INF_SX );
            }

            // 緊急停止モードからの自動復旧?
            if( this->m_serIO->isAutoRecovery() )
            {
                if( m_serIO->isScibotX() && ((RobotModeNum)num == RobotMode_FAIL) )
                {
                    // 待機モードになっても自動で運転モードにしない
                    m_InitFlag = false;
                }
                else
                {
                    // 待機モードになったら自動で運転モードする
                    m_InitFlag = true;
                }
            }
            else
            {
                // 待機モードになっても自動で運転モードにしない
                m_InitFlag = false;
            }
        }
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 状態（旧ロボットベースの場合は待機/運転のみ）を取得
 *
 *  @copybrief
 *
 *  @param  num        [in]     モード
 */
// モード移行
int TRobotIF::GetMode()
{
    return SM.GetMode();
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  nop
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::nop( const unsigned char * const buf )
{
    LOG_PRINTF( "No Operation. Now State=%d\n", SM._stat );

    if( GetMode() == RobotMode_ERROR_STOP )
    {
        // 緊急停止モードからの自動復旧?
        if( m_serIO->isAutoRecovery() )
        {
            // SCIBOT-Xフォーマット
            if( m_serIO->isScibotX() )
            {
                // 待機モードに遷移を試みる
                m_serIO->sendCmd( (int)Command_RECOVERY_SX );
            }
            // L-MES
            else if( m_serIO->isLMES() )
            {
                // 待機モードに遷移を試みる
                m_serIO->sendCmd( (int)Command_RECOVERY_LM );
            }
        }
    }
    else if( GetMode() == RobotMode_REBOOT )
    {
        // 緊急停止モードからの自動復旧?
        if( m_serIO->isAutoRecovery() )
        {
            // L-MES
            if( m_serIO->isLMES() )
            {
                // 待機モードに遷移を試みる
                m_serIO->sendCmd( (int)Command_SET_IDLE_LM );
            }
        }
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  タイムアウト(SCIBOT-X)
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::TimeOut_SX( const unsigned char * const buf )
{
    //LOG_PRINTF( "Response TIMEOUT=%d\n", SM._stat );

    //
    // ロボットベース側の要因にて何の通知も無く
    // 故障/緊急停止に遷移している場合がある
    // 現在のモードを取得する
    m_serIO->sendCmd( (int)Command_GET_MODE_SX );

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  タイムアウト(L-MES)
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::TimeOut_LM( const unsigned char * const buf )
{
    LOG_PRINTF( "Response TIMEOUT=%d\n", SM._stat );

    // 現在のモードを取得する
    //m_serIO->sendCmd( (int)Command_SCAN_CONDITION_LM );
    m_serIO->sendCmd( (int)Command_GET_CONFIG_LM );

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  運転モード応答の処理(SCIBOT-X)
 *  stat_func_SX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResDrive_SX( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResDrive_SX\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    if( buf[ ResModePos ] != RobotMode_DRIVE )
    {
        LOG_PRINTF( "ERROR_ON_STATE\n" );

        //
        // 現在情報を取得する。
        // (まれに、成功しているのに"待機"で返ってくる時あり
        //  （ロボットベース側の応答送信の処理がロボットベース側の
        //    モード変更処理と待ち合わせできていない？？？
        //    = 成功/失敗で無く現在のモード？？？）
        //  この場合、他のコマンド応答で同じフィールドが"運転"になっている。
        //  再送することにより流れに乗せる。
        //
        m_serIO->sendCmd( (int)Command_SET_DRIVE_SX );
        return;
    }

    // 次回、待機モードになっても自動で運転モードにしない
    m_InitFlag = false;


    //
    // 速度制御値コマンド送信は上位からの指示時
    //

    return;
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
void TRobotIF::TRobotStateM::ResDrive_LM( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResDrive_SX\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    if( buf[ ResModePos ] != RobotMode_DRIVE )
    {
        LOG_PRINTF( "ERROR_ON_STATE\n" );

        //
        // 現在情報を取得する。
        // 成功しているのに"待機"で返ってくる時のため。
        //
        m_serIO->sendCmd( (int)Command_SET_DRIVE_LM );
        return;
    }

    // 次回、待機モードになっても自動で運転モードにしない
    m_InitFlag = false;


    //
    // 速度制御値コマンド送信は上位からの指示時
    //

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  現在モード取得応答の処理(SCIBOT-X)
 *  stat_func_SX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResGetMode_SX( const unsigned char * const buf )
{
    LOG_PRINTF( "ResGetMode_SX\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 現在モード取得応答受信
    ResGetMode_f =  true;

    // まだ、現在情報を取得していなければ
    if( !ResGetInf_f )
    {
        // 現在情報を取得する
        m_serIO->sendCmd( (int)Command_GET_INF_SX );
        return;
    }

    // 待機以外
    //if( GetMode() != RobotMode_IDLE )
    //{
    //    // 現在情報を取得する。(エラー取得の為)
    //    m_serIO->sendCmd( (int)Command_GET_INF_SX );
    //    return;
    //}


    // 待機
    if( SM.GetMode() == RobotMode_IDLE )
    {
        // 最初の1回だけ
        if( m_InitFlag )
        {
            // 運転モード
            m_serIO->sendCmd( (int)Command_SET_DRIVE_SX );
        }
    }
    else if( GetMode() == RobotMode_ERROR_STOP )
    {
        // 緊急停止モードからの自動復旧?
        if( this->m_serIO->isAutoRecovery() )
        {
            // 待機モードに遷移を試みる
            this->m_serIO->sendCmd( (int)Command_RECOVERY_SX );
        }
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  現在情報取得応答の処理(SCIBOT-X)
 *  stat_func_SX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResGetInf_SX( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResGetInf_SX\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 現在情報取得応答受信
    ResGetInf_f =  true;

    // メンテナンス情報をデコードする
    decodeMntInfData_SX( buf,  &this->mntInfData.MntInfData_SX );

    // メンテナンス情報をキューに入れる
    m_serIO->sendMntInfData( this->mntInfData );

    // まだ、モードを取得していなければ
    if( !ResGetMode_f )
    {
        // 現在のモードを取得する
        m_serIO->sendCmd( (int)Command_GET_MODE_SX );
        return;
    }

    // 待機以外
    //if( GetMode() != RobotMode_IDLE )
    //{
    //    // 現在情報を取得する。(エラー取得の為)
    //    m_serIO->sendCmd( (int)Command_GET_INF_SX );
    //    return;
    //}

    // 最初の1回だけ
    if( m_InitFlag )
    {
        // 待機
        if( GetMode() == RobotMode_IDLE )
        {
            // 運転モード
            m_serIO->sendCmd( (int)Command_SET_DRIVE_SX );
        }
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  Mode情報取得応答の処理(L-MES)
 *  stat_func_LM ("TRobotSerialLM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResScnCnd_LM( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResScnCnd_LM\n" );
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  設定情報取得応答の処理(L-MES)
 *  stat_func_LM ("TRobotSerialLM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResGetCnf_LM( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResScnCnd_LM\n" );
    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 現在情報取得応答受信
    ResGetInf_f =  true;

    // メンテナンス情報をデコードする
    decodeMntInfData_LM( buf,  &this->mntInfData.MntInfData_LM );

    // メンテナンス情報をキューに入れる
    m_serIO->sendMntInfData( this->mntInfData );

    // まだ、モードを取得していなければ
    if( !ResGetMode_f )
    {
        // 現在のモードを取得する
        //m_serIO->sendCmd( (int)Command_SCAN_CONDITION_LM );
        //return;
        ResGetMode_f =  true;
    }

    // 待機以外
    //if( GetMode() != RobotMode_IDLE )
    //{
    //    // 現在情報を取得する。(エラー取得の為)
    //    m_serIO->sendCmd( (int)Command_GET_CONFIG_LM );
    //    return;
    //}


    // つながって最初の１回目はエラークリアをする
    if( m_ErrClearFlag )
    {
        m_serIO->sendCmd( (int)Command_ERROR_CLEAR_LM );
        return;
    }

    // 待機
    if( GetMode() == RobotMode_IDLE )
    {
        // 最初の1回だけ
        if( m_InitFlag )
        {
            // 運転モード
            m_serIO->sendCmd( (int)Command_SET_DRIVE_LM );
        }
    }
    else if( GetMode() == RobotMode_ERROR_STOP )
    {
        // 緊急停止モードからの自動復旧?
        if( this->m_serIO->isAutoRecovery() )
        {
            // 待機モードに遷移を試みる
            this->m_serIO->sendCmd( (int)Command_RECOVERY_LM );
        }
    }
    else if( GetMode() == RobotMode_FAIL )
    {
        // 緊急停止モードからの自動復旧?
        if( this->m_serIO->isAutoRecovery() && m_serIO->isLMES() )
        {
            // 待機モードに遷移を試みる
            this->m_serIO->sendCmd( (int)Command_RECOVERY_LM );
        }
    }
    else if( GetMode() == RobotMode_REBOOT )
    {
        // 緊急停止モードからの自動復旧?
        if( this->m_serIO->isAutoRecovery() && m_serIO->isLMES() )
        {
            // 待機モードに遷移を試みる
            this->m_serIO->sendCmd( (int)Command_SET_IDLE_LM );
        }
    }


    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  待機モード応答の処理(SCIBOT-X)
 *  stat_func_SX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResIdele_SX( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResIdele_SX\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    //if( buf[ ResModePos ] != RobotMode_IDLE )
    //{
    //    // 現在情報を取得する。(エラー取得の為)
    //    m_serIO->sendCmd( (int)Command_GET_INF_SX );
    //    return;
    //}

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  待機モード応答の処理(L-MES)
 *  stat_func_LM ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResIdele_LM( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResIdele_LM\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    //if( buf[ ResModePos ] != RobotMode_IDLE )
    //{
    //    // 現在情報を取得する。(エラー取得の為)
    //    m_serIO->sendCmd( (int)Command_SCAN_CONDITION_LM );
    //    return;
    //}

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  緊急停止モード応答の処理(SCIBOT-X)
 *  stat_func_SX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResErrorStop_SX( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResErrorStop_SX\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    if( buf[ ResModePos ] != RobotMode_ERROR_STOP )
    {
        LOG_PRINTF( "ERROR_ON_STATE\n" );
        // 再送する
        m_serIO->sendCmd( (int)Command_ERROR_STOP_SX );
        return;
    }

    // 自分で緊急停止しているのだけども、自動復旧させる。
    if( GetMode() == RobotMode_ERROR_STOP )
    {
        // 緊急停止モードからの自動復旧?
        if( m_serIO->isAutoRecovery() )
        {
            // 待機モードに遷移を試みる
            m_serIO->sendCmd( (int)Command_RECOVERY_SX );
        }
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  速度制御値応答の処理(SCIBOT-X)
 *  stat_func_SX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResSpeed_SX( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResSpeed_SX\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    if( buf[ ResModePos ] != RobotMode_DRIVE )
    {
        //return;   // センサデータのエラーコードを処理する。
    }

    // 受信電文をデコードする(SCIBOT-X)
    decodeBaseSensorData_SX( buf, &this->sensorData.SensorData_SX );

    // センサデータをキューに入れる
    m_serIO->sendSensorData( this->sensorData );


    if( GetMode() == RobotMode_ERROR_STOP )
    {
        // 緊急停止モードからの自動復旧?
        if( m_serIO->isAutoRecovery() )
        {
            // 待機モードに遷移を試みる
            m_serIO->sendCmd( (int)Command_RECOVERY_SX );
            //LOG_PRINTF( "recv ResSpeed_SX  send Command_RECOVERY_SX\n" );
        }
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  速度制御値応答の処理(L-MES)
 *  stat_func_LM ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResSpeed_LM( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResSpeed_LM\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    if( buf[ ResModePos ] != RobotMode_DRIVE )
    {
        //return;   // センサデータのエラーコードを処理する。
    }

    // 受信電文をデコードする(L-MES)
    decodeBaseSensorData_LM( buf, &this->sensorData.SensorData_LM );

    // センサデータをキューに入れる
    m_serIO->sendSensorData( this->sensorData );


    if( (GetMode() == RobotMode_ERROR_STOP) || (GetMode() == RobotMode_FAIL) )
    {
        // 緊急停止モードからの自動復旧?
        if( m_serIO->isAutoRecovery() )
        {
            // 待機モードに遷移を試みる
            m_serIO->sendCmd( (int)Command_RECOVERY_LM );
            //LOG_PRINTF( "recv ResSpeed_LM  send Command_RECOVERY_LM\n" );
        }
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  緊急停止モードから待機モード応答の処理(SCIBOT-X)
 *  stat_func_SX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResRecovery_SX( const unsigned char * const buf )
{
    LOG_PRINTF( "ResRecovery_SX\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    if( buf[ ResModePos ] != RobotMode_IDLE )
    {
        LOG_PRINTF( "ERROR_ON_STATE\n" );
        // 再送する
        m_serIO->sendCmd( (int)Command_RECOVERY_SX );
        return;
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  緊急停止モードから待機モード応答の処理(L-MES)
 *  stat_func_LM ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResRecovery_LM( const unsigned char * const buf )
{
    LOG_PRINTF( "ResRecovery_LM\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    if( buf[ ResModePos ] != RobotMode_IDLE )
    {
        LOG_PRINTF( "ERROR_ON_STATE\n" );
        // 再送する
        m_serIO->sendCmd( (int)Command_RECOVERY_LM );
        return;
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  REBOOT応答の処理(L-MES)
 *  stat_func_LM ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResREBOOT_LM( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResREBOOT_LM\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // 応答モードチェック
    if( buf[ ResModePos ] != RobotMode_REBOOT )
    {
        LOG_PRINTF( "ERROR_ON_STATE\n" );
        // 再送する
        //m_serIO->sendCmd( (int)Command_REBOOT_LM );
        return;
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  Errクリア応答の処理(L-MES)
 *  stat_func_LM ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResErrClr_LM( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResErrClr_LM\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    // Errクリアフラグクリア
    m_ErrClearFlag = false;

    // 現在のモードを取得する
    m_serIO->sendCmd( (int)Command_GET_CONFIG_LM );

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  省電力応答の処理(L-MES)
 *  stat_func_LM ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_LMからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResSUSPEND_LM( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResSUSPEND_LM\n" );

    // モードセット
    SM.SetMode( (int)buf[ ResModePos ] );

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  速度制御値応答の処理(旧ロボットベース)
 *  stat_func_EX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResSpeed_EX( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResSpeed_EX\n" );

    // 受信電文をデコードする(旧ロボットベース)
    decodeBaseSensorData_EX( buf, &this->sensorData.SensorData_EX );

    // センサデータをキューに入れる
    m_serIO->sendSensorData( this->sensorData );
    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  速度制御値応答の処理(CIRCINUS（メカナム）)
 *  stat_func_EX ("TRobotSerialSM.hpp内で定義")の状態遷移定義から分岐。
 *  AnalysisMessage_SXからコールされる。
 *
 *  @param  buf   [in]        受信電文
 *
 */
void TRobotIF::TRobotStateM::ResSpeed_MN( const unsigned char * const buf )
{
    //LOG_PRINTF( "ResSpeed_MN\n" );

    // 受信電文をデコードする(旧ロボットベース)
    decodeBaseSensorData_MN( buf, &this->sensorData.SensorData_MN );

    // センサデータをキューに入れる
    m_serIO->sendSensorData( this->sensorData );
    return;
}
