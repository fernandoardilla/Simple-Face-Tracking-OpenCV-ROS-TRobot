// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotSerialSM.hpp
 * @brief TRobotSerialSMヘッダ
 *
 */


#ifndef TROBOT_SERIALSM_HPP
#define TROBOT_SERIALSM_HPP

#include "TRobotUtil.hpp"

//-------------------------------------------------------------------
//-------------------------------------------------------------------


namespace TRobotIF
{
    //
    // ステートマシーンクラス
    //
    class TRobotStateM
    { 
    public:
        // 状態を初期化
        TRobotStateM() : _stat( RobotMode_INIT ) { m_InitFlag = true; m_ErrClearFlag = true; }

        // シリアル入出力用インスタンスへのポインタ
        void SetserIO( SerialIOIface *serIO ) { m_serIO = serIO; }

        // シリアル入出力用インスタンスへのポインタ
        SerialIOIface * GetserIO() { return m_serIO; }

        // 初回運転状態へ自動遷移させる
        void SetInitFlag()
        {
            //
            // クリア
            //
            SetMode( RobotMode_INIT );
            m_InitFlag = true;
            m_ErrClearFlag = true;

            //
            // 相手が接続しなおしを認識していない時のため
            //
            // SCIBOT-Xフォーマット
            if( m_serIO->isScibotX() )
            {
                // 現在のモードを取得する
                m_serIO->sendCmd( (int)Command_GET_MODE_SX );
            }
            // L-MES
            else if( m_serIO->isLMES() )
            {
                // 待機モードに遷移を試みる
                m_serIO->sendCmd( (int)Command_SET_IDLE_LM );
            }
        }

        // 状態、イベントで分岐(旧ロボットベース)
        void GoEventFunc_EX( int num, const unsigned char * buf )
        {
            ( this->*stat_func_EX[ (int)(_stat-1) ][ num-1 ] )( buf );
        }

        // 状態、イベントで分岐(SCIBOT-X)
        void GoEventFunc_SX( int num, const unsigned char * buf )
        {
            ( this->*stat_func_SX[ (int)(_stat-1) ][ num-1 ] )( buf );
        }

        // 状態、イベントで分岐(L-MES)
        void GoEventFunc_LM( int num, const unsigned char * buf )
        {
            ( this->*stat_func_LM[ (int)(_stat-1) ][ num-1 ] )( buf );
        }

        // 状態、イベントで分岐(CIRCINUS（メカナム）)
        void GoEventFunc_MN( int num, const unsigned char * buf )
        {
            ( this->*stat_func_MN[ (int)(_stat-1) ][ num-1 ] )( buf );
        }

        // モード移行
        void SetMode( int num );

        // 状態（旧ロボットベースの場合は待機/運転のみ）を取得
        int GetMode() { return (int)this->_stat; }

        // 送信データ長を設定する
        void setSndLength(unsigned int v){ m_serIO->setSndLength(v); };

        // センサデータ格納用
        BaseSensorData sensorData;

        // メンテナンス情報格納用
        MntInfData mntInfData;

        //
        // 状態遷移分岐関数群
        //
        // 処理なし
        void nop( const unsigned char * buf );

        // 運転モード応答の処理(SCIBOT-X)
        void ResDrive_SX( const unsigned char * buf );

        // 現在モード取得応答の処理(SCIBOT-X)
        void ResGetMode_SX( const unsigned char * buf );

        // 現在情報取得応答の処理(SCIBOT-X)
        void ResGetInf_SX( const unsigned char * buf );

        // 待機モード応答の処理(SCIBOT-X)
        void ResIdele_SX( const unsigned char * buf );

        // 緊急停止モード応答の処理(SCIBOT-X)
        void ResErrorStop_SX( const unsigned char * buf );

        // 速度制御値応答の処理(SCIBOT-X)
        void ResSpeed_SX( const unsigned char * buf );

        // 緊急停止モードから待機モード応答の処理(SCIBOT-X)
        void ResRecovery_SX( const unsigned char * buf );

        // タイムアウトの処理(SCIBOT-X)
        void TimeOut_SX( const unsigned char * buf );

        // 速度制御値応答の処理(旧ロボットベース)
        void ResSpeed_EX( const unsigned char * buf );

        // 運転モード応答の処理(L-MES)
        void ResDrive_LM( const unsigned char * buf );

        // 現在情報取得応答の処理(L-MES)
        void ResScnCnd_LM( const unsigned char * buf );

        // 設定情報取得応答の処理(L-MES)
        void ResGetCnf_LM( const unsigned char * buf );

        // 待機モード応答の処理(L-MES)
        void ResIdele_LM( const unsigned char * buf );

        // 速度制御値応答の処理(L-MES)
        void ResSpeed_LM( const unsigned char * buf );

        // 緊急停止モードから待機モード応答の処理(L-MES)
        void ResRecovery_LM( const unsigned char * buf );

        // リブート応答の処理(L-MES)
        void ResREBOOT_LM( const unsigned char * buf );

        // 省電力応答の処理(L-MES)
        void ResSUSPEND_LM( const unsigned char * buf );

        // エラークリア応答の処理(L-MES)
        void ResErrClr_LM( const unsigned char * buf );

        // タイムアウト(L-MES)
        void TimeOut_LM( const unsigned char * const buf );

        // 速度制御値応答の処理(CIRCINUS（メカナム）)
        void ResSpeed_MN( const unsigned char * buf );


    private:

        // シリアル入出力用インスタンスへのポインタ
        SerialIOIface *m_serIO;

        // 初回運転状態へ自動遷移させる為のフラグ
        bool m_InitFlag;

        // 初回エラークリアさせる為のフラグ
        bool m_ErrClearFlag;

        // 状態（旧ロボットベースの場合は待機/運転のみ）
        RobotModeNum _stat;

        // 現在モード取得応答受信フラグ
        bool ResGetMode_f;

        // 現在情報取得応答受信フラグ
        bool ResGetInf_f;

        // マトリックス(SCIBOT-X)
        static void (TRobotStateM::*const stat_func_SX[RobotMode_MAX-1][Event_MAX_SX-1])( const unsigned char * buf ); 

        // マトリックス(L-MES)
        static void (TRobotStateM::*const stat_func_LM[RobotMode_MAX-1][Event_MAX_LM-1])( const unsigned char * buf ); 

        // マトリックス(旧ロボットベース)
        static void (TRobotStateM::*const stat_func_EX[TRobotIF::RobotMode_MAX-1][1])( const unsigned char * buf ); 
        
        // マトリックス(CIRCINUS（メカナム）)
        static void (TRobotStateM::*const stat_func_MN[TRobotIF::RobotMode_MAX-1][1])( const unsigned char * buf ); 
    }; 



//
// 状態遷移表用デファイン
//
#define SmNop &TRobotStateM::nop

        // 運転モード応答の処理(SCIBOT-X)
#define SmResDrive_SX &TRobotStateM::ResDrive_SX

        // 現在モード取得応答の処理(SCIBOT-X)
#define SmResGetMode_SX &TRobotStateM::ResGetMode_SX

        // 現在情報取得応答の処理(SCIBOT-X)
#define SmResGetInf_SX &TRobotStateM::ResGetInf_SX

        // 待機モード応答の処理(SCIBOT-X)
#define SmResIdele_SX &TRobotStateM::ResIdele_SX

        // 緊急停止モード応答の処理(SCIBOT-X)
#define SmResErrorStop_SX &TRobotStateM::ResErrorStop_SX

        // 速度制御値応答の処理(SCIBOT-X)
#define SmResSpeed_SX &TRobotStateM::ResSpeed_SX

        // リカバリ応答の処理(SCIBOT-X)
#define SmResRecovery_SX &TRobotStateM::ResRecovery_SX

        // タイムアウトの処理(SCIBOT-X)
#define SmResTMOUT_SX &TRobotStateM::TimeOut_SX

        // 速度制御値応答の処理(旧ロボットベース)
#define SmResSpeed &TRobotStateM::ResSpeed

        // 運転モード応答の処理(L-MES)
#define SmResDrive_LM &TRobotStateM::ResDrive_LM

        // 現在モード情報取得応答の処理(L-MES)
#define SmResScnCnd_LM &TRobotStateM::ResScnCnd_LM

        // 現在モード情報取得応答の処理(L-MES)
#define SmResGetCnf_LM &TRobotStateM::ResGetCnf_LM

        // 待機モード応答の処理(L-MES)
#define SmResIdele_LM &TRobotStateM::ResIdele_LM

        // 速度制御値応答の処理(L-MES)
#define SmResSpeed_LM &TRobotStateM::ResSpeed_LM

        // 緊急停止モード応答の処理(L-MES)
#define SmResRecovery_LM &TRobotStateM::ResRecovery_LM

        // リブート応答の処理(L-MES)
#define SmResREBOOT_LM &TRobotStateM::ResREBOOT_LM

        // 省電力応答の処理(L-MES)
#define SmResSUSPEND_LM &TRobotStateM::ResSUSPEND_LM

        // エラークリア応答の処理(L-MES)
#define SmResErrClr_LM &TRobotStateM::ResErrClr_LM

        // タイムアウトの処理(L-MES)
#define SmResTMOUT_LM &TRobotStateM::TimeOut_LM

        // 速度制御値応答の処理(CIRCINUS（メカナム）)
#define SmResSpeed_MN &TRobotStateM::ResSpeed_MN



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 状態遷移(SCIBOT-X)
    //
    void (TRobotStateM::*const TRobotStateM::stat_func_SX[TRobotIF::RobotMode_MAX-1][TRobotIF::Event_MAX_SX-1])(const unsigned char * buf) =
    {
    // 運転応答       待機応答       緊急停止応答       リカバリ応答      速度制御応答   モード取得応答   情報取得応答    タイムアウト
    // 起動・診断モード
      {SmNop,         SmNop,         SmNop,             SmNop,            SmResSpeed_SX, SmResGetMode_SX, SmResGetInf_SX, SmResTMOUT_SX},

    // 待機モード
      {SmResDrive_SX, SmResIdele_SX, SmNop,             SmResRecovery_SX, SmResSpeed_SX, SmResGetMode_SX, SmResGetInf_SX, SmResTMOUT_SX},

    // 運転モード
      {SmResDrive_SX, SmResIdele_SX, SmResErrorStop_SX, SmNop,            SmResSpeed_SX, SmResGetMode_SX, SmResGetInf_SX, SmResTMOUT_SX},

    // 緊急停止モード
      {SmNop,         SmNop,         SmResErrorStop_SX, SmResRecovery_SX, SmResSpeed_SX, SmResGetMode_SX, SmResGetInf_SX, SmResTMOUT_SX},

    // 故障モード 
      {SmNop,         SmNop,         SmNop,             SmNop,            SmResSpeed_SX, SmResGetMode_SX, SmResGetInf_SX, SmResTMOUT_SX},

    // 省電力モード(ここへは遷移しない)
      {SmNop,         SmNop,         SmNop,             SmNop,            SmNop,         SmNop,           SmNop,          SmNop},

    // リブートモード(ここへは遷移しない)
      {SmNop,         SmNop,         SmNop,             SmNop,            SmNop,         SmNop,           SmNop,          SmNop}
    };


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 状態遷移(L-MES)
    //
    void (TRobotStateM::*const TRobotStateM::stat_func_LM[TRobotIF::RobotMode_MAX-1][TRobotIF::Event_MAX_LM-1])(const unsigned char * buf) =
    {
    // 運転応答       待機応答       診断応答 リカバリ応答      Reboot応答      省電力応答       速度制御応答   Mode情報取得応答 設定書換応答 設定取得応答    PortWrite応答 PortRead応答 Errクリア応答   PARA1応答 PARA2応答 タイムアウト
    // 起動・診断モード
      {SmNop,         SmResIdele_LM, SmNop,   SmNop,            SmNop,          SmNop,           SmResSpeed_LM, SmResScnCnd_LM,  SmNop,       SmResGetCnf_LM, SmNop,        SmNop,       SmNop,          SmNop,    SmNop,    SmResTMOUT_LM},

    // 待機モード
      {SmResDrive_LM, SmResIdele_LM, SmNop,   SmResRecovery_LM, SmResREBOOT_LM, SmResSUSPEND_LM, SmResSpeed_LM, SmResScnCnd_LM,  SmNop,       SmResGetCnf_LM, SmNop,        SmNop,       SmResErrClr_LM, SmNop,    SmNop,    SmResTMOUT_LM},

    // 運転モード
      {SmResDrive_LM, SmResIdele_LM, SmNop,   SmNop,            SmNop,          SmNop,           SmResSpeed_LM, SmResScnCnd_LM,  SmNop,       SmResGetCnf_LM, SmNop,        SmNop,       SmResErrClr_LM, SmNop,    SmNop,    SmResTMOUT_LM},

    // 緊急停止モード
      {SmNop,         SmNop,         SmNop,   SmResRecovery_LM, SmNop,          SmNop,           SmResSpeed_LM, SmResScnCnd_LM,  SmNop,       SmResGetCnf_LM, SmNop,        SmNop,       SmNop,          SmNop,    SmNop,    SmResTMOUT_LM},

    // 故障モード
      {SmNop,         SmNop,         SmNop,   SmResRecovery_LM, SmNop,          SmNop,           SmResSpeed_LM, SmResScnCnd_LM,  SmNop,       SmResGetCnf_LM, SmNop,        SmNop,       SmNop,          SmNop,    SmNop,    SmResTMOUT_LM},

    // 省電力モード
      {SmNop,         SmResIdele_LM, SmNop,   SmResRecovery_LM, SmNop,          SmResSUSPEND_LM, SmResSpeed_LM, SmResScnCnd_LM,  SmNop,       SmResGetCnf_LM, SmNop,        SmNop,       SmNop,          SmNop,    SmNop,    SmResTMOUT_LM},

    // リブートモード
      {SmNop,         SmResIdele_LM, SmNop,   SmResRecovery_LM, SmResREBOOT_LM, SmNop,           SmResSpeed_LM, SmResScnCnd_LM,  SmNop,       SmResGetCnf_LM, SmNop,        SmNop,       SmNop,          SmNop,    SmNop,    SmResTMOUT_LM}
    };


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 状態遷移(旧ロボットベース)
    //
    void (TRobotStateM::*const TRobotStateM::stat_func_EX[TRobotIF::RobotMode_MAX-1][1])(const unsigned char * buf) =
    {
    // 速度制御応答
    // 起動・診断モード
      {&TRobotStateM::ResSpeed_EX},
    // 待機モード
      {&TRobotStateM::ResSpeed_EX},
    // 運転モード(ここから開始して遷移しない)
      {&TRobotStateM::ResSpeed_EX},
    // 緊急停止モード
      {&TRobotStateM::ResSpeed_EX},
    // 故障モード
      {&TRobotStateM::ResSpeed_EX},
    // 省電力モード
      {&TRobotStateM::ResSpeed_EX},
    // リブートモード
      {&TRobotStateM::ResSpeed_EX}
    };


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 状態遷移(旧ロボットベース)
    //
    void (TRobotStateM::*const TRobotStateM::stat_func_MN[TRobotIF::RobotMode_MAX-1][1])(const unsigned char * buf) =
    {
    // 速度制御応答
    // 起動・診断モード
      {&TRobotStateM::ResSpeed_MN},
    // 待機モード
      {&TRobotStateM::ResSpeed_MN},
    // 運転モード(ここから開始して遷移しない)
      {&TRobotStateM::ResSpeed_MN},
    // 緊急停止モード
      {&TRobotStateM::ResSpeed_MN},
    // 故障モード
      {&TRobotStateM::ResSpeed_MN},
    // 省電力モード
      {&TRobotStateM::ResSpeed_MN},
    // リブートモード
      {&TRobotStateM::ResSpeed_MN}
    };

}

#endif // TROBOT_SERIALSM_HPP
