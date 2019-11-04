// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotSerialIOIface.hpp
 * @brief TRobotSerialIOIfaceヘッダ
 *
 */

#ifndef TROBOT_SERIAL_IO_IFACE_HPP
#define TROBOT_SERIAL_IO_IFACE_HPP

#include "TRobotConstants.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <deque>
#include <mutex>
#include <functional>
#include <condition_variable>


namespace TRobotIF
{
    // メイン基板に接続されたセンサのデータを集約する構造体(全体)
    struct BaseSensorData;

    // メイン基板に接続されたセンサのデータを集約する構造体(旧ロボットベース)
    struct BaseSensorData_EX;

    // メイン基板に接続されたセンサのデータを集約する構造体(SCIBOT-X)
    struct BaseSensorData_SX;

    // メイン基板に接続されたセンサのデータを集約する構造体(L-MES)
    struct BaseSensorData_LM;

    // メイン基板に接続されたセンサのデータを集約する構造体(CIRCINUS（メカナム）)
    struct BaseSensorData_MN;

    // メンテナンス情報のデータを集約する構造体(全体)
    struct MntInfData;

    // メンテナンス情報のデータを集約する構造体(SCIBOT-X)
    struct MntInfData_SX;

    // メンテナンス情報のデータを集約する構造体(L-MES)
    struct MntInfData_LM;

    // cmd/left/rightのペアを作成
    // class RotVelCmdPair : std::pair<int, int>
    typedef std::pair<int, std::pair< std::pair<int, int>, std::pair<int, int> > > CmdLeftRightPair;
    class RotVelCmdPair : CmdLeftRightPair
    {
    public:
        RotVelCmdPair( int cmd = 0, int left = 0, int right = 0, int left_r = 0, int right_r = 0 )
            : CmdLeftRightPair(cmd, std::pair< std::pair<int, int>, std::pair<int, int> >( std::pair<int, int>( left, right ), std::pair<int, int>( left_r, right_r )) ) {}
        inline int cmd()   { return this->first; } 
        inline int left()  { return this->second.first.first; } 
        inline int right() { return this->second.first.second; }
        inline int left_r()  { return this->second.second.first; } 
        inline int right_r() { return this->second.second.second; }
    };


    //! スレッドセーフなキューのクラス
    template <typename T>
    class TRobotIFDeQueue
    {
    private:
        std::mutex m_mutex;
        std::deque<T> m_queue;

    public:
        TRobotIFDeQueue()
        {
            this->clear();
        }
        //! push
        void push(const T &value)
        {
            std::lock_guard<std::mutex> lock(this->m_mutex);
            m_queue.push_back(value);
        }
        //! pop
        void pop(T &value)
        {
            std::lock_guard<std::mutex> lock(this->m_mutex);
            value = this->m_queue.front();
            this->m_queue.pop_front();
        }
        //! empty
        bool empty()
        {
            std::lock_guard<std::mutex> lock(this->m_mutex);
            return this->m_queue.empty();
        }
        //! clear
        void clear()
        {
            std::lock_guard<std::mutex> lock(this->m_mutex);
            this->m_queue.clear();
        }

    };


    // スレッドセーフなフラグのクラス
    class TRobotIFFlag
    {
    private:
        std::mutex m_mutex;
        bool m_flag;

    public:
        TRobotIFFlag()
        {
            this->setOFF();
        }
        //! ON
        void setON()
        {
            std::lock_guard<std::mutex> lock(this->m_mutex);
            this->m_flag = true;
        }
        //! OFF
        void setOFF()
        {
            std::lock_guard<std::mutex> lock(this->m_mutex);
            this->m_flag = false;
        }
        //! look
        bool look()
        {
            std::lock_guard<std::mutex> lock(this->m_mutex);
            return this->m_flag;
        }

    };

    //! 同期クラス
    class TRobotIFThreadSync
    {
        std::mutex m_mutex;
        std::condition_variable m_cvar;
        bool m_started;

     public:
        //! コンストラクタ
        explicit TRobotIFThreadSync()
        {
            this->clear();
        }

        //! 待機する
        void wait()
        {
            {
                std::unique_lock<std::mutex> lock(this->m_mutex);
                this->m_cvar.wait(lock, [this] { return this->m_started; });
            }
            this->m_started = false;
        }

        //! 待機を完了する
        void done()
        {
            {
                std::lock_guard<std::mutex> lock(this->m_mutex);
                this->m_started = true;
            }
            this->m_cvar.notify_one();
        }
        //! clear
        void clear()
        {
            std::lock_guard<std::mutex> lock(this->m_mutex);
            this->m_started = false;
        }
    };


    //////////////////////////////////////////////////////////////////////
    /*!
     *  @brief Tロボットベースとのシリアル通信のためのクラス
     *
     *  @copybrief
     *
     */
    class SerialIOIface
    {
    public:
      SerialIOIface();                                                     //!< SerialIOIfaceコンストラクタ
      ~SerialIOIface();                                                    //!< SerialIOIfaceデストラクタ

      RetCode start( const char *port );                                   //!< 通信を開始する
      RetCode stop();                                                      //!< 通信を停止する

      RetCode sendWheelRotVel( int leftRotVel,     int rightRotVel,
                               int leftRotVel_r,   int rightRotVel_r );    //!< 回転速度指令を送信する
      RetCode sendCmd( int cmd );                                          //!< コマンドを送信する
      RetCode receiveSensorData( BaseSensorData& sensorData );             //!< センサデータを受信する
      RetCode sendSensorData( BaseSensorData& sensorData );                //!< センサデータを送信する
      RetCode receiveMntInfData( MntInfData& mntInfData );                 //!< メンテナンス情報を受信する
      RetCode sendMntInfData( MntInfData& mntInfData );                    //!< メンテナンス情報を送信する
      RetCode receiveRobotStatData( int& robotStat );                      //!< 運転状態を受信する
      RetCode sendRobotStatData( int& robotStat );                         //!< 運転状態を送信する

      bool stopRequested(){ return m_stopRequested.look(); };              //!< 通信スレッドの停止要求の有無を確認する
      bool isRunning(){ return m_isRunning.look(); };                      //!< 通信スレッド通信状態を確認する

      bool isScibotX(){ return m_ScibotXFlag; };                           //!< 現在の通信方法がSCIBOT-Xであるかを返す
      void setScibotXFlag(bool f){ m_ScibotXFlag=f; };                     //!< 現在の通信方法をSCIBOT-Xに設定する

      bool isLMES(){ return m_LMESFlag; };                                 //!< 現在の通信方法がL-MESであるかを返す
      void setLMESFlag(bool f){ m_LMESFlag=f; };                           //!< 現在の通信方法をL-MESに設定する

      bool isCIRCINUS(){ return m_CIRCINUSFlag; };                         //!< 現在の通信方法がCIRCINUS（メカナム）であるかを返す
      void setCIRCINUSFlag(bool f){ m_CIRCINUSFlag=f; };                   //!< 現在の通信方法をCIRCINUS（メカナム）に設定する

      bool isTraceLogFlag(){ return m_trace_log; };                         //!< 現在のトレースログ出力フラグを返す
      void setTraceLogFlag(bool f){ m_trace_log=f; };                       //!< 現在のトレースログ出力フラグを設定する

      bool isAutoRecovery(){ return m_autoRecovery; };                     //!< 緊急停止モードからの自動復旧するかを返す
      void setAutoRecovery(bool f){ m_autoRecovery=f; };                   //!< 緊急停止モードからの自動復旧するかに設定する

      unsigned int getCtrlRate(){ return m_ctrl_rate; };                   //!< サンプリングレートを返す
      void setCtrlRate(unsigned int v){ m_ctrl_rate=v; };                  //!< サンプリングレートを設定する

      useconds_t getWaitDuration(){ return m_wait_duration; };             //!< 送受信間隔を返す
      void setWaitDuration(unsigned int v){ m_wait_duration=v; };          //!< 送受信間隔を設定する

      size_t getSndLength(){ return m_snd_length; };                       //!< 送信データ長を返す
      void setSndLength(unsigned int v){ m_snd_length=v; };                //!< 送信データ長を設定する

      size_t getRcvLength(){ return m_rcv_length; };                       //!< 受信データ長を返す
      void setRcvLength(unsigned int v){ m_rcv_length=v; };                //!< 受信データ長を設定する

      // 設定情報書換コマンドのデータを保持するバッファ(CIRCINUS（メカナム）)
      unsigned char CmdConfig_MN[TR_ROT_VEL_CMDCONFIG_LEN_MN];

    private:
      TRobotIFFlag m_stopRequested;                                        //!< 通信停止要求フラグ 
      TRobotIFFlag m_isRunning;                                            //!< 通信中か否かを表すフラグ 
      TRobotIFThreadSync m_startWait;                                      //!< 開始待ち合わせ

      RetCode receiveExactNumBytes(
                    int fd, unsigned char *buf, size_t numBytes );         //!< 指定された分のデータを全て受信する
      RetCode receiveAndThrow( int fd  );
      RetCode sendExactNumBytes(
                    int fd, unsigned char *buf, size_t numBytes );         //!< 指定された分のデータを全て送信する
      void recvThreadFunc();                                               //!< 受信スレッドのメイン
      void sendThreadFunc();                                               //!< 送信スレッドのメイン

      RetCode seekToNextHeader( unsigned char *buf, size_t bufLen );       //!< 次のヘッダが見つかるまで読み込む

      int m_portFd;                                                        //!< シリアルポートのファイルディスクリプタ

      std::thread *m_recvThrId;                                            //!< 受信スレッドのスレッドID
      std::thread *m_sendThrId;                                            //!< 送信スレッドのスレッドID

      pthread_cond_t m_condFirstSendDone;                                  //!< 送信スレッドと受信スレッドの待ち合わせに用いる

      TRobotIFDeQueue<BaseSensorData> m_sensorDataQueue;                   //!< センサデータのキュー
      TRobotIFDeQueue<MntInfData>     m_mntInfDataQueue;                   //!< メンテナンス情報のキュー
      TRobotIFDeQueue<int>            m_robotStatDataQueue;                //!< 運転状態のキュー
      TRobotIFDeQueue<RotVelCmdPair>  m_rotVelCmdQueue;                    //!< 回転速度指令値のキュー

      bool m_ScibotXFlag;                                                  //!< SCIBOT-Xフラグ

      bool m_LMESFlag;                                                     //!< L-MESフラグ

      bool m_CIRCINUSFlag;                                                 //!< CIRCINUS（メカナム）フラグ

      bool m_autoRecovery;                                                 //!< 緊急停止モードからの自動復旧

      bool m_trace_log;                                                    //!< トレースログ出力フラグ

      useconds_t m_wait_duration;                                          //!< 送受信間隔

      size_t m_snd_length;                                                 //!< 送信データ長
      size_t m_rcv_length;                                                 //!< 受信データ長

      unsigned int  m_ctrl_rate;                                           //!< サンプリングレート

    };
} // namespace TRobotIF

#endif // TROBOT_SERIAL_IO_IFACE_HPP
