// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotSerialIOIface.cpp
 *  @brief TRobotSerialIOIfaceの実装
 *
 */

#include "TRobotSerialIOIface.hpp"

#include "TRobotSensorData.hpp"
#include "TRobotUtil.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>

#include <iostream>
#include <algorithm>
#include <cstdio>
#include <cstring>



////////////////////////////////////////////////////////////////////////////////
/*!
 *  SerialIOIfaceコンストラクタ
 */
TRobotIF::SerialIOIface::SerialIOIface()
    : m_recvThrId(nullptr),
      m_sendThrId(nullptr)
{
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  SerialIOIfaceデストラクタ
 */
TRobotIF::SerialIOIface::~SerialIOIface()
{
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief ロボットベースのファームウェアとの通信を開始する
 *
 *  @copybrief
 *
 *  @param  port   [in]    通信ポートを表す文字列(/dev/ttyUSB0 など) 
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::start( const char *port )
{
    using namespace std;

    struct termios termCfg;
    int retVal;


    //
    // 通信ポートオープン
    //
    m_portFd = open( port, O_RDWR | O_NOCTTY| O_NONBLOCK );
    if( m_portFd < 0 )
    {
        LOG_ERROUT( << "Cannot open " << port << ": \n" << flush );
        LOG_PERR( "Error" );
        return TRobotIF::ERROR_OPEN_PORT;
    }

    //
    // 通信ポート設定
    //
    memset( &termCfg, 0, sizeof(termCfg) );

    cfmakeraw( &termCfg );
    termCfg.c_iflag |= IGNPAR;                    // パリティエラーを無視
    termCfg.c_iflag &= ~( IXON | IXOFF | INPCK ); // フロー制御なし，パリティチェックなし
    termCfg.c_cflag |= ( CREAD| CS8 );            // 受信有効, データビット数 8bit

    if( isScibotX() || isLMES() )
    {
        // SCIBOT-X or L-MES
        termCfg.c_cflag |= CSTOPB;                // ストップビット 2 bit
    }
    else
    {
        // 旧ロボットベース or CIRCINUS（メカナム）
        termCfg.c_cflag &= ~( CSTOPB );           // ストップビット 1 bit
    }

    termCfg.c_cflag &= ~( CRTSCTS );              // CTS/RTS(ハードウェア)フロー制御無効
    termCfg.c_lflag &= ~( PARENB );               // パリティチェックなし
    termCfg.c_lflag &= ~( ECHO | ICANON );        // エコー，行編集を無効化

    termCfg.c_cc[ VMIN ] = 0;                     // non-blocking
    termCfg.c_cc[ VTIME ] = 0;

    cfsetispeed( &termCfg, B115200 );
    cfsetospeed( &termCfg, B115200 );
    retVal = tcsetattr( m_portFd, TCSANOW, &termCfg );
    if( retVal == -1 )
    {
        LOG_PERR( "tcsetattr():" );
    }
    retVal = tcflush( m_portFd, TCIOFLUSH );
    if( retVal == -1 )
    {
        LOG_PERR( "tcflush():" );
    }


    // 旧ロボットベース or CIRCINUS（メカナム）
    if( (!isScibotX()) && (!isLMES()) )
    {
#ifdef FIRSTSEND_WAIT
        // 開始待ち合わせをクリア
        m_startWait.clear();
#endif
    }

    // 残っているデータのせいで遅延が生じないようにする
    m_sensorDataQueue.clear();
    m_mntInfDataQueue.clear();
    m_robotStatDataQueue.clear();
    m_rotVelCmdQueue.clear();


    //
    // 受信スレッド作成
    //
    m_recvThrId = new std::thread([this]{ recvThreadFunc(); });
    if( m_recvThrId == nullptr )
    {
        LOG_PERR( "Failed to create a receiver thread" );
        close( m_portFd );
        return TRobotIF::ERROR_CREATE_RECV_THREAD;
    }
    LOG_OUT( << "Created a receiver thread" << endl );


    //
    // 送信スレッド作成
    //
    m_sendThrId = new std::thread([this]{ sendThreadFunc(); });
    if( m_sendThrId == nullptr )
    {
        LOG_PERR( "Failed to create a sender thread" );
        close(m_portFd);
        return TRobotIF::ERROR_CREATE_SEND_THREAD;
    }
    LOG_OUT( << "Created a sender thread" << endl );


    // 通信スレッド通信状態のON
    m_isRunning.setON();

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 通信を終了する
 *
 *  @copybrief
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::stop()
{
    // 通信スレッドの停止要求のON
    m_stopRequested.setON();

    // 待機を完了(旧ロボットベース or CIRCINUS（メカナム）)
    if( (!isScibotX()) && (!isLMES()) )
    {
#ifdef FIRSTSEND_WAIT
        m_startWait.done();
#endif
    }

    // スレッドの終了を待つ
    this->m_recvThrId->join();
    this->m_recvThrId = nullptr;
    this->m_sendThrId->join();
    this->m_sendThrId = nullptr;

    close( m_portFd );

    // 次にstart()が呼ばれるときに，残っているデータのせいで遅延が生じないようにする
    m_sensorDataQueue.clear();
    m_mntInfDataQueue.clear();
    m_robotStatDataQueue.clear();
    m_rotVelCmdQueue.clear();

    // 通信スレッドの停止要求のOFF
    m_stopRequested.setOFF();

    // 通信スレッド通信状態のOFF
    m_isRunning.setOFF();

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 指定された分のデータを全て受信する
 *
 *  @copybrief
 *
 *  @param  fd         [in]     通信ポートのファイルディスクリプタ
 *  @param  buf        [out]    受信データを格納するバッファ
 *  @param  numBytes   [in]     受信するデータのサイズ [bytes]
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::receiveExactNumBytes( int fd, unsigned char *buf, size_t numBytes )
{
    size_t   nrecv            = 0;
    ssize_t  retVal           = 0;
    fd_set   fdSet;
    int      selectRes        = 0;
    int      selectTimeOutCNT = 0;
    RetCode  return_Code      = TRobotIF::SUCCESS;
    FD_ZERO( &fdSet );

    // 指定サイズ受信するまでループ
    while( nrecv < numBytes )
    {
        // select() は timeout を変更しうるので呼び出し前に初期化が必要
        //struct timeval timeout = { 0, 20 * m_wait_duration };
        //struct timeval timeout = { 0, 1000000 };
        struct timeval timeout = { 0, m_wait_duration };


        // 停止要求？
        if( stopRequested() )
        {
            LOG_PERR( "recvThr stop Requested" );
            return_Code = TRobotIF::STOPREQUESTED;
            break;
        }

        FD_ZERO( &fdSet );
        FD_SET( fd, &fdSet );
        selectRes = select( fd + 1, &fdSet, NULL, NULL, &timeout );
        if( selectRes < 0 )
        {
            LOG_PERR( "recvThr Error on read select()" );
            return_Code = TRobotIF::ERROR_ON_READERR;
            break;
        }

        // タイムアウト？
        else if( selectRes == 0 )
        {
            if( selectTimeOutCNT >= 3 )
            {
                selectTimeOutCNT = 0;
                return_Code = TRobotIF::ERROR_ON_TIMEOUT;
                LOG_ERROUT( << "select() read timed out " << std::endl );
                break;
            }
            selectTimeOutCNT ++;
            //LOG_ERROUT( << "select() read timed out " << selectTimeOutCNT << std::endl );
            continue;
        }

        selectTimeOutCNT = 0;

        if( !FD_ISSET( fd, &fdSet ) )
        {
            continue;
        }

        retVal = read( fd, buf + nrecv, numBytes - nrecv );
        if( retVal > 0 )
        {
            nrecv += retVal;
            //LOG_DBGOUT( << "Sreceived " << retVal << " bytes." << std::endl );
        }
        else if( retVal < 0 )
        {
            LOG_PERR( "recvThr read Error" );
            return_Code = TRobotIF::ERROR_ON_READERR;
            break;
        }
        else if( retVal == 0 )
        {
            LOG_ERRPRINTF( "recvThr read Error size=%d\n", (int)retVal );
            return_Code = TRobotIF::ERROR_ON_READERR;
            break;
        }
    }

    if( m_trace_log )
    {
        LOG_DBGOUT( << "received: " << std::endl );
        LOG_DBGDUMP( buf, nrecv );
        LOG_DBGOUT( << std::endl );
    }

    return return_Code;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief データを全て空受信する
 *
 *  @copybrief
 *
 *  @param  fd         [in]     通信ポートのファイルディスクリプタ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::receiveAndThrow( int fd )
{
    ssize_t  retVal      = 0;
    fd_set   fdSet;
    int      selectRes   = 0;
    RetCode  return_Code = TRobotIF::SUCCESS;

    unsigned char  buf[256];

    FD_ZERO( &fdSet );

    // 指定サイズ受信するまでループ
    while( true )
    {
        // select() は timeout を変更しうるので呼び出し前に初期化が必要
        struct timeval timeout = { 0, m_wait_duration/4 };


        // 停止要求？
        if( stopRequested() )
        {
            LOG_PERR( "recvThr stop Requested" );
            return_Code = TRobotIF::STOPREQUESTED;
            break;
        }

        FD_ZERO( &fdSet );
        FD_SET( fd, &fdSet );
        selectRes = select( fd + 1, &fdSet, NULL, NULL, &timeout );
        if( selectRes < 0 )
        {
            LOG_PERR( "recvThr Error on read select()" );
            break;

        }

        // タイムアウト？（全て抜ききった）
        else if( selectRes == 0 )
        {
            break;
        }


        if( !FD_ISSET( fd, &fdSet ) )
        {
            continue;
        }

        retVal = read( fd, buf, sizeof(buf) );
        if( retVal <= 0 )
        {
            LOG_PERR( "recvThr read Error" );
            return_Code = TRobotIF::ERROR_ON_READERR;
            break;
        }
    }

    return return_Code;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 指定された分のデータを全て送信する
 *
 *  @copybrief
 *
 *  @param  fd         [in]    通信ポートのファイルディスクリプタ
 *  @param  buf        [in]    送信データを格納するバッファ
 *  @param  numBytes   [in]    送信するデータのサイズ [bytes] 
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::sendExactNumBytes( int fd, unsigned char *buf, size_t numBytes )
{
    fd_set  fdSet;
    size_t  nsent            = 0;
    ssize_t retVal           = 0;
    int     selectRes        = 0;
    int     selectTimeOutCNT = 0;
    RetCode return_Code      = TRobotIF::SUCCESS;

    FD_ZERO( &fdSet );

    while( nsent < numBytes )
    {
        // select() は timeout を変更しうるので呼び出し前に初期化が必要
        //struct timeval timeout = { 0, 20 * m_wait_duration };
        struct timeval timeout = { 0, 1000000 };

        // 停止要求？
        if( stopRequested() )
        {
            LOG_PERR( "sendThr stop Requested" );
            return_Code = TRobotIF::STOPREQUESTED;
            break;
        }

        FD_ZERO( &fdSet );
        FD_SET( fd, &fdSet );
        selectRes = select( fd + 1, NULL, &fdSet, NULL, &timeout );
        if( selectRes < 0 )
        {
            LOG_PERR( "sendThr Error on write select()" );
            return_Code = TRobotIF::ERROR_ON_WRITEERR;
            break;

        }

        // タイムアウト？
        else if( selectRes == 0 )
        {
            if( selectTimeOutCNT >= 3 )
            {
                selectTimeOutCNT = 0;
                return_Code = TRobotIF::ERROR_ON_TIMEOUT;
                LOG_ERROUT( << "select() write timed out " << std::endl );
                break;
            }
            selectTimeOutCNT ++;
            //LOG_ERROUT( << "select() write timed out " << selectTimeOutCNT << std::endl );
            continue;
        }

        if( !FD_ISSET( fd, &fdSet ) )
        {
            continue;
        }

        retVal = write( m_portFd, buf + nsent, numBytes - nsent );
        if( retVal > 0 )
        {
            nsent += retVal;
        }
        else if( retVal < 0 )
        {
            LOG_PERR( "sendThr write Error" );
            return_Code = TRobotIF::ERROR_ON_WRITEERR;
            break;
        }
        else if( retVal == 0 )
        {
            LOG_ERRPRINTF( "sendThr write Error size=%d\n", (int)retVal );
            return_Code = TRobotIF::ERROR_ON_WRITEERR;
            break;
        }
    }

    if( m_trace_log )
    {
        LOG_DBGOUT( << "sent: " << std::endl );
        LOG_DBGDUMP( buf, nsent );
        LOG_DBGOUT( << std::endl );
    }

    return return_Code;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *
 *  次のヘッダが見つかるまで読み込む
 *
 *  ポーリングでなく一方的なストリーミングのため，
 *  読み込んだバッファの先頭がデータのヘッダとは限らない
 *
 *  @param  buf    [out]  読み込み用のバッファ
 *  @param  bufLen [in]   buf の長さ [bytes]
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::seekToNextHeader( unsigned char *buf, size_t bufLen )
{
    using namespace std;

    const unsigned char     *headPos    = NULL;
    unsigned char           *Pos        = buf;
    unsigned char           *bufLast    = &buf[ bufLen ];
    size_t                  remDataLen  = 0;
    size_t                  recvlen     = bufLen;
    RetCode                 return_Code = TRobotIF::SUCCESS;

    unsigned char *tmpbuf[ bufLen ];

#ifdef TERMINAL_LOG
    int numSeekRetry = 0;
#endif

    // ヘッダが見つかるまでループ
    while( !stopRequested() )
    {
        // とりあえず受信サイズ分受信
        return_Code = receiveExactNumBytes( m_portFd, Pos, recvlen );
        if( return_Code == ERROR_ON_TIMEOUT )
        {
            //LOG_DBGOUT( << "seekToNextHeader ERROR_ON_TIMEOUT" << std::endl );
            break;
        }
        else if( return_Code != TRobotIF::SUCCESS )
        {
            break;
        }

        // 電文の先頭を検索
        if( isScibotX() )
        {
            // SCIBOT-Xフォーマット
            headPos = searchForValidHeader_SX( Pos, bufLast );
        }
        else if( isLMES() )
        {
            // L-MESフォーマット
            headPos = searchForValidHeader_LM( Pos, bufLast );
        }
        else if( isCIRCINUS() )
        {
            // CIRCINUS（メカナム）フォーマット
            headPos = searchForValidHeader_MN( Pos, bufLast );
        }
        else
        {
            // 旧ロボットベースフォーマット
            headPos = searchForValidHeader_EX( Pos, bufLast );
        }

        // 電文の先頭が見つからなければ読み捨てる
        if( headPos == (unsigned char *)NULL )
        {
            // ヘッダが途中で切れていた場合も捨てる
            LOG_OUT( << "numSeekRetry == " << numSeekRetry++ << endl );
            Pos = buf;
            headPos = NULL;
            recvlen = bufLen;
            //usleep( m_wait_duration );
            continue;
        }

        // まだ読み込んでいない残りのデータ長
        remDataLen = headPos - buf;
        // LOG_OUT( << "remDataLen == " << remDataLen << endl );

        // remDataLen == 0 であれば buf の先頭にヘッダがある
        if( remDataLen != 0 )
        {
            // ずらして残りのデータ長読み込む
            memcpy( tmpbuf, headPos, (bufLen - remDataLen) );
            memcpy( buf, tmpbuf, (bufLen - remDataLen) );
            return_Code = receiveExactNumBytes( m_portFd, (bufLast - remDataLen), remDataLen );
            if( return_Code == ERROR_ON_TIMEOUT )
            {
                break;
            }
            else if( return_Code != TRobotIF::SUCCESS )
            {
                break;
            }
        }

        // 旧ロボットベースorCIRCINUS（メカナム）フォーマットではチェックサムはなし
        if( (!isScibotX()) && (!isLMES()) )
        {
            // 電文の先頭が見つかった
            break;
        }

        // SUM値をチェック
        if( checkSum( *bufLast, buf, bufLast-1 ) )
        {
            // 電文の先頭が見つかった
            break;
        }

        // 1byteずらして最初から(読み込まないで、残りの電文で先頭を探す)
        recvlen = 0;        // 読み込むとき：bufLen - 1;
        Pos = buf + 1;
        memcpy( tmpbuf, Pos, (bufLen - 1) );
        memcpy( buf, tmpbuf, (bufLen - 1) );
        //usleep( m_wait_duration );
    }

    return return_Code;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 送信スレッドのメイン
 *
 *  @copybrief
 */
void TRobotIF::SerialIOIface::sendThreadFunc()
{
    using namespace std;

    static unsigned char cmdbuf[ TR_SNDBUF_LENGTH ];

    RetCode retCode;


    RotVelCmdPair cmdPair;
    bool firstSend = true;

    LOG_OUT( << "send thread starting..." << endl );

    // 通信スレッド停止要求が来るまでループ
    while( !stopRequested() )
    {
        // 送信指示なし
        if( m_rotVelCmdQueue.empty() )
        {
            usleep( m_wait_duration/4 );
            continue;
        }
        while( !m_rotVelCmdQueue.empty() )
        {
            m_rotVelCmdQueue.pop( cmdPair );

            // 速度指令コマンド以外は間引かず送信する
            if( ( isScibotX() && (cmdPair.cmd()!=Command_SPEED_CONTROL_MN) ) ||
                ( isLMES() && (cmdPair.cmd()!=Command_SPEED_CONTROL_LM) ) ||
                ( isCIRCINUS() && (cmdPair.cmd()!=Command_SPEED_CONTROL_MN) ) )
            {
                break;
            }
        }

        // 電文作成
        if( isScibotX() )
        {
            // SCIBOT-Xフォーマット
            retCode = buildCmdSeqSet_SX( cmdPair.cmd(), cmdPair.left(), cmdPair.right(), cmdbuf );
        }
        else if( isLMES() )
        {
            // L-MESフォーマット
            retCode = buildCmdSeqSet_LM( cmdPair.cmd(), cmdPair.left(), cmdPair.right(), cmdbuf );
        }
        else if( isCIRCINUS() )
        {
            // CIRCINUS（メカナム）フォーマット
            retCode = buildCmdSeqSet_MN( cmdPair.cmd(), cmdPair.left(), cmdPair.right(), cmdPair.left_r(), cmdPair.right_r(), cmdbuf );
        }
        else
        {
            // 旧ロボットベース
            retCode = buildCmdSeqSetRotVel( cmdPair.left(), cmdPair.right(), cmdbuf );
        }

        // 状態不一致
        if( retCode == TRobotIF::ERROR_ON_STATE )
        {
            continue;
        }

        // 異常
        if( retCode != TRobotIF::SUCCESS )
        {
            LOG_ERROUT( << "Failed to build a command sequence" << endl );

            // 通信スレッドの停止を要求
            m_stopRequested.setON();
            break;
        }

        // 送信
        retCode = sendExactNumBytes( m_portFd, cmdbuf, m_snd_length );
        if( retCode != TRobotIF::SUCCESS )
        {
            // SCIBOT-X or L-MES
            if( (isScibotX()) || (isLMES()) )
            {
                // 緊急停止モードに設定
                SetMode( (int)TRobotIF::RobotMode_ERROR_STOP );
            }

            // 通信スレッドの停止を要求
            m_stopRequested.setON();
        }

        // 初回送信時受信スレッドにシグナル
        if( firstSend )
        {
#ifdef FIRSTSEND_WAIT
            m_startWait.done();
#endif
            firstSend = false;
        }

        usleep( m_wait_duration/4 );
    }
    LOG_ERROUT( << "Stoped sendThreadFunc" << endl );

    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 受信スレッドのメインルーチン
 *
 *  @copybrief
 */
void TRobotIF::SerialIOIface::recvThreadFunc()
{
    using namespace TRobotIF;
    using namespace std;

    static unsigned char   buf[ TR_RCVBUF_LENGTH ];

    RetCode                return_Code         = TRobotIF::SUCCESS;


    LOG_OUT( << "recv thread starting..." << endl );

    //
    // データを全て空受信する
    //
    return_Code = receiveAndThrow( m_portFd );
    if( return_Code != TRobotIF::SUCCESS )
    {
        // SCIBOT-X
        if( isScibotX() )
        {
            // 緊急停止モードに設定
            SetMode( (int)TRobotIF::RobotMode_ERROR_STOP );
        }
        // L-MES
        else if( isLMES() )
        {
            // 故障モードに設定
            SetMode( (int)TRobotIF::RobotMode_FAIL );
        }

        // 通信スレッドの停止を要求
        m_stopRequested.setON();

        return;
    }

    //
    // 現在緊急停止モードなら、ロボットベースも緊急停止モードに
    //
    if( GetMode() == RobotMode_ERROR_STOP )
    {
        // SCIBOT-Xフォーマット
        if( isScibotX() )
        {
             // 緊急停止コマンド送信
             sendCmd( (int)Command_ERROR_STOP_SX );
        }
        // L-MES
        else if( isLMES() )
        {
             SetMode( (int)TRobotIF::RobotMode_INIT );
        }
    }


    //
    // 旧ロボットベースorCIRCINUS（メカナム）は最初のコマンド送信を待つ（デッドロックを防ぐため）
    //
    // 旧ロボットベース or CIRCINUS（メカナム）
    if( (!isScibotX()) && (!isLMES()) )
    {
#ifdef FIRSTSEND_WAIT
        m_startWait.wait();
#endif
    }

    //
    // 通信スレッド停止要求が来るまで無限ループ
    //
    while( !stopRequested() )
    {
        // 受信しつつ電文の先頭を探す
        return_Code = seekToNextHeader( buf, m_rcv_length );
        if( return_Code == ERROR_ON_TIMEOUT )
        {
            // SCIBOT-Xフォーマット
            if( isScibotX() )
            {
                // タイムアウトとして受信電文を解析処理する(SCIBOT-X)
                buf[ResponsePos_SX] = (unsigned char)Response_TIMEOUT_SX;
                AnalysisMessage_SX( buf );
            }
            // L-MES
            else if( isLMES() )
            {
                // タイムアウトとして受信電文を解析処理する(L-MES)
                buf[ResponsePos_LM] = (unsigned char)Response_TIMEOUT_LM;
                AnalysisMessage_LM( buf );
            }
            continue;
        }
        else if( return_Code != TRobotIF::SUCCESS )
        {
            // SCIBOT-X
            if( isScibotX() )
            {
                // 緊急停止モードに設定
                SetMode( (int)TRobotIF::RobotMode_ERROR_STOP );
            }
            // L-MES
            else if( isLMES() )
            {
                // 故障モードに設定
                SetMode( (int)TRobotIF::RobotMode_FAIL );
            }

            // 通信スレッドの停止を要求
            m_stopRequested.setON();
        }

        // SCIBOT-Xフォーマット
        if( isScibotX() )
        {
            // 受信電文を解析処理する(SCIBOT-X)
            AnalysisMessage_SX( buf );
        }
        // L-MES
        else if( isLMES() )
        {
            // 受信電文を解析処理する(L-MES)
            AnalysisMessage_LM( buf );
        }
        // CIRCINUS（メカナム）
        else if( isCIRCINUS() )
        {
            // 受信電文を解析処理する(CIRCINUS（メカナム）)
            AnalysisMessage_MN( buf );
        }
        else
        {
            // 受信電文を解析処理する(旧ロボットベース)
            AnalysisMessage_EX( buf );
        }

        //usleep( m_wait_duration );
    }
    LOG_ERROUT( << "Stoped recvThreadFunc" << endl );


    return;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 左右の車輪の回転速度の目標値を送信する
 *
 *  @copybrief
 *
 *  @param  leftRotVel    [in]  左車輪の回転速度指令値
 *  @param  rightRotVel   [in]  右車輪の回転速度指令値
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::sendWheelRotVel( int leftRotVel, int rightRotVel, int leftRotVel_r=0, int rightRotVel_r=0 )
{
    m_rotVelCmdQueue.push( RotVelCmdPair(
        isScibotX()?(int)Command_SPEED_CONTROL_SX:isLMES()?(int)Command_SPEED_CONTROL_LM:(int)Command_SPEED_CONTROL_MN,
        leftRotVel, rightRotVel, leftRotVel_r, rightRotVel_r) );
    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief コマンドを送信する
 *
 *  @copybrief
 *
 *  @param  cmd    [in]  コマンド(TRobotIF::CommandNum)
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::sendCmd( int cmd )
{
    m_rotVelCmdQueue.push( RotVelCmdPair(cmd, 0, 0, 0, 0) );
    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief センサデータを受信する
 *
 *  @copybrief
 *
 *  @param  sensorData [out] センサデータ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::receiveSensorData( TRobotIF::BaseSensorData& sensorData )
{

    if( m_sensorDataQueue.empty() )
    {
        return TRobotIF::ERROR_QUEUE_EMPTY;
    }
    m_sensorDataQueue.pop( sensorData );
    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief センサデータを送信する
 *
 *  @copybrief
 *
 *  @param  sensorData [in] センサデータ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::sendSensorData( TRobotIF::BaseSensorData& sensorData )
{

    m_sensorDataQueue.push( sensorData );
    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief メンテナンス情報を受信する
 *
 *  @copybrief
 *
 *  @param  mntInfData [out] メンテナンス情報
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::receiveMntInfData( TRobotIF::MntInfData& mntInfData )
{

    if( m_mntInfDataQueue.empty() )
    {
        return TRobotIF::ERROR_QUEUE_EMPTY;
    }
    m_mntInfDataQueue.pop( mntInfData );
    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief メンテナンス情報を送信する
 *
 *  @copybrief
 *
 *  @param  mntInfData [in] メンテナンス情報
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::sendMntInfData( TRobotIF::MntInfData& mntInfData )
{

    m_mntInfDataQueue.push( mntInfData );
    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 運転状態を受信する
 *
 *  @copybrief
 *
 *  @param  robotStat [out] 運転状態
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::receiveRobotStatData( int& robotStat )
{

    if( m_robotStatDataQueue.empty() )
    {
        return TRobotIF::ERROR_QUEUE_EMPTY;
    }
    m_robotStatDataQueue.pop( robotStat );
    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief 運転状態を送信する
 *
 *  @copybrief
 *
 *  @param  robotStat [in] 運転状態
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::SerialIOIface::sendRobotStatData( int& robotStat )
{

    m_robotStatDataQueue.push( robotStat );
    return TRobotIF::SUCCESS;
}
