// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotUtil.hpp
 * @brief TRobotUtilヘッダ
 *
 */


#ifndef TROBOT_UTIL_HPP
#define TROBOT_UTIL_HPP

#include <cstdio>
#include <cstddef>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <cassert>
#include <algorithm>
#include <vector>


#include "TRobotConstants.hpp"
#include "TRobotSerialIOIface.hpp"
#include "TRobotSensorData.hpp"

//-------------------------------------------------------------------
//---------------------- 有効無効 -----------------------------------
//-------------------------------------------------------------------
#define TERMINAL_LOG
#define TERMINAL_ERRLOG
#define TERMINAL_DBGLOG
//-------------------------------------------------------------------
//-------------------------------------------------------------------



namespace TRobotIF
{
    // 受信電文を解析処理する(旧ロボットベース)
    RetCode AnalysisMessage_EX( const unsigned char * const buf );

    // 受信電文を解析処理する(SCIBOT-X)
    RetCode AnalysisMessage_SX( const unsigned char * const buf );

    // 受信電文を解析処理する(L-MES)
    RetCode AnalysisMessage_LM( const unsigned char * const buf );

    // 受信電文を解析処理する(CIRCINUS（メカナム）)
    RetCode AnalysisMessage_MN( const unsigned char * const buf );

    // 受信電文をデコードする(旧ロボットベース)
    RetCode decodeBaseSensorData_EX( const unsigned char * const buf,
                                     BaseSensorData_EX *sensorData );

    // 受信電文をデコードする(SCIBOT-X)
    RetCode decodeBaseSensorData_SX( const unsigned char * const buf,
                                     BaseSensorData_SX *sensorData );

    // 受信電文をデコードする(L-MES)
    RetCode decodeBaseSensorData_LM( const unsigned char * const buf,
                                     BaseSensorData_LM *sensorData );

    // 受信電文をデコードする(L-MES)
    RetCode decodeBaseSensorData_MN( const unsigned char * const buf,
                                     BaseSensorData_MN *sensorData );

    // メンテナンス情報をデコードする(SCIBOT-X)
    RetCode decodeMntInfData_SX( const unsigned char * const buf,
                                 MntInfData_SX *mntInfData );

    // メンテナンス情報をデコードする(L-MES)
    RetCode decodeMntInfData_LM( const unsigned char * const buf,
                                 MntInfData_LM *mntInfData );

    // 受信電文の先頭を検索する(旧ロボットベース)
    const unsigned char *searchForValidHeader_EX( const unsigned char *bufFirst,
                                                  const unsigned char *bufLast );
    // 受信電文の先頭を検索する(SCIBOT-X)
    const unsigned char *searchForValidHeader_SX( const unsigned char *bufFirst,
                                                  const unsigned char *bufLast );

    // 受信電文の先頭を検索する(L-MES)
    const unsigned char *searchForValidHeader_LM( const unsigned char *bufFirst,
                                                  const unsigned char *bufLast );
    // 受信電文の先頭を検索する(CIRCINUS（メカナム）)
    const unsigned char *searchForValidHeader_MN( const unsigned char *bufFirst,
                                                  const unsigned char *bufLast );

    // SUM値作成
    unsigned char makeSum( const unsigned char *bufFirst,
                                 const unsigned char *bufLast );

    // SUM値をチェックする
    bool checkSum( const unsigned char sum, const unsigned char *bufFirst,
                                            const unsigned char *bufLast );

    // 数値を最大/最小内の値にする
    template<class T> inline const T& limitRange( const T& val, const T& minVal, const T& maxVal )
    {
        return std::max( std::min( val, maxVal ), minVal );
    }

    // 数値を加速度制限内の値にする
    template<class T> inline T AccellimitRange( T& TargetVal, T& Val, T& TargetVal_r, T& Val_r, T& AccelVal, T& DecelVal )
    {
        if( TargetVal == Val )
        {
           return Val;
        }

        // 現在前進
        if( Val >= 0 )
        {
            // 目標前進
            if( TargetVal >= 0 )
            {
                // 加速
                if(  (std::abs(TargetVal) - std::abs(Val)) >= 0 )
                {
                    return (Val + std::min( (std::abs(TargetVal) - std::abs(Val)), AccelVal ));
                }
                // 減速
                else
                {
                    return (Val - std::min( (std::abs(Val) - std::abs(TargetVal)), DecelVal ));
                }
            }
            // 目標後退(加速として扱う)
            else
            {
                return (Val - std::min( (std::abs(Val) + std::abs(TargetVal)), AccelVal ));
            }
        }
        // 現在後退
        else
        {
            // 目標後退
            if( TargetVal <= 0 )
            {
                // 加速
                if( (std::abs(TargetVal) - std::abs(Val)) >= 0 )
                {
                    return (Val - std::min( (std::abs(TargetVal) - std::abs(Val)), AccelVal ));
                }
                // 減速
                else
                {
                    return (Val + std::min( (std::abs(Val) - std::abs(TargetVal)), DecelVal ));
                }
            }
            // 目標前進(加速として扱う)
            else
            {
                return (Val + std::min( (std::abs(TargetVal) + std::abs(Val)), AccelVal ));
            }
        }
    }


    // 数値を加速度制限内の値にする
    inline void AccellimitRange_Smooth_MN(
                     int Wheel_Num,                                          // 駆動輪の数
                     int &LeftRotVelCmd_TVal,   int &RightRotVelCmd_TVal,    // 目標速度 前(左,右)
                     int &LeftRotVelCmd_TVal_r, int &RightRotVelCmd_TVal_r,  // 目標速度 後(左,右)
                     int &LeftRotVelCmd,        int &RightRotVelCmd,         // 現在速度 前(左,右)
                     int &LeftRotVelCmd_r,      int &RightRotVelCmd_r,       // 現在速度 後(左,右)
                     int &VelAccelMax,          int &VelDecelMax,            // 加速MAX値, 減速MAX値
                     int &RetLeftRotVelCmd,     int &RetRightRotVelCmd,      // 結果速度 前(左,右)
                     int &RetLeftRotVelCmd_r,   int &RetRightRotVelCmd_r )   // 結果速度 後(左,右)
    {

        // 目標速度をvectorに格納
        std::vector<int> VelCmd_TVal;
        VelCmd_TVal.push_back( LeftRotVelCmd_TVal );
        VelCmd_TVal.push_back( RightRotVelCmd_TVal );

        // 現在速度をvectorに格納
        std::vector<int> RotVelCmd;
        RotVelCmd.push_back( LeftRotVelCmd );
        RotVelCmd.push_back( RightRotVelCmd );

        // 結果速度アドレスをvectorに格納
        std::vector<int*> RetRotVelCmd_p;
        RetRotVelCmd_p.push_back( &RetLeftRotVelCmd );
        RetRotVelCmd_p.push_back( &RetRightRotVelCmd );

        // 加速をvectorに格納
        std::vector<int> Veldiff;
        Veldiff.push_back( abs(LeftRotVelCmd_TVal - LeftRotVelCmd) );
        Veldiff.push_back( abs(RightRotVelCmd_TVal - RightRotVelCmd) );

        // 4輪駆動（メカナム）は後輪も格納
        if( Wheel_Num == 4 )
        {
            // 目標速度
            VelCmd_TVal.push_back( LeftRotVelCmd_TVal_r );
            VelCmd_TVal.push_back( RightRotVelCmd_TVal_r );

            // 現在速度
            RotVelCmd.push_back( LeftRotVelCmd_r );
            RotVelCmd.push_back( RightRotVelCmd_r );

            // 結果速度アドレス
            RetRotVelCmd_p.push_back( &RetLeftRotVelCmd_r );
            RetRotVelCmd_p.push_back( &RetRightRotVelCmd_r );

            // 加速
            Veldiff.push_back( abs(LeftRotVelCmd_TVal_r - LeftRotVelCmd_r) );
            Veldiff.push_back( abs(RightRotVelCmd_TVal_r - RightRotVelCmd_r) );
        }

        // 加速最大の車輪のインデックス
        std::vector<int>::iterator maxIt = std::max_element( Veldiff.begin(), Veldiff.end() );
        size_t difmaxIdx = std::distance(Veldiff.begin(), maxIt);


        // 加速し終わった
        if( Veldiff[difmaxIdx] == 0 )
        {
            // 結果に目標速度
            for( size_t i=0; i<Veldiff.size(); i++ )
            {
                *RetRotVelCmd_p[i]  = (double)VelCmd_TVal[i];
            }

            // vector開放
            std::vector<int>().swap(VelCmd_TVal);
            std::vector<int>().swap(RotVelCmd);
            std::vector<int*>().swap(RetRotVelCmd_p);
            std::vector<int>().swap(Veldiff);

            return;
        }


        // 加速or減速
        bool AccelFlag;
        // 現在と目標が順方向：目標前進で現在前進or目標後退で現在後退
        if( ( ( VelCmd_TVal[difmaxIdx] >= 0 ) && ( RotVelCmd[difmaxIdx] >= 0 ) ) || 
            ( ( VelCmd_TVal[difmaxIdx] <= 0 ) && ( RotVelCmd[difmaxIdx] <= 0 ) ) )
        {
            AccelFlag = ( ( abs(VelCmd_TVal[difmaxIdx]) - abs(RotVelCmd[difmaxIdx]) ) >= 0 ) ? true : false;
        }
        // 現在と目標が反転(一律加速として扱う)
        else
        {
            AccelFlag =  true;
        }

        // 制限値：加速の時はVelAccelMax、減速の時はVelDecelMax
        int VelMax = ( AccelFlag ) ? VelAccelMax : VelDecelMax;

        // 加速値（差分の絶対値が制限値を超えていれば制限値）
        int Veladd  = ( Veldiff[difmaxIdx] >= VelMax ) ? VelMax : Veldiff[difmaxIdx];

        // 目標が前進
        if( VelCmd_TVal[difmaxIdx] > 0 )
        {
            // 加速なら正を足しこむ、減速なら負を足しこむ
            Veladd *= ( AccelFlag ) ? 1 : -1;
        }
        // 目標が停止
        else if( VelCmd_TVal[difmaxIdx] == 0 )
        {
            // 減速のみ。現在が前進なら負を足しこむ、後退なら正を足しこむ
            Veladd *= ( RotVelCmd[difmaxIdx] >= 0 )  ? -1 : 1;
        }
        // 目標が後退
        else
        {
            // 加速なら負を足しこむ、減速なら正を足しこむ
            Veladd *= ( AccelFlag ) ? -1 : 1;
        }


        // 結果速度を格納
        double ratio = fabs(( (double)Veladd / (double)Veldiff[difmaxIdx] ) );
        for( size_t i=0; i<Veldiff.size(); i++ )
        {
            if( i == difmaxIdx )
            {
                *RetRotVelCmd_p[i]  = (double)RotVelCmd[i] + Veladd;
            }
            else
            {
                *RetRotVelCmd_p[i] = (double)RotVelCmd[i] + ((double)( ( VelCmd_TVal[i]  - RotVelCmd[i] ) ) * ratio);
            }
        }

        // vector開放
        std::vector<int>().swap(VelCmd_TVal);
        std::vector<int>().swap(RotVelCmd);
        std::vector<int*>().swap(RetRotVelCmd_p);
        std::vector<int>().swap(Veldiff);

        return;
    }


    // ファームウェア用指令シーケンスを組み立てる
    RetCode buildCmdSeqSet_SX( int cmd, int leftRotVel, int rightRotVel, unsigned char *cmdbuf );

    // 速度指令指令シーケンスを組み立てる(旧ロボットベース)
    RetCode buildCmdSeqSetRotVel( int leftRotVel, int rightRotVel, unsigned char *cmdbuf );

    // 速度指令指令シーケンスを組み立てる(SCIBOT-X)
    RetCode buildCmdSeqSetRotVel_SX( int leftRotVel, int rightRotVel, unsigned char *cmdbuf );

    // 速度指令指令シーケンスを組み立てる(CIRCINUS（メカナム）)
    RetCode buildCmdSeqSetRotVel_MN( int leftRotVel, int rightRotVel, int leftRotVel_r, int rightRotVel_r, unsigned char *cmdbuf );

    // ファームウェア用運転モードコマンドを組み立てる(SCIBOT-X)
    RetCode buildCmdSeqSetDrive_SX( unsigned char *cmdbuf );

    // ファームウェア用待機コマンドを組み立てる(SCIBOT-X)
    RetCode buildCmdSeqSetIdle_SX( unsigned char *cmdbuf );

    // ファームウェア用緊急停止コマンドを組み立てる(SCIBOT-X)
    RetCode buildCmdSeqSetErrStop_SX( unsigned char *cmdbuf );

    // ファームウェア用リカバリコマンドを組み立てる(SCIBOT-X)
    RetCode buildCmdSeqSetRecovery_SX( unsigned char *cmdbuf );

    // ファームウェア用モード取得コマンドを組み立てる(SCIBOT-X)
    RetCode buildCmdSeqSetGetMode_SX( unsigned char *cmdbuf );

    // ファームウェア用情報取得コマンドを組み立てる(SCIBOT-X)
    RetCode buildCmdSeqSetGetInf_SX( unsigned char *cmdbuf );


    // ファームウェア用指令シーケンスを組み立てる
    RetCode buildCmdSeqSet_LM( int cmd, int leftRotVel, int rightRotVel, unsigned char *cmdbuf );

    // 速度指令指令シーケンスを組み立てる(L-MES)
    RetCode buildCmdSeqSetRotVel_LM( int leftRotVel, int rightRotVel, unsigned char *cmdbuf );

    // ファームウェア用運転モードコマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetDrive_LM( unsigned char *cmdbuf );

    // ファームウェア用緊急停止コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetErrStop_LM( unsigned char *cmdbuf );

    // ファームウェア用待機コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetIdle_LM( unsigned char *cmdbuf );

    // ファームウェア用緊急停止システムの自己診断要求コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetDIAG_LM( unsigned char *cmdbuf );

    // ファームウェア用リカバリコマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetRecovery_LM( unsigned char *cmdbuf );

    // ファームウェア用再起動コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetReboot_LM( unsigned char *cmdbuf );

    // ファームウェア用省電力モード移行コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetSuspend_LM( unsigned char *cmdbuf );

    // ファームウェア用モード、センサ情報取得コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetScanCondition_LM( unsigned char *cmdbuf );

    // ファームウェア用設定情報書換コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetSetConfig_LM( unsigned char *cmdbuf );

    // ファームウェア用設定情報取得コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetGetConfig_LM( unsigned char *cmdbuf );

    // ファームウェア用I2C/SPIインタフェース・デバイスへのライトアクセスコマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetPortWrite_LM( unsigned char *cmdbuf );

    // ファームウェア用I2C/SPIインタフェース・デバイスへのリードアクセスコマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetPortRead_LM( unsigned char *cmdbuf );

    // ファームウェア用エラー、軽故障データクリアコマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetErrorClear_LM( unsigned char *cmdbuf );

    // ファームウェア用パラメータ1設定コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetSetPara1_LM( unsigned char *cmdbuf );

    // ファームウェア用パラメータ2設定コマンドを組み立てる(L-MES)
    RetCode buildCmdSeqSetSetPara2_LM( unsigned char *cmdbuf );


    // ファームウェア用指令シーケンスを組み立てる(CIRCINUS（メカナム）)
    RetCode buildCmdSeqSet_MN( int cmd, int leftRotVel, int rightRotVel, int leftRotVel_r, int rightRotVel_r, unsigned char *cmdbuf );

    // ファームウェア用設定情報書換コマンドを組み立てる((CIRCINUS（メカナム）))
    RetCode buildCmdSeqSetSetConfig_MN( unsigned char *cmdbuf );


    // シリアル入出力用インスタンスへのポインタをステートマシーンクラスに登録
    void SetserIO( SerialIOIface *serIO );

    // シリアル入出力用インスタンスへのポインタをステートマシーンクラスからGET
    SerialIOIface * GetserIO();

    // モードをステートマシーンクラスに登録
    void SetMode( int num );

    // 状態、イベントで分岐(旧ロボットベース)
    void GoEventFunc_EX( int num, const unsigned char * buf );

    // 状態、イベントで分岐(SCIBOT-X)
    void GoEventFunc_SX( int num, const unsigned char * buf );

    // 状態、イベントで分岐(L-MES)
    void GoEventFunc_LM( int num, const unsigned char * buf );

    // 状態、イベントで分岐(CIRCINUS（メカナム）)
    void GoEventFunc_MN( int num, const unsigned char * buf );

    // 初回運転状態へ自動遷移させる
    void SetInitFlag();

    // 状態（旧ロボットベースの場合は待機/運転のみ）を取得
    int GetMode();
}


//-----------------
// ログ出力マクロ
//-----------------
#ifdef TERMINAL_LOG
#    define LOG_OUT(line)                                             \
     {                                                                \
         struct timeval nowTime;                                      \
         gettimeofday(&nowTime, NULL);                                \
         struct tm *pnow = localtime(&nowTime.tv_sec);                \
         std::fprintf( stdout,                                        \
                "[%02d-%02d-%02d %02d:%02d:%02d.%03d] ",              \
                pnow->tm_year-100, pnow->tm_mon+1, pnow->tm_mday,     \
                pnow->tm_hour, pnow->tm_min,                          \
                pnow->tm_sec, (int)(nowTime.tv_usec/1000) );          \
         std::cout line;                                              \
         std::fflush(stdout);                                         \
     }

#    define LOG_OUT_NOTTIME(line) { std::cout line; }
#else
#    define LOG_OUT(line) ( (void)0 )
#    define LOG_OUT_NOTTIME(line) ( (void)0 )
#endif

#ifdef TERMINAL_ERRLOG
#    define LOG_ERROUT(line)                                          \
     {                                                                \
         struct timeval nowTime;                                      \
         gettimeofday(&nowTime, NULL);                                \
         struct tm *pnow = localtime(&nowTime.tv_sec);                \
         std::fprintf( stderr,                                        \
                "[%02d-%02d-%02d %02d:%02d:%02d.%03d] ",              \
                pnow->tm_year-100, pnow->tm_mon+1, pnow->tm_mday,     \
                pnow->tm_hour, pnow->tm_min,                          \
                pnow->tm_sec, (int)(nowTime.tv_usec/1000) );          \
         std::cerr line;                                              \
         std::fflush( stderr );                                       \
     }

#    define LOG_PRINTF(...)                                           \
     {                                                                \
         struct timeval nowTime;                                      \
         gettimeofday(&nowTime, NULL);                                \
         struct tm *pnow = localtime(&nowTime.tv_sec);                \
         std::fprintf( stdout,                                        \
                "[%02d-%02d-%02d %02d:%02d:%02d.%03d] ",              \
                pnow->tm_year-100, pnow->tm_mon+1, pnow->tm_mday,     \
                pnow->tm_hour, pnow->tm_min,                          \
                pnow->tm_sec, (int)(nowTime.tv_usec/1000) );          \
         std::printf( __VA_ARGS__);                                   \
         std::fflush( stdout );                                       \
     }

#    define LOG_PERR(line)                                            \
     {                                                                \
         struct timeval nowTime;                                      \
         gettimeofday(&nowTime, NULL);                                \
         struct tm *pnow = localtime(&nowTime.tv_sec);                \
         std::fprintf( stderr,                                        \
                "[%02d-%02d-%02d %02d:%02d:%02d.%03d] ",              \
                pnow->tm_year-100, pnow->tm_mon+1, pnow->tm_mday,     \
                pnow->tm_hour, pnow->tm_min,                          \
                pnow->tm_sec, (int)(nowTime.tv_usec/1000) );          \
         std::perror( line );                                         \
         std::fflush( stderr );                                       \
     }

#    define LOG_ERRPRINTF(...)                                        \
     {                                                                \
         struct timeval nowTime;                                      \
         gettimeofday(&nowTime, NULL);                                \
         struct tm *pnow = localtime(&nowTime.tv_sec);                \
         std::fprintf( stderr,                                        \
                "[%02d-%02d-%02d %02d:%02d:%02d.%03d] ",              \
                pnow->tm_year-100, pnow->tm_mon+1, pnow->tm_mday,     \
                pnow->tm_hour, pnow->tm_min,                          \
                pnow->tm_sec, (int)(nowTime.tv_usec/1000) );          \
         std::fprintf(stderr, __VA_ARGS__);                           \
         std::fflush( stderr );                                       \
     }
#else
#    define LOG_ERROUT(line) ( (void)0 )
#    define LOG_PERR(line) ( (void)0 )
#    define LOG_ERRPRINTF(...) ( (void)0 )
#endif

#ifdef TERMINAL_DBGLOG
#    define LOG_DBGOUT(line)                                          \
     {                                                                \
         struct timeval nowTime;                                      \
         gettimeofday(&nowTime, NULL);                                \
         struct tm *pnow = localtime(&nowTime.tv_sec);                \
         std::fprintf( stdout,                                        \
                "[%02d-%02d-%02d %02d:%02d:%02d.%03d] ",              \
                pnow->tm_year-100, pnow->tm_mon+1, pnow->tm_mday,     \
                pnow->tm_hour, pnow->tm_min,                          \
                pnow->tm_sec, (int)(nowTime.tv_usec/1000) );          \
         std::cout line;                                              \
         std::fflush( stdout );                                       \
     }

#    define LOG_DBGPRINTF(...)                                        \
     {                                                                \
         struct timeval nowTime;                                      \
         gettimeofday(&nowTime, NULL);                                \
         struct tm *pnow = localtime(&nowTime.tv_sec);                \
         std::fprintf( stdout,                                        \
                "[%02d-%02d-%02d %02d:%02d:%02d.%03d] ",              \
                pnow->tm_year-100, pnow->tm_mon+1, pnow->tm_mday,     \
                pnow->tm_hour, pnow->tm_min,                          \
                pnow->tm_sec, (int)(nowTime.tv_usec/1000) );          \
         std::printf( __VA_ARGS__);                                   \
         std::fflush( stdout );                                       \
     }

#    define LOG_DBGDUMP(b,l)                                          \
     {                                                                \
         struct timeval nowTime;                                      \
         gettimeofday(&nowTime, NULL);                                \
         struct tm *pnow = localtime(&nowTime.tv_sec);                \
         for( size_t iiii = 0; iiii < l; )                            \
         {                                                            \
             std::fprintf( stdout,                                    \
                    "[%02d-%02d-%02d %02d:%02d:%02d.%03d] ",          \
                    pnow->tm_year-100, pnow->tm_mon+1, pnow->tm_mday, \
                    pnow->tm_hour, pnow->tm_min,                      \
                    pnow->tm_sec, (int)(nowTime.tv_usec/1000) );      \
             for( size_t jjjj = 0; (iiii<l) && (jjjj<2); jjjj++ )     \
             {                                                        \
                 for( size_t kkkk = 0; (iiii<l) && (kkkk<8); kkkk++ ) \
                 {                                                    \
                     std::printf( "%02x ",  b[iiii] ); iiii++;        \
                 }                                                    \
                 std::printf( " " );                                  \
             }                                                        \
             std::printf( "\n" );                                     \
             std::fflush( stdout );                                   \
         }                                                            \
     }
#else
#    define LOG_DBGOUT(line) ( (void)0 )
#    define LOG_DBGPRINTF(...) ( (void)0 )
#    define LOG_DBGDUMP(line) ( (void)0 )
#endif


#endif // TROBOT_UTIL_HPP
