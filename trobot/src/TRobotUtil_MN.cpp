// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotUtil.cpp
 *  @brief ユーティリティ(CIRCINUS（メカナム）)の実装
 *
 */

#include <cmath>
#include <algorithm>
#include <string.h>

#include "TRobotSerialIOIface.hpp"
#include "TRobotUtil.hpp"



////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文を解析処理する(CIRCINUS（メカナム）)
 *
 *  @param  buf        受信電文
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::AnalysisMessage_MN( const unsigned char * buf )
{
    int event = 0;

    // 速度制御値応答のみ
    event = 1;

    // 状態、イベントで分岐
    GoEventFunc_MN( event, buf );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  受信電文の先頭を検索する(CIRCINUS（メカナム）)
 *
 *  @param  bufFirst   [in]        検索開始ポインタ
 *  @param  bufLast    [in]        検索終了ポインタ
 *
 *  @return 指定されたバッファ中のヘッダの先頭へのポインタ。
 *          ヘッダが見つからなければバッファの末尾(有効部分の終端+1)へのポインタ
 */
const unsigned char *TRobotIF::searchForValidHeader_MN( const unsigned char *bufFirst,
                                                        const unsigned char *bufLast )
{
    using namespace std;

    //
    // 電文ヘッダ構造
    //
    // 上位コントローラから受信有り
    static const unsigned char HTS[] = {
        TR_START_MN, TR_PREV_DATA_RECV_SUCCESS_MN, TR_DATA_CHUNK_LEN_MN };
    static const unsigned char *HTS_LAST = HTS + sizeof(HTS);

    // 上位コントローラから受信無
    static const unsigned char HTF[] = {
        TR_START_MN, TR_PREV_DATA_RECV_FAILURE_MN, TR_DATA_CHUNK_LEN_MN };
    static const unsigned char *HTF_LAST = HTF + sizeof(HTF);


    const unsigned char *headPos = NULL;


    // 上位コントローラから受信有り構造を探す
    headPos = search( bufFirst, bufLast, HTS, HTS_LAST );
    if (headPos != bufLast) {
        return headPos;
    }

    // 上位コントローラから受信無構造を探す
    headPos = search( bufFirst, bufLast, HTF, HTF_LAST );
    if (headPos != bufLast) {
        return headPos;
    }

    return (unsigned char *)NULL;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文をデコードする(CIRCINUS（メカナム）)
 *
 *  @param  buf          [in]    受信電文
 *  @param  sensorData   [out]   センサーデータ
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::decodeBaseSensorData_MN( const unsigned char * const buf,
                                                     BaseSensorData_MN *sensorData )
{
    ////
    //
    // エンコーダ、サイクルカウント値GET
    //
    ////
    const unsigned char R_ENC_MSB_MASK     = 0xf0;
    const unsigned char L_ENC_MSB_MASK     = 0x0f;
    const unsigned char R_ENC_MSB_SIGN_BIT = 0x80;
    const unsigned char L_ENC_MSB_SIGN_BIT = 0x08;

    const size_t R_ENC_MSB_SHIFT = 4;
    const size_t L_ENC_MSB_SHIFT = 8;

    const int U12MAX = 0x0fff;                         // 12 bit 符号なし整数の最大値

    // モータ制御およびエンコーダデータのサンプルの間隔 [bytes]
    const size_t CTL_ENC_SMP_STRIDE = 7;

    // データ数分ループ
    const size_t TR_RL_ENC_MSB_OFFSET_from_CTL = TR_RL_ENC_MSB_OFFSET_MN - TR_R_CTL_LSB_OFFSET_MN;
    const size_t TR_L_ENC_LSB_OFFSET_from_CTL = TR_L_ENC_LSB_OFFSET_MN - TR_R_CTL_LSB_OFFSET_MN;
    const size_t TR_R_ENC_LSB_OFFSET_from_CTL = TR_R_ENC_LSB_OFFSET_MN - TR_R_CTL_LSB_OFFSET_MN;
    const size_t TR_CYCLE_CNT_OFFSET_from_CTL = TR_CYCLE_CNT_OFFSET_MN - TR_R_CTL_LSB_OFFSET_MN;
    size_t smpIdx = TR_R_CTL_LSB_OFFSET_MN;
    unsigned char msb;
    int sgnBits;
    for( size_t i = 0; i < BaseSensorData_MN::NUM_ENC_COUNTS; i++ )
    {
        //
        // 前輪
        // 符号ビットによって必要に応じて補数表現上妥当なように変換する
        // (符号付き 12bit 整数 -> 符号付き 32bit 整数)
        //
        msb = buf[ smpIdx + TR_RL_ENC_MSB_OFFSET_from_CTL ];

        // 左車輪エンコーダ
        sgnBits = (msb & L_ENC_MSB_SIGN_BIT)? ~U12MAX : 0x00;
        sensorData->m_leftEnc[ i ] = sgnBits |
            ((msb & L_ENC_MSB_MASK) << L_ENC_MSB_SHIFT) | buf[ smpIdx + TR_L_ENC_LSB_OFFSET_from_CTL ];

        // 右車輪エンコーダ
        sgnBits = (msb & R_ENC_MSB_SIGN_BIT)? ~U12MAX : 0x00;
        sensorData->m_rightEnc[ i ] = sgnBits |
            ((msb & R_ENC_MSB_MASK) << R_ENC_MSB_SHIFT) | buf[ smpIdx + TR_R_ENC_LSB_OFFSET_from_CTL ];

        // サイクルカウント
        sensorData->m_cycleCount[ i ] = buf[ smpIdx + TR_CYCLE_CNT_OFFSET_from_CTL ];

        smpIdx += CTL_ENC_SMP_STRIDE;

        //
        // 後輪
        //
        msb = buf[ smpIdx + TR_RL_ENC_MSB_OFFSET_from_CTL ];

        // 左車輪エンコーダ
        sgnBits = (msb & L_ENC_MSB_SIGN_BIT)? ~U12MAX : 0x00;
        sensorData->m_leftEnc_r[ i ] = sgnBits |
            ((msb & L_ENC_MSB_MASK) << L_ENC_MSB_SHIFT) | buf[ smpIdx + TR_L_ENC_LSB_OFFSET_from_CTL ];

        // 右車輪エンコーダ
        sgnBits = (msb & R_ENC_MSB_SIGN_BIT)? ~U12MAX : 0x00;
        sensorData->m_rightEnc_r[ i ] = sgnBits |
            ((msb & R_ENC_MSB_MASK) << R_ENC_MSB_SHIFT) | buf[ smpIdx + TR_R_ENC_LSB_OFFSET_from_CTL ];

        // サイクルカウント
        sensorData->m_cycleCount[ i ] = buf[ smpIdx + TR_CYCLE_CNT_OFFSET_from_CTL ];

        smpIdx += CTL_ENC_SMP_STRIDE;
    }


    ////
    //
    // 9軸センサモジュールデータGET
    // 前提: 姿勢センサのデータは little endian
    //
    ////

    // 各軸データの間隔 [bytes]
    const size_t ATT_SEN_SMP_STRIDE = 2;

    // 9軸センサモジュールオフセット
    const size_t ACC_OFFSET = TR_R_CTL_LSB_OFFSET_MN + ( BaseSensorData_MN::NUM_ENC_COUNTS * CTL_ENC_SMP_STRIDE * 2 );
    const size_t GYR_OFFSET = ACC_OFFSET + BaseSensorData_EX::NUM_XYZ * ATT_SEN_SMP_STRIDE;
    const size_t MAG_OFFSET = GYR_OFFSET + BaseSensorData_EX::NUM_XYZ * ATT_SEN_SMP_STRIDE;

    // X,Y,Z分ループ
    for( size_t ax = 0; ax < BaseSensorData_MN::NUM_XYZ; ax++ )
    {
        size_t accIdx = ACC_OFFSET + ax * ATT_SEN_SMP_STRIDE;
        size_t gyrIdx = GYR_OFFSET + ax * ATT_SEN_SMP_STRIDE;
        size_t magIdx = MAG_OFFSET + ax * ATT_SEN_SMP_STRIDE;

        sensorData->m_acc[ ax ] = (int)(buf[ accIdx + 1 ] << 8) | buf[ accIdx ];
        sensorData->m_gyr[ ax ] = (int)(buf[ gyrIdx + 1 ] << 8) | buf[ gyrIdx ];
        sensorData->m_mag[ ax ] = (int)(buf[ magIdx + 1 ] << 8) | buf[ magIdx ];
    }


    ////
    //
    // バッテリ電圧GET
    // 分圧抵抗 82:12
    // V = v * 5/255 * ((82+12)/12) = v * 5/255 * (47/6)
    //
    ////

    // バッテリ電圧オフセット
    const size_t BAT_OFFSET = MAG_OFFSET + BaseSensorData_MN::NUM_XYZ * ATT_SEN_SMP_STRIDE;

    // バッテリ電圧GET
    unsigned int voltage = (unsigned int)buf[ BAT_OFFSET ];
    sensorData->m_BatteryVoltage = voltage * 5.0 / 255.0 * 47.0 / 6.0;


    ////
    //
    // ダイ温度GET
    // レジスタ値から40 を引いた値が温度
    //
    ////

    // ダイ温度オフセット
    const size_t DAI_OFFSET = BAT_OFFSET + sizeof(unsigned char);

    // ダイ温度GET
    unsigned int temp = (unsigned int)buf[ DAI_OFFSET ];
    sensorData->m_daiTemp = temp - 40.0;

    ////
    //
    // 超音波センサ値GET
    // 取り付けるセンサーにより値域が違うのでそのまま格納
    //
    ////
    // 超音波センサ値
    const size_t DISTANCE_OFFSET = DAI_OFFSET + sizeof(unsigned char);
    for( size_t i = 0; i < BaseSensorData_MN::NUM_DISTANCE; i++ )
    {
        sensorData->m_Distance[ i ] =  buf[ DISTANCE_OFFSET + i ];
    }

    // バンパセンサ値
    const size_t BUMPER_OFFSET = DISTANCE_OFFSET + (sizeof(unsigned char)*BaseSensorData_MN::NUM_DISTANCE);
    sensorData->m_BumperSensor = (unsigned char)buf[ BUMPER_OFFSET ];


    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用指令シーケンスを組み立てる(CIRCINUS（メカナム）)
 *
 *  @param  cmd           [in]  コマンド(TRobotIF::CommandNum)
 *  @param  leftRotVel    [in]  前左車輪の回転速度指令値
 *                            （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  rightRotVel   [in]  前右車輪の回転速度指令値
 *                            （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  leftRotVel_r  [in]  後左車輪の回転速度指令値
 *                            （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  rightRotVel_r [in]  後右車輪の回転速度指令値
 *                            （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  cmdbuf        [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSet_MN( int cmd,
                                               int leftRotVel,   int rightRotVel,
                                               int leftRotVel_r, int rightRotVel_r,
                                               unsigned char *cmdbuf )
{
    TRobotIF::RetCode retCode;

    switch(cmd)
    {
    case Command_SPEED_CONTROL_MN:      // 速度制御値を指定する。
        //LOG_PRINTF( "send Command_SPEED_CONTROL_MN\n" );
        retCode = buildCmdSeqSetRotVel_MN( leftRotVel, rightRotVel, leftRotVel_r, rightRotVel_r, cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    case Command_SET_CONFIG_MN:           // 設定情報を書き換える。
        LOG_PRINTF( "send Command_SET_CONFIG_MN\n" );
        retCode = buildCmdSeqSetSetConfig_MN( cmdbuf );
        if( retCode != TRobotIF::SUCCESS )
        {
            return retCode;
        }
        break;


    default:
        break;
    }

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  速度指令指令シーケンスを組み立てる(CIRCINUS（メカナム）)
 *
 *  @param  leftRotVel     [in]  左車輪の回転速度指令値
 *                             （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  rightRotVel    [in]  右車輪の回転速度指令値
 *                             （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  leftRotVel_r   [in]  左車輪の回転速度指令値
 *                             （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  rightRotVel_r  [in]  右車輪の回転速度指令値
 *                             （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  cmdbuf         [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetRotVel_MN(
                                                  int leftRotVel,   int rightRotVel,
                                                  int leftRotVel_r, int rightRotVel_r,
                                                  unsigned char *cmdbuf )
{
    static int count = 0;
    const int COUNT_MAX = 256;


    if( (leftRotVel  < TR_ROT_VEL_CMD_MIN_MN || TR_ROT_VEL_CMD_MAX_MN < leftRotVel)  ||
        (rightRotVel < TR_ROT_VEL_CMD_MIN_MN || TR_ROT_VEL_CMD_MAX_MN < rightRotVel) )
    {
        return TRobotIF::ERROR_OUT_OF_RANGE;
    }
    if( (leftRotVel_r  < TR_ROT_VEL_CMD_MIN_MN || TR_ROT_VEL_CMD_MAX_MN < leftRotVel_r)  ||
        (rightRotVel_r < TR_ROT_VEL_CMD_MIN_MN || TR_ROT_VEL_CMD_MAX_MN < rightRotVel_r) )
    {
        return TRobotIF::ERROR_OUT_OF_RANGE;
    }

    GetserIO()->setSndLength( TR_ROT_VEL_CMDSPEED_LEN_MN );

    cmdbuf[  0 ] = TR_START_MN;
    cmdbuf[  1 ] = Command_SPEED_CONTROL_MN;
    cmdbuf[  2 ] = TR_ROT_VEL_CMDSPEED_LEN_MN;
    cmdbuf[  3 ] = TR_RESERVED;
    cmdbuf[  4 ] = (unsigned char)((leftRotVel    & 0x0000ff00)>>8);
    cmdbuf[  5 ] = (unsigned char)((leftRotVel    & 0x000000ff));
    cmdbuf[  6 ] = (unsigned char)((rightRotVel   & 0x0000ff00)>>8);
    cmdbuf[  7 ] = (unsigned char)((rightRotVel   & 0x000000ff));
    cmdbuf[  8 ] = (unsigned char)((leftRotVel_r  & 0x0000ff00)>>8);
    cmdbuf[  9 ] = (unsigned char)((leftRotVel_r  & 0x000000ff));
    cmdbuf[ 10 ] = (unsigned char)((rightRotVel_r & 0x0000ff00)>>8);
    cmdbuf[ 11 ] = (unsigned char)((rightRotVel_r & 0x000000ff));
    cmdbuf[ 12 ] = (unsigned char)(count);
    cmdbuf[ 13 ] = TR_END;

    count = (count + 1) % COUNT_MAX;

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  ファームウェア用設定情報を書き換えるコマンドを組み立てる(CIRCINUS（メカナム）)
 *
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetSetConfig_MN( unsigned char *cmdbuf )
{
    GetserIO()->setSndLength( TR_ROT_VEL_CMDCONFIG_LEN_MN );

    memset( cmdbuf, 0x00, GetserIO()->getSndLength() );

    cmdbuf[ 0 ] = TR_START_MN_CMDCONFIG;
    cmdbuf[ 1 ] = 0x00;
    cmdbuf[ 2 ] = 0x00;
    cmdbuf[ 3 ] = Command_SET_CONFIG_MN;   // 設定情報を書き換える。

    memcpy( &cmdbuf[ 4 ], &GetserIO()->CmdConfig_MN[ 4 ], GetserIO()->getSndLength() - 5);

    // サム値
    cmdbuf[ GetserIO()->getSndLength() - 1 ] = makeSum( &cmdbuf[0], &cmdbuf[ GetserIO()->getSndLength() - 2 ] );

    return TRobotIF::SUCCESS;
}
