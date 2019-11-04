// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotUtil_EX.cpp
 *  @brief ユーティリティ(旧ロボットベース)の実装
 *
 */

#include <cmath>
#include <algorithm>
#include <string.h>

#include "TRobotSerialIOIface.hpp"
#include "TRobotUtil.hpp"



////////////////////////////////////////////////////////////////////////////////
/*!
 *  受信電文を解析処理する(旧ロボットベース)
 *
 *  @param  buf        受信電文
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::AnalysisMessage_EX( const unsigned char * buf )
{
    int event = 0;

    // 速度制御値応答のみ
    event = 1;

    // 状態、イベントで分岐
    GoEventFunc_EX( event, buf );

    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  受信電文の先頭を検索する(旧ロボットベース)
 *
 *  @param  bufFirst   [in]        検索開始ポインタ
 *  @param  bufLast    [in]        検索終了ポインタ
 *
 *  @return 指定されたバッファ中のヘッダの先頭へのポインタ。
 *          ヘッダが見つからなければバッファの末尾(有効部分の終端+1)へのポインタ
 */
const unsigned char *TRobotIF::searchForValidHeader_EX( const unsigned char *bufFirst,
                                                        const unsigned char *bufLast )
{
    using namespace std;

    //
    // 電文ヘッダ構造
    //
    // 上位コントローラから受信有り
    static const unsigned char HTS[] = {
        TR_START, TR_PREV_DATA_RECV_SUCCESS, TR_DATA_CHUNK_LEN };
    static const unsigned char *HTS_LAST = HTS + sizeof(HTS);

    // 上位コントローラから受信無
    static const unsigned char HTF[] = {
        TR_START, TR_PREV_DATA_RECV_FAILURE, TR_DATA_CHUNK_LEN };
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
 *  受信電文をデコードする(旧ロボットベース)
 *
 *  @param  buf          [in]    受信電文
 *  @param  sensorData   [out]   センサーデータ
 *
 *  @return TRobotIF::RetCode
 *
 */
TRobotIF::RetCode TRobotIF::decodeBaseSensorData_EX( const unsigned char * const buf,
                                                     BaseSensorData_EX *sensorData )
{
    ////
    //
    // エンコーダ、サイクルカウント値GET
    //
    ////
    const unsigned char R_ENC_MSB_MASK     = 0x0f;
    const unsigned char L_ENC_MSB_MASK     = 0xf0;
    const unsigned char R_ENC_MSB_SIGN_BIT = 0x08;
    const unsigned char L_ENC_MSB_SIGN_BIT = 0x80;

    const size_t R_ENC_MSB_SHIFT = 8;  
    const size_t L_ENC_MSB_SHIFT = 4;

    const int U12MAX = 0x0fff;                         // 12 bit 符号なし整数の最大値

    // モータ制御およびエンコーダデータのサンプルの間隔 [bytes]
    const size_t CTL_ENC_SMP_STRIDE = 7;

    // データ数分ループ
    size_t smpIdx = 0;
    unsigned char msb;
    int sgnBits;
    for( size_t i = 0; i < BaseSensorData_EX::NUM_ENC_COUNTS; ++ i )
    {
        // 符号ビットによって必要に応じて補数表現上妥当なように変換する
        // (符号付き 12bit 整数 -> 符号付き 32bit 整数)
        msb = buf[ smpIdx + TR_RL_ENC_MSB_OFFSET ];

        // 右車輪エンコーダ
        sgnBits = (msb & R_ENC_MSB_SIGN_BIT)? ~U12MAX : 0x00;
        sensorData->m_rightEnc[ i ] = sgnBits |
            ((msb & R_ENC_MSB_MASK) << R_ENC_MSB_SHIFT) | buf[ smpIdx + TR_R_ENC_LSB_OFFSET ];

        // 左車輪エンコーダ
        sgnBits = (msb & L_ENC_MSB_SIGN_BIT)? ~U12MAX : 0x00;
        sensorData->m_leftEnc[ i ] = sgnBits |
            ((msb & L_ENC_MSB_MASK) << L_ENC_MSB_SHIFT) | buf[ smpIdx + TR_L_ENC_LSB_OFFSET ];

        // サイクルカウント
        sensorData->m_cycleCount[ i ] = buf[ smpIdx + TR_CYCLE_CNT_OFFSET ];

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
    const size_t ACC_OFFSET = TR_R_CTL_LSB_OFFSET + BaseSensorData_EX::NUM_ENC_COUNTS * CTL_ENC_SMP_STRIDE;
    const size_t GYR_OFFSET = ACC_OFFSET + BaseSensorData_EX::NUM_XYZ * ATT_SEN_SMP_STRIDE;
    const size_t MAG_OFFSET = GYR_OFFSET + BaseSensorData_EX::NUM_XYZ * ATT_SEN_SMP_STRIDE;

    // X,Y,Z分ループ
    for( size_t ax = 0; ax < BaseSensorData_EX::NUM_XYZ; ++ ax )
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
    const size_t BAT_OFFSET = MAG_OFFSET + BaseSensorData_EX::NUM_XYZ * ATT_SEN_SMP_STRIDE;

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
    // 赤外線センサーGET
    // 取り付けるセンサーにより値域が違うのでそのまま格納
    //
    ////
    // 赤外線センサー0
    const size_t INFRARED0_OFFSET = DAI_OFFSET + sizeof(unsigned char);
    sensorData->m_InfraredSensor0 = (unsigned char)buf[ INFRARED0_OFFSET ];

    // 赤外線センサー1
    const size_t INFRARED1_OFFSET = INFRARED0_OFFSET + sizeof(unsigned char);
    sensorData->m_InfraredSensor1 = (unsigned char)buf[ INFRARED1_OFFSET ];

    // 赤外線センサー0
    const size_t INFRARED2_OFFSET = INFRARED1_OFFSET + sizeof(unsigned char);
    sensorData->m_InfraredSensor2 = (unsigned char)buf[ INFRARED2_OFFSET ];

    // 赤外線センサー3
    const size_t INFRARED3_OFFSET = INFRARED2_OFFSET + sizeof(unsigned char);
    sensorData->m_InfraredSensor3 = (unsigned char)buf[ INFRARED3_OFFSET ];


    ////
    //
    // エラーステータスGET
    //
    ////
    // 内蔵ADC エラーステータス
    const size_t ADCERR_OFFSET = INFRARED3_OFFSET + sizeof(unsigned char);
    sensorData->m_ADCErrSTS = (unsigned char)buf[ ADCERR_OFFSET ];

    // 通信エラーステータス
    const size_t COMERR_OFFSET = ADCERR_OFFSET + sizeof(unsigned char);
    sensorData->m_COMErrSTS = (unsigned char)buf[ COMERR_OFFSET ];

    // I2C 通信 エラーステータス0
    const size_t I2CERR0_OFFSET = COMERR_OFFSET + sizeof(unsigned char);
    sensorData->m_I2CErrSTS0 = (unsigned char)buf[ I2CERR0_OFFSET ];

    // I2C 通信 エラーステータス1
    const size_t I2CERR1_OFFSET = I2CERR0_OFFSET + sizeof(unsigned char);
    sensorData->m_I2CErrSTS1 = (unsigned char)buf[ I2CERR1_OFFSET ];


    return TRobotIF::SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
/*!
 *  速度指令指令シーケンスを組み立てる(旧ロボットベース)
 *
 *  @param  leftRotVel   [in]  左車輪の回転速度指令値
 *                           （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  rightRotVel  [in]  右車輪の回転速度指令値
 *                           （値域: [TR_ROT_VEL_CMD_MIN, TR_ROT_VEL_CMD_MAX]）
 *  @param  cmdbuf       [out] 送信バッファ
 *
 *  @return TRobotIF::RetCode
 */
TRobotIF::RetCode TRobotIF::buildCmdSeqSetRotVel( int leftRotVel, int rightRotVel,
                                                  unsigned char *cmdbuf )
{
    static int count = 0;
    const int COUNT_MAX = 256;

    if( (leftRotVel  < TR_ROT_VEL_CMD_MIN || TR_ROT_VEL_CMD_MAX < leftRotVel)  ||
        (rightRotVel < TR_ROT_VEL_CMD_MIN || TR_ROT_VEL_CMD_MAX < rightRotVel) )
    {
        return TRobotIF::ERROR_OUT_OF_RANGE;
    }

    cmdbuf[ 0 ] = TR_START;
    cmdbuf[ 1 ] = TR_ROT_VEL_CMD_ID;
    cmdbuf[ 2 ] = TR_ROT_VEL_CMD_LEN;
    cmdbuf[ 3 ] = TR_RESERVED;
    cmdbuf[ 4 ] = (unsigned char)(rightRotVel & 0xff);
    cmdbuf[ 5 ] = (unsigned char)(leftRotVel  & 0xff);
    cmdbuf[ 6 ] = (unsigned char)(count);
    cmdbuf[ 7 ] = TR_END;

    count = (count + 1) % COUNT_MAX;

    return TRobotIF::SUCCESS;
}
