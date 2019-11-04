// -*- C++ -*-
// -*- coding: utf-8 -*- 
/*!
 *  @file TRobotUtil.cpp
 *  @brief TRobotUtil ユーティリティの実装
 *
 */

#include <cmath>
#include <algorithm>
#include <string.h>

#include "TRobotSerialIOIface.hpp"
#include "TRobotUtil.hpp"


////////////////////////////////////////////////////////////////////////////////
/**
 *  SUM値作成
 *
 *  @param  bufFirst   [in]        指定開始ポインタ
 *  @param  bufLast    [in]        指定終了ポインタ
 *
 *  @return 指定された範囲のSUM値
 */
unsigned char TRobotIF::makeSum( const unsigned char *bufFirst,
                                 const unsigned char *bufLast )
{
    unsigned long suml = 0;
    unsigned char sumc = 0;

    // 指定されたバッファ中の先頭から末尾までループ
    for( unsigned char *buf=(unsigned char *)bufFirst; buf<=(unsigned char *)bufLast; buf++ )
    {
        suml += (unsigned long)buf[ 0 ];
    }
    sumc = (unsigned char)( suml & 0x000000ff );

    return sumc;
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  SUM値をチェックする
 *
 *  @param  bufFirst   [in]        指定開始ポインタ
 *  @param  bufLast    [in]        指定終了ポインタ
 *
 *  @return 指定された範囲のSUM値
 */
bool TRobotIF::checkSum( const unsigned char sum,
                         const unsigned char *bufFirst,
                         const unsigned char *bufLast )
{
    unsigned char sumc = makeSum( bufFirst, bufLast );

    return (sumc==sumc);
}
