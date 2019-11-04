// -*- C++ -*-
// -*- coding: utf-8 -*-
/*!
 * @file  TRobotConstants.hpp
 * @brief TRobotConstantsヘッダ
 *
 */

#ifndef TROBOT_CONSTANTS_HPP
#define TROBOT_CONSTANTS_HPP

#include "TRobotConstants_COM.hpp"
#include "TRobotConstants_EX.hpp"
#include "TRobotConstants_SX.hpp"
#include "TRobotConstants_LM.hpp"
#include "TRobotConstants_MN.hpp"


// 送信バッファ長
#if ( TR_CMD_LENGTH_SX > TR_CMD_LENGTH_LM )
#    if ( TR_CMD_LENGTH_SX > TR_ROT_VEL_CMD_LEN )
#        if ( TR_CMD_LENGTH_SX > TR_ROT_VEL_CMD_LEN_MN )
#            define TR_SNDBUF_LENGTH    TR_CMD_LENGTH_SX
#        else
#            define TR_SNDBUF_LENGTH    TR_ROT_VEL_CMD_LEN_MN
#        endif
#    else
#        if ( TR_ROT_VEL_CMD_LEN > TR_ROT_VEL_CMD_LEN_MN )
#            define TR_SNDBUF_LENGTH    TR_ROT_VEL_CMD_LEN
#        else
#            define TR_SNDBUF_LENGTH    TR_ROT_VEL_CMD_LEN_MN
#        endif
#    endif
#else
#    if ( TR_CMD_LENGTH_LM > TR_ROT_VEL_CMD_LEN )
#        if ( TR_CMD_LENGTH_LM > TR_ROT_VEL_CMD_LEN_MN )
#            define TR_SNDBUF_LENGTH    TR_CMD_LENGTH_LM
#        else
#            define TR_SNDBUF_LENGTH    TR_ROT_VEL_CMD_LEN_MN
#        endif
#    else
#        if ( TR_ROT_VEL_CMD_LEN > TR_ROT_VEL_CMD_LEN_MN )
#            define TR_SNDBUF_LENGTH    TR_ROT_VEL_CMD_LEN
#        else
#            define TR_SNDBUF_LENGTH    TR_ROT_VEL_CMD_LEN_MN
#        endif
#    endif
#endif


// 受信バッファ長
#if ( TR_RCV_LENGTH_SX > TR_RCV_LENGTH_LM )
#    if ( TR_RCV_LENGTH_SX > TR_BASE_SENSOR_DATA_LEN )
#        if ( TR_RCV_LENGTH_SX > TR_BASE_SENSOR_DATA_LEN_MN )
#            define TR_RCVBUF_LENGTH    TR_RCV_LENGTH_SX
#        else
#            define TR_RCVBUF_LENGTH    TR_BASE_SENSOR_DATA_LEN_MN
#        endif
#    else
#        if ( TR_BASE_SENSOR_DATA_LEN > TR_BASE_SENSOR_DATA_LEN_MN )
#            define TR_RCVBUF_LENGTH    TR_BASE_SENSOR_DATA_LEN
#        else
#            define TR_RCVBUF_LENGTH    TR_BASE_SENSOR_DATA_LEN_MN
#        endif
#    endif
#else
#    if ( TR_RCV_LENGTH_LM > TR_BASE_SENSOR_DATA_LEN )
#        if ( TR_RCV_LENGTH_LM > TR_BASE_SENSOR_DATA_LEN_MN )
#            define TR_RCVBUF_LENGTH    TR_RCV_LENGTH_LM
#        else
#            define TR_RCVBUF_LENGTH    TR_BASE_SENSOR_DATA_LEN_MN
#        endif
#    else
#        if ( TR_BASE_SENSOR_DATA_LEN > TR_BASE_SENSOR_DATA_LEN_MN )
#            define TR_RCVBUF_LENGTH    TR_BASE_SENSOR_DATA_LEN
#        else
#            define TR_RCVBUF_LENGTH    TR_BASE_SENSOR_DATA_LEN_MN
#        endif
#    endif
#endif


#endif // TROBOT_CONSTANTS_HPP

