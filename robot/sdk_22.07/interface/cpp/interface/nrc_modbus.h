/*
 * nrc_modbus.h
 *
 *  Created on: 2025年3月19日
 *      Author: yiixiong
 */

#ifndef INCLUDE_CPP_INTERFACE_NRC_MODBUS_H_
#define INCLUDE_CPP_INTERFACE_NRC_MODBUS_H_

#include "cpp/parameter/nrc_define.h"
#include "cpp/parameter/nrc_modbus_parameter.h"

//==========================
//==========主站=============
//==========================
/**
 * @brief 设置主站参数
 * @param processNumber 配方id参数,最多保存9个id
 * @param TCPMasterParameter 参数
 */
EXPORT_API Result modbus_set_master_parameter(SOCKETFD socketFd, int id, const ModbusMasterParameter& param);

/**
 * @brief 打开主站
 * @param TCPMasterParameter 参数
 */
EXPORT_API Result modbus_open_master(SOCKETFD socketFd, int id);

/**
 * @brief 获取主站连接状态
 * @param TCPMasterParameter 参数
 */
EXPORT_API Result modbus_get_master_connection_status(SOCKETFD socketFd, int id, int& status);

/**
 * 功能码 01H
 */
EXPORT_API Result modbus_read_coil_status(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);
/**
 * 功能码 02H
 */
EXPORT_API Result modbus_read_input_status(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);
/**
 * 功能码 03H
 */
EXPORT_API Result modbus_read_holding_registers(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);
/**
 * 功能码 04H
 */
EXPORT_API Result modbus_read_input_registers(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);
/**
 * 功能码 05H
 */
EXPORT_API Result modbus_write_signal_coil_status(SOCKETFD socketFd, int id, int address, int data);
/**
 * 功能码 06H
 */
EXPORT_API Result modbus_write_signal_holding_registers(SOCKETFD socketFd, int id, int address, int data);
/**
 * 功能码 0FH
 */
EXPORT_API Result modbus_write_multiple_coil_status(SOCKETFD socketFd, int id, int address, const std::vector<int>& data);
/**
 * 功能码 10H
 */
EXPORT_API Result modbus_write_multiple_holding_registers(SOCKETFD socketFd, int id, int address, const std::vector<int>& data);

//==========================
//==========从站=============
//==========================
// 待开放

#endif /* INTERFACE_CPP_INTERFACE_NRC_MODBUS_H_ */
