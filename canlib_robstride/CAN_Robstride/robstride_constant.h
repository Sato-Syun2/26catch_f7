/*
 * robstride_constant.h
 *
 *  Created on: Dec 7, 2024
 *      Author: tetud
 */

#ifndef INC_ROBSTRIDE_CONSTANT_H_
#define INC_ROBSTRIDE_CONSTANT_H_

/**
 * @brief ファームウェアのバージョンに応じてアドレス定義を切り替えるためのマクロ。
 * 古いバージョンのファームウェア（日本語マニュアル準拠）を使用する場合は、このマクロをコメント解除してください。
 */
#define USE_OLD_FIRMWARE_ADDRESSES

// --- 共通の通信コマンドID (バージョン間で変更なし) ---
#define CMD_DEVICEID 0
#define CMD_OPERATION 1
#define CMD_RESPONSE 2
#define CMD_ENABLE 3
#define CMD_RESET 4
#define CMD_SET_MECH_POSITION_TO_ZERO 6
#define CMD_CHANGE_CAN_ID 7
#define CMD_RAM_READ 17
#define CMD_RAM_WRITE 18
#define CMD_GET_MOTOR_FAIL 21

// --- バージョン別の通信コマンドID ---
#ifdef USE_OLD_FIRMWARE_ADDRESSES
// 古いファームウェア
#define CMD_BAUD_RATE_MOD 22
#define CMD_ACTIVE_REPORT 23
#define CMD_DATA_SAVE 24
// 古いファームウェアには存在しないコマンド
#define CMD_PROTOCOL_MOD 0xFF // Dummy Command
#else
// 新しいファームウェア
#define CMD_DATA_SAVE 22
#define CMD_BAUD_RATE_MOD 23
#define CMD_ACTIVE_REPORT 24
#define CMD_PROTOCOL_MOD 25
#endif

// --- 共通パラメータアドレス (バージョン間で変更なし) ---
#define ADDR_RUN_MODE 0x7005
#define ADDR_IQ_REF 0x7006
#define ADDR_SPEED_REF 0x700A
#define ADDR_LIMIT_TORQUE 0x700B
#define ADDR_CURRENT_KP 0x7010
#define ADDR_CURRENT_KI 0x7011
#define ADDR_CURRENT_FILTER_GAIN 0x7014
#define ADDR_LOC_REF 0x7016
#define ADDR_LIMIT_SPEED 0x7017
#define ADDR_LIMIT_CURRENT 0x7018
#define ADDR_MECH_POS 0x7019
#define ADDR_IQF 0x701A
#define ADDR_MECH_VEL 0x701B
#define ADDR_VBUS 0x701C
#define ADDR_ROTATION 0x701D
#define ADDR_LOC_KP 0x701E
#define ADDR_SPD_KP 0x701F
#define ADDR_SPD_KI 0x7020
#define ADDR_CAN_TIMEOUT 0x7028

// --- バージョン別パラメータアドレス ---
#ifdef USE_OLD_FIRMWARE_ADDRESSES
// 古いファームウェア用のアドレス定義
#define ADDR_EPSCAN_TIME 0x7024
#define ADDR_VEL_MAX_PP 0x7025
#define ADDR_ACC_SET_PP 0x7026
#define ADDR_SPD_FILTER_GAIN 0x7027

// 存在しない変数のためのダミーアドレス (switch文のコンパイルエラー回避用)
#define ADDR_ACC_RAD 0x7FFE  // Dummy address
#define ADDR_ZERO_STA 0x7FFF // Dummy address
#define ADDR_ADD_OFFSET 0x7FFD // Dummy address
#else
// 新しいファームウェア用のアドレス定義 (デフォルト)
#define ADDR_SPD_FILTER_GAIN 0x7021
#define ADDR_ACC_RAD 0x7022
#define ADDR_VEL_MAX_PP 0x7024
#define ADDR_ACC_SET_PP 0x7025
#define ADDR_EPSCAN_TIME 0x7026
#define ADDR_ZERO_STA 0x7029
#define ADDR_ADD_OFFSET 0x702B
#endif

// --- 全パラメータ読み出し用リスト ---
#ifdef USE_OLD_FIRMWARE_ADDRESSES
// 古いファームウェアで実際に読み出すアドレスのリスト
#define ROBSTRIDE_READABLE_ADDRESS_LIST {                                           \
    ADDR_RUN_MODE, ADDR_IQ_REF, ADDR_SPEED_REF, ADDR_LIMIT_TORQUE, ADDR_CURRENT_KP, \
    ADDR_CURRENT_KI, ADDR_CURRENT_FILTER_GAIN, ADDR_LOC_REF, ADDR_LIMIT_SPEED,      \
    ADDR_LIMIT_CURRENT, ADDR_MECH_POS, ADDR_IQF, ADDR_MECH_VEL, ADDR_VBUS,          \
    ADDR_LOC_KP, ADDR_SPD_KP, ADDR_SPD_KI, ADDR_SPD_FILTER_GAIN,                    \
    ADDR_VEL_MAX_PP, ADDR_ACC_SET_PP, ADDR_EPSCAN_TIME, ADDR_CAN_TIMEOUT            \
}
#else
// 新しいファームウェアで読み出すアドレスのリスト
#define ROBSTRIDE_READABLE_ADDRESS_LIST {                                               \
    ADDR_RUN_MODE, ADDR_IQ_REF, ADDR_SPEED_REF, ADDR_LIMIT_TORQUE, ADDR_CURRENT_KP,     \
    ADDR_CURRENT_KI, ADDR_CURRENT_FILTER_GAIN, ADDR_LOC_REF, ADDR_LIMIT_SPEED,          \
    ADDR_LIMIT_CURRENT, ADDR_MECH_POS, ADDR_IQF, ADDR_MECH_VEL, ADDR_VBUS,              \
    ADDR_LOC_KP, ADDR_SPD_KP, ADDR_SPD_KI, ADDR_SPD_FILTER_GAIN, ADDR_ACC_RAD,          \
    ADDR_VEL_MAX_PP, ADDR_ACC_SET_PP, ADDR_EPSCAN_TIME, ADDR_CAN_TIMEOUT, ADDR_ZERO_STA , ADDR_ADD_OFFSET\
}
#endif

// --- モーターモードステータス定義 (CMD_ResponseでIDのbit22-23から取得) ---
#define ROBSTRIDE_STATE_DISABLE 0 // Resetモード
#define ROBSTRIDE_STATE_CALIB 1   // Caliモード (キャリブレーション中)
#define ROBSTRIDE_STATE_ENABLE 2  // Motorモード (運転中)
// Operation Control Mode用
#define P_MIN_ROBSTRIDE02 -12.56637061435917295384f
#define P_MAX_ROBSTRIDE02 12.56637061435917295384f
#define V_MIN_ROBSTRIDE02 -44.0f
#define V_MAX_ROBSTRIDE02 44.0f
#define T_MIN_ROBSTRIDE02 -17.0f
#define T_MAX_ROBSTRIDE02 17.0f
#define KP_MIN_ROBSTRIDE02 0.0f
#define KP_MAX_ROBSTRIDE02 500.0f
#define KD_MIN_ROBSTRIDE02 0.0f
#define KD_MAX_ROBSTRIDE02 5.0f

#define P_MIN_ROBSTRIDE04 -12.56637061435917295384f
#define P_MAX_ROBSTRIDE04 12.56637061435917295384f
#define V_MIN_ROBSTRIDE04 -15.0f
#define V_MAX_ROBSTRIDE04 15.0f
#define T_MIN_ROBSTRIDE04 -120.0f
#define T_MAX_ROBSTRIDE04 120.0f
#define KP_MIN_ROBSTRIDE04 0.0f
#define KP_MAX_ROBSTRIDE04 500.0f
#define KD_MIN_ROBSTRIDE04 0.0f
#define KD_MAX_ROBSTRIDE04 5.0f

#define P_MIN_ROBSTRIDE05 -12.56637061435917295384f
#define P_MAX_ROBSTRIDE05 12.56637061435917295384f
#define V_MIN_ROBSTRIDE05 -50.0f
#define V_MAX_ROBSTRIDE05 50.0f
#define T_MIN_ROBSTRIDE05 -6.0f
#define T_MAX_ROBSTRIDE05 6.0f
#define KP_MIN_ROBSTRIDE05 0.0f
#define KP_MAX_ROBSTRIDE05 500.0f
#define KD_MIN_ROBSTRIDE05 0.0f
#define KD_MAX_ROBSTRIDE05 5.0f

#endif /* INC_ROBSTRIDE_CONSTANT_H_ */
