/*
 * SDO_COB.h
 *
 *  Created on: Sep 28, 2025
 *      Author: pc
 */

#ifndef APP_CAN_SDO_COB_H_
#define APP_CAN_SDO_COB_H_

#include "def.h"

// CAN SDO地址定义
#define COB_Tr_Req_ADDR	    (0x602)
#define COB_OP_Req_ADDR	    (0x603)
#define COB_Tr_RES_ADDR	    (0x582)
#define COB_OP_RES_ADDR	    (0x583)

// DMCU索引组定义
#define DMCU_COB_INDEX          (0x2024)  // DMCU COB数据
#define DMCU_CTRL_BASIC_INDEX   (0x2023)  // 驱动控制基础数据
#define DMCU_M_PAR_INDEX        (0x2022)  // 电机参数数据
#define DMCU_CTRL_INDEX         (0x2021)  // DMCU控制器参数
#define DMCU_SPD_INDEX          (0x2020)  // DMCU速度控制参数
#define DMCU_TORQUE_INDEX       (0x201F)  // DMCU扭矩控制参数
#define DMCU_GAIN_INDEX         (0x201E)  // DMCU增益参数
#define DMCU_CTRLOPT_INDEX      (0x201D)  // DMCU控制优化参数
#define DMCU_FORK_INDEX         (0x201C)  // DMCU叉车参数
#define DMCU_COMM_INDEX         (0x201B)  // DMCU通讯参数
#define DMCU_PROT_INDEX         (0x201A)  // DMCU保护参数
#define DMCU_DISCHG_INDEX       (0x2019)  // DMCU泄放参数

// 各组参数数量定义 (包含校验和字段)
#define COB_MAX_ID2	            (29)      // 0x2024组参数数量
#define CTRL_BASIC_MAX          (9)       // 0x2023组参数数量 (8+1校验和)
#define M_PAR_MAX               (24)      // 0x2022组参数数量 (23+1校验和)
#define CTRL_MAX                (10)      // 0x2021组参数数量 (9+1校验和)
#define SPD_MAX                 (6)       // 0x2020组参数数量 (5+1校验和)
#define TORQUE_MAX              (6)       // 0x201F组参数数量 (5+1校验和)
#define GAIN_MAX                (21)      // 0x201E组参数数量 (20+1校验和)
#define CTRLOPT_MAX             (5)       // 0x201D组参数数量 (4+1校验和)
#define FORK_MAX                (2)       // 0x201C组参数数量 (1+1校验和)
#define COMM_MAX                (3)       // 0x201B组参数数量 (2+1校验和)
#define PROT_MAX                (19)      // 0x201A组参数数量 (18+1校验和)
#define DISCHG_MAX              (3)       // 0x2019组参数数量 (2+1校验和)

#define COB_MAX_ID3	            (21)

#define EEPROM_SDO_Adr          1000
#define EEPROM_DMCU_BASE        (COB_MAX_ID2+1)      // DMCU其他组的EEPROM基地址

typedef union{  
    uint32_t U32t;
    uint16_t U16t[2]; // 
    uint8_t U8t[4]; //     
}Data_COB;

typedef union{  
    uint32_t U32t;
    uint16_t U16t[2]; // 
    uint8_t U8t[4]; //     
}Data_COB_32b;


typedef union{ 
    struct
    {
    uint32_t byte0:8; // 
    uint32_t index:16; // 
    uint32_t SUBindex:8; // 
    Data_COB_32b data4; // 
    }msg;
    uint8_t U8t[8]; //     
}Data_COB_64b;

typedef struct
{	
    uint16_t    Msg_COB_ID;
    uint16_t	Index;
    uint8_t	    Subindex;
    uint8_t     Data_len;
}CanMsgSDO_DATA;

// 索引组信息结构
typedef struct
{
    uint16_t    index;          // 索引号
    const char* var_name;       // 变量名
    const char* description;    // 中文描述
}IndexGroupInfo;

 enum
 {
    data_cks = 0,
    T_M_Spd_max_e,//牵引最大转速
    Pmode_spd_percentage_e,//P模式速度百分比
    Emode_spd_percentage_e,//E模式速度百分比
    Smode_spd_percentage_e,//S模式速度百分比
    Tr_Pmode_acc_time_e,//P模式加速时间
    Tr_Pmode_dec_time_e,//P模式减速时间
    Tr_Emode_acc_time_e,//E模式加速时间
    Tr_Emode_dec_time_e,//E模式减速时间
    Tr_Smode_acc_time_e,//S模式加速时间
    Tr_Smode_dec_time_e,//S模式减速时间
    H_spd_brake_mode_dec_time_e,//高速时踩下刹车减速时间
    L_spd_brake_mode_dec_time_e,//低速时踩下刹车减速时间
    Crawl_brake_mode_dec_time_e,//蠕行时踩下刹车减速时间
    Half_Throttle_dec_time_e,//半松油门减速时间
    Reverse_dec_time_e,//反向制动减速时间
    Crawl_acc_dec_time_e,   //蠕行加减速时间
    Unseat_delay_time_e,//座椅开关延时
    Unseatbelt_delay_time_e,//安全带开关延时
    PSlope_time_MAX_e,//最大驻坡时间
    Throttle_analog_input_gain_e,//油门模拟量增益
    Throttle_analog_input_offset_e,//油门模拟量偏置
    Tyre_angle_analog_input_gain_e,//转角模拟量增益
    Tyre_angle_analog_input_offset_e,//转角模拟量偏置
    Australian_seatbelt_logic_e,//澳标安全带逻辑
    Tracti_seatbelt_check_e,//牵引安全带检测功能
    TractioPanelLoc_check_e,//牵引仪表刷卡功能
    Back_off_mod_sp_percentage_e,//倒车速度百分比
    S_curve_mode_time1_e,//S曲线时间1
    S_curve_mode_time2_e,//S曲线时间2

/*
    Pump_motor_spd_max_e,//泵最大转速
    First_stick_pump_spd_e,//第一联最大转速
    Second_stick_pump_spd_e,//第二联最大转速
    Third_stick_pump_spd_e,//第三联最大转速
    Fourth_stick_pump_spd_e,//第四联最大转速
    RPM_of_Standb_Unit_e,//备用联最大转速
    Steerin_pump_spd_e,//转向怠速
    Low_SOC_first_stick_pump_spd_e,//低电量状态最大起升转速
    Pump_P_mode_acc_time_e,//泵P模式加速时间
    Pump_P_mode_dec_time_e,//泵P模式减速时间
    Pump_E_mode_acc_time_e,//泵E模式加速时间
    Pump_E_mode_dec_time_e,//泵E模式减速时间
    Pump_S_mode_acc_time_e,//泵S模式加速时间
    Pump_S_mode_dec_time_e,//泵S模式减速时间
    Steering_pump_delay_time_e,//转向助力怠速延时
    Lift_analog_input_gain_e,//起升模拟量增益
    Lift_analog_input_offset_e,//起升模拟偏置
    Onseat_enable_steering_pump_e,//落座触发转向助力
    Steering_wheel_enable_steering_e,//方向盘触发转向助力
    Pump_seatbelt_check_e,//油泵安全带检测功能
    Pump_PanelLock_check_e,//油泵仪表刷卡功能
    */
    COB_MAX	
};



// 原有函数声明
void CanMsgSDO_SetDataID2(uint8_t* data_set);
void CanMsgSDO_SetDataID3(uint8_t* data_set);
uint16_t SDO_SetData_GET(uint8_t data_index);
void SDO_SetData_EEP_def(void);
void SDO_GetData_EEP_Read(void);

// DMCU扩展功能函数声明
void CanMsgSDO_DMCU_Handler(uint16_t index, uint8_t* data_set);
uint32_t SDO_DMCU_GetData(uint16_t index, uint8_t subindex);
void SDO_DMCU_SetData(uint16_t index, uint8_t subindex, uint32_t value);
void SDO_DMCU_EEP_Init(void);
void SDO_DMCU_EEP_Read(uint16_t index);
void SDO_DMCU_EEP_Write(uint16_t index, uint8_t subindex, uint32_t value);
void SDO_DMCU_EEP_WriteGroup(uint16_t index);  // 整组写入EEPROM

// ============================================================================
// DMCU参数数组索引宏定义 (用于COB_data数组访问)
// ============================================================================

// 0x2023 - 驱动控制基础数据数组索引
#define DCTRL_BASIC_DATA_CKS                    0   // 校验和
#define DCTRL_BASIC_CMD_SRC                     1   // 指令源
#define DCTRL_BASIC_CMD_T_ON                    2   // 指令使能
#define DCTRL_BASIC_M_ROTATION_DIRECTION        3   // 电机旋转方向
#define DCTRL_BASIC_M_CTR_MODE                  4   // 电机控制模式
#define DCTRL_BASIC_SPD_CMD                     5   // 速度指令
#define DCTRL_BASIC_TORQUE_CMD                  6   // 扭矩指令
#define DCTRL_BASIC_D_AXIS_CURRENT              7   // D轴电流
#define DCTRL_BASIC_Q_AXIS_CURRENT              8   // Q轴电流

// 0x2022 - 电机参数数据数组索引
#define M_PAR_DATA_CKS                          0   // 校验和
#define M_PAR_SN                                1   // 电机序列号
#define M_PAR_TYPE                              2   // 电机类型
#define M_PAR_NEGATIVE_SEQUENCE_CTRL            3   // 负序控制
#define M_PAR_RATED_POWER                       4   // 额定功率
#define M_PAR_PEAK_POWER                        5   // 峰值功率
#define M_PAR_RATED_SPD                         6   // 额定转速
#define M_PAR_PEAK_SPD                          7   // 峰值转速
#define M_PAR_NUMBER_OF_POLE_PAIRS              8   // 极对数
#define M_PAR_ROTOR_SENSOR_POLE_PAIRS           9   // 转子传感器极对数
#define M_PAR_RATED_VOL                         10  // 额定电压
#define M_PAR_RATED_CUR                         11  // 额定电流
#define M_PAR_PEAK_CUR                          12  // 峰值电流
#define M_PAR_PEAK_TORQUE                       13  // 峰值扭矩
#define M_PAR_RESOLVER_ZERO_POS                 14  // 旋变零位
#define M_PAR_RS                                15  // 定子电阻
#define M_PAR_D_AXIS_INDUCTANCE                 16  // D轴电感
#define M_PAR_Q_AXIS_INDUCTANCE                 17  // Q轴电感
#define M_PAR_BACK_EMF_CONSTANT                 18  // 反电动势常数
#define M_PAR_FIELD_WEAKENING_CUR               19  // 弱磁电流
#define M_PAR_MAX_FEEDBACK_CUR                  20  // 最大回馈电流
#define M_PAR_INIT_POS_IDEN_CUR                 21  // 初始位置辨识电流
#define M_PAR_STATOR_RES_IDEN_CUR1              22  // 定子电阻辨识电流1
#define M_PAR_STATOR_RES_IDEN_CUR2              23  // 定子电阻辨识电流2

// 0x2021 - DMCU控制器参数数组索引
#define DM_CTRL_DATA_CKS                        0   // 校验和
#define DM_CTRL_RATED_POWER                     1   // 控制器额定功率
#define DM_CTRL_RATED_VOLTAGE                   2   // 控制器额定电压
#define DM_CTRL_RATED_CURRENT                   3   // 控制器额定电流
#define DM_CTRL_PEAK_CURRENT                    4   // 控制器峰值电流
#define DM_CTRL_DEADTIME                        5   // 死区时间
#define DM_CTRL_DEADTIME_COMP                   6   // 死区补偿时间
#define DM_CTRL_BUSVOLTAGE_CAL                  7   // 母线电压采样校准系数
#define DM_CTRL_PHASECURRENT_CAL                8   // 相电流采样校准系数
#define DM_CTRL_CONTROLVOLTAGE_CAL              9   // 控制电压采样校准系数

// 0x2020 - DMCU速度控制参数数组索引
#define DM_SPD_DATA_CKS                         0   // 校验和
#define DM_SPD_ACCEL_TIMECONST                  1   // 加速时间常数
#define DM_SPD_DECEL_TIMECONST                  2   // 减速时间常数
#define DM_SPD_LIMIT_SOURCE                     3   // 速度上限指令来源
#define DM_SPD_POS_LIMIT                        4   // 正转上限速度设定
#define DM_SPD_NEG_LIMIT                        5   // 反转上限速度设定

// 0x201F - DMCU扭矩控制参数数组索引
#define DM_TORQUE_DATA_CKS                      0   // 校验和
#define DM_TORQUE_LIMIT_SOURCE                  1   // 扭矩上限指令来源
#define DM_TORQUE_MOTOR_LIMIT                   2   // 电动扭矩上限设定
#define DM_TORQUE_FEEDBACK_LIMIT                3   // 回馈扭矩上限设定
#define DM_TORQUE_ACCEL_TIME                    4   // 扭矩加速时间
#define DM_TORQUE_DECEL_TIME                    5   // 扭矩减速时间

// 0x201E - DMCU增益参数数组索引
#define DM_GAIN_DATA_CKS                        0   // 校验和
#define DM_GAIN_SPD_KP1                         1   // 速度环Kp1
#define DM_GAIN_SPD_KI1                         2   // 速度环Ki1
#define DM_GAIN_SPD_PARAM_SWITCH_LOWSPEED       3   // 速度环参数切换低点转速
#define DM_GAIN_SPD_KP2                         4   // 速度环Kp2
#define DM_GAIN_SPD_KI2                         5   // 速度环Ki2
#define DM_GAIN_SPD_PARAM_SWITCH_HIGHSPEED      6   // 速度环参数切换高点转速
#define DM_GAIN_SPD_OUTPUT_FILTER               7   // 速度环输出滤波参数
#define DM_GAIN_SPD_FEEDBACK_FILTER             8   // 反馈速度滤波设置
#define DM_GAIN_CUR_GAIN_SWITCH_SELECT          9   // 电流环增益切换选择
#define DM_GAIN_CUR_GAIN_SWITCH_SPEEDPOINT      10  // 电流环增益切换速度点
#define DM_GAIN_CUR_GAIN_SWITCH_CURRENTPOINT    11  // 电流环增益切换电流点
#define DM_GAIN_CUR_GAIN_SWITCH_HYSTERESIS      12  // 电流环增益切换滞环值
#define DM_GAIN_DCUR_KP1                        13  // D轴电流环KP1
#define DM_GAIN_DCUR_KI1                        14  // D轴电流环KI1
#define DM_GAIN_DCUR_KP2                        15  // D轴电流环KP2
#define DM_GAIN_DCUR_KI2                        16  // D轴电流环KI2
#define DM_GAIN_QCUR_KP1                        17  // Q轴电流环KP1
#define DM_GAIN_QCUR_KI1                        18  // Q轴电流环KI1
#define DM_GAIN_QCUR_KP2                        19  // Q轴电流环KP2
#define DM_GAIN_QCUR_KI2                        20  // Q轴电流环KI2

// 0x201D - DMCU控制优化参数数组索引
#define DM_CTRLOPT_DATA_CKS                     0   // 校验和
#define DM_CTRLOPT_LIMITENABLE                  1   // 限速使能开启
#define DM_CTRLOPT_TORQUECOMPENABLE             2   // 扭矩补偿功能开启
#define DM_CTRLOPT_IDENZEROONLY                 3   // 仅辨识零位功能
#define DM_CTRLOPT_CARRIERFREQ                  4   // 载波频率

// 0x201C - DMCU叉车参数数组索引
#define DM_FORK_DATA_CKS                        0   // 校验和
#define DM_FORK_SPD_MODE_ENABLE                 1   // 速度控行驶工况使能开启

// 0x201B - DMCU通讯参数数组索引
#define DM_COMM_DATA_CKS                        0   // 校验和
#define DM_COMM_CAN1_BAUD                       1   // CAN1波特率
#define DM_COMM_CAN2_BAUD                       2   // CAN2波特率

// 0x201A - DMCU保护参数数组索引
#define DM_PROT_DATA_CKS                        0   // 校验和
#define DM_PROT_CUROFFSET_THRES                 1   // 电流零漂检测故障阈值
#define DM_PROT_MOTOR_OVERTEMP                  2   // 电机过温保护点
#define DM_PROT_MOTOR_OVERTEMP_WARN             3   // 电机过温预警点
#define DM_PROT_CTRL_OVERTEMP                   4   // 控制器过温保护点
#define DM_PROT_CTRL_OVERTEMP_WARN              5   // 控制器过温预警点
#define DM_PROT_MOTOR_POS_OVERSPEED             6   // 电机正转超速点
#define DM_PROT_MOTOR_NEG_OVERSPEED             7   // 电机反转超速点
#define DM_PROT_INSTANT_OC_THRES                8   // 瞬时过流保护点
#define DM_PROT_FILTER_OC_THRES                 9   // 滤波过流保护点
#define DM_PROT_PWRON_VOLT_HYST                 10  // 开机电压滞环
#define DM_PROT_LOWVOLT_UNDER                   11  // 低压欠压保护阈值
#define DM_PROT_LOWVOLT_OVER                    12  // 低压过压保护阈值
#define DM_PROT_HIGHVOLT_UNDER                  13  // 高压欠压保护阈值
#define DM_PROT_HIGHVOLT_OVER                   14  // 高压过压保护阈值
#define DM_PROT_LOCKED_ROTOR_FREQ               15  // 堵转载频
#define DM_PROT_STALL_TORQUE                    16  // 堵转保护扭矩
#define DM_PROT_STALL_TIME                      17  // 堵转保护时间
#define DM_PROT_STALL_SAFE_TORQUE               18  // 堵转安全扭矩

// 0x2019 - DMCU泄放参数数组索引
#define DM_DISCHG_DATA_CKS                      0   // 校验和
#define DM_DISCHG_SAFE_VOL                      1   // 安全电压
#define DM_DISCHG_TIMEOUT                       2   // 泄放超时时间

// ============================================================================
// DMCU参数子索引快速参考宏定义 (用于SDO通信)
// ============================================================================

// 0x2024 - DMCU COB数据子索引 (29参数)
#define DMCU_COB_T_M_Spd_max                    1   // 驱动电机最大转速
#define DMCU_COB_Pmode_spd_percentage           2   // P模式速度百分比
#define DMCU_COB_Emode_spd_percentage           3   // E模式速度百分比
#define DMCU_COB_Smode_spd_percentage           4   // S模式速度百分比
#define DMCU_COB_Tr_Pmode_acc_time              5   // P模式加速时间
#define DMCU_COB_Tr_Pmode_dec_time              6   // P模式减速时间
#define DMCU_COB_Tr_Emode_acc_time              7   // E模式加速时间
#define DMCU_COB_Tr_Emode_dec_time              8   // E模式减速时间
#define DMCU_COB_Tr_Smode_acc_time              9   // S模式加速时间
#define DMCU_COB_Tr_Smode_dec_time              10  // S模式减速时间
#define DMCU_COB_H_spd_brake_mode_dec_time      11  // 高速制动模式减速时间
#define DMCU_COB_L_spd_brake_mode_dec_time      12  // 低速制动模式减速时间
#define DMCU_COB_Crawl_brake_mode_dec_time      13  // 爬行制动模式减速时间
#define DMCU_COB_Half_Throttle_dec_time         14  // 半油门减速时间
#define DMCU_COB_Reverse_dec_time               15  // 倒车减速时间
#define DMCU_COB_Crawl_acc_dec_time             16  // 爬行加减速时间
#define DMCU_COB_Unseat_delay_time              17  // 离座延迟时间
#define DMCU_COB_Unseatbelt_delay_time          18  // 未系安全带延迟时间
#define DMCU_COB_PSlope_time_MAX                19  // 坡道时间最大值
#define DMCU_COB_Throttle_analog_input_gain     20  // 油门模拟输入增益
#define DMCU_COB_Throttle_analog_input_offset   21  // 油门模拟输入偏移
#define DMCU_COB_Tyre_angle_analog_input_gain   22  // 轮胎角度模拟输入增益
#define DMCU_COB_Tyre_angle_analog_input_offset 23  // 轮胎角度模拟输入偏移
#define DMCU_COB_Australia_seatbelt_logic       24  // 澳洲安全带逻辑
#define DMCU_COB_Tracti_seatbelt_check          25  // 牵引安全带检查
#define DMCU_COB_TractioPanelLoc_check          26  // 牵引面板锁检查
#define DMCU_COB_Back_off_mod_sp_percentage     27  // 后退模式速度百分比
#define DMCU_COB_S_curve_mode_time1             28  // S曲线模式时间1
#define DMCU_COB_S_curve_mode_time2             29  // S曲线模式时间2

// 0x2023 - 驱动控制基础数据子索引 (8参数)
#define DMCU_CTRL_BASIC_CMD_Src                 1   // 指令源
#define DMCU_CTRL_BASIC_CMD_T_ON                2   // 指令使能
#define DMCU_CTRL_BASIC_M_Rotation_Direction    3   // 电机旋转方向
#define DMCU_CTRL_BASIC_M_Ctr_mode              4   // 电机控制模式
#define DMCU_CTRL_BASIC_Spd_CMD                 5   // 速度指令
#define DMCU_CTRL_BASIC_Torque_CMD              6   // 扭矩指令
#define DMCU_CTRL_BASIC_D_Axis_Current          7   // D轴电流
#define DMCU_CTRL_BASIC_Q_Axis_Current          8   // Q轴电流

// 0x2022 - 电机参数数据子索引 (23参数)
#define DMCU_M_PAR_SN                           1   // 电机序列号
#define DMCU_M_PAR_Type                         2   // 电机类型
#define DMCU_M_PAR_Negative_Sequence_Ctrl       3   // 负序控制
#define DMCU_M_PAR_Rated_Power                  4   // 额定功率
#define DMCU_M_PAR_Peak_Power                   5   // 峰值功率
#define DMCU_M_PAR_Rated_Spd                    6   // 额定转速
#define DMCU_M_PAR_Peak_Spd                     7   // 峰值转速
#define DMCU_M_PAR_Number_Of_Pole_Pairs         8   // 极对数
#define DMCU_M_PAR_Rotor_Sensor_Pole_Pairs      9   // 转子传感器极对数
#define DMCU_M_PAR_Rated_Vol                    10  // 额定电压
#define DMCU_M_PAR_Rated_Cur                    11  // 额定电流
#define DMCU_M_PAR_Peak_Cur                     12  // 峰值电流
#define DMCU_M_PAR_Peak_Torque                  13  // 峰值扭矩
#define DMCU_M_PAR_Resolver_Zero_Pos            14  // 旋变零位
#define DMCU_M_PAR_Rs                           15  // 定子电阻
#define DMCU_M_PAR_D_Axis_Inductance            16  // D轴电感
#define DMCU_M_PAR_Q_Axis_Inductance            17  // Q轴电感
#define DMCU_M_PAR_Back_EMF_Constant            18  // 反电动势常数
#define DMCU_M_PAR_Field_Weakening_Cur          19  // 弱磁电流
#define DMCU_M_PAR_Max_Feedback_Cur             20  // 最大回馈电流
#define DMCU_M_PAR_Init_Pos_Iden_Cur            21  // 初始位置辨识电流
#define DMCU_M_PAR_Stator_Res_Iden_Cur1         22  // 定子电阻辨识电流1
#define DMCU_M_PAR_Stator_Res_Iden_Cur2         23  // 定子电阻辨识电流2

// 0x2021 - DMCU控制器参数子索引 (9参数)
#define DMCU_CTRL_Rated_Power                   1   // 控制器额定功率
#define DMCU_CTRL_Rated_Voltage                 2   // 控制器额定电压
#define DMCU_CTRL_Rated_Current                 3   // 控制器额定电流
#define DMCU_CTRL_Peak_Current                  4   // 控制器峰值电流
#define DMCU_CTRL_Deadtime                      5   // 死区时间
#define DMCU_CTRL_Deadtime_Comp                 6   // 死区补偿时间
#define DMCU_CTRL_BusVoltage_Cal                7   // 母线电压采样校准系数
#define DMCU_CTRL_PhaseCurrent_Cal              8   // 相电流采样校准系数
#define DMCU_CTRL_ControlVoltage_Cal            9   // 控制电压采样校准系数

// 0x2020 - DMCU速度控制参数子索引 (5参数)
#define DMCU_SPD_Accel_TimeConst                1   // 加速时间常数
#define DMCU_SPD_Decel_TimeConst                2   // 减速时间常数
#define DMCU_SPD_Limit_Source                   3   // 速度上限指令来源
#define DMCU_SPD_Pos_Limit                      4   // 正转上限速度设定
#define DMCU_SPD_Neg_Limit                      5   // 反转上限速度设定

// 0x201F - DMCU扭矩控制参数子索引 (5参数)
#define DMCU_TORQUE_Limit_Source                1   // 扭矩上限指令来源
#define DMCU_TORQUE_Motor_Limit                 2   // 电动扭矩上限设定
#define DMCU_TORQUE_Feedback_Limit              3   // 回馈扭矩上限设定
#define DMCU_TORQUE_Accel_Time                  4   // 扭矩加速时间
#define DMCU_TORQUE_Decel_Time                  5   // 扭矩减速时间

// 0x201E - DMCU增益参数子索引 (20参数)
#define DMCU_GAIN_Spd_Kp1                       1   // 速度环Kp1
#define DMCU_GAIN_Spd_Ki1                       2   // 速度环Ki1
#define DMCU_GAIN_Spd_Param_Switch_LowSpeed     3   // 速度环参数切换低点转速
#define DMCU_GAIN_Spd_Kp2                       4   // 速度环Kp2
#define DMCU_GAIN_Spd_Ki2                       5   // 速度环Ki2
#define DMCU_GAIN_Spd_Param_Switch_HighSpeed    6   // 速度环参数切换高点转速
#define DMCU_GAIN_Spd_Output_Filter             7   // 速度环输出滤波参数
#define DMCU_GAIN_Spd_Feedback_Filter           8   // 反馈速度滤波设置
#define DMCU_GAIN_Cur_Gain_Switch_Select        9   // 电流环增益切换选择
#define DMCU_GAIN_Cur_Gain_Switch_SpeedPoint    10  // 电流环增益切换速度点
#define DMCU_GAIN_Cur_Gain_Switch_CurrentPoint  11  // 电流环增益切换电流点
#define DMCU_GAIN_Cur_Gain_Switch_Hysteresis    12  // 电流环增益切换滞环值
#define DMCU_GAIN_Dcur_KP1                      13  // D轴电流环KP1
#define DMCU_GAIN_Dcur_KI1                      14  // D轴电流环KI1
#define DMCU_GAIN_Dcur_KP2                      15  // D轴电流环KP2
#define DMCU_GAIN_Dcur_KI2                      16  // D轴电流环KI2
#define DMCU_GAIN_Qcur_KP1                      17  // Q轴电流环KP1
#define DMCU_GAIN_Qcur_KI1                      18  // Q轴电流环KI1
#define DMCU_GAIN_Qcur_KP2                      19  // Q轴电流环KP2
#define DMCU_GAIN_Qcur_KI2                      20  // Q轴电流环KI2

// 0x201D - DMCU控制优化参数子索引 (4参数)
#define DMCU_CTRLOPT_LimitEnable                1   // 限速使能开启
#define DMCU_CTRLOPT_TorqueCompEnable           2   // 扭矩补偿功能开启
#define DMCU_CTRLOPT_IdenZeroOnly               3   // 仅辨识零位功能
#define DMCU_CTRLOPT_CarrierFreq                4   // 载波频率

// 0x201C - DMCU叉车参数子索引 (1参数)
#define DMCU_FORK_Spd_Mode_Enable               1   // 速度控行驶工况使能开启

// 0x201B - DMCU通讯参数子索引 (2参数)
#define DMCU_COMM_CAN1_Baud                     1   // CAN1波特率
#define DMCU_COMM_CAN2_Baud                     2   // CAN2波特率

// 0x201A - DMCU保护参数子索引 (18参数)
#define DMCU_PROT_CurOffset_Thres               1   // 电流零漂检测故障阈值
#define DMCU_PROT_Motor_OverTemp                2   // 电机过温保护点
#define DMCU_PROT_Motor_OverTemp_Warn           3   // 电机过温预警点
#define DMCU_PROT_Ctrl_OverTemp                 4   // 控制器过温保护点
#define DMCU_PROT_Ctrl_OverTemp_Warn            5   // 控制器过温预警点
#define DMCU_PROT_Motor_Pos_OverSpeed           6   // 电机正转超速点
#define DMCU_PROT_Motor_Neg_OverSpeed           7   // 电机反转超速点
#define DMCU_PROT_Instant_OC_Thres              8   // 瞬时过流保护点
#define DMCU_PROT_Filter_OC_Thres               9   // 滤波过流保护点
#define DMCU_PROT_PwrOn_Volt_Hyst               10  // 开机电压滞环
#define DMCU_PROT_LowVolt_Under                 11  // 低压欠压保护阈值
#define DMCU_PROT_LowVolt_Over                  12  // 低压过压保护阈值
#define DMCU_PROT_HighVolt_Under                13  // 高压欠压保护阈值
#define DMCU_PROT_HighVolt_Over                 14  // 高压过压保护阈值
#define DMCU_PROT_Locked_Rotor_Freq             15  // 堵转载频
#define DMCU_PROT_Stall_Torque                  16  // 堵转保护扭矩
#define DMCU_PROT_Stall_Time                    17  // 堵转保护时间
#define DMCU_PROT_Stall_Safe_Torque             18  // 堵转安全扭矩

// 0x2019 - DMCU泄放参数子索引 (2参数)
#define DMCU_DISCHG_Safe_Vol                    1   // 安全电压
#define DMCU_DISCHG_Timeout                     2   // 泄放超时时间

// ============================================================================
// 使用示例:
// 读取电机额定转速: SDO_DMCU_GetData(DMCU_M_PAR_INDEX, DMCU_M_PAR_Rated_Spd)
// 设置速度环Kp1: SDO_DMCU_SetData(DMCU_GAIN_INDEX, DMCU_GAIN_Spd_Kp1, value)
// ============================================================================

#endif /* APP_CAN_SDO_COB_H_ */


// ============================================================================
// DMCU各组参数获取函数声明
// ============================================================================

// 0x2023 - 驱动控制基础数据
uint32_t DCtrlBasic_Get_CMD_Src(void);                    // 指令源
uint32_t DCtrlBasic_Get_CMD_T_ON(void);                   // 指令使能
uint32_t DCtrlBasic_Get_M_Rotation_Direction(void);       // 电机旋转方向
uint32_t DCtrlBasic_Get_M_Ctr_mode(void);                 // 电机控制模式
uint32_t DCtrlBasic_Get_Spd_CMD(void);                    // 速度指令
uint32_t DCtrlBasic_Get_Torque_CMD(void);                 // 扭矩指令
uint32_t DCtrlBasic_Get_D_Axis_Current(void);             // D轴电流
uint32_t DCtrlBasic_Get_Q_Axis_Current(void);             // Q轴电流

// 0x2022 - 电机参数数据
uint32_t M_Par_Get_SN(void);                              // 电机序列号
uint32_t M_Par_Get_Type(void);                            // 电机类型
uint32_t M_Par_Get_Negative_Sequence_Ctrl(void);          // 负序控制
uint32_t M_Par_Get_Rated_Power(void);                     // 额定功率
uint32_t M_Par_Get_Peak_Power(void);                      // 峰值功率
uint32_t M_Par_Get_Rated_Spd(void);                       // 额定转速
uint32_t M_Par_Get_Peak_Spd(void);                        // 峰值转速
uint32_t M_Par_Get_Number_Of_Pole_Pairs(void);            // 极对数
uint32_t M_Par_Get_Rotor_Sensor_Pole_Pairs(void);         // 转子传感器极对数
uint32_t M_Par_Get_Rated_Vol(void);                       // 额定电压
uint32_t M_Par_Get_Rated_Cur(void);                       // 额定电流
uint32_t M_Par_Get_Peak_Cur(void);                        // 峰值电流
uint32_t M_Par_Get_Peak_Torque(void);                     // 峰值扭矩
uint32_t M_Par_Get_Resolver_Zero_Pos(void);               // 旋变零位
uint32_t M_Par_Get_Rs(void);                              // 定子电阻
uint32_t M_Par_Get_D_Axis_Inductance(void);               // D轴电感
uint32_t M_Par_Get_Q_Axis_Inductance(void);               // Q轴电感
uint32_t M_Par_Get_Back_EMF_Constant(void);               // 反电动势常数
uint32_t M_Par_Get_Field_Weakening_Cur(void);             // 弱磁电流
uint32_t M_Par_Get_Max_Feedback_Cur(void);                // 最大回馈电流
uint32_t M_Par_Get_Init_Pos_Iden_Cur(void);               // 初始位置辨识电流
uint32_t M_Par_Get_Stator_Res_Iden_Cur1(void);            // 定子电阻辨识电流1
uint32_t M_Par_Get_Stator_Res_Iden_Cur2(void);            // 定子电阻辨识电流2

// 0x2021 - DMCU控制器参数
uint32_t DM_Ctrl_Get_Rated_Power(void);                   // 控制器额定功率
uint32_t DM_Ctrl_Get_Rated_Voltage(void);                 // 控制器额定电压
uint32_t DM_Ctrl_Get_Rated_Current(void);                 // 控制器额定电流
uint32_t DM_Ctrl_Get_Peak_Current(void);                  // 控制器峰值电流
uint32_t DM_Ctrl_Get_Deadtime(void);                      // 死区时间
uint32_t DM_Ctrl_Get_Deadtime_Comp(void);                 // 死区补偿时间
uint32_t DM_Ctrl_Get_BusVoltage_Cal(void);                // 母线电压采样校准系数
uint32_t DM_Ctrl_Get_PhaseCurrent_Cal(void);              // 相电流采样校准系数
uint32_t DM_Ctrl_Get_ControlVoltage_Cal(void);            // 控制电压采样校准系数

// 0x2020 - DMCU速度控制参数
uint32_t DM_Spd_Get_Accel_TimeConst(void);                // 加速时间常数
uint32_t DM_Spd_Get_Decel_TimeConst(void);                // 减速时间常数
uint32_t DM_Spd_Get_Limit_Source(void);                   // 速度上限指令来源
uint32_t DM_Spd_Get_Pos_Limit(void);                      // 正转上限速度设定
uint32_t DM_Spd_Get_Neg_Limit(void);                      // 反转上限速度设定

// 0x201F - DMCU扭矩控制参数
uint32_t DM_Torque_Get_Limit_Source(void);                // 扭矩上限指令来源
uint32_t DM_Torque_Get_Motor_Limit(void);                 // 电动扭矩上限设定
uint32_t DM_Torque_Get_Feedback_Limit(void);              // 回馈扭矩上限设定
uint32_t DM_Torque_Get_Accel_Time(void);                  // 扭矩加速时间
uint32_t DM_Torque_Get_Decel_Time(void);                  // 扭矩减速时间

// 0x201E - DMCU增益参数
uint32_t DM_Gain_Get_Spd_Kp1(void);                       // 速度环Kp1
uint32_t DM_Gain_Get_Spd_Ki1(void);                       // 速度环Ki1
uint32_t DM_Gain_Get_Spd_Param_Switch_LowSpeed(void);     // 速度环参数切换低点转速
uint32_t DM_Gain_Get_Spd_Kp2(void);                       // 速度环Kp2
uint32_t DM_Gain_Get_Spd_Ki2(void);                       // 速度环Ki2
uint32_t DM_Gain_Get_Spd_Param_Switch_HighSpeed(void);    // 速度环参数切换高点转速
uint32_t DM_Gain_Get_Spd_Output_Filter(void);             // 速度环输出滤波参数
uint32_t DM_Gain_Get_Spd_Feedback_Filter(void);           // 反馈速度滤波设置
uint32_t DM_Gain_Get_Cur_Gain_Switch_Select(void);        // 电流环增益切换选择
uint32_t DM_Gain_Get_Cur_Gain_Switch_SpeedPoint(void);    // 电流环增益切换速度点
uint32_t DM_Gain_Get_Cur_Gain_Switch_CurrentPoint(void);  // 电流环增益切换电流点
uint32_t DM_Gain_Get_Cur_Gain_Switch_Hysteresis(void);    // 电流环增益切换滞环值
uint32_t DM_Gain_Get_Dcur_KP1(void);                      // D轴电流环KP1
uint32_t DM_Gain_Get_Dcur_KI1(void);                      // D轴电流环KI1
uint32_t DM_Gain_Get_Dcur_KP2(void);                      // D轴电流环KP2
uint32_t DM_Gain_Get_Dcur_KI2(void);                      // D轴电流环KI2
uint32_t DM_Gain_Get_Qcur_KP1(void);                      // Q轴电流环KP1
uint32_t DM_Gain_Get_Qcur_KI1(void);                      // Q轴电流环KI1
uint32_t DM_Gain_Get_Qcur_KP2(void);                      // Q轴电流环KP2
uint32_t DM_Gain_Get_Qcur_KI2(void);                      // Q轴电流环KI2

// 0x201D - DMCU控制优化参数
uint32_t DM_CtrlOpt_Get_LimitEnable(void);                // 限速使能开启
uint32_t DM_CtrlOpt_Get_TorqueCompEnable(void);           // 扭矩补偿功能开启
uint32_t DM_CtrlOpt_Get_IdenZeroOnly(void);               // 仅辨识零位功能
uint32_t DM_CtrlOpt_Get_CarrierFreq(void);                // 载波频率

// 0x201C - DMCU叉车参数
uint32_t DM_Fork_Get_Spd_Mode_Enable(void);               // 速度控行驶工况使能开启

// 0x201B - DMCU通讯参数
uint32_t DM_Comm_Get_CAN1_Baud(void);                     // CAN1波特率
uint32_t DM_Comm_Get_CAN2_Baud(void);                     // CAN2波特率

// 0x201A - DMCU保护参数
uint32_t DM_Prot_Get_CurOffset_Thres(void);               // 电流零漂检测故障阈值
uint32_t DM_Prot_Get_Motor_OverTemp(void);                // 电机过温保护点
uint32_t DM_Prot_Get_Motor_OverTemp_Warn(void);           // 电机过温预警点
uint32_t DM_Prot_Get_Ctrl_OverTemp(void);                 // 控制器过温保护点
uint32_t DM_Prot_Get_Ctrl_OverTemp_Warn(void);            // 控制器过温预警点
uint32_t DM_Prot_Get_Motor_Pos_OverSpeed(void);           // 电机正转超速点
uint32_t DM_Prot_Get_Motor_Neg_OverSpeed(void);           // 电机反转超速点
uint32_t DM_Prot_Get_Instant_OC_Thres(void);              // 瞬时过流保护点
uint32_t DM_Prot_Get_Filter_OC_Thres(void);               // 滤波过流保护点
uint32_t DM_Prot_Get_PwrOn_Volt_Hyst(void);               // 开机电压滞环
uint32_t DM_Prot_Get_LowVolt_Under(void);                 // 低压欠压保护阈值
uint32_t DM_Prot_Get_LowVolt_Over(void);                  // 低压过压保护阈值
uint32_t DM_Prot_Get_HighVolt_Under(void);                // 高压欠压保护阈值
uint32_t DM_Prot_Get_HighVolt_Over(void);                 // 高压过压保护阈值
uint32_t DM_Prot_Get_Locked_Rotor_Freq(void);             // 堵转载频
uint32_t DM_Prot_Get_Stall_Torque(void);                  // 堵转保护扭矩
uint32_t DM_Prot_Get_Stall_Time(void);                    // 堵转保护时间
uint32_t DM_Prot_Get_Stall_Safe_Torque(void);             // 堵转安全扭矩

// 0x2019 - DMCU泄放参数
uint32_t DM_Dischg_Get_Safe_Vol(void);                    // 安全电压
uint32_t DM_Dischg_Get_Timeout(void);                     // 泄放超时时间
