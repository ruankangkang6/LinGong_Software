/*
 * SDO_COB.c
 *
 *  Created on: Sep 28, 2025
 *      Author: pc
 */

#include "string.h"
#include "def.h"
#include "main.h"
#include "can.h"
#include "uds.h"
#include "tp.h"
#include "fbl.h"
#include "SDO_COB.h"

union{
    uint32_t EEPROM_data[COB_MAX];
    Data_COB_32b COB_data[COB_MAX];
    struct
    {
        Data_COB_32b data_cks;
        Data_COB_32b T_M_Spd_max;//牵引最大转速
        Data_COB_32b Pmode_spd_percentage;//P模式速度百分比
        Data_COB_32b Emode_spd_percentage;//E模式速度百分比
        Data_COB_32b Smode_spd_percentage;//S模式速度百分比
        Data_COB_32b Tr_Pmode_acc_time;//P模式加速时间
        Data_COB_32b Tr_Pmode_dec_time;//P模式减速时间
        Data_COB_32b Tr_Emode_acc_time;//E模式加速时间
        Data_COB_32b Tr_Emode_dec_time;//E模式减速时间
        Data_COB_32b Tr_Smode_acc_time;//S模式加速时间
        Data_COB_32b Tr_Smode_dec_time;//S模式减速时间
        Data_COB_32b H_spd_brake_mode_dec_time;//高速时踩下刹车减速时间
        Data_COB_32b L_spd_brake_mode_dec_time;//低速时踩下刹车减速时间
        Data_COB_32b Crawl_brake_mode_dec_time;//蠕行时踩下刹车减速时间
        Data_COB_32b Half_Throttle_dec_time;//半松油门减速时间
        Data_COB_32b Reverse_dec_time;//反向制动减速时间
        Data_COB_32b Crawl_acc_dec_time;   //蠕行加减速时间
        Data_COB_32b Unseat_delay_time;//座椅开关延时
        Data_COB_32b Unseatbelt_delay_time;//安全带开关延时
        Data_COB_32b PSlope_time_MAX;//最大驻坡时间
        Data_COB_32b Throttle_analog_input_gain;//油门模拟量增益
        Data_COB_32b Throttle_analog_input_offset;//油门模拟量偏置
        Data_COB_32b Tyre_angle_analog_input_gain;//转角模拟量增益
        Data_COB_32b Tyre_angle_analog_input_offset;//转角模拟量偏置
        Data_COB_32b Australian_seatbelt_logic;//澳标安全带逻辑
        Data_COB_32b Tracti_seatbelt_check;//牵引安全带检测功能
        Data_COB_32b TractioPanelLoc_check;//牵引仪表刷卡功能
        Data_COB_32b Back_off_mod_sp_percentage;//倒车速度百分比
        Data_COB_32b S_curve_mode_time1;//S曲线时间1
        Data_COB_32b S_curve_mode_time2;//S曲线时间2


    }st;

}SDO_SET_data;

const Data_COB_32b SDO_SET_data_def[COB_MAX] = \
{       0x0000AA55,
        4570,//牵引最大转速
        100,//P模式速度百分比
        100,//E模式速度百分比
        45,//S模式速度百分比
        635,//P模式加速时间
        635,//P模式减速时间
        1016,//E模式加速时间
        762,//E模式减速时间
        831,//S模式加速时间
        1143,//S模式减速时间

        500,//高速时踩下刹车减速时间
        500,//低速时踩下刹车减速时间
        500,//蠕行时踩下刹车减速时间
        500,//半松油门减速时间
        571,//反向制动减速时间
        3808,   //蠕行加减速时间
        15,//座椅开关延时
        10,//安全带开关延时
        30,//最大驻坡时间
        500,//油门模拟量增益
        202,//油门模拟量偏置
        500,//转角模拟量增益
        200,//转角模拟量偏置
        1,//澳标安全带逻辑
        1,//牵引安全带检测功能
        1,//牵引仪表刷卡功能
        80,//倒车速度百分比
        500,//S曲线时间1
        500,//S曲线时间2


};

void SDO_SetData_W_EEPROM(uint8_t indx,uint32_t data_stor)
{

    I2CWriteInt32WaitMode((EEPROM_SDO_Adr+indx),data_stor);

}

uint32_t SDO_SetData_R_EEPROM(uint8_t indx)
{
    uint32_t   Read_data;
    I2CReadInt32WaitMode((EEPROM_SDO_Adr+indx),&Read_data);
    return Read_data;
}

void CanMsgSDO_SetDataID2(uint8_t* data_set)
{
    Data_COB_64b msg_get_data;
    Data_COB_64b get_Req_data;
    uint8_t i;
    for(i = 0;i<8;i++)
    {
        msg_get_data.U8t[i] = data_set[i];
        get_Req_data.U8t[i] = 0;
    }
    
    // 检查是否为DMCU其他索引组 (0x2019-0x2023)
    if(msg_get_data.msg.index >= 0x2019 && msg_get_data.msg.index <= 0x2023)
    {
        CanMsgSDO_DMCU_Handler(msg_get_data.msg.index, data_set);
        return;
    }

    if((msg_get_data.msg.SUBindex)&&(msg_get_data.msg.index == 0x2024))
    {
        switch (msg_get_data.msg.byte0)
        {
        case 0x23:
        /* code */
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[0] = msg_get_data.msg.data4.U8t[0];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[1] = msg_get_data.msg.data4.U8t[1];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[2] = msg_get_data.msg.data4.U8t[2];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[3] = msg_get_data.msg.data4.U8t[3];
            SDO_SetData_W_EEPROM(msg_get_data.msg.SUBindex,SDO_SET_data.EEPROM_data[msg_get_data.msg.SUBindex]);
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x60;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
            break;
        break;
        case 0x27:
        /* code */
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[0] = msg_get_data.msg.data4.U8t[0];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[1] = msg_get_data.msg.data4.U8t[1];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[2] = msg_get_data.msg.data4.U8t[2];
            
            SDO_SetData_W_EEPROM(msg_get_data.msg.SUBindex,SDO_SET_data.EEPROM_data[msg_get_data.msg.SUBindex]);
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x60;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
            break;
        break;
        case 0x2B:
        /* code */
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[0] = msg_get_data.msg.data4.U8t[0];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[1] = msg_get_data.msg.data4.U8t[1];
            
            SDO_SetData_W_EEPROM(msg_get_data.msg.SUBindex,SDO_SET_data.EEPROM_data[msg_get_data.msg.SUBindex]);
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x60;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
            break;

        break;
        case 0x2F:
        /* code */
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex].U8t[0] = msg_get_data.msg.data4.U8t[0];
            
            SDO_SetData_W_EEPROM(msg_get_data.msg.SUBindex,SDO_SET_data.EEPROM_data[msg_get_data.msg.SUBindex]);
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x60;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
            break;
        break;
        case 0x40:
        /* code */
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x4B;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
            break;
        break;    
        default:

            get_Req_data.msg.byte0 = 0x80;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;    
            break;
        }
    }
    else
    {
        get_Req_data.msg.byte0 = 0x80;
        get_Req_data.msg.index = msg_get_data.msg.index;
        get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex; 
    }
    IL_FillSendMessage0x582_SET(&get_Req_data.U8t[0]);
    
}

void CanMsgSDO_SetDataID3(uint8_t* data_set)
{
    Data_COB_64b msg_get_data;
    Data_COB_64b get_Req_data;
    uint8_t i;
    for(i = 0;i<8;i++)
    {
        msg_get_data.U8t[i] = data_set[i];
        get_Req_data.U8t[i] = 0;
    }

    if((msg_get_data.msg.SUBindex)&&(msg_get_data.msg.index == 0x2028))
    {
        switch (msg_get_data.msg.byte0)
        {
        case 0x23:
        /* code */
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[0] = msg_get_data.msg.data4.U8t[0];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[1] = msg_get_data.msg.data4.U8t[1];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[2] = msg_get_data.msg.data4.U8t[2];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[3] = msg_get_data.msg.data4.U8t[3];
            SDO_SetData_W_EEPROM((msg_get_data.msg.SUBindex + COB_MAX_ID2),SDO_SET_data.EEPROM_data[msg_get_data.msg.SUBindex + COB_MAX_ID2]);
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x60;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;

            break;
        break;
        case 0x27:
        /* code */
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[0] = msg_get_data.msg.data4.U8t[0];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[1] = msg_get_data.msg.data4.U8t[1];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[2] = msg_get_data.msg.data4.U8t[2];

            SDO_SetData_W_EEPROM((msg_get_data.msg.SUBindex + COB_MAX_ID2),SDO_SET_data.EEPROM_data[msg_get_data.msg.SUBindex + COB_MAX_ID2]);
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x60;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;

            break;
        break;
        case 0x2B:
        /* code */
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[0] = msg_get_data.msg.data4.U8t[0];
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[1] = msg_get_data.msg.data4.U8t[1];
           
            SDO_SetData_W_EEPROM((msg_get_data.msg.SUBindex + COB_MAX_ID2),SDO_SET_data.EEPROM_data[msg_get_data.msg.SUBindex + COB_MAX_ID2]);
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x60;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
            break;
        break;
        case 0x2F:
        /* code */
            SDO_SET_data.COB_data[msg_get_data.msg.SUBindex + COB_MAX_ID2].U8t[0] = msg_get_data.msg.data4.U8t[0];
            SDO_SetData_W_EEPROM((msg_get_data.msg.SUBindex + COB_MAX_ID2),SDO_SET_data.EEPROM_data[msg_get_data.msg.SUBindex + COB_MAX_ID2]);
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x60;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
            break;
        break;
        case 0x40:
        /* code */
            get_Req_data.msg.data4.U32t  = SDO_SetData_R_EEPROM(msg_get_data.msg.SUBindex);
            get_Req_data.msg.byte0 = 0x4B;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
        break;

        default:

            get_Req_data.msg.byte0 = 0x80;
            get_Req_data.msg.index = msg_get_data.msg.index;
            get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;    
            break;
        }
    }
    IL_FillSendMessage0x583_SET(&get_Req_data.U8t[0]);


}

uint16_t SDO_SetData_GET(uint8_t data_index)
{
    return SDO_SET_data.COB_data[data_index].U16t[0];
}

void SDO_SetData_EEP_def(void)
{
    uint8_t i;
    SDO_SET_data.EEPROM_data[data_cks] = SDO_SetData_R_EEPROM(data_cks);
    if(SDO_SET_data.EEPROM_data[data_cks] != SDO_SET_data_def[i].U32t)
    {
        for(i=0;i<COB_MAX;i++)
        {
            SDO_SetData_W_EEPROM(i,SDO_SET_data_def[i].U32t);
        }
        
    }
    SDO_GetData_EEP_Read();
    
    // 初始化DMCU参数
    SDO_DMCU_EEP_Init();
}

void SDO_GetData_EEP_Read(void)
{
    uint8_t i;
    for(i=0;i<COB_MAX;i++)
    {
        SDO_SET_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(i);
    }

}

// ============================================================================
// DMCU参数管理 - 索引组信息表
// ============================================================================
static const IndexGroupInfo g_indexGroupTable_DMCU[] = {
    { 0x2024, "DMCUCOB_Data_v",        "DMCU COB数据" },
    { 0x2023, "DCtrlBasic_Data_v",     "驱动控制基础数据" },
    { 0x2022, "DM_Par_Data_v",          "电机参数数据" },
    { 0x2021, "DM_Ctrl_Data_v",        "DMCU控制器参数" },
    { 0x2020, "DM_Spd_Data_v",         "DMCU速度控制参数" },
    { 0x201F, "DM_Torque_Data_v",      "DMCU扭矩控制参数" },
    { 0x201E, "DM_Gain_Data_v",        "DMCU增益参数" },
    { 0x201D, "DM_CtrlOpt_Data_v",     "DMCU控制优化参数" },
    { 0x201C, "DM_Fork_Data_v",        "DMCU叉车参数" },
    { 0x201B, "DM_Comm_Data_v",        "DMCU通讯参数" },
    { 0x201A, "DM_Prot_Data_v",        "DMCU保护参数" },
    { 0x2019, "DM_Dischg_Data_v",      "DMCU泄放参数" }
};

// ============================================================================
// DMCU参数存储结构定义
// ============================================================================

// 0x2023 - 驱动控制基础数据
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b CMD_Src;                    // 指令源
    Data_COB_32b CMD_T_ON;                   // 指令使能
    Data_COB_32b M_Rotation_Direction;       // 电机旋转方向
    Data_COB_32b M_Ctr_mode;                 // 电机控制模式
    Data_COB_32b Spd_CMD;                    // 速度指令
    Data_COB_32b Torque_CMD;                 // 扭矩指令
    Data_COB_32b D_Axis_Current;             // D轴电流
    Data_COB_32b Q_Axis_Current;             // Q轴电流
} DCtrlBasic_Data_t;

// 0x2022 - 电机参数数据
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b SN;                         // 电机序列号
    Data_COB_32b Type;                       // 电机类型
    Data_COB_32b Negative_Sequence_Ctrl;     // 负序控制
    Data_COB_32b Rated_Power;                // 额定功率
    Data_COB_32b Peak_Power;                 // 峰值功率
    Data_COB_32b Rated_Spd;                  // 额定转速
    Data_COB_32b Peak_Spd;                   // 峰值转速
    Data_COB_32b Number_Of_Pole_Pairs;       // 极对数
    Data_COB_32b Rotor_Sensor_Pole_Pairs;    // 转子传感器极对数
    Data_COB_32b Rated_Vol;                  // 额定电压
    Data_COB_32b Rated_Cur;                  // 额定电流
    Data_COB_32b Peak_Cur;                   // 峰值电流
    Data_COB_32b Peak_Torque;                // 峰值扭矩
    Data_COB_32b Resolver_Zero_Pos;          // 旋变零位
    Data_COB_32b Rs;                         // 定子电阻
    Data_COB_32b D_Axis_Inductance;          // D轴电感
    Data_COB_32b Q_Axis_Inductance;          // Q轴电感
    Data_COB_32b Back_EMF_Constant;          // 反电动势常数
    Data_COB_32b Field_Weakening_Cur;        // 弱磁电流
    Data_COB_32b Max_Feedback_Cur;           // 最大回馈电流
    Data_COB_32b Init_Pos_Iden_Cur;          // 初始位置辨识电流
    Data_COB_32b Stator_Res_Iden_Cur1;       // 定子电阻辨识电流1
    Data_COB_32b Stator_Res_Iden_Cur2;       // 定子电阻辨识电流2
} M_Par_Data_t;

// 0x2021 - DMCU控制器参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b Rated_Power;                // 控制器额定功率
    Data_COB_32b Rated_Voltage;              // 控制器额定电压
    Data_COB_32b Rated_Current;              // 控制器额定电流
    Data_COB_32b Peak_Current;               // 控制器峰值电流
    Data_COB_32b Deadtime;                   // 死区时间
    Data_COB_32b Deadtime_Comp;              // 死区补偿时间
    Data_COB_32b BusVoltage_Cal;             // 母线电压采样校准系数
    Data_COB_32b PhaseCurrent_Cal;           // 相电流采样校准系数
    Data_COB_32b ControlVoltage_Cal;         // 控制电压采样校准系数
} DM_Ctrl_Data_t;

// 0x2020 - DMCU速度控制参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b Accel_TimeConst;            // 加速时间常数
    Data_COB_32b Decel_TimeConst;            // 减速时间常数
    Data_COB_32b Limit_Source;               // 速度上限指令来源
    Data_COB_32b Pos_Limit;                  // 正转上限速度设定
    Data_COB_32b Neg_Limit;                  // 反转上限速度设定
} DM_Spd_Data_t;

// 0x201F - DMCU扭矩控制参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b Limit_Source;               // 扭矩上限指令来源
    Data_COB_32b Motor_Limit;                // 电动扭矩上限设定
    Data_COB_32b Feedback_Limit;             // 回馈扭矩上限设定
    Data_COB_32b Accel_Time;                 // 扭矩加速时间
    Data_COB_32b Decel_Time;                 // 扭矩减速时间
} DM_Torque_Data_t;

// 0x201E - DMCU增益参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b Spd_Kp1;                    // 速度环Kp1
    Data_COB_32b Spd_Ki1;                    // 速度环Ki1
    Data_COB_32b Spd_Param_Switch_LowSpeed;  // 速度环参数切换低点转速
    Data_COB_32b Spd_Kp2;                    // 速度环Kp2
    Data_COB_32b Spd_Ki2;                    // 速度环Ki2
    Data_COB_32b Spd_Param_Switch_HighSpeed; // 速度环参数切换高点转速
    Data_COB_32b Spd_Output_Filter;          // 速度环输出滤波参数
    Data_COB_32b Spd_Feedback_Filter;        // 反馈速度滤波设置
    Data_COB_32b Cur_Gain_Switch_Select;     // 电流环增益切换选择
    Data_COB_32b Cur_Gain_Switch_SpeedPoint; // 电流环增益切换速度点
    Data_COB_32b Cur_Gain_Switch_CurrentPoint; // 电流环增益切换电流点
    Data_COB_32b Cur_Gain_Switch_Hysteresis; // 电流环增益切换滞环值
    Data_COB_32b Dcur_KP1;                   // D轴电流环KP1
    Data_COB_32b Dcur_KI1;                   // D轴电流环KI1
    Data_COB_32b Dcur_KP2;                   // D轴电流环KP2
    Data_COB_32b Dcur_KI2;                   // D轴电流环KI2
    Data_COB_32b Qcur_KP1;                   // Q轴电流环KP1
    Data_COB_32b Qcur_KI1;                   // Q轴电流环KI1
    Data_COB_32b Qcur_KP2;                   // Q轴电流环KP2
    Data_COB_32b Qcur_KI2;                   // Q轴电流环KI2
} DM_Gain_Data_t;

// 0x201D - DMCU控制优化参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b LimitEnable;                // 限速使能开启
    Data_COB_32b TorqueCompEnable;           // 扭矩补偿功能开启
    Data_COB_32b IdenZeroOnly;               // 仅辨识零位功能
    Data_COB_32b CarrierFreq;                // 载波频率
} DM_CtrlOpt_Data_t;

// 0x201C - DMCU叉车参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b Spd_Mode_Enable;            // 速度控行驶工况使能开启
} DM_Fork_Data_t;

// 0x201B - DMCU通讯参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b CAN1_Baud;                  // CAN1波特率
    Data_COB_32b CAN2_Baud;                  // CAN2波特率
} DM_Comm_Data_t;

// 0x201A - DMCU保护参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b CurOffset_Thres;            // 电流零漂检测故障阈值
    Data_COB_32b Motor_OverTemp;             // 电机过温保护点
    Data_COB_32b Motor_OverTemp_Warn;        // 电机过温预警点
    Data_COB_32b Ctrl_OverTemp;              // 控制器过温保护点
    Data_COB_32b Ctrl_OverTemp_Warn;         // 控制器过温预警点
    Data_COB_32b Motor_Pos_OverSpeed;        // 电机正转超速点
    Data_COB_32b Motor_Neg_OverSpeed;        // 电机反转超速点
    Data_COB_32b Instant_OC_Thres;           // 瞬时过流保护点
    Data_COB_32b Filter_OC_Thres;            // 滤波过流保护点
    Data_COB_32b PwrOn_Volt_Hyst;            // 开机电压滞环
    Data_COB_32b LowVolt_Under;              // 低压欠压保护阈值
    Data_COB_32b LowVolt_Over;               // 低压过压保护阈值
    Data_COB_32b HighVolt_Under;             // 高压欠压保护阈值
    Data_COB_32b HighVolt_Over;              // 高压过压保护阈值
    Data_COB_32b Locked_Rotor_Freq;          // 堵转载频
    Data_COB_32b Stall_Torque;               // 堵转保护扭矩
    Data_COB_32b Stall_Time;                 // 堵转保护时间
    Data_COB_32b Stall_Safe_Torque;          // 堵转安全扭矩
} DM_Prot_Data_t;

// 0x2019 - DMCU泄放参数
typedef struct {
    Data_COB_32b data_cks;                   // 校验和
    Data_COB_32b Safe_Vol;                   // 安全电压
    Data_COB_32b Timeout;                    // 泄放超时时间
} DM_Dischg_Data_t;

// ============================================================================
// DMCU参数实例 - 使用联合体方便EEPROM整体读写
// ============================================================================

// 0x2023 - 驱动控制基础数据联合体
union {
    uint32_t EEPROM_data[CTRL_BASIC_MAX];
    Data_COB_32b COB_data[CTRL_BASIC_MAX];
    DCtrlBasic_Data_t st;
} DCtrlBasic_data;

// 0x2022 - 电机参数数据联合体
union {
    uint32_t EEPROM_data[M_PAR_MAX];
    Data_COB_32b COB_data[M_PAR_MAX];
    M_Par_Data_t st;
} M_Par_data;

// 0x2021 - DMCU控制器参数联合体
union {
    uint32_t EEPROM_data[CTRL_MAX];
    Data_COB_32b COB_data[CTRL_MAX];
    DM_Ctrl_Data_t st;
} DM_Ctrl_data;

// 0x2020 - DMCU速度控制参数联合体
union {
    uint32_t EEPROM_data[SPD_MAX];
    Data_COB_32b COB_data[SPD_MAX];
    DM_Spd_Data_t st;
} DM_Spd_data;

// 0x201F - DMCU扭矩控制参数联合体
union {
    uint32_t EEPROM_data[TORQUE_MAX];
    Data_COB_32b COB_data[TORQUE_MAX];
    DM_Torque_Data_t st;
} DM_Torque_data;

// 0x201E - DMCU增益参数联合体
union {
    uint32_t EEPROM_data[GAIN_MAX];
    Data_COB_32b COB_data[GAIN_MAX];
    DM_Gain_Data_t st;
} DM_Gain_data;

// 0x201D - DMCU控制优化参数联合体
union {
    uint32_t EEPROM_data[CTRLOPT_MAX];
    Data_COB_32b COB_data[CTRLOPT_MAX];
    DM_CtrlOpt_Data_t st;
} DM_CtrlOpt_data;

// 0x201C - DMCU叉车参数联合体
union {
    uint32_t EEPROM_data[FORK_MAX];
    Data_COB_32b COB_data[FORK_MAX];
    DM_Fork_Data_t st;
} DM_Fork_data;

// 0x201B - DMCU通讯参数联合体
union {
    uint32_t EEPROM_data[COMM_MAX];
    Data_COB_32b COB_data[COMM_MAX];
    DM_Comm_Data_t st;
} DM_Comm_data;

// 0x201A - DMCU保护参数联合体
union {
    uint32_t EEPROM_data[PROT_MAX];
    Data_COB_32b COB_data[PROT_MAX];
    DM_Prot_Data_t st;
} DM_Prot_data;

// 0x2019 - DMCU泄放参数联合体
union {
    uint32_t EEPROM_data[DISCHG_MAX];
    Data_COB_32b COB_data[DISCHG_MAX];
    DM_Dischg_Data_t st;
} DM_Dischg_data;

// ============================================================================
// DMCU参数默认值定义
// ============================================================================

// 0x2023 - 驱动控制基础数据默认值
static const DCtrlBasic_Data_t DCtrlBasic_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 0},  // CMD_Src
    {.U32t = 0},  // CMD_T_ON
    {.U32t = 0},  // M_Rotation_Direction
    {.U32t = 0},  // M_Ctr_mode
    {.U32t = 0},  // Spd_CMD
    {.U32t = 0},  // Torque_CMD
    {.U32t = 0},  // D_Axis_Current
    {.U32t = 0}   // Q_Axis_Current
};

// 0x2022 - 电机参数默认值
static const M_Par_Data_t M_Par_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 0},      // SN
    {.U32t = 0},      // Type
    {.U32t = 0},      // Negative_Sequence_Ctrl
    {.U32t = 15000},  // Rated_Power (15kW)
    {.U32t = 30000},  // Peak_Power (30kW)
    {.U32t = 3000},   // Rated_Spd (3000rpm)
    {.U32t = 5212},   // Peak_Spd (5212rpm)
    {.U32t = 4},      // Number_Of_Pole_Pairs
    {.U32t = 4},      // Rotor_Sensor_Pole_Pairs
    {.U32t = 72},     // Rated_Vol (72V)
    {.U32t = 200},    // Rated_Cur (200A)
    {.U32t = 400},    // Peak_Cur (400A)
    {.U32t = 100},    // Peak_Torque (100Nm)
    {.U32t = 0},      // Resolver_Zero_Pos
    {.U32t = 50},     // Rs (50mΩ)
    {.U32t = 100},    // D_Axis_Inductance
    {.U32t = 100},    // Q_Axis_Inductance
    {.U32t = 50},     // Back_EMF_Constant
    {.U32t = 50},     // Field_Weakening_Cur
    {.U32t = 100},    // Max_Feedback_Cur
    {.U32t = 50},     // Init_Pos_Iden_Cur
    {.U32t = 50},     // Stator_Res_Iden_Cur1
    {.U32t = 100}     // Stator_Res_Iden_Cur2
};

// 0x2021 - 控制器参数默认值
static const DM_Ctrl_Data_t DM_Ctrl_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 15000},  // Rated_Power
    {.U32t = 72},     // Rated_Voltage
    {.U32t = 200},    // Rated_Current
    {.U32t = 400},    // Peak_Current
    {.U32t = 2000},   // Deadtime (ns)
    {.U32t = 1000},   // Deadtime_Comp (ns)
    {.U32t = 100},    // BusVoltage_Cal
    {.U32t = 100},    // PhaseCurrent_Cal
    {.U32t = 100}     // ControlVoltage_Cal
};

// 0x2020 - 速度控制参数默认值
static const DM_Spd_Data_t DM_Spd_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 500},    // Accel_TimeConst
    {.U32t = 500},    // Decel_TimeConst
    {.U32t = 0},      // Limit_Source
    {.U32t = 5212},   // Pos_Limit
    {.U32t = 5212}    // Neg_Limit
};

// 0x201F - 扭矩控制参数默认值
static const DM_Torque_Data_t DM_Torque_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 0},      // Limit_Source
    {.U32t = 100},    // Motor_Limit
    {.U32t = 50},     // Feedback_Limit
    {.U32t = 100},    // Accel_Time
    {.U32t = 100}     // Decel_Time
};

// 0x201E - 增益参数默认值
static const DM_Gain_Data_t DM_Gain_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 100},    // Spd_Kp1
    {.U32t = 50},     // Spd_Ki1
    {.U32t = 500},    // Spd_Param_Switch_LowSpeed
    {.U32t = 80},     // Spd_Kp2
    {.U32t = 40},     // Spd_Ki2
    {.U32t = 3000},   // Spd_Param_Switch_HighSpeed
    {.U32t = 10},     // Spd_Output_Filter
    {.U32t = 10},     // Spd_Feedback_Filter
    {.U32t = 0},      // Cur_Gain_Switch_Select
    {.U32t = 1000},   // Cur_Gain_Switch_SpeedPoint
    {.U32t = 100},    // Cur_Gain_Switch_CurrentPoint
    {.U32t = 50},     // Cur_Gain_Switch_Hysteresis
    {.U32t = 100},    // Dcur_KP1
    {.U32t = 50},     // Dcur_KI1
    {.U32t = 80},     // Dcur_KP2
    {.U32t = 40},     // Dcur_KI2
    {.U32t = 100},    // Qcur_KP1
    {.U32t = 50},     // Qcur_KI1
    {.U32t = 80},     // Qcur_KP2
    {.U32t = 40}      // Qcur_KI2
};

// 0x201D - 控制优化参数默认值
static const DM_CtrlOpt_Data_t DM_CtrlOpt_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 1},      // LimitEnable
    {.U32t = 1},      // TorqueCompEnable
    {.U32t = 0},      // IdenZeroOnly
    {.U32t = 10000}   // CarrierFreq (10kHz)
};

// 0x201C - 叉车参数默认值
static const DM_Fork_Data_t DM_Fork_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 1}       // Spd_Mode_Enable
};

// 0x201B - 通讯参数默认值
static const DM_Comm_Data_t DM_Comm_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 125},    // CAN1_Baud (125kbps)
    {.U32t = 500}     // CAN2_Baud (500kbps)
};

// 0x201A - 保护参数默认值
static const DM_Prot_Data_t DM_Prot_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 50},     // CurOffset_Thres
    {.U32t = 120},    // Motor_OverTemp (120°C)
    {.U32t = 100},    // Motor_OverTemp_Warn (100°C)
    {.U32t = 85},     // Ctrl_OverTemp (85°C)
    {.U32t = 75},     // Ctrl_OverTemp_Warn (75°C)
    {.U32t = 5500},   // Motor_Pos_OverSpeed
    {.U32t = 5500},   // Motor_Neg_OverSpeed
    {.U32t = 500},    // Instant_OC_Thres
    {.U32t = 450},    // Filter_OC_Thres
    {.U32t = 5},      // PwrOn_Volt_Hyst
    {.U32t = 60},     // LowVolt_Under
    {.U32t = 90},     // LowVolt_Over
    {.U32t = 60},     // HighVolt_Under
    {.U32t = 90},     // HighVolt_Over
    {.U32t = 5000},   // Locked_Rotor_Freq
    {.U32t = 80},     // Stall_Torque
    {.U32t = 5000},   // Stall_Time (ms)
    {.U32t = 50}      // Stall_Safe_Torque
};

// 0x2019 - 泄放参数默认值
static const DM_Dischg_Data_t DM_Dischg_default = {
    {.U32t = 0x0000AA55},  // data_cks 校验和
    {.U32t = 36},     // Safe_Vol (36V)
    {.U32t = 60000}   // Timeout (60s)
};

// ============================================================================
// DMCU辅助函数：根据索引和子索引获取数据指针
// 注意：subindex直接对应数组索引，subindex=0是校验和
// ============================================================================
static Data_COB_32b* SDO_DMCU_GetDataPtr(uint16_t index, uint8_t subindex)
{
    Data_COB_32b* ptr = NULL;
    
    switch(index)
    {
        case 0x2023: // DCtrlBasic_data
            if(subindex < CTRL_BASIC_MAX) {
                ptr = &DCtrlBasic_data.COB_data[subindex];
            }
            break;
            
        case 0x2022: // M_Par_data
            if(subindex < M_PAR_MAX) {
                ptr = &M_Par_data.COB_data[subindex];
            }
            break;
            
        case 0x2021: // DM_Ctrl_data
            if(subindex < CTRL_MAX) {
                ptr = &DM_Ctrl_data.COB_data[subindex];
            }
            break;
            
        case 0x2020: // DM_Spd_data
            if(subindex < SPD_MAX) {
                ptr = &DM_Spd_data.COB_data[subindex];
            }
            break;
            
        case 0x201F: // DM_Torque_data
            if(subindex < TORQUE_MAX) {
                ptr = &DM_Torque_data.COB_data[subindex];
            }
            break;
            
        case 0x201E: // DM_Gain_data
            if(subindex < GAIN_MAX) {
                ptr = &DM_Gain_data.COB_data[subindex];
            }
            break;
            
        case 0x201D: // DM_CtrlOpt_data
            if(subindex < CTRLOPT_MAX) {
                ptr = &DM_CtrlOpt_data.COB_data[subindex];
            }
            break;
            
        case 0x201C: // DM_Fork_data
            if(subindex < FORK_MAX) {
                ptr = &DM_Fork_data.COB_data[subindex];
            }
            break;
            
        case 0x201B: // DM_Comm_data
            if(subindex < COMM_MAX) {
                ptr = &DM_Comm_data.COB_data[subindex];
            }
            break;
            
        case 0x201A: // DM_Prot_data
            if(subindex < PROT_MAX) {
                ptr = &DM_Prot_data.COB_data[subindex];
            }
            break;
            
        case 0x2019: // DM_Dischg_data
            if(subindex < DISCHG_MAX) {
                ptr = &DM_Dischg_data.COB_data[subindex];
            }
            break;
            
        default:
            break;
    }
    
    return ptr;
}

// ============================================================================
// DMCU辅助函数：计算EEPROM地址
// 注意：subindex直接对应数组索引，subindex=0是校验和
// ============================================================================
static uint16_t SDO_DMCU_CalcEEPAddr(uint16_t index, uint8_t subindex)
{
    uint16_t base_addr = EEPROM_DMCU_BASE;
    uint16_t offset = 0;
    
    // 根据索引计算基地址偏移
    switch(index)
    {
        case 0x2023: offset = 0; break;
        case 0x2022: offset = CTRL_BASIC_MAX; break;
        case 0x2021: offset = (CTRL_BASIC_MAX + M_PAR_MAX); break;
        case 0x2020: offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX); break;
        case 0x201F: offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX); break;
        case 0x201E: offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX); break;
        case 0x201D: offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX); break;
        case 0x201C: offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX); break;
        case 0x201B: offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX); break;
        case 0x201A: offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX + COMM_MAX); break;
        case 0x2019: offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX + COMM_MAX + PROT_MAX); break;
        default: return 0;
    }
    
    return base_addr + offset + subindex;
}

// ============================================================================
// DMCU外部接口函数实现
// ============================================================================

/**
 * @brief  DMCU参数EEPROM初始化
 * 
 * 对每个参数组:
 * 1. 读取EEPROM中的校验和(data_cks)
 * 2. 与默认值的校验和对比
 * 3. 如果不一致,将此块数据的所有默认值写入EEPROM
 * 4. 如果一致,将整块数据从EEPROM读出来使用
 */
void SDO_DMCU_EEP_Init(void)
{
    uint32_t eep_cks;
    uint16_t base_addr = EEPROM_DMCU_BASE;
    uint16_t offset = 0;
    uint8_t i;
    
    // ========================================================================
    // 0x2023 - 驱动控制基础数据
    // ========================================================================
    offset = 0;
    // 先读取EEPROM中的校验和
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    // 与默认值的校验和对比
    if(eep_cks != DCtrlBasic_default.data_cks.U32t)
    {
        // 校验和不一致,将此块数据的所有默认值写入EEPROM
        const uint32_t* default_ptr = (const uint32_t*)&DCtrlBasic_default;
        for(i = 0; i < CTRL_BASIC_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        // 写入后更新RAM中的数据
        memcpy(&DCtrlBasic_data.st, &DCtrlBasic_default, sizeof(DCtrlBasic_Data_t));
    }
    else
    {
        // 校验和一致,将整块数据从EEPROM读出来使用
        for(i = 0; i < CTRL_BASIC_MAX; i++) {
            DCtrlBasic_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x2022 - 电机参数数据
    // ========================================================================
    offset = CTRL_BASIC_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != M_Par_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&M_Par_default;
        for(i = 0; i < M_PAR_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&M_Par_data.st, &M_Par_default, sizeof(M_Par_Data_t));
    }
    else
    {
        for(i = 0; i < M_PAR_MAX; i++) {
            M_Par_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x2021 - DMCU控制器参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_Ctrl_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_Ctrl_default;
        for(i = 0; i < CTRL_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_Ctrl_data.st, &DM_Ctrl_default, sizeof(DM_Ctrl_Data_t));
    }
    else
    {
        for(i = 0; i < CTRL_MAX; i++) {
            DM_Ctrl_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x2020 - DMCU速度控制参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_Spd_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_Spd_default;
        for(i = 0; i < SPD_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_Spd_data.st, &DM_Spd_default, sizeof(DM_Spd_Data_t));
    }
    else
    {
        for(i = 0; i < SPD_MAX; i++) {
            DM_Spd_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x201F - DMCU扭矩控制参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_Torque_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_Torque_default;
        for(i = 0; i < TORQUE_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_Torque_data.st, &DM_Torque_default, sizeof(DM_Torque_Data_t));
    }
    else
    {
        for(i = 0; i < TORQUE_MAX; i++) {
            DM_Torque_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x201E - DMCU增益参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_Gain_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_Gain_default;
        for(i = 0; i < GAIN_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_Gain_data.st, &DM_Gain_default, sizeof(DM_Gain_Data_t));
    }
    else
    {
        for(i = 0; i < GAIN_MAX; i++) {
            DM_Gain_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x201D - DMCU控制优化参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_CtrlOpt_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_CtrlOpt_default;
        for(i = 0; i < CTRLOPT_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_CtrlOpt_data.st, &DM_CtrlOpt_default, sizeof(DM_CtrlOpt_Data_t));
    }
    else
    {
        for(i = 0; i < CTRLOPT_MAX; i++) {
            DM_CtrlOpt_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x201C - DMCU叉车参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_Fork_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_Fork_default;
        for(i = 0; i < FORK_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_Fork_data.st, &DM_Fork_default, sizeof(DM_Fork_Data_t));
    }
    else
    {
        for(i = 0; i < FORK_MAX; i++) {
            DM_Fork_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x201B - DMCU通讯参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_Comm_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_Comm_default;
        for(i = 0; i < COMM_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_Comm_data.st, &DM_Comm_default, sizeof(DM_Comm_Data_t));
    }
    else
    {
        for(i = 0; i < COMM_MAX; i++) {
            DM_Comm_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x201A - DMCU保护参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX + COMM_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_Prot_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_Prot_default;
        for(i = 0; i < PROT_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_Prot_data.st, &DM_Prot_default, sizeof(DM_Prot_Data_t));
    }
    else
    {
        for(i = 0; i < PROT_MAX; i++) {
            DM_Prot_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
    
    // ========================================================================
    // 0x2019 - DMCU泄放参数
    // ========================================================================
    offset = CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX + COMM_MAX + PROT_MAX;
    eep_cks = SDO_SetData_R_EEPROM(base_addr + offset);
    
    if(eep_cks != DM_Dischg_default.data_cks.U32t)
    {
        const uint32_t* default_ptr = (const uint32_t*)&DM_Dischg_default;
        for(i = 0; i < DISCHG_MAX; i++) {
            SDO_SetData_W_EEPROM(base_addr + offset + i, default_ptr[i]);
        }
        memcpy(&DM_Dischg_data.st, &DM_Dischg_default, sizeof(DM_Dischg_Data_t));
    }
    else
    {
        for(i = 0; i < DISCHG_MAX; i++) {
            DM_Dischg_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
        }
    }
}

/**
 * @brief  读取DMCU参数
 */
uint32_t SDO_DMCU_GetData(uint16_t index, uint8_t subindex)
{
    Data_COB_32b* ptr = SDO_DMCU_GetDataPtr(index, subindex);
    if(ptr != NULL) {
        return ptr->U32t;
    }
    return 0;
}

/**
 * @brief  设置DMCU参数
 */
void SDO_DMCU_SetData(uint16_t index, uint8_t subindex, uint32_t value)
{
    Data_COB_32b* ptr = SDO_DMCU_GetDataPtr(index, subindex);
    if(ptr != NULL) {
        ptr->U32t = value;
    }
}

/**
 * @brief  DMCU参数写入EEPROM
 */
void SDO_DMCU_EEP_Write(uint16_t index, uint8_t subindex, uint32_t value)
{
    uint16_t eep_addr = SDO_DMCU_CalcEEPAddr(index, subindex);
    if(eep_addr != 0) {
        // TODO: 调用I2C EEPROM写入函数
         SDO_SetData_W_EEPROM(eep_addr, value);
    }
}

/**
 * @brief  从EEPROM读取DMCU参数组(整组读取)
 */
void SDO_DMCU_EEP_Read(uint16_t index)
{
    uint16_t base_addr = EEPROM_DMCU_BASE;
    uint16_t offset = 0;
    uint8_t i;
    
    switch(index)
    {
        case 0x2023: // DCtrlBasic_data
            offset = 0;
            for(i = 0; i < CTRL_BASIC_MAX; i++) {
                DCtrlBasic_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x2022: // M_Par_data
            offset = CTRL_BASIC_MAX;
            for(i = 0; i < M_PAR_MAX; i++) {
                M_Par_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x2021: // DM_Ctrl_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX);
            for(i = 0; i < CTRL_MAX; i++) {
                DM_Ctrl_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x2020: // DM_Spd_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX);
            for(i = 0; i < SPD_MAX; i++) {
                DM_Spd_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x201F: // DM_Torque_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX);
            for(i = 0; i < TORQUE_MAX; i++) {
                DM_Torque_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x201E: // DM_Gain_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX);
            for(i = 0; i < GAIN_MAX; i++) {
                DM_Gain_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x201D: // DM_CtrlOpt_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX);
            for(i = 0; i < CTRLOPT_MAX; i++) {
                DM_CtrlOpt_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x201C: // DM_Fork_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX);
            for(i = 0; i < FORK_MAX; i++) {
                DM_Fork_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x201B: // DM_Comm_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX);
            for(i = 0; i < COMM_MAX; i++) {
                DM_Comm_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x201A: // DM_Prot_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX + COMM_MAX);
            for(i = 0; i < PROT_MAX; i++) {
                DM_Prot_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        case 0x2019: // DM_Dischg_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX + COMM_MAX + PROT_MAX);
            for(i = 0; i < DISCHG_MAX; i++) {
                DM_Dischg_data.EEPROM_data[i] = SDO_SetData_R_EEPROM(base_addr + offset + i);
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief  将DMCU参数组整体写入EEPROM
 */
void SDO_DMCU_EEP_WriteGroup(uint16_t index)
{
    uint16_t base_addr = EEPROM_DMCU_BASE;
    uint16_t offset = 0;
    uint8_t i;
    
    switch(index)
    {
        case 0x2023: // DCtrlBasic_data
            offset = 0;
            for(i = 0; i < CTRL_BASIC_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DCtrlBasic_data.EEPROM_data[i]);
            }
            break;
            
        case 0x2022: // M_Par_data
            offset = CTRL_BASIC_MAX;
            for(i = 0; i < M_PAR_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, M_Par_data.EEPROM_data[i]);
            }
            break;
            
        case 0x2021: // DM_Ctrl_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX);
            for(i = 0; i < CTRL_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_Ctrl_data.EEPROM_data[i]);
            }
            break;
            
        case 0x2020: // DM_Spd_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX);
            for(i = 0; i < SPD_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_Spd_data.EEPROM_data[i]);
            }
            break;
            
        case 0x201F: // DM_Torque_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX);
            for(i = 0; i < TORQUE_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_Torque_data.EEPROM_data[i]);
            }
            break;
            
        case 0x201E: // DM_Gain_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX);
            for(i = 0; i < GAIN_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_Gain_data.EEPROM_data[i]);
            }
            break;
            
        case 0x201D: // DM_CtrlOpt_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX);
            for(i = 0; i < CTRLOPT_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_CtrlOpt_data.EEPROM_data[i]);
            }
            break;
            
        case 0x201C: // DM_Fork_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX);
            for(i = 0; i < FORK_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_Fork_data.EEPROM_data[i]);
            }
            break;
            
        case 0x201B: // DM_Comm_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX);
            for(i = 0; i < COMM_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_Comm_data.EEPROM_data[i]);
            }
            break;
            
        case 0x201A: // DM_Prot_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX + COMM_MAX);
            for(i = 0; i < PROT_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_Prot_data.EEPROM_data[i]);
            }
            break;
            
        case 0x2019: // DM_Dischg_data
            offset = (CTRL_BASIC_MAX + M_PAR_MAX + CTRL_MAX + SPD_MAX + TORQUE_MAX + GAIN_MAX + CTRLOPT_MAX + FORK_MAX + COMM_MAX + PROT_MAX);
            for(i = 0; i < DISCHG_MAX; i++) {
                SDO_SetData_W_EEPROM(base_addr + offset + i, DM_Dischg_data.EEPROM_data[i]);
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief  DMCU SDO消息处理
 * 注意：subindex=0对应校验和，允许读取但禁止写入
 */
void CanMsgSDO_DMCU_Handler(uint16_t index, uint8_t* data_set)
{
    Data_COB_64b msg_get_data;
    Data_COB_64b get_Req_data;
    uint8_t i;
    
    for(i = 0; i < 8; i++) {
        msg_get_data.U8t[i] = data_set[i];
        get_Req_data.U8t[i] = 0;
    }
    
    // 检查索引是否匹配
    if(msg_get_data.msg.index != index) {
        get_Req_data.msg.byte0 = 0x80;  // 错误响应
        get_Req_data.msg.index = msg_get_data.msg.index;
        get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
        IL_FillSendMessage0x582_SET(&get_Req_data.U8t[0]);
        return;
    }
    
    Data_COB_32b* data_ptr = SDO_DMCU_GetDataPtr(index, msg_get_data.msg.SUBindex);
    
    if(data_ptr == NULL) {
        get_Req_data.msg.byte0 = 0x80;  // 错误响应
        get_Req_data.msg.index = msg_get_data.msg.index;
        get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
        IL_FillSendMessage0x582_SET(&get_Req_data.U8t[0]);
        return;
    }
    
    switch(msg_get_data.msg.byte0)
    {
        case 0x23: // 写4字节
            // 禁止写入校验和(subindex=0)
            if(msg_get_data.msg.SUBindex == 0) {
                get_Req_data.msg.byte0 = 0x80;  // 错误响应
                break;
            }
            data_ptr->U8t[0] = msg_get_data.msg.data4.U8t[0];
            data_ptr->U8t[1] = msg_get_data.msg.data4.U8t[1];
            data_ptr->U8t[2] = msg_get_data.msg.data4.U8t[2];
            data_ptr->U8t[3] = msg_get_data.msg.data4.U8t[3];
            SDO_DMCU_EEP_Write(index, msg_get_data.msg.SUBindex, data_ptr->U32t);
            get_Req_data.msg.data4.U32t = data_ptr->U32t;
            get_Req_data.msg.byte0 = 0x60;
            break;
            
        case 0x27: // 写3字节
            // 禁止写入校验和(subindex=0)
            if(msg_get_data.msg.SUBindex == 0) {
                get_Req_data.msg.byte0 = 0x80;  // 错误响应
                break;
            }
            data_ptr->U8t[0] = msg_get_data.msg.data4.U8t[0];
            data_ptr->U8t[1] = msg_get_data.msg.data4.U8t[1];
            data_ptr->U8t[2] = msg_get_data.msg.data4.U8t[2];
            SDO_DMCU_EEP_Write(index, msg_get_data.msg.SUBindex, data_ptr->U32t);
            get_Req_data.msg.data4.U32t = data_ptr->U32t;
            get_Req_data.msg.byte0 = 0x60;
            break;
            
        case 0x2B: // 写2字节
            // 禁止写入校验和(subindex=0)
            if(msg_get_data.msg.SUBindex == 0) {
                get_Req_data.msg.byte0 = 0x80;  // 错误响应
                break;
            }
            data_ptr->U8t[0] = msg_get_data.msg.data4.U8t[0];
            data_ptr->U8t[1] = msg_get_data.msg.data4.U8t[1];
            SDO_DMCU_EEP_Write(index, msg_get_data.msg.SUBindex, data_ptr->U32t);
            get_Req_data.msg.data4.U32t = data_ptr->U32t;
            get_Req_data.msg.byte0 = 0x60;
            break;
            
        case 0x2F: // 写1字节
            // 禁止写入校验和(subindex=0)
            if(msg_get_data.msg.SUBindex == 0) {
                get_Req_data.msg.byte0 = 0x80;  // 错误响应
                break;
            }
            data_ptr->U8t[0] = msg_get_data.msg.data4.U8t[0];
            SDO_DMCU_EEP_Write(index, msg_get_data.msg.SUBindex, data_ptr->U32t);
            get_Req_data.msg.data4.U32t = data_ptr->U32t;
            get_Req_data.msg.byte0 = 0x60;
            break;
            
        case 0x40: // 读请求 - 允许读取校验和
            get_Req_data.msg.data4.U32t = data_ptr->U32t;
            get_Req_data.msg.byte0 = 0x4B;
            break;
            
        default:
            get_Req_data.msg.byte0 = 0x80;  // 错误响应
            break;
    }
    
    get_Req_data.msg.index = msg_get_data.msg.index;
    get_Req_data.msg.SUBindex = msg_get_data.msg.SUBindex;
    IL_FillSendMessage0x582_SET(&get_Req_data.U8t[0]);
}


// ============================================================================
// DMCU各组参数获取函数实现
// ============================================================================

// 0x2023 - 驱动控制基础数据获取函数
uint32_t DCtrlBasic_Get_CMD_Src(void) { return DCtrlBasic_data.COB_data[DCTRL_BASIC_CMD_SRC].U32t; }
uint32_t DCtrlBasic_Get_CMD_T_ON(void) { return DCtrlBasic_data.COB_data[DCTRL_BASIC_CMD_T_ON].U32t; }
uint32_t DCtrlBasic_Get_M_Rotation_Direction(void) { return DCtrlBasic_data.COB_data[DCTRL_BASIC_M_ROTATION_DIRECTION].U32t; }
uint32_t DCtrlBasic_Get_M_Ctr_mode(void) { return DCtrlBasic_data.COB_data[DCTRL_BASIC_M_CTR_MODE].U32t; }
uint32_t DCtrlBasic_Get_Spd_CMD(void) { return DCtrlBasic_data.COB_data[DCTRL_BASIC_SPD_CMD].U32t; }
uint32_t DCtrlBasic_Get_Torque_CMD(void) { return DCtrlBasic_data.COB_data[DCTRL_BASIC_TORQUE_CMD].U32t; }
uint32_t DCtrlBasic_Get_D_Axis_Current(void) { return DCtrlBasic_data.COB_data[DCTRL_BASIC_D_AXIS_CURRENT].U32t; }
uint32_t DCtrlBasic_Get_Q_Axis_Current(void) { return DCtrlBasic_data.COB_data[DCTRL_BASIC_Q_AXIS_CURRENT].U32t; }

// 0x2022 - 电机参数数据获取函数
uint32_t M_Par_Get_SN(void) { return M_Par_data.COB_data[M_PAR_SN].U32t; }
uint32_t M_Par_Get_Type(void) { return M_Par_data.COB_data[M_PAR_TYPE].U32t; }
uint32_t M_Par_Get_Negative_Sequence_Ctrl(void) { return M_Par_data.COB_data[M_PAR_NEGATIVE_SEQUENCE_CTRL].U32t; }
uint32_t M_Par_Get_Rated_Power(void) { return M_Par_data.COB_data[M_PAR_RATED_POWER].U32t; }
uint32_t M_Par_Get_Peak_Power(void) { return M_Par_data.COB_data[M_PAR_PEAK_POWER].U32t; }
uint32_t M_Par_Get_Rated_Spd(void) { return M_Par_data.COB_data[M_PAR_RATED_SPD].U32t; }
uint32_t M_Par_Get_Peak_Spd(void) { return M_Par_data.COB_data[M_PAR_PEAK_SPD].U32t; }
uint32_t M_Par_Get_Number_Of_Pole_Pairs(void) { return M_Par_data.COB_data[M_PAR_NUMBER_OF_POLE_PAIRS].U32t; }
uint32_t M_Par_Get_Rotor_Sensor_Pole_Pairs(void) { return M_Par_data.COB_data[M_PAR_ROTOR_SENSOR_POLE_PAIRS].U32t; }
uint32_t M_Par_Get_Rated_Vol(void) { return M_Par_data.COB_data[M_PAR_RATED_VOL].U32t; }
uint32_t M_Par_Get_Rated_Cur(void) { return M_Par_data.COB_data[M_PAR_RATED_CUR].U32t; }
uint32_t M_Par_Get_Peak_Cur(void) { return M_Par_data.COB_data[M_PAR_PEAK_CUR].U32t; }
uint32_t M_Par_Get_Peak_Torque(void) { return M_Par_data.COB_data[M_PAR_PEAK_TORQUE].U32t; }
uint32_t M_Par_Get_Resolver_Zero_Pos(void) { return M_Par_data.COB_data[M_PAR_RESOLVER_ZERO_POS].U32t; }
uint32_t M_Par_Get_Rs(void) { return M_Par_data.COB_data[M_PAR_RS].U32t; }
uint32_t M_Par_Get_D_Axis_Inductance(void) { return M_Par_data.COB_data[M_PAR_D_AXIS_INDUCTANCE].U32t; }
uint32_t M_Par_Get_Q_Axis_Inductance(void) { return M_Par_data.COB_data[M_PAR_Q_AXIS_INDUCTANCE].U32t; }
uint32_t M_Par_Get_Back_EMF_Constant(void) { return M_Par_data.COB_data[M_PAR_BACK_EMF_CONSTANT].U32t; }
uint32_t M_Par_Get_Field_Weakening_Cur(void) { return M_Par_data.COB_data[M_PAR_FIELD_WEAKENING_CUR].U32t; }
uint32_t M_Par_Get_Max_Feedback_Cur(void) { return M_Par_data.COB_data[M_PAR_MAX_FEEDBACK_CUR].U32t; }
uint32_t M_Par_Get_Init_Pos_Iden_Cur(void) { return M_Par_data.COB_data[M_PAR_INIT_POS_IDEN_CUR].U32t; }
uint32_t M_Par_Get_Stator_Res_Iden_Cur1(void) { return M_Par_data.COB_data[M_PAR_STATOR_RES_IDEN_CUR1].U32t; }
uint32_t M_Par_Get_Stator_Res_Iden_Cur2(void) { return M_Par_data.COB_data[M_PAR_STATOR_RES_IDEN_CUR2].U32t; }

// 0x2021 - DMCU控制器参数获取函数
uint32_t DM_Ctrl_Get_Rated_Power(void) { return DM_Ctrl_data.COB_data[DM_CTRL_RATED_POWER].U32t; }
uint32_t DM_Ctrl_Get_Rated_Voltage(void) { return DM_Ctrl_data.COB_data[DM_CTRL_RATED_VOLTAGE].U32t; }
uint32_t DM_Ctrl_Get_Rated_Current(void) { return DM_Ctrl_data.COB_data[DM_CTRL_RATED_CURRENT].U32t; }
uint32_t DM_Ctrl_Get_Peak_Current(void) { return DM_Ctrl_data.COB_data[DM_CTRL_PEAK_CURRENT].U32t; }
uint32_t DM_Ctrl_Get_Deadtime(void) { return DM_Ctrl_data.COB_data[DM_CTRL_DEADTIME].U32t; }
uint32_t DM_Ctrl_Get_Deadtime_Comp(void) { return DM_Ctrl_data.COB_data[DM_CTRL_DEADTIME_COMP].U32t; }
uint32_t DM_Ctrl_Get_BusVoltage_Cal(void) { return DM_Ctrl_data.COB_data[DM_CTRL_BUSVOLTAGE_CAL].U32t; }
uint32_t DM_Ctrl_Get_PhaseCurrent_Cal(void) { return DM_Ctrl_data.COB_data[DM_CTRL_PHASECURRENT_CAL].U32t; }
uint32_t DM_Ctrl_Get_ControlVoltage_Cal(void) { return DM_Ctrl_data.COB_data[DM_CTRL_CONTROLVOLTAGE_CAL].U32t; }

// 0x2020 - DMCU速度控制参数获取函数
uint32_t DM_Spd_Get_Accel_TimeConst(void) { return DM_Spd_data.COB_data[DM_SPD_ACCEL_TIMECONST].U32t; }
uint32_t DM_Spd_Get_Decel_TimeConst(void) { return DM_Spd_data.COB_data[DM_SPD_DECEL_TIMECONST].U32t; }
uint32_t DM_Spd_Get_Limit_Source(void) { return DM_Spd_data.COB_data[DM_SPD_LIMIT_SOURCE].U32t; }
uint32_t DM_Spd_Get_Pos_Limit(void) { return DM_Spd_data.COB_data[DM_SPD_POS_LIMIT].U32t; }
uint32_t DM_Spd_Get_Neg_Limit(void) { return DM_Spd_data.COB_data[DM_SPD_NEG_LIMIT].U32t; }

// 0x201F - DMCU扭矩控制参数获取函数
uint32_t DM_Torque_Get_Limit_Source(void) { return DM_Torque_data.COB_data[DM_TORQUE_LIMIT_SOURCE].U32t; }
uint32_t DM_Torque_Get_Motor_Limit(void) { return DM_Torque_data.COB_data[DM_TORQUE_MOTOR_LIMIT].U32t; }
uint32_t DM_Torque_Get_Feedback_Limit(void) { return DM_Torque_data.COB_data[DM_TORQUE_FEEDBACK_LIMIT].U32t; }
uint32_t DM_Torque_Get_Accel_Time(void) { return DM_Torque_data.COB_data[DM_TORQUE_ACCEL_TIME].U32t; }
uint32_t DM_Torque_Get_Decel_Time(void) { return DM_Torque_data.COB_data[DM_TORQUE_DECEL_TIME].U32t; }

// 0x201E - DMCU增益参数获取函数
uint32_t DM_Gain_Get_Spd_Kp1(void) { return DM_Gain_data.COB_data[DM_GAIN_SPD_KP1].U32t; }
uint32_t DM_Gain_Get_Spd_Ki1(void) { return DM_Gain_data.COB_data[DM_GAIN_SPD_KI1].U32t; }
uint32_t DM_Gain_Get_Spd_Param_Switch_LowSpeed(void) { return DM_Gain_data.COB_data[DM_GAIN_SPD_PARAM_SWITCH_LOWSPEED].U32t; }
uint32_t DM_Gain_Get_Spd_Kp2(void) { return DM_Gain_data.COB_data[DM_GAIN_SPD_KP2].U32t; }
uint32_t DM_Gain_Get_Spd_Ki2(void) { return DM_Gain_data.COB_data[DM_GAIN_SPD_KI2].U32t; }
uint32_t DM_Gain_Get_Spd_Param_Switch_HighSpeed(void) { return DM_Gain_data.COB_data[DM_GAIN_SPD_PARAM_SWITCH_HIGHSPEED].U32t; }
uint32_t DM_Gain_Get_Spd_Output_Filter(void) { return DM_Gain_data.COB_data[DM_GAIN_SPD_OUTPUT_FILTER].U32t; }
uint32_t DM_Gain_Get_Spd_Feedback_Filter(void) { return DM_Gain_data.COB_data[DM_GAIN_SPD_FEEDBACK_FILTER].U32t; }
uint32_t DM_Gain_Get_Cur_Gain_Switch_Select(void) { return DM_Gain_data.COB_data[DM_GAIN_CUR_GAIN_SWITCH_SELECT].U32t; }
uint32_t DM_Gain_Get_Cur_Gain_Switch_SpeedPoint(void) { return DM_Gain_data.COB_data[DM_GAIN_CUR_GAIN_SWITCH_SPEEDPOINT].U32t; }
uint32_t DM_Gain_Get_Cur_Gain_Switch_CurrentPoint(void) { return DM_Gain_data.COB_data[DM_GAIN_CUR_GAIN_SWITCH_CURRENTPOINT].U32t; }
uint32_t DM_Gain_Get_Cur_Gain_Switch_Hysteresis(void) { return DM_Gain_data.COB_data[DM_GAIN_CUR_GAIN_SWITCH_HYSTERESIS].U32t; }
uint32_t DM_Gain_Get_Dcur_KP1(void) { return DM_Gain_data.COB_data[DM_GAIN_DCUR_KP1].U32t; }
uint32_t DM_Gain_Get_Dcur_KI1(void) { return DM_Gain_data.COB_data[DM_GAIN_DCUR_KI1].U32t; }
uint32_t DM_Gain_Get_Dcur_KP2(void) { return DM_Gain_data.COB_data[DM_GAIN_DCUR_KP2].U32t; }
uint32_t DM_Gain_Get_Dcur_KI2(void) { return DM_Gain_data.COB_data[DM_GAIN_DCUR_KI2].U32t; }
uint32_t DM_Gain_Get_Qcur_KP1(void) { return DM_Gain_data.COB_data[DM_GAIN_QCUR_KP1].U32t; }
uint32_t DM_Gain_Get_Qcur_KI1(void) { return DM_Gain_data.COB_data[DM_GAIN_QCUR_KI1].U32t; }
uint32_t DM_Gain_Get_Qcur_KP2(void) { return DM_Gain_data.COB_data[DM_GAIN_QCUR_KP2].U32t; }
uint32_t DM_Gain_Get_Qcur_KI2(void) { return DM_Gain_data.COB_data[DM_GAIN_QCUR_KI2].U32t; }

// 0x201D - DMCU控制优化参数获取函数
uint32_t DM_CtrlOpt_Get_LimitEnable(void) { return DM_CtrlOpt_data.COB_data[DM_CTRLOPT_LIMITENABLE].U32t; }
uint32_t DM_CtrlOpt_Get_TorqueCompEnable(void) { return DM_CtrlOpt_data.COB_data[DM_CTRLOPT_TORQUECOMPENABLE].U32t; }
uint32_t DM_CtrlOpt_Get_IdenZeroOnly(void) { return DM_CtrlOpt_data.COB_data[DM_CTRLOPT_IDENZEROONLY].U32t; }
uint32_t DM_CtrlOpt_Get_CarrierFreq(void) { return DM_CtrlOpt_data.COB_data[DM_CTRLOPT_CARRIERFREQ].U32t; }

// 0x201C - DMCU叉车参数获取函数
uint32_t DM_Fork_Get_Spd_Mode_Enable(void) { return DM_Fork_data.COB_data[DM_FORK_SPD_MODE_ENABLE].U32t; }

// 0x201B - DMCU通讯参数获取函数
uint32_t DM_Comm_Get_CAN1_Baud(void) { return DM_Comm_data.COB_data[DM_COMM_CAN1_BAUD].U32t; }
uint32_t DM_Comm_Get_CAN2_Baud(void) { return DM_Comm_data.COB_data[DM_COMM_CAN2_BAUD].U32t; }

// 0x201A - DMCU保护参数获取函数
uint32_t DM_Prot_Get_CurOffset_Thres(void) { return DM_Prot_data.COB_data[DM_PROT_CUROFFSET_THRES].U32t; }
uint32_t DM_Prot_Get_Motor_OverTemp(void) { return DM_Prot_data.COB_data[DM_PROT_MOTOR_OVERTEMP].U32t; }
uint32_t DM_Prot_Get_Motor_OverTemp_Warn(void) { return DM_Prot_data.COB_data[DM_PROT_MOTOR_OVERTEMP_WARN].U32t; }
uint32_t DM_Prot_Get_Ctrl_OverTemp(void) { return DM_Prot_data.COB_data[DM_PROT_CTRL_OVERTEMP].U32t; }
uint32_t DM_Prot_Get_Ctrl_OverTemp_Warn(void) { return DM_Prot_data.COB_data[DM_PROT_CTRL_OVERTEMP_WARN].U32t; }
uint32_t DM_Prot_Get_Motor_Pos_OverSpeed(void) { return DM_Prot_data.COB_data[DM_PROT_MOTOR_POS_OVERSPEED].U32t; }
uint32_t DM_Prot_Get_Motor_Neg_OverSpeed(void) { return DM_Prot_data.COB_data[DM_PROT_MOTOR_NEG_OVERSPEED].U32t; }
uint32_t DM_Prot_Get_Instant_OC_Thres(void) { return DM_Prot_data.COB_data[DM_PROT_INSTANT_OC_THRES].U32t; }
uint32_t DM_Prot_Get_Filter_OC_Thres(void) { return DM_Prot_data.COB_data[DM_PROT_FILTER_OC_THRES].U32t; }
uint32_t DM_Prot_Get_PwrOn_Volt_Hyst(void) { return DM_Prot_data.COB_data[DM_PROT_PWRON_VOLT_HYST].U32t; }
uint32_t DM_Prot_Get_LowVolt_Under(void) { return DM_Prot_data.COB_data[DM_PROT_LOWVOLT_UNDER].U32t; }
uint32_t DM_Prot_Get_LowVolt_Over(void) { return DM_Prot_data.COB_data[DM_PROT_LOWVOLT_OVER].U32t; }
uint32_t DM_Prot_Get_HighVolt_Under(void) { return DM_Prot_data.COB_data[DM_PROT_HIGHVOLT_UNDER].U32t; }
uint32_t DM_Prot_Get_HighVolt_Over(void) { return DM_Prot_data.COB_data[DM_PROT_HIGHVOLT_OVER].U32t; }
uint32_t DM_Prot_Get_Locked_Rotor_Freq(void) { return DM_Prot_data.COB_data[DM_PROT_LOCKED_ROTOR_FREQ].U32t; }
uint32_t DM_Prot_Get_Stall_Torque(void) { return DM_Prot_data.COB_data[DM_PROT_STALL_TORQUE].U32t; }
uint32_t DM_Prot_Get_Stall_Time(void) { return DM_Prot_data.COB_data[DM_PROT_STALL_TIME].U32t; }
uint32_t DM_Prot_Get_Stall_Safe_Torque(void) { return DM_Prot_data.COB_data[DM_PROT_STALL_SAFE_TORQUE].U32t; }

// 0x2019 - DMCU泄放参数获取函数
uint32_t DM_Dischg_Get_Safe_Vol(void) { return DM_Dischg_data.COB_data[DM_DISCHG_SAFE_VOL].U32t; }
uint32_t DM_Dischg_Get_Timeout(void) { return DM_Dischg_data.COB_data[DM_DISCHG_TIMEOUT].U32t; }
