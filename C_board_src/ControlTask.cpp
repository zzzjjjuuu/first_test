#include "ControlTask.h"
#include "Gm6020.h"
#include "Pid.h"
#include "main.h"   // 提供 HAL 库基础类型定义
#include "can.h"    // 提供 hcan1 声明
#include "tim.h"    // 提供 htim6 声明
#include  <cmath>   // 提供 std::sin
#include  <cstdint>

namespace {
constexpr float kFreq  = 1.0f;           // 正弦频率 1Hz
constexpr float kAmp   = 3.14159265f;    // 幅值 ±π rad
constexpr float kDt    = 0.001f;         // 1ms (1kHz) 控制周期
constexpr float kTwoPi = 2.0f * 3.14159265358979323846f;
}

volatile float g_out_current = 0.0f;
// 1. 硬件实体与算法实例
static Gm6020 g_motor(1); // 电机拨码 ID 设为 1

// 外环位置 PID: 误差(rad) -> 目标角速度限幅(±30 rad/s)
static Pid g_pos_pid(
    /*kp=*/150.0f, /*ki=*/0.0f, /*kd=*/0.0f,
    /*out_max=*/30.0f, /*out_min=*/-30.0f
);

// 内环速度 PID: 误差(rad/s) -> 电流给定限幅(±16384, 对应 ±3A)
static Pid g_vel_pid(
    /*kp=*/120.0f, /*ki=*/1.5f, /*kd=*/0.0f,
    /*out_max=*/16384.0f, /*out_min=*/-16384.0f
);

// 2. 观测监视变量 (可在 Ozone 中画图)
volatile float g_target_angle = 0.0f;
volatile float g_actual_angle = 0.0f;
volatile float g_target_vel   = 0.0f;
volatile float g_actual_vel   = 0.0f;
static float   g_time_s       = 0.0f;

// 3. 上电初始化接口
void ControlTaskInit(void) {
    // 配置 CAN 过滤器放行所有反馈
    CAN_FilterTypeDef filter;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &filter);

    // 启动 CAN1 及接收中断
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 启动 TIM6 1kHz 周期中断
    HAL_TIM_Base_Start_IT(&htim6);
}

// 4. 1kHz 控制节拍执行体
void MainTask(void) {
    // 步骤 A: 产生正弦波位置期望
    g_time_s += kDt;
    g_target_angle = kAmp * std::sin(kTwoPi * kFreq * g_time_s);

    // 步骤 B: 采样电机物理反馈
    g_actual_angle = g_motor.angle();
    g_actual_vel   = g_motor.vel();

    // 步骤 C: 双环串级计算
    g_target_vel = g_pos_pid.calc(g_target_angle, g_actual_angle);
    float out_current = g_vel_pid.calc(g_target_vel, g_actual_vel);

    // 步骤 D: 下发电流给定到电机类对象
    g_out_current = out_current;
    g_motor.setCurrent(static_cast<int16_t>(out_current));

    // 步骤 E: 打包并通过 CAN1 发送
    uint8_t tx_data[8] = {0};
    g_motor.encode(tx_data);

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId              = g_motor.txId(); // 0x1FE
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox);
}

// 5. CAN 接收中断回调
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id == g_motor.rxId()) {
        g_motor.decode(data);
    }
}