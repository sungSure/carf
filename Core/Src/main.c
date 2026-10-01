/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 电赛小车 —— 全流程状态机(验收版)
  ******************************************************************************
  * @attention
  *
  * 流程: 待命 → 启动(上电延时自启 / PA15声音 / 蓝牙'G') → 直行20cm → 停车计时5s → 蜂鸣
  *       → 第1圈自动循迹(雷达扫描+点云图+Y路口+避障) → 回起点停车
  *       → 第2圈手动遥控(蓝牙) → 数据显示 → (拓展)逆向循迹返程
  *
  * 架构: 主循环无阻塞, 全部基于 HAL_GetTick() 时间片调度
  *       电机统一接口 Motor_Set(左,右) ∈ [-199,199]
  *       循迹/雷达/避障/蜂鸣/OLED 各自独立任务函数
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "oled.h"
#include "font.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ============ 硬件接线(与 carf.ioc / configs.txt 一致) ============ *
 * 电机: TB6612  PWMA=TIM1_CH1(PA8) 左轮  PWMB=TIM1_CH2(PA9) 右轮
 *       AIN1/2=PB12/13  BIN1/2=PB14/15  STBY=3.3V
 * 红外: PA2(L2) PA3(L1) PA4(R1) PA5(R2) 左→右, 压黑线输出低电平
 * 超声: Trig=PB5  Echo=PB6(TIM4_CH1 输入捕获, 1us计数)
 * 舵机: TIM3_CH1(PA6) 50Hz, 0.5~2.5ms 对应 0~180°
 * 蜂鸣: PA12 低电平响
 * 蓝牙: USART3 PB10/11 9600
 * 声音: DO→PA15(模块未到货, 暂关)
 * ============ 蓝牙指令表(单字符, 需BT_ENABLE=1; 当前=0模块未接) ============ *
 *  G: 启动(声音关闭时的启动信号)   L/R: Y路口选左/右(默认左)
 *  M: 自动圈结束后进入手动圈       B: 完成后触发逆向返程(拓展)
 *  W/A/S/D/X: 手动圈 遥控 前/左/后/右/停
 *  U/J: 循迹基础速度 ±10(现场免重烧调参)
 * ==================================== */

/* ---------- 启动方式(现场可改) ----------
 * SOUND_ENABLE=1  : PA15声音触发(验收要求, 模块到货后改1)
 * AUTO_START_MS>0 : 上电延时自启(当前方案: 无声音模块无蓝牙)
 * 两者都关        : 等蓝牙'G'启动(需BT_ENABLE=1)
 * BT_ENABLE=0     : 蓝牙未接, 不挂串口接收(防悬空RX收到噪声指令);
 *                   接上模块后改1                                     */
#define SOUND_ENABLE      0
#define SOUND_ACTIVE      0        /* 检测到声音时DO电平(拿到模块实测后改) */
#define AUTO_START_MS     3000u    /* 上电到自启延时ms, 0=禁用; 留时间摆车 */
#define BT_ENABLE         0        /* 蓝牙模块接上后改1 */

/* ---------- 调试开关 ---------- */
#define TRACK_DBG         1        /* 1=调试模式: 延时后直接纯红外循迹, 跳过全部流程/
                                        雷达/避障/圈计时, OLED显示传感器状态; 调完改0 */

/* ---------- 电机 ---------- */
#define PWM_MAX           199      /* TIM1 ARR, PWM满占空比 */

/* ---------- 基本动作标定(无编码器, 全部时间法, 现场调) ---------- */
#define RUN20_PWM         130      /* 直行20cm的占空比 */
#define RUN20_MS          1200u    /* 直行20cm的时长, 现场标定 */
#define TURN90_MS         520u     /* 原地转约90°的时长 */
#define Y_TURN_MS         550u     /* Y路口转向时长 */
#define AVOID_FWD_MS      750u     /* 避障侧移前进时长 */
#define SPIN_PWM          150      /* 原地转占空比 */

/* ---------- 循迹 ---------- */
#define TRACK_BASE        115      /* 循迹基础巡航占空比(U/J可在线调) */
#define TRACK_K           40       /* 转向增益: 轮速 = base ± K*err */
#define LOST_LINE_MS      2500u    /* 丢线保护时间 */

/* ---------- 线事件判定 ---------- */
#define LINE_MARK_MS       60u      /* 全黑持续<此值 → 横线(起点/终点) */
#define JUNCTION_MS       200u     /* 全黑持续≥此值 → 宽路口(Y) */
#define MIN_LAP_MS        15000u   /* 圈内最早接受"终点"的时间 */

/* ---------- 避障 ---------- */
#define AVOID_TRIGGER_CM  40.0f    /* 前方距离阈值(含雷达刷新延迟余量) */
#define GLANCE_SETTLE_MS  250u     /* 舵机偏转稳定时间 */
#define REACQ_MS          1500u    /* 避障/路口后重新找线时限 */

/* ---------- 雷达 ---------- */
#define RADAR_SETTLE_MS   180u     /* 扫描点间舵机稳定时间 */
#define RADAR_MAX_CM      90.0f    /* 点云图量程 */

/* ---------- 数据 ---------- */
#define TRACK_LEN_CM      0        /* 赛道一圈长度cm, 现场量好后填, 0=不显示圈均速 */

/* ---------- 超声波 ---------- */
#define US_TIMEOUT_MS     30u      /* 单次测距超时(无回波=远) */

/* ---------- OLED ---------- */
#define UI_PERIOD_MS      250u     /* 屏幕刷新周期 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 压黑线时红外DO输出低电平 → 1=在黑线上 */
#define IR_ON_LINE        GPIO_PIN_RESET
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* ---------- 状态机 ---------- */
typedef enum {
    ST_STANDBY,       /* 待命等启动信号 */
    ST_RUN20,         /* 直行20cm */
    ST_WAIT5,         /* 停车计时5s */
    ST_LAP1,          /* 自动圈: 循迹+雷达+避障+Y路口 */
    ST_LAP1_DONE,     /* 自动圈完成, 等M/B指令 */
    ST_MANUAL,        /* 手动遥控圈 */
    ST_FINISHED,      /* 全部完成, 显示数据 */
    ST_REVERSE        /* 拓展: 逆向循迹返程 */
} CarState;

/* 避障子流程 */
typedef enum {
    AV_IDLE,
    AV_STOP,
    AV_GL_L_SETTLE, AV_GL_L_MEAS,
    AV_GL_R_SETTLE, AV_GL_R_MEAS,
    AV_TURN_AWAY,
    AV_FWD,
    AV_TURN_BACK,
    AV_REACQ
} AvoidStep;

/* Y路口子流程 */
typedef enum { YS_IDLE, YS_TURN, YS_REACQ } YStep;

static CarState  state = ST_STANDBY;
static uint32_t  t_state;            /* 进入当前状态的时刻 */
static uint32_t  lap1_ms = 0, lap2_ms = 0, rev_ms = 0;
static float     speed_calib = 0;    /* 20cm标定速度 cm/s */

static int16_t   base_speed = TRACK_BASE;
static uint8_t   y_choice_left = 1;  /* Y路口: 1=左 0=右 */
static uint8_t   y_fired = 0;        /* 本圈Y是否已处理 */
static uint8_t   lost_err = 0;       /* 丢线保护触发标志 */

/* ---------- 红外循迹 ---------- */
static uint8_t   g_ir;               /* 本轮采样 bit3..0 = L2 L1 R1 R2, 1=压线 */
static int8_t    last_err = 0;       /* 记忆上次偏差(丢线找回用) */
static uint32_t  t_lost = 0;

/* 线事件(全黑起止分类) */
static struct {
    uint8_t  in_black;
    uint8_t  fired_junction;
    uint32_t t_start;
} le;

/* ---------- 避障 / Y ---------- */
static AvoidStep av = AV_IDLE;
static uint32_t   av_t;
static float      av_dl, av_dr;
static YStep      ys = YS_IDLE;
static uint32_t   ys_t;

/* ---------- 超声波 ---------- */
static volatile uint8_t  us_done = 0;
static volatile uint32_t us_width = 0;

/* ---------- 雷达 ---------- */
/* 扫描序列: 每2个点插一次90°正前方, 保证避障检测刷新率 */
static const uint8_t radar_seq[] = {
    0, 90, 15, 90, 30, 90, 45, 90, 60, 90, 75, 90,
    90, 90, 105, 90, 120, 90, 135, 90, 150, 90, 165, 90, 180, 90
};
#define RADAR_SEQ_N   (sizeof(radar_seq) / sizeof(radar_seq[0]))
static uint8_t   radar_pos = 0;     /* 序列索引 */
static uint8_t   radar_stage = 0;   /* 0=舵机稳定期 1=测距期 */
static uint32_t  radar_t = 0;
static float     radar_dist[13];   /* 0°..180° 每15°一个点, 0=无目标 */
static uint8_t   radar_on = 0;

/* ---------- 蜂鸣(非阻塞) ---------- */
static const uint16_t *beep_pat = 0; /* on,off,on,...,0结束 */
static uint8_t   beep_idx = 0;
static uint32_t  beep_t = 0;
static uint8_t   beep_level = 0;
static const uint16_t P_LONG[] = {700, 0};                       /* 长鸣 */
static const uint16_t P_LAP[]  = {120, 80, 120, 0};              /* 两短 */
static const uint16_t P_DONE[] = {400, 150, 400, 0};             /* 两长 */
static const uint16_t P_ERR[]  = {80, 80, 80, 80, 80, 0};         /* 五短 */

/* ---------- 蓝牙 ---------- */
static volatile uint8_t bt_flag = 0;
static volatile uint8_t bt_data = 0;

/* ---------- OLED ---------- */
static uint32_t ui_t = 0;
static uint8_t  ui_init = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Goto(CarState s);
static void Motor_Set(int16_t l, int16_t r);
static void Motor_Stop(void);
static void Buzzer_Set(uint8_t on);
static void Buzzer_Beep(const uint16_t *pat);
static void Buzzer_Task(void);
static void US_Start(void);
static uint8_t US_Poll(float *cm);
static void Servo_SetAngle(uint8_t angle);
static void Radar_Enable(void);
static void Radar_Disable(void);
static void Radar_Task(void);
static float Front_Min(void);
static void LineEvt_Reset(void);
static uint8_t LineEvt_Task(void);   /* 返回: 0无 1横线 2宽路口 */
static void Track_Drive(uint8_t reverse);
static void Avoid_Task(void);
static void Y_Task(void);
static void BT_Process(uint8_t c);
static void UI_Task(void);
static void Main_Loop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================= 状态切换 ================= */
static void Goto(CarState s)
{
    state  = s;
    t_state = HAL_GetTick();
}

/* ================= 电机: 统一符号接口 ================= */
/* 左轮A: IN1/IN2(PB12/13)+TIM1_CH1   右轮B: IN3/IN4(PB14/15)+TIM1_CH2 */
static void Motor_Set(int16_t l, int16_t r)
{
    if (l >  PWM_MAX) l =  PWM_MAX;
    if (l < -PWM_MAX) l = -PWM_MAX;
    if (r >  PWM_MAX) r =  PWM_MAX;
    if (r < -PWM_MAX) r = -PWM_MAX;

    if (l > 0)      { HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET); }
    else if (l < 0) { HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET); }
    else            { HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET); }

    if (r > 0)      { HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET); }
    else if (r < 0) { HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_SET); }
    else            { HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET); }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(l < 0 ? -l : l));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(r < 0 ? -r : r));
}

static void Motor_Stop(void) { Motor_Set(0, 0); }

/* ================= 蜂鸣器(PA12 低电平响) ================= */
static void Buzzer_Set(uint8_t on)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Buzzer_Beep(const uint16_t *pat)
{
    beep_pat = pat;
    beep_idx = 0;
    beep_t   = HAL_GetTick();
    beep_level = 1;
    Buzzer_Set(1);
}

static void Buzzer_Task(void)
{
    if (beep_pat == 0) return;
    uint32_t now = HAL_GetTick();
    if (now - beep_t < beep_pat[beep_idx]) return;
    beep_t += beep_pat[beep_idx];
    beep_idx++;
    if (beep_pat[beep_idx] == 0) {       /* 序列结束 */
        Buzzer_Set(0);
        beep_pat = 0;
    } else {
        beep_level = !beep_level;
        Buzzer_Set(beep_level);
    }
}

/* ================= 超声波 HC-SR04 ================= */
/* 前置: TIM4_CH1 1us计数, 输入捕获中断已在初始化时开启 */
static void US_Start(void)
{
    us_done  = 0;
    us_width = 0;
    /* 回到上升沿捕获并清标志, 然后给触发脉冲(>10us) */
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
    for (volatile uint16_t i = 0; i < 500; i++) __NOP();   /* ~35us@72MHz */
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
}

static uint8_t US_Poll(float *cm)
{
    if (!us_done) return 0;
    uint32_t w = us_width;
    if (w > 30000) { *cm = 999; }                 /* 回波异常长 → 视为无目标 */
    else           { *cm = (float)w * 0.017f; }    /* us → cm (声速340,往返) */
    return 1;
}

/* ================= 舵机(50Hz, 0.5~2.5ms → 0~180°) ================= */
static void Servo_SetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint32_t pulse_us = 500 + (uint32_t)angle * 2000 / 180;
    if (pulse_us > 19999) pulse_us = 19999;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
}

/* ================= 雷达扫描 + 点云数据 ================= */
static void Radar_Enable(void)
{
    radar_on   = 1;
    radar_pos  = 0;
    radar_stage = 0;
    radar_t    = HAL_GetTick();
    for (uint8_t i = 0; i < 13; i++) radar_dist[i] = 0;
    Servo_SetAngle(radar_seq[0]);
}

static void Radar_Disable(void)
{
    radar_on = 0;
    Servo_SetAngle(90);   /* 回正前方 */
}

static void Radar_Task(void)
{
    if (!radar_on) return;
    uint32_t now = HAL_GetTick();

    if (radar_stage == 0) {                       /* 舵机稳定期 */
        if (now - radar_t >= RADAR_SETTLE_MS) {
            US_Start();
            radar_stage = 1;
            radar_t = now;
        }
    } else {                                     /* 测距期 */
        float cm;
        uint8_t ang = radar_seq[radar_pos];
        if (US_Poll(&cm) || now - radar_t > US_TIMEOUT_MS) {
            radar_dist[ang / 15] = (cm > 1 && cm < RADAR_MAX_CM) ? cm : 0;
            radar_pos = (radar_pos + 1) % RADAR_SEQ_N;
            radar_stage = 0;
            radar_t = now;
            Servo_SetAngle(radar_seq[radar_pos]);
        }
    }
}

/* 正前方±30°(60~120°)最近距离 */
static float Front_Min(void)
{
    float m = 999;
    for (uint8_t i = 4; i <= 8; i++)
        if (radar_dist[i] > 0 && radar_dist[i] < m) m = radar_dist[i];
    return m;
}

/* ================= 红外采样与线事件 ================= */
static uint8_t IR_Read(void)
{
    uint8_t v = 0;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == IR_ON_LINE) v |= 0x8;  /* L2 */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == IR_ON_LINE) v |= 0x4;  /* L1 */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == IR_ON_LINE) v |= 0x2;  /* R1 */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == IR_ON_LINE) v |= 0x1;  /* R2 */
    return v;
}

static void LineEvt_Reset(void)
{
    le.in_black = 0;
    le.fired_junction = 0;
    le.t_start = 0;
}

/* 全黑起止分类: 短黑=横线(1), 持续黑=宽路口(2) */
static uint8_t LineEvt_Task(void)
{
    uint32_t now = HAL_GetTick();
    if (g_ir == 0x0F) {
        if (!le.in_black) { le.in_black = 1; le.t_start = now; le.fired_junction = 0; }
        else if (!le.fired_junction && now - le.t_start >= JUNCTION_MS) {
            le.fired_junction = 1;
            return 2;
        }
    } else if (le.in_black) {
        le.in_black = 0;
        if (!le.fired_junction) return 1;   /* 短全黑结束 → 横线 */
    }
    return 0;
}

/* ================= 循迹差速驱动 ================= */
/* 加权: L2=-3 L1=-1 R1=+1 R2=+3; err>0线在右侧 → 左轮加速右转 */
static void Track_Drive(uint8_t reverse)
{
    int16_t l, r;

    if (g_ir == 0x0F) {            /* 全黑(横线/路口): 事件机负责判定, 车直行通过 */
        l = base_speed; r = base_speed;
    } else if (g_ir == 0) {         /* 丢线: 按记忆方向加强转 */
        int8_t e = (last_err >= 0) ? 4 : -4;
        l = base_speed + TRACK_K * e;
        r = base_speed - TRACK_K * e;
    } else {
        int8_t e = 0;
        if (g_ir & 0x8) e -= 3;
        if (g_ir & 0x4) e -= 1;
        if (g_ir & 0x2) e += 1;
        if (g_ir & 0x1) e += 3;
        last_err = e;
        l = base_speed + TRACK_K * e;
        r = base_speed - TRACK_K * e;
    }

    /* 倒车循迹: 转向物理方向随行进方向反转, 公式取负即可 */
    if (reverse) { l = -l; r = -r; }
    Motor_Set(l, r);
}

/* ================= 避障子流程 ================= */
/* 触发: 前方距离 < 阈值; 动作: 停车→左右张望→往空侧转90°→侧移→转回→找线 */
static void Avoid_Task(void)
{
    uint32_t now = HAL_GetTick();
    float cm;

    switch (av) {
    case AV_IDLE:
        return;

    case AV_STOP:                                   /* 刹车缓冲 */
        if (now - av_t >= 200) {
            Servo_SetAngle(150);                    /* 张望左(150°) */
            av = AV_GL_L_SETTLE; av_t = now;
        }
        break;

    case AV_GL_L_SETTLE:
        if (now - av_t >= GLANCE_SETTLE_MS) { US_Start(); av = AV_GL_L_MEAS; av_t = now; }
        break;
    case AV_GL_L_MEAS:
        if (US_Poll(&cm)) { av_dl = (cm > 1 && cm < 300) ? cm : 300; }
        else if (now - av_t > US_TIMEOUT_MS) { av_dl = 300; }
        else break;
        Servo_SetAngle(30);                         /* 张望右(30°) */
        av = AV_GL_R_SETTLE; av_t = now;
        break;

    case AV_GL_R_SETTLE:
        if (now - av_t >= GLANCE_SETTLE_MS) { US_Start(); av = AV_GL_R_MEAS; av_t = now; }
        break;
    case AV_GL_R_MEAS:
        if (US_Poll(&cm)) { av_dr = (cm > 1 && cm < 300) ? cm : 300; }
        else if (now - av_t > US_TIMEOUT_MS) { av_dr = 300; }
        else break;
        /* 往空间大的一侧绕: 左侧空→左转 */
        if (av_dl >= av_dr) Motor_Set(-SPIN_PWM,  SPIN_PWM);
        else                Motor_Set( SPIN_PWM, -SPIN_PWM);
        av = AV_TURN_AWAY; av_t = now;
        break;

    case AV_TURN_AWAY:                              /* 转离原航线 ~90° */
        if (now - av_t >= TURN90_MS) {
            Motor_Set(base_speed, base_speed);
            av = AV_FWD; av_t = now;
        }
        break;

    case AV_FWD:                                    /* 侧移越过障碍 */
        if (now - av_t >= AVOID_FWD_MS) {
            if (av_dl >= av_dr) Motor_Set( SPIN_PWM, -SPIN_PWM);
            else                Motor_Set(-SPIN_PWM,  SPIN_PWM);
            av = AV_TURN_BACK; av_t = now;
        }
        break;

    case AV_TURN_BACK:                              /* 转回原方向 */
        if (now - av_t >= TURN90_MS) {
            Motor_Set(base_speed * 7 / 10, base_speed * 7 / 10);
            av = AV_REACQ; av_t = now;
        }
        break;

    case AV_REACQ:                                  /* 低速找回黑线 */
        if (g_ir != 0 && g_ir != 0x0F) {           /* 找到线 */
            av = AV_IDLE;
            Radar_Enable();                        /* 雷达继续扫 */
            LineEvt_Reset();
        } else if (now - av_t >= REACQ_MS) {        /* 超时也恢复循迹(靠记忆找回) */
            av = AV_IDLE;
            Radar_Enable();
            LineEvt_Reset();
        }
        break;
    }
}

/* ================= Y路口子流程 ================= */
static void Y_Task(void)
{
    uint32_t now = HAL_GetTick();
    switch (ys) {
    case YS_IDLE:
        return;
    case YS_TURN:                                   /* 向选定分支差速转向 */
        if (now - ys_t >= Y_TURN_MS) {
            Motor_Set(base_speed * 7 / 10, base_speed * 7 / 10);
            ys = YS_REACQ; ys_t = now;
        }
        break;
    case YS_REACQ:                                  /* 低速找回分支线 */
        if (g_ir != 0 && g_ir != 0x0F) { ys = YS_IDLE; }
        else if (now - ys_t >= REACQ_MS)  { ys = YS_IDLE; }
        break;
    }
}

/* ================= 蓝牙指令处理 ================= */
static void BT_Process(uint8_t c)
{
    switch (c) {
    /* --- 全局: 参数在线调整 --- */
    case 'U':                                       /* 循迹速度+10 */
        base_speed += 10;
        if (base_speed > PWM_MAX - TRACK_K * 6) base_speed = PWM_MAX - TRACK_K * 6;
        break;
    case 'J':                                       /* 循迹速度-10 */
        base_speed -= 10;
        if (base_speed < 60) base_speed = 60;
        break;
    case 'L': y_choice_left = 1; break;             /* Y路口选左 */
    case 'R': y_choice_left = 0; break;             /* Y路口选右 */

    /* --- 启动 --- */
    case 'G':
        if (state == ST_STANDBY) {
            Motor_Set(RUN20_PWM, RUN20_PWM);
            Goto(ST_RUN20);
        }
        break;

    /* --- 圈切换 --- */
    case 'M':
        if (state == ST_LAP1_DONE) {
            LineEvt_Reset();
            Goto(ST_MANUAL);
        }
        break;
    case 'B':
        if (state == ST_LAP1_DONE || state == ST_FINISHED) {
            LineEvt_Reset();
            ys = YS_IDLE; av = AV_IDLE;
            Radar_Disable();
            Goto(ST_REVERSE);
        }
        break;

    /* --- 手动遥控(仅手动圈生效) --- */
    case 'W': if (state == ST_MANUAL) Motor_Set( base_speed,  base_speed); break;
    case 'S': if (state == ST_MANUAL) Motor_Set(-base_speed, -base_speed); break;
    case 'A': if (state == ST_MANUAL) Motor_Set(-SPIN_PWM,  SPIN_PWM);      break;
    case 'D': if (state == ST_MANUAL) Motor_Set( SPIN_PWM, -SPIN_PWM);     break;
    case 'X': if (state == ST_MANUAL) Motor_Stop();                        break;

    default: break;
    }
}

/* ================= OLED 显示 ================= */
static void UI_Task(void)
{
    uint32_t now = HAL_GetTick();
    if (now - ui_t < UI_PERIOD_MS) return;
    ui_t = now;
    if (!ui_init) return;

    char buf[24];
    OLED_NewFrame();

    switch (state) {
    case ST_STANDBY: {
        OLED_PrintASCIIString(16, 0, "== CAR READY ==", &afont12x6, OLED_COLOR_NORMAL);
#if SOUND_ENABLE
        OLED_PrintASCIIString(16, 20, "WAIT SOUND ...", &afont12x6, OLED_COLOR_NORMAL);
#elif AUTO_START_MS > 0
        uint32_t left_ms = (now - t_state >= AUTO_START_MS) ? 0
                                                       : (AUTO_START_MS - (now - t_state));
        sprintf(buf, "START IN: %us", (unsigned)((left_ms + 999u) / 1000u));
        OLED_PrintASCIIString(13, 20, buf, &afont12x6, OLED_COLOR_NORMAL);
#else
        OLED_PrintASCIIString(10, 20, "BT 'G' TO START", &afont12x6, OLED_COLOR_NORMAL);
#endif
        sprintf(buf, "Y-%s  SPD:%d", y_choice_left ? "L" : "R", base_speed);
        OLED_PrintASCIIString(10, 40, buf, &afont12x6, OLED_COLOR_NORMAL);
        break;
    }

    case ST_RUN20:
        OLED_PrintASCIIString(28, 8, "RUN 20 CM", &afont16x8, OLED_COLOR_NORMAL);
        break;

    case ST_WAIT5: {
        uint32_t left = 5 - (now - t_state) / 1000;
        OLED_PrintASCIIString(28, 8, "WAIT 5S", &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "COUNTDOWN: %u", (unsigned)left);
        OLED_PrintASCIIString(16, 32, buf, &afont12x6, OLED_COLOR_NORMAL);
        break;
    }

    case ST_LAP1: {
#if TRACK_DBG
        /* 调试页: 传感器方块(实心=压线) + 偏差 + 速度 */
        OLED_PrintASCIIString(16, 0, "TRACK DEBUG", &afont16x8, OLED_COLOR_NORMAL);
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t x = 14 + i * 26;
            if ((g_ir >> (3 - i)) & 1) OLED_DrawFilledRectangle(x, 22, 24, 20, OLED_COLOR_NORMAL);
            else                       OLED_DrawRectangle(x, 22, 24, 20, OLED_COLOR_NORMAL);
        }
        OLED_PrintASCIIString(20, 44, "L2", &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(46, 44, "L1", &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(72, 44, "R1", &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(98, 44, "R2", &afont8x6, OLED_COLOR_NORMAL);
        sprintf(buf, "E:%d  SPD:%d  %s", last_err, base_speed, lost_err ? "LOST" : "RUN");
        OLED_PrintASCIIString(4, 54, buf, &afont8x6, OLED_COLOR_NORMAL);
        break;
#endif
        /* 顶部信息行 */
        sprintf(buf, "T:%lu.%01lus v:%d.%d", (unsigned long)((now - t_state) / 1000),
                (unsigned long)(((now - t_state) / 100) % 10),
                (int)speed_calib, (int)(speed_calib * 10) % 10);
        OLED_PrintASCIIString(0, 0, buf, &afont8x6, OLED_COLOR_NORMAL);
        /* 雷达点云图: 圆心(63,60), 量程90cm→半径52px */
        OLED_DrawCircle(63, 60, 17, OLED_COLOR_NORMAL);
        OLED_DrawCircle(63, 60, 34, OLED_COLOR_NORMAL);
        OLED_DrawCircle(63, 60, 52, OLED_COLOR_NORMAL);
        for (uint8_t i = 0; i < 13; i++) {
            if (radar_dist[i] <= 0) continue;
            float rad = i * 15.0f * 3.14159f / 180.0f;   /* 0°=右 90°=前 180°=左 */
            float r   = radar_dist[i] / RADAR_MAX_CM * 52.0f;
            int16_t x = 63 + (int16_t)(r * cosf(rad));
            int16_t y = 60 - (int16_t)(r * sinf(rad));
            OLED_DrawFilledRectangle(x > 0 ? (uint8_t)(x - 1) : 0,
                                    y > 0 ? (uint8_t)(y - 1) : 0, 3, 3, OLED_COLOR_NORMAL);
        }
        if (lost_err) OLED_PrintASCIIString(34, 12, "LOST!", &afont8x6, OLED_COLOR_NORMAL);
        break;
    }

    case ST_LAP1_DONE:
        OLED_PrintASCIIString(28, 0, "LAP1 DONE", &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "LAP1: %lu.%01lus", (unsigned long)(lap1_ms / 1000), (unsigned long)((lap1_ms / 100) % 10));
        OLED_PrintASCIIString(10, 20, buf, &afont12x6, OLED_COLOR_NORMAL);
        sprintf(buf, "v: %d.%d cm/s", (int)speed_calib, (int)(speed_calib * 10) % 10);
        OLED_PrintASCIIString(10, 34, buf, &afont12x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(4, 48, "BT 'M':manual 'B':rev", &afont8x6, OLED_COLOR_NORMAL);
        break;

    case ST_MANUAL:
        OLED_PrintASCIIString(16, 0, "MANUAL LAP", &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "T: %lu.%01lus", (unsigned long)((now - t_state) / 1000),
                (unsigned long)(((now - t_state) / 100) % 10));
        OLED_PrintASCIIString(10, 24, buf, &afont12x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(4, 44, "W/A/S/D/X drive", &afont8x6, OLED_COLOR_NORMAL);
        break;

    case ST_FINISHED: {
        uint32_t tot = lap1_ms + lap2_ms;
        OLED_PrintASCIIString(16, 0, "ALL  DONE", &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "L1:%lus L2:%lus", (unsigned long)(lap1_ms / 1000), (unsigned long)(lap2_ms / 1000));
        OLED_PrintASCIIString(10, 18, buf, &afont12x6, OLED_COLOR_NORMAL);
        sprintf(buf, "TOTAL: %lu.%01lus", (unsigned long)(tot / 1000), (unsigned long)((tot / 100) % 10));
        OLED_PrintASCIIString(10, 30, buf, &afont12x6, OLED_COLOR_NORMAL);
#if TRACK_LEN_CM > 0                                 /* 圈平均速度 = 赛道长/圈时 */
        sprintf(buf, "V1:%d V2:%d cm/s",
                (int)((float)TRACK_LEN_CM * 1000.0f / lap1_ms),
                (int)((float)TRACK_LEN_CM * 1000.0f / lap2_ms));
        OLED_PrintASCIIString(10, 42, buf, &afont12x6, OLED_COLOR_NORMAL);
#else                                                 /* 无赛道长: 用20cm标定速度 */
        sprintf(buf, "v: %d.%d cm/s", (int)speed_calib, (int)(speed_calib * 10) % 10);
        OLED_PrintASCIIString(10, 42, buf, &afont12x6, OLED_COLOR_NORMAL);
#endif
        if (rev_ms) {
            sprintf(buf, "REV: %lu.%01lus", (unsigned long)(rev_ms / 1000), (unsigned long)((rev_ms / 100) % 10));
            OLED_PrintASCIIString(10, 54, buf, &afont8x6, OLED_COLOR_NORMAL);
        } else {
            OLED_PrintASCIIString(10, 54, "BT 'B': reverse", &afont8x6, OLED_COLOR_NORMAL);
        }
        break;
    }

    case ST_REVERSE:
        OLED_PrintASCIIString(10, 0, "REVERSE LAP", &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "T: %lu.%01lus", (unsigned long)((now - t_state) / 1000),
                (unsigned long)(((now - t_state) / 100) % 10));
        OLED_PrintASCIIString(10, 32, buf, &afont12x6, OLED_COLOR_NORMAL);
        break;
    }

    OLED_ShowFrame();
}

/* ================= 主循环 ================= */
static void Main_Loop(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 公共任务 ---- */
    Buzzer_Task();
    if (bt_flag) { uint8_t c = bt_data; bt_flag = 0; BT_Process(c); }
    g_ir = IR_Read();
    Radar_Task();
    UI_Task();

    /* ---- 声音启动检测(启用时): PA15 活动电平持续30ms ---- */
#if SOUND_ENABLE
    if (state == ST_STANDBY) {
        static uint32_t t_snd = 0;
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == (GPIO_PinState)SOUND_ACTIVE) {
            if (t_snd == 0) t_snd = now;
            else if (now - t_snd >= 30) {
                t_snd = 0;
                Motor_Set(RUN20_PWM, RUN20_PWM);
                Goto(ST_RUN20);
            }
        } else t_snd = 0;
    }
#endif

    /* ---- 状态机 ---- */
    switch (state) {

    case ST_STANDBY:                                /* 待命: 电机停, 等启动 */
        Motor_Stop();
#if !SOUND_ENABLE
        /* 无声音模块: 上电延时自动启动 */
        if (AUTO_START_MS > 0 && now - t_state >= AUTO_START_MS) {
#if TRACK_DBG
            last_err = 0; t_lost = 0; lost_err = 0;
            Goto(ST_LAP1);                          /* 调试: 直接进纯循迹 */
#else
            Motor_Set(RUN20_PWM, RUN20_PWM);
            Goto(ST_RUN20);
#endif
        }
#endif
        break;

    case ST_RUN20:                                  /* 直行20cm(时间法标定) */
        if (now - t_state >= RUN20_MS) {
            Motor_Stop();
            speed_calib = 20000.0f / (float)RUN20_MS;   /* 20cm/时长 → cm/s */
            Goto(ST_WAIT5);
        }
        break;

    case ST_WAIT5:                                  /* 停车计时5s后蜂鸣起步 */
        if (now - t_state >= 5000) {
            Buzzer_Beep(P_LONG);                    /* 5s到: 鸣笛 */
            Radar_Enable();                         /* 循迹中持续扫描 */
            LineEvt_Reset();
            y_fired  = 0;
            lost_err = 0;
            t_lost   = 0;
            last_err = 0;
            Goto(ST_LAP1);
        }
        break;

    case ST_LAP1: {                                 /* 自动圈: 循迹+避障+Y+雷达 */
        uint32_t lap = now - t_state;
        uint8_t  evt;

        /* 丢线保护触发后停车, 人工把车放回线上自动恢复 */
        if (lost_err) {
            Motor_Stop();
            if (g_ir != 0 && g_ir != 0x0F) { lost_err = 0; t_lost = 0; }
            break;
        }

        /* 避障子流程优先接管 */
        if (av != AV_IDLE) { Avoid_Task(); break; }
        /* Y路口子流程接管 */
        if (ys != YS_IDLE) { Y_Task(); break; }

#if TRACK_DBG
        /* ---- 调试模式: 只做红外循迹(保留丢线保护), 其余全跳过 ---- */
        if (g_ir == 0) {
            if (t_lost == 0) t_lost = now;
            else if (now - t_lost > LOST_LINE_MS) {
                Motor_Stop(); lost_err = 1; Buzzer_Beep(P_ERR); t_lost = 0;
                break;
            }
        } else t_lost = 0;
        Track_Drive(0);
        break;
#endif

        /* 线事件分类 */
        evt = LineEvt_Task();

        /* 前方避障触发(跑起来3s后才开始判, 避开起步误判) */
        if (evt != 2 && lap > 3000 && Front_Min() < AVOID_TRIGGER_CM) {
            Motor_Stop();
            av = AV_STOP; av_t = now;
            Radar_Disable();                        /* 舵机先给张望用 */
            break;
        }

        /* Y路口: 宽黑且本圈未处理 */
        if (evt == 2 && lap > 3000 && !y_fired) {
            y_fired = 1;
            if (y_choice_left) Motor_Set(base_speed / 4, base_speed);       /* 左分支 */
            else               Motor_Set(base_speed, base_speed / 4);       /* 右分支 */
            ys = YS_TURN; ys_t = now;
            break;
        }

        /* 横线 = 回到起点(圈时保护) */
        if (evt == 1 && lap > MIN_LAP_MS) {
            Motor_Stop();
            Radar_Disable();
            lap1_ms = lap;
            Buzzer_Beep(P_LAP);
            Goto(ST_LAP1_DONE);
            break;
        }

        /* 丢线保护 */
        if (g_ir == 0) {
            if (t_lost == 0) t_lost = now;
            else if (now - t_lost > LOST_LINE_MS) {
                Motor_Stop();
                lost_err = 1;
                Buzzer_Beep(P_ERR);
                t_lost = 0;
                break;
            }
        } else t_lost = 0;

        Track_Drive(0);
        break;
    }

    case ST_LAP1_DONE:                              /* 完成自动圈, 等'M'/'B' */
        Motor_Stop();
        break;

    case ST_MANUAL: {                               /* 手动遥控圈, 线终点 */
        uint32_t lap = now - t_state;
        uint8_t  evt = LineEvt_Task();
        if ((evt == 1 || evt == 2) && lap > MIN_LAP_MS) {
            Motor_Stop();
            lap2_ms = lap;
            Buzzer_Beep(P_DONE);
            Goto(ST_FINISHED);
        }
        break;
    }

    case ST_FINISHED:                               /* 数据总览 */
        Motor_Stop();
        break;

    case ST_REVERSE: {                              /* 拓展: 逆向循迹返程 */
        uint32_t lap = now - t_state;
        uint8_t  evt;

        /* 丢线保护触发后停车, 人工放回线上自动恢复 */
        if (lost_err) {
            Motor_Stop();
            if (g_ir != 0 && g_ir != 0x0F) { lost_err = 0; t_lost = 0; }
            break;
        }

        evt = LineEvt_Task();
        if ((evt == 1 || evt == 2) && lap > MIN_LAP_MS) {
            Motor_Stop();
            rev_ms = lap;
            Buzzer_Beep(P_DONE);
            Goto(ST_FINISHED);
            break;
        }
        /* 丢线保护(倒车) */
        if (g_ir == 0) {
            if (t_lost == 0) t_lost = now;
            else if (now - t_lost > LOST_LINE_MS) {
                Motor_Stop(); lost_err = 1;
                Buzzer_Beep(P_ERR);
                t_lost = 0;
                break;
            }
        } else t_lost = 0;

        Track_Drive(1);
        break;
    }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  Buzzer_Set(0);                        /* 上电先关蜂鸣(gpio初始电平为响) */
  Motor_Stop();                         /* 电机归零 */

  /* 电机PWM + 舵机PWM 输出 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  Servo_SetAngle(90);                   /* 超声波朝正前 */

  /* TIM2编码器配置是无效残留(无编码器硬件), 忽略之, 不启动 */

  /* 超声波回波捕获 */
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  /* 蓝牙接收(中断, 单字节; 模块未接时不挂, 防悬空RX噪声) */
#if BT_ENABLE
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&bt_data, 1);
#endif

#if SOUND_ENABLE
  /* PA15原为JTDI, 需释放JTAG保留SWD后才可作GPIO */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  {
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_15;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);
  }
#endif

  /* OLED(上电等待稳定再初始化) */
  HAL_Delay(50);
  OLED_Init();
  OLED_NewFrame();
  OLED_ShowFrame();
  ui_init = 1;

  Goto(ST_STANDBY);
  t_lost = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Main_Loop();
    /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* ================= 蓝牙接收回调 ================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        /* HAL已把收到的字节写入bt_data缓冲, 置标志即可 */
        bt_flag = 1;
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&bt_data, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&bt_data, 1);   /* 出错重挂, 防接收卡死 */
}

/* ================= 超声波回波捕获 ================= */
/* 上升沿记起点并切下降沿捕获, 下降沿算脉宽 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if (!us_done) {
            if (us_width == 0) {                     /* 上升沿 */
                us_width = val;                       /* 暂存起点(置非0表示等待下降沿) */
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            } else {                                 /* 下降沿 */
                us_width = (val >= us_width) ? (val - us_width)
                                             : (65536u + val - us_width);
                us_done  = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error occurred line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
