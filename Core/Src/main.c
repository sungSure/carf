/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 电赛小车 —— 全流程状态机(验收版)
  ******************************************************************************
  * @attention
  *
  * 流程: 待命 → 启动(PA15声音) → 直行20cm → 停车计时5s → 蜂鸣
  *       → 去程循迹(雷达扫描+点云图+Y路口分叉识别+避障) → 到终点横线停车
  *       → 原地掉头 → 沿原路返回起点横线停车(记去程/返程/总时)
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
 * 红外: PA2(L2) PA3(L1) PA4(R1) PA5(R2) 左→右, 压黑线输出高电平
 * 超声: Trig=PB5  Echo=PB6(TIM4_CH1 输入捕获, 1us计数)
 * 舵机: TIM3_CH1(PA6) 50Hz, 0.5~2.5ms 对应 0~180°
 * 蜂鸣: PA12 低电平响(有源, 已匹配)
 * 蓝牙: USART3 PB10/11 9600(EN/STATE悬空不接)
 * 声音: HW-484 DO→PA15, 有声音输出高电平
 * ============ 蓝牙指令表(单字符) ============ *
 *  G: 启动(声音关闭时的启动信号)   L/R: Y路口选左/右(默认左)
 *  M: 自动圈结束后进入手动圈       B: 完成后触发逆向返程(拓展)
 *  W/A/S/D/X: 手动圈 遥控 前/左/后/右/停
 *  U/J: 循迹基础速度 ±10(现场免重烧调参)
 * ==================================== */

/* ---------- 启动方式(现场可改) ----------
 * SOUND_ENABLE=1  : PA15声音触发(验收要求, HW-484已接入)
 * AUTO_START_MS>0 : 上电延时自启(SOUND_ENABLE=1时编译剔除, 仅备用)
 * 两者都关        : 等蓝牙'G'启动(需BT_ENABLE=1)
 * BT_ENABLE=1     : 蓝牙已接 USART3(PB10/PB11, 9600)                    */
#define SOUND_ENABLE      1
#define SOUND_ACTIVE      1        /* HW-484: 检测到声音DO输出高电平(实测资料) */
#define SND_TEST_MS       0   /* 上电声音自检时长ms(显示DO电平+脉冲计数), 0=跳过 */
#define SND_HOLD_MS       1u  /* 声音触发启动: 活动电平需持续时长ms(越小越灵敏, 太小易误触发) */
#define AUTO_START_MS     3000u    /* 上电到自启延时ms, 0=禁用; 留时间摆车 */
#define BT_ENABLE         1        /* 蓝牙模块已接入 */

/* ---------- 调试开关 ---------- */
#define TRACK_DBG         0        /* 1=调试模式: 延时后直接纯红外循迹, 跳过全部流程/
                                        雷达/避障/圈计时, OLED显示传感器状态; 调完改0 */
#define REC_DBG           1        /* 1=录制模式(临时诊断): 声控启动后纯循迹, 记录传感器
                                        变化沿, 到终点横线或录满后经蓝牙(USART3)回传文本;
                                        与 TRACK_DBG 互斥; 采完改0恢复验收流程 */

/* ---------- 数据录制(临时诊断) ---------- */
#define REC_MAX           2000u    /* 最大记录条数(仅记变化沿), 每条3字节 ≈6KB RAM */
#define REC_MIN_GAP_MS    2u       /* 最小记录间隔ms(抗传感器抖动) */

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
#define TRACK_BASE        100      /* 循迹基础巡航占空比(U/J可在线调) */
#define TRACK_K           28       /* 转向增益: 差速 = K*err, 摆动大→减, 弯道跟不上→加 */
#define TRACK_DIFF_MAX    70       /* 差速限幅: 单侧加减的上限, 削峰防甩尾 */
#define GAP_HOLD_MS       120u     /* 线在传感器间隙(全白)时保持原曲率的时间, 超过才算真丢线 */
#define SHARP_TRIG_MS     120u     /* 最外传感器持续压线时长→判定U型急弯 */
#define SHARP_PWM_OUT     150      /* 急弯旋转: 外侧轮速 */
#define SHARP_PWM_IN      (-60)   /* 急弯旋转: 内侧轮速(负=倒转→小半径) */
#define LOST_LINE_MS      2500u    /* 丢线保护时间 */

/* ---------- 线事件判定 ---------- */
#define LINE_MARK_MAX_MS  250u     /* 全黑持续<此值 → 横线(起点/终点); 超时按宽黑区处理 */
#define FORK_MIN_MS        40u     /* 分叉图案"黑-白-黑"持续此值 → Y路口(两线分叉型) */
#define MIN_LAP_MS        15000u   /* 去程最早接受"终点"的时间 */
#define MIN_RET_MS         1500u   /* 返程最早接受"起点"的时间 */
#define RET_TURN_MS       900u     /* 终点原地掉头180°的旋转时长(现场标定: 掉头不准则调此值) */
#define RET_TURN_PWM      150      /* 掉头旋转轮速(左正右负, 原地转) */
#define RET_ACQ_MS        2000u    /* 掉头后重新找线的时限; 超时则按丢线保护处理 */

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
/* 压黑线时红外DO输出高电平(实测, 调试屏实心=压线) → 1=在黑线上 */
#define IR_ON_LINE        GPIO_PIN_SET
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* ---------- 状态机 ---------- */
typedef enum {
    ST_STANDBY,       /* 待命等启动信号 */
    ST_RUN20,         /* 直行20cm */
    ST_WAIT5,         /* 停车计时5s */
    ST_LAP1,          /* 自动圈去程: 循迹+雷达+避障+Y路口选择 */
    ST_RETURN,        /* 终点掉头, 沿原路返回起点并停车 */
    ST_RECORD,        /* 临时: 纯循迹并录制传感器数据(诊断用) */
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
static uint32_t  snd_seen = 0;      /* 最近一次检测到声音的时刻(待命屏显示用) */
static uint32_t  t_lost = 0;

/* 线事件(横线 / 分叉路口) */
static struct {
    uint8_t  in_black;       /* 全黑进行中 */
    uint8_t  in_fork;        /* 分叉图案进行中 */
    uint8_t  fired;          /* 本次事件已上报(防重复) */
    uint32_t t_black;
    uint32_t t_fork;
} le;

/* 回到起点子流程 */
typedef enum { RT_TURN, RT_REACQ, RT_TRACK } RetStep;
static RetStep   rt = RT_TURN;
static uint32_t  rt_t;               /* 当前返回子步骤起始时刻 */
static uint32_t  out_ms = 0;         /* 去程用时(起点→终点) */
static uint32_t  back_ms = 0;        /* 返程用时(终点→起点) */

#if REC_DBG
/* ---------- 传感器录制(仅记录变化沿) ---------- */
static uint16_t  rec_t[REC_MAX];     /* 相对起点的时间戳ms(16位, 65s回绕) */
static uint8_t   rec_v[REC_MAX];     /* 4位传感器值: bit3=L2 bit2=L1 bit1=R1 bit0=R2 */
static uint16_t  rec_n   = 0;        /* 已记录条数 */
static uint32_t  rec_t0  = 0;        /* 录制起点(HAL tick), 首个采样时确定 */
static uint8_t   rec_started = 0;    /* 已采到第一条 */
static uint8_t   rec_done = 0;       /* 录制已结束(不再采样) */
static uint8_t   rec_full = 0;       /* 缓冲已满被截断 */
#endif

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
#if REC_DBG
static void Rec_Sample(uint32_t now);
static void Rec_Dump(void);
static void Rec_Reset(void);
#endif
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
    le.in_fork  = 0;
    le.fired    = 0;
    le.t_black  = 0;
    le.t_fork   = 0;
}

/* 分叉图案识别: 4路按 L2 L1 R1 R2 顺序中若出现"黑-白-黑"(中间有白),
   说明传感器下同时有两条分离的黑线 → Y路口分叉。
   正常循迹时压到的黑线是连续一段, 不会出现中间空洞。 */
static uint8_t IR_Fork(uint8_t v)
{
    int8_t l = -1, r = -1;
    for (uint8_t i = 0; i < 4; i++) {
        if ((v >> (3 - i)) & 1) { if (l < 0) l = (int8_t)i; r = (int8_t)i; }
    }
    if (l < 0 || (r - l) < 2) return 0;              /* 无黑 或 连续黑 */
    for (uint8_t i = (uint8_t)l + 1; i < (uint8_t)r; i++)
        if (!((v >> (3 - i)) & 1)) return 1;         /* 中间有白 → 分叉 */
    return 0;
}

/* 事件: 1=横线(起点/终点, 短全黑) 2=Y路口(分叉图案 或 超长全黑区) */
static uint8_t LineEvt_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  evt = 0;

    /* --- Y路口(分叉型): 黑-白-黑 持续 FORK_MIN_MS --- */
    if (IR_Fork(g_ir)) {
        if (!le.in_fork) { le.in_fork = 1; le.fired = 0; le.t_fork = now; }
        else if (!le.fired && now - le.t_fork >= FORK_MIN_MS) { le.fired = 1; evt = 2; }
    } else le.in_fork = 0;

    /* --- 横线(起点/终点): 全黑出现后短时间结束 --- */
    if (g_ir == 0x0F) {
        if (!le.in_black) { le.in_black = 1; le.fired = 0; le.t_black = now; }
        else if (!le.fired && now - le.t_black >= LINE_MARK_MAX_MS) { le.fired = 1; evt = 2; }
    } else if (le.in_black) {
        le.in_black = 0;
        if (!le.fired) evt = 1;                      /* 短全黑 → 横线 */
    }
    return evt;
}

/* ================= 循迹差速驱动 ================= */
/* 加权: L2=-3 L1=-1 R1=+1 R2=+3; err>0线在右侧 → 左轮加速右转 */
/* U型急弯: 差速再大也只是大半径弧线, 必须切原地小半径旋转(内侧轮反转) */
static int8_t    sharp_turn = 0;     /* 急弯旋转中: +1右转 -1左转 0无(调试屏要显示) */

static void Track_Drive(uint8_t reverse)
{
    static uint32_t t_gap = 0;     /* 全白持续时长: 短=线在传感器间隙, 长=真丢线 */
    static uint32_t t_out = 0;     /* 最外传感器持续压线时长 */
    int16_t l, r, diff;

    if (sharp_turn) {              /* ---- 急弯旋转: 转到有传感器重新压线为止 ---- */
        if (g_ir != 0 && g_ir != 0x0F) {          /* 找到线(全黑=横穿线, 继续转) */
            sharp_turn = 0; t_gap = 0; t_out = 0;
        } else {
            l = (sharp_turn > 0) ? SHARP_PWM_OUT : SHARP_PWM_IN;
            r = (sharp_turn > 0) ? SHARP_PWM_IN  : SHARP_PWM_OUT;
            Motor_Set(l, r);
            return;
        }
    }

    if (g_ir == 0x0F) {            /* 全黑(横线/路口): 事件机负责判定, 车直行通过 */
        diff = 0;
    } else if (g_ir == 0) {        /* 全白: 先保持原曲率滑过间隙, 持续过久才强打方向找线 */
        if (t_gap == 0) t_gap = HAL_GetTick();
        if (last_err >= 2) {       /* 外侧压线后线消失 → U型急弯, 切旋转 */
            sharp_turn = +1;
            return;
        } else if (last_err <= -2) {
            sharp_turn = -1;
            return;
        }
        if (HAL_GetTick() - t_gap <= GAP_HOLD_MS)
            diff = TRACK_K * last_err;                       /* 间隙: 维持当前转向 */
        else
            diff = (last_err >= 0) ? TRACK_DIFF_MAX : -TRACK_DIFF_MAX;   /* 真丢线: 满差速 */
    } else {
        int8_t e = 0;
        if (g_ir & 0x8) e -= 3;
        if (g_ir & 0x4) e -= 1;
        if (g_ir & 0x2) e += 1;
        if (g_ir & 0x1) e += 3;
        last_err = e;
        t_gap = 0;
        /* 最外传感器持续压线: 满差速也拉不回中心 → 急弯 */
        if (e == 3 || e == -3) {
            if (t_out == 0) t_out = HAL_GetTick();
            else if (HAL_GetTick() - t_out >= SHARP_TRIG_MS) { sharp_turn = (e > 0) ? 1 : -1; t_out = 0; }
        } else t_out = 0;
        diff = TRACK_K * e;
    }

    /* 差速限幅: 防止单侧饱和/反转造成甩尾 */
    if (diff >  TRACK_DIFF_MAX) diff =  TRACK_DIFF_MAX;
    if (diff < -TRACK_DIFF_MAX) diff = -TRACK_DIFF_MAX;

    l = base_speed + diff;
    r = base_speed - diff;

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
        if (base_speed > PWM_MAX - TRACK_DIFF_MAX) base_speed = PWM_MAX - TRACK_DIFF_MAX;
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

#if REC_DBG
    /* --- 录制模式控制 --- */
    case 'P':                                       /* 立即回传已录数据 */
        Motor_Stop();
        Rec_Dump();
        break;
    case 'C':                                       /* 清空并重新开始录制 */
        Motor_Stop();
        Rec_Reset();
        Goto(ST_RECORD);
        break;
#endif

    default: break;
    }
}

/* ================= OLED 显示 ================= */
static void UI_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t period = UI_PERIOD_MS;
#if REC_DBG
    if (state == ST_RECORD) period = 1000u;   /* 录制时降低刷屏频率, 缩小I2C阻塞盲区 */
#endif
    if (now - ui_t < period) return;
    ui_t = now;
    if (!ui_init) return;

    char buf[24];
    OLED_NewFrame();

    switch (state) {
    case ST_STANDBY: {
        OLED_PrintASCIIString(16, 0, "== CAR READY ==", &afont12x6, OLED_COLOR_NORMAL);
#if SOUND_ENABLE
        OLED_PrintASCIIString(16, 16, "WAIT SOUND ...", &afont12x6, OLED_COLOR_NORMAL);
        /* 声音触发可视化: 拍手后显示ACTIVE!持续1s, 用于验证极性/灵敏度 */
        if (snd_seen != 0 && now - snd_seen < 1000)
            OLED_PrintASCIIString(22, 30, "SND: ACTIVE !", &afont12x6, OLED_COLOR_NORMAL);
        else
            OLED_PrintASCIIString(28, 30, "SND: ----", &afont12x6, OLED_COLOR_NORMAL);
#elif AUTO_START_MS > 0
        uint32_t left_ms = (now - t_state >= AUTO_START_MS) ? 0
                                                       : (AUTO_START_MS - (now - t_state));
        sprintf(buf, "START IN: %us", (unsigned)((left_ms + 999u) / 1000u));
        OLED_PrintASCIIString(13, 20, buf, &afont12x6, OLED_COLOR_NORMAL);
#else
        OLED_PrintASCIIString(10, 20, "BT 'G' TO START", &afont12x6, OLED_COLOR_NORMAL);
#endif
        sprintf(buf, "Y-%s  SPD:%d", y_choice_left ? "L" : "R", base_speed);
        OLED_PrintASCIIString(10, 48, buf, &afont12x6, OLED_COLOR_NORMAL);
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
        sprintf(buf, "E:%d K%d S%d D%d %s", last_err, TRACK_K, base_speed, TRACK_DIFF_MAX,
                sharp_turn ? "TURN" : (lost_err ? "LOST" : "RUN"));
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

    case ST_RETURN:
        OLED_PrintASCIIString(20, 0, "RETURNING", &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "OUT: %lu.%01lus", (unsigned long)(out_ms / 1000), (unsigned long)((out_ms / 100) % 10));
        OLED_PrintASCIIString(10, 18, buf, &afont12x6, OLED_COLOR_NORMAL);
        sprintf(buf, "T: %lu.%01lus", (unsigned long)((now - t_state) / 1000),
                (unsigned long)(((now - t_state) / 100) % 10));
        OLED_PrintASCIIString(10, 32, buf, &afont12x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(10, 50, rt == RT_TURN ? "TURN AROUND" : "BACK TO START", &afont8x6, OLED_COLOR_NORMAL);
        break;

#if REC_DBG
    case ST_RECORD:
        OLED_PrintASCIIString(20, 0, "RECORDING", &afont16x8, OLED_COLOR_NORMAL);
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t x = 14 + i * 26;
            if ((g_ir >> (3 - i)) & 1) OLED_DrawFilledRectangle(x, 18, 24, 18, OLED_COLOR_NORMAL);
            else                       OLED_DrawRectangle(x, 18, 24, 18, OLED_COLOR_NORMAL);
        }
        sprintf(buf, "n:%u  t:%lus  %s", (unsigned)rec_n,
                (unsigned long)((now - t_state) / 1000), rec_full ? "FULL" : "REC");
        OLED_PrintASCIIString(4, 40, buf, &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(4, 52, "BT 'P':dump 'C':clr", &afont8x6, OLED_COLOR_NORMAL);
        break;
#endif

    case ST_LAP1_DONE:
        OLED_PrintASCIIString(24, 0, "LAP1 DONE", &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "OUT: %lu.%01lus", (unsigned long)(out_ms / 1000), (unsigned long)((out_ms / 100) % 10));
        OLED_PrintASCIIString(10, 18, buf, &afont12x6, OLED_COLOR_NORMAL);
        sprintf(buf, "RET: %lu.%01lus", (unsigned long)(back_ms / 1000), (unsigned long)((back_ms / 100) % 10));
        OLED_PrintASCIIString(10, 30, buf, &afont12x6, OLED_COLOR_NORMAL);
        sprintf(buf, "TOT: %lu.%01lus", (unsigned long)((out_ms + back_ms) / 1000),
                (unsigned long)(((out_ms + back_ms) / 100) % 10));
        OLED_PrintASCIIString(10, 42, buf, &afont12x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(4, 54, "BT 'M':man 'B':rev", &afont8x6, OLED_COLOR_NORMAL);
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

#if REC_DBG
/* ================= 传感器录制(临时诊断) =================
 * 采样策略: 主循环每次迭代采样 g_ir, 仅当值与上条记录不同且距上条≥REC_MIN_GAP_MS
 *           时记录一条(时间戳+4位值), 记录的是"变化沿"序列, 数据量小且能无损还原方波。
 * 蓝牙回传(USART3, 9600): 文本, 每行 "t,pattern"
 *   t       = 相对录制起点的时间ms
 *   pattern = 4 个字符, 依次 L2 L1 R1 R2 (1=压到黑线, 0=白地)
 * 例:  0,0000
 *      12,1000      ← 左2(最外左)压线
 *      37,1001      ← 左右最外侧同时压线(分叉口特征)
 */
static void Rec_Sample(uint32_t now)
{
    if (rec_done) return;
    if (!rec_started) {                            /* 第一条: 以当前时刻为录制起点 */
        rec_started = 1; rec_t0 = now;
        rec_t[0] = 0; rec_v[0] = g_ir; rec_n = 1;
        return;
    }
    uint16_t t = (uint16_t)(now - rec_t0);
    if (g_ir == rec_v[rec_n - 1]) return;                            /* 值未变化 */
    if ((uint16_t)(t - rec_t[rec_n - 1]) < REC_MIN_GAP_MS) return;  /* 抗传感器抖动 */
    if (rec_n >= REC_MAX) { rec_full = 1; rec_done = 1; return; }
    rec_t[rec_n] = t; rec_v[rec_n] = g_ir; rec_n++;
}

static void Rec_Send(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 2000);
}

/* 经蓝牙回传全部记录 */
static void Rec_Dump(void)
{
    char b[40];
    sprintf(b, "\r\n#REC n=%u full=%u gap=%ums\r\n", (unsigned)rec_n,
            (unsigned)rec_full, (unsigned)REC_MIN_GAP_MS);
    Rec_Send(b);
    Rec_Send("t,L2L1R1R2\r\n");
    for (uint16_t i = 0; i < rec_n; i++) {
        uint8_t v = rec_v[i];
        sprintf(b, "%u,%d%d%d%d\r\n", (unsigned)rec_t[i],
                (v >> 3) & 1, (v >> 2) & 1, (v >> 1) & 1, v & 1);
        Rec_Send(b);
    }
    Rec_Send("#END\r\n");
}

static void Rec_Reset(void)
{
    rec_n = 0; rec_t0 = 0; rec_started = 0; rec_done = 0; rec_full = 0;
}
#endif

/* ================= 主循环 ================= */
static void Main_Loop(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 公共任务 ---- */
    Buzzer_Task();
    if (bt_flag) { uint8_t c = bt_data; bt_flag = 0; BT_Process(c); }
    g_ir = IR_Read();
#if REC_DBG
    if (state == ST_RECORD) Rec_Sample(now);      /* 录制: 每次迭代采样(仅记变化沿) */
#endif
    Radar_Task();
    UI_Task();
#if REC_DBG
    /* UI_Task 内含I2C阻塞(约90ms), 刷新后再采一次, 补上阻塞期间漏掉的变化 */
    if (state == ST_RECORD) Rec_Sample(HAL_GetTick());
#endif

    /* ---- 声音检测(启用时): PA15 活动电平持续 SND_HOLD_MS 才启动, 防误触发 ---- */
#if SOUND_ENABLE
    {
        static uint32_t t_snd = 0;
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == (GPIO_PinState)SOUND_ACTIVE) {
            snd_seen = now;                       /* 记录, 待命屏显示1s */
            if (state == ST_STANDBY) {
                if (t_snd == 0) t_snd = now;
                else if (now - t_snd >= SND_HOLD_MS) {
                    t_snd = 0;
#if REC_DBG
                    Rec_Reset();
                    last_err = 0; t_lost = 0; lost_err = 0;
                    Goto(ST_RECORD);            /* 录制模式: 直接进纯循迹录制 */
#else
                    Motor_Set(RUN20_PWM, RUN20_PWM);
                    Goto(ST_RUN20);
#endif
                }
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
#if REC_DBG
            Rec_Reset();
            last_err = 0; t_lost = 0; lost_err = 0;
            Goto(ST_RECORD);                        /* 录制模式 */
#elif TRACK_DBG
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

        /* 终点横线: 停车记时 → 掉头返回起点 */
        if (evt == 1 && lap > MIN_LAP_MS) {
            Motor_Stop();
            Radar_Disable();
            out_ms   = lap;                         /* 去程用时 */
            lost_err = 0;
            t_lost   = 0;
            rt       = RT_TURN;
            rt_t     = now;
            Buzzer_Beep(P_LAP);
            Goto(ST_RETURN);
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

    case ST_RETURN: {                               /* 终点掉头 → 沿原路返回起点 */
        uint8_t  evt;

        /* 丢线保护触发后停车, 人工把车放回线上自动恢复 */
        if (lost_err) {
            Motor_Stop();
            if (g_ir != 0 && g_ir != 0x0F) { lost_err = 0; t_lost = 0; }
            break;
        }

        switch (rt) {
        case RT_TURN:                               /* 原地掉头180° */
            Motor_Set(RET_TURN_PWM, -RET_TURN_PWM);
            if (now - rt_t >= RET_TURN_MS) {
                Motor_Stop();
                LineEvt_Reset();                    /* 清掉停在终点线上产生的黑事件 */
                t_lost = 0;
                rt = RT_REACQ; rt_t = now;
            }
            break;

        case RT_REACQ:                              /* 前进离开终点线, 直到重新压到循迹线 */
            Motor_Set(TRACK_BASE / 2, TRACK_BASE / 2);
            if (g_ir != 0 && g_ir != 0x0F) {        /* 重新看到单条黑线 → 接管循迹 */
                rt = RT_TRACK; rt_t = now; t_lost = 0;
            } else if (now - rt_t > RET_ACQ_MS) {   /* 时限内找不到线 → 交丢线保护 */
                Motor_Stop();
                lost_err = 1;
                Buzzer_Beep(P_ERR);
            }
            break;

        default:
        case RT_TRACK:                              /* 正常循迹返回, 检测起点横线 */
            evt = LineEvt_Task();
            if (evt == 1 && now - rt_t > MIN_RET_MS) {
                Motor_Stop();
                back_ms = now - t_state;            /* 返程用时(含掉头) */
                lap1_ms = out_ms + back_ms;         /* 自动圈总时 */
                Buzzer_Beep(P_DONE);
                Goto(ST_LAP1_DONE);
                break;
            }
            /* 丢线保护 */
            if (g_ir == 0) {
                if (t_lost == 0) t_lost = now;
                else if (now - t_lost > LOST_LINE_MS) {
                    Motor_Stop(); lost_err = 1; Buzzer_Beep(P_ERR); t_lost = 0;
                    break;
                }
            } else t_lost = 0;
            Track_Drive(0);
            break;
        }
        break;
    }

#if REC_DBG
    case ST_RECORD: {                               /* 临时: 纯循迹 + 录制传感器数据 */
        uint32_t lap = now - t_state;
        uint8_t  evt = LineEvt_Task();

        /* 丢线保护: 停车, 人工放回线上自动恢复 */
        if (lost_err) {
            Motor_Stop();
            if (g_ir != 0 && g_ir != 0x0F) { lost_err = 0; t_lost = 0; }
            break;
        }

        /* 录满 → 停车并回传 */
        if (rec_full) {
            Motor_Stop();
            Buzzer_Beep(P_DONE);
            Rec_Dump();
            Goto(ST_LAP1_DONE);
            break;
        }

        /* 终点/起点横线(启动2.5s后) → 结束录制并回传 */
        if (evt == 1 && lap > 2500) {
            Motor_Stop();
            rec_done = 1;
            Buzzer_Beep(P_DONE);
            Rec_Dump();                             /* 阻塞回传(车已停) */
            Goto(ST_LAP1_DONE);
            break;
        }

        /* 丢线保护 */
        if (g_ir == 0) {
            if (t_lost == 0) t_lost = now;
            else if (now - t_lost > LOST_LINE_MS) {
                Motor_Stop(); lost_err = 1; Buzzer_Beep(P_ERR); t_lost = 0;
                break;
            }
        } else t_lost = 0;

        Track_Drive(0);
        break;
    }
#endif

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
  /* PA15初始化(CubeMX未配置此脚)。SWJ_NOJTAG重映射在 hal_msp.c 里CubeMX已做
     (SYS=Serial Wire), 此处再写一次属冗余保险, 保证本块自包含 */
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

#if SOUND_ENABLE && SND_TEST_MS > 0
  /* ---- 上电声音自检: 无论是否触发都等满时长, 实时显示DO电平+跳变计数 ----
   * 判读: 拍手时 DO 变 HIGH 且 PULSES 增加 → 极性对(SOUND_ACTIVE=1)
   *       拍手时 DO 变 LOW(平时HIGH)     → 极性反了, SOUND_ACTIVE 改 0
   *       DO 恒 LOW/恒 HIGH 且 PULSES 不动 → 查接线/供电/灵敏度电位器        */
  {
    char tbuf[24];
    GPIO_PinState lvl = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    uint32_t t0 = HAL_GetTick(), t_ui = 0;
    uint16_t pulses = 0;

    while (HAL_GetTick() - t0 < SND_TEST_MS) {
      GPIO_PinState cur = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
      if (cur != lvl) { lvl = cur; pulses++; }         /* 电平跳变计数 */
      uint32_t n = HAL_GetTick();
      if (n - t_ui >= 100) {                           /* 10Hz 刷新屏幕 */
        t_ui = n;
        OLED_NewFrame();
        sprintf(tbuf, "SND TEST %us",
                (unsigned)((SND_TEST_MS - (n - t0) + 999u) / 1000u));
        OLED_PrintASCIIString(16, 0, tbuf, &afont16x8, OLED_COLOR_NORMAL);
        sprintf(tbuf, "DO: %s", lvl ? "HIGH" : "LOW");
        OLED_PrintASCIIString(28, 20, tbuf, &afont16x8, OLED_COLOR_NORMAL);
        sprintf(tbuf, "PULSES: %u", (unsigned)pulses);
        OLED_PrintASCIIString(28, 42, tbuf, &afont12x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(34, 56, "CLAP TO TEST", &afont8x6, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
      }
    }
  }
#endif

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
