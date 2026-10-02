/*
 * band_config.h -- 智能健康监测手环固件：编译期配置与协议常量
 *
 * 汇总 MCU / 外设地址 / 协议帧 / 离线缓存 / 调度器等所有可配置项，
 * 便于在不同硬件版本之间切换而不改动业务代码。
 */
#ifndef BAND_CONFIG_H
#define BAND_CONFIG_H

/* ------------------------------------------------------------------ */
/* 固件版本                                                            */
/* ------------------------------------------------------------------ */
#define BAND_FW_NAME             "stm32-wearable-health-band"
#define BAND_FW_VERSION_MAJOR    1
#define BAND_FW_VERSION_MINOR    0
#define BAND_FW_VERSION_PATCH    3
#define BAND_FW_VERSION_STR      "1.0.3"

/* ------------------------------------------------------------------ */
/* MCU 与时钟树 (STM32F405RGT6 / LQFP64)                               */
/* ------------------------------------------------------------------ */
#define BAND_MCU_PART            "STM32F405RGT6"
#define BAND_SYSCLK_HZ           168000000UL   /* HSE 8MHz -> PLL -> 168MHz */
#define BAND_APB1_HZ             42000000UL    /* I2C1 / USART2 / TIM 挂在 APB1 */
#define BAND_APB2_HZ             84000000UL    /* USART1 / SPI1 挂在 APB2 */
#define BAND_LSI_HZ              32000UL       /* 独立看门狗时钟源 */
#define BAND_LSE_HZ              32768UL       /* RTC 时钟源 */

/* 板级引脚分配（原理图 net 名 -> MCU 引脚） */
#define BAND_PIN_I2C1_SCL        "PB6"
#define BAND_PIN_I2C1_SDA        "PB7"
#define BAND_PIN_MPU6050_INT     "PC0"         /* 六轴中断 / DMP 数据就绪 */
#define BAND_PIN_FT6236_INT      "PC1"         /* 电容触摸中断 */
#define BAND_PIN_KEY_WAKE        "PC13"        /* 按键，兼唤醒源 */
#define BAND_PIN_MOTOR_EN        "PB0"         /* 振动马达 MOS 驱动 */
#define BAND_PIN_LCD_BL          "PB1"         /* 背光 PWM */
#define BAND_PIN_WIFI_EN         "PB2"         /* Wi-Fi 模组使能 */
#define BAND_PIN_USART1_TX       "PA9"
#define BAND_PIN_USART1_RX       "PA10"
#define BAND_PIN_BAT_ADC         "PC4"         /* ADC1_IN14 电池分压采样 */
#define BAND_PIN_VREFINT         "ADC1_IN17"   /* 内部 1.21V 基准，用于校准 VDDA */
#define BAND_PIN_ANT_KEEP        "PA13"        /* 板载天线净空区，禁止走线 */

/* ------------------------------------------------------------------ */
/* I2C 从机地址 (7bit)                                                 */
/* ------------------------------------------------------------------ */
#define BAND_I2C_ADDR_MPU6050    0x68
#define BAND_I2C_ADDR_FT6236     0x38
#define BAND_I2C_ADDR_MLX90615   0x5A
#define BAND_I2C_ADDR_SSD1306    0x3C

/* ------------------------------------------------------------------ */
/* 自定义应用层帧协议                                                  */
/*   +------+------+-------+-------+-----------+-------+
 *   | SOF0 | SOF1 | TYPE  | SEQ   | LEN       | ...   |
 *   | 0xAA | 0x55 | 1B    | 2B LE | 2B LE     |       |
 *   +------+------+-------+-------+-----------+-------+
 *   完整帧 = SOF(2) + TYPE(1) + SEQ(2) + LEN(2) + PAYLOAD(LEN) + CRC16(2)
 *   CRC16/CCITT-FALSE(poly=0x1021, init=0xFFFF) 覆盖 TYPE..PAYLOAD
 * ------------------------------------------------------------------ */
#define FRAME_SOF0               0xAAu
#define FRAME_SOF1               0x55u
#define FRAME_HEADER_LEN         7u    /* SOF0 SOF1 TYPE SEQ(2) LEN(2) */
#define FRAME_CRC_LEN            2u
#define FRAME_OVERHEAD           (FRAME_HEADER_LEN + FRAME_CRC_LEN)  /* 9 */
#define FRAME_MAX_PAYLOAD        256u
#define FRAME_MAX_SIZE           (FRAME_OVERHEAD + FRAME_MAX_PAYLOAD)
#define FRAME_CRC_INIT           0xFFFFu
#define FRAME_CRC_POLY           0x1021u

/* 帧类型 */
#define MSG_HEART_RATE           0x01u  /* 上行：心率 */
#define MSG_STEP_COUNT           0x02u  /* 上行：步数 */
#define MSG_TEMPERATURE          0x03u  /* 上行：体温 */
#define MSG_BATTERY              0x04u  /* 上行：电量 */
#define MSG_ATTITUDE             0x05u  /* 上行：姿态角 */
#define MSG_HEALTH_BATCH         0x06u  /* 上行：多体征聚合包 */
#define MSG_ACK                  0x10u  /* 双向：确认 */
#define MSG_NACK                 0x11u  /* 双向：否定确认 */
#define MSG_PARAM_SET            0x20u  /* 下行：远程参数下发 */
#define MSG_PARAM_QUERY          0x21u  /* 下行：参数查询 */
#define MSG_PARAM_REPORT         0x22u  /* 上行：参数上报 */
#define MSG_TIME_SYNC            0x23u  /* 下行：时间同步 */
#define MSG_OFFLINE_BATCH        0x30u  /* 上行：离线补传数据 */
#define MSG_PING                 0x40u  /* 双向：心跳 */
#define MSG_PONG                 0x41u  /* 双向：心跳应答 */
#define MSG_LOG                  0x80u  /* 上行：调试日志 */
#define MSG_OTA_BEGIN            0x90u  /* 下行：OTA 开始（预留） */

/* ACK 载荷中的状态码 */
#define ACK_STATUS_OK            0x00u
#define ACK_STATUS_CRC_ERR       0x01u
#define ACK_STATUS_SEQ_DUP       0x02u
#define ACK_STATUS_UNSUPPORTED   0x03u
#define ACK_STATUS_PARAM_ERR     0x04u

/* ------------------------------------------------------------------ */
/* 远程参数下发 TLV                                                    */
/* ------------------------------------------------------------------ */
#define TLV_TAG_HR_INTERVAL      0x01u  /* uint16 心率采样周期 ms */
#define TLV_TAG_STEP_SENSITIVITY 0x02u  /* uint8  计步灵敏度 0..100 */
#define TLV_TAG_UPLOAD_INTERVAL  0x03u  /* uint16 上传周期 ms */
#define TLV_TAG_SLEEP_TIMEOUT    0x04u  /* uint16 自动休眠超时 s */
#define TLV_TAG_LCD_BRIGHTNESS   0x05u  /* uint8  背光 0..100 */
#define TLV_TAG_MOTOR_ENABLE     0x06u  /* uint8  振动马达开关 */
#define TLV_TAG_WIFI_SSID        0x07u  /* string Wi-Fi SSID */
#define TLV_TAG_WIFI_PASS        0x08u  /* string Wi-Fi 密码 */
#define TLV_TAG_DEVICE_NAME      0x09u  /* string 设备名 */
#define TLV_MAX_COUNT            16u
#define TLV_MAX_VALUE_LEN        32u

/* ------------------------------------------------------------------ */
/* 缓冲区                                                              */
/* ------------------------------------------------------------------ */
#define RING_BUFFER_CAP          2048u  /* UART/WS 收包字节环形缓冲 */
#define OFFLINE_PAYLOAD_RING_CAP 4096u  /* 原始待发字节流缓冲 */

/* ------------------------------------------------------------------ */
/* 离线缓存与补传                                                      */
/* ------------------------------------------------------------------ */
#define OFFLINE_SLOT_MAX         64u    /* 环形缓存槽位数（一条记录一槽） */
#define OFFLINE_REC_MAX          128u   /* 单条记录最大字节数 */
#define OFFLINE_RESEND_BURST     8u     /* 单轮补传最多连发条数，防占满链路 */

/* ------------------------------------------------------------------ */
/* WebSocket (RFC6455)                                                 */
/* ------------------------------------------------------------------ */
#define WS_GUID                  "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
#define WS_HANDSHAKE_BUF         1024u
#define WS_KEY_LEN               16u    /* 客户端随机数 -> base64 后 24B */
#define WS_KEY_B64_LEN           24u
#define WS_ACCEPT_LEN            28u
#define WS_RX_BUF                1024u
#define WS_TX_BUF                1024u
#define WS_MAX_FRAME_PAYLOAD     1024u
#define WS_DEFAULT_PORT          9001u
#define WS_DEFAULT_PATH          "/band"
#define WS_PING_INTERVAL_MS      15000u /* 空闲超过该时间发送 ping */
#define WS_PONG_TIMEOUT_MS       5000u  /* ping 后等待 pong 的超时 */
#define WS_RECONNECT_BASE_MS     200u   /* 指数退避基数 */
#define WS_RECONNECT_MAX_MS      8000u  /* 指数退避上限 */
#define WS_RECONNECT_MAX_RETRY   6u

/* WebSocket opcode */
#define WS_OP_CONT               0x0u
#define WS_OP_TEXT               0x1u
#define WS_OP_BINARY             0x2u
#define WS_OP_CLOSE              0x8u
#define WS_OP_PING               0x9u
#define WS_OP_PONG               0xAu

/* WebSocket close code */
#define WS_CLOSE_NORMAL          1000u
#define WS_CLOSE_GOING_AWAY      1001u
#define WS_CLOSE_PROTOCOL_ERR    1002u
#define WS_CLOSE_BAD_DATA        1003u
#define WS_CLOSE_INTERNAL_ERR    1011u

/* ------------------------------------------------------------------ */
/* 调度器                                                              */
/* ------------------------------------------------------------------ */
#define SCHED_TASK_MAX           8u
#define SCHED_PRIO_LEVELS        6u
#define SCHED_DEFAULT_SLICE_MS   5u
#define SCHED_MAX_BLOCK_MS       60000u

/* 任务优先级（数值越大越紧急） */
#define PRIO_POWER               1u
#define PRIO_COMM                2u
#define PRIO_UI                  3u
#define PRIO_KEY                 4u
#define PRIO_SENSOR              5u

/* 邮箱深度与消息长度 */
#define MBOX_DEPTH_SENSOR        8u
#define MBOX_DEPTH_UI            4u
#define MBOX_DEPTH_COMM          8u
#define MBOX_MSG_LEN             16u

/* ------------------------------------------------------------------ */
/* 低功耗                                                              */
/* ------------------------------------------------------------------ */
#define POWER_IDLE_TO_SLEEP_MS   15000u /* 无活动该时长后进入休眠 */
#define POWER_TOUCH_WAKE_MS      8000u  /* 触摸唤醒后的亮屏保持时间 */

/* ------------------------------------------------------------------ */
/* 计步算法                                                            */
/* ------------------------------------------------------------------ */
#define STEP_FILTER_SHIFT        3u     /* 加速度模值一阶低通：1/8 新值 */
#define STEP_BASELINE_SHIFT      5u     /* 重力基线滑动平均：1/32 */
#define STEP_MIN_THRESHOLD_MG    55    /* 最小动态阈值 (mg) */
#define STEP_HYSTERESIS_MG       25    /* 迟滞带 (mg) */
#define STEP_REFRACTORY_MS       250u   /* 不应期：<250ms 不可能是新步 */
#define STEP_MAX_CADENCE_SPM     240u
#define STEP_MIN_CADENCE_SPM     30u

/* ------------------------------------------------------------------ */
/* 电量计                                                              */
/* ------------------------------------------------------------------ */
#define BAT_CELL_CAPACITY_MAH    180    /* 手表电池额定容量 */
#define BAT_CELL_FULL_MV         4200   /* 满电电压 */
#define BAT_CELL_EMPTY_MV        3300   /* 截止电压 */
#define BAT_DIVIDER_NUM          2      /* 分压比 R1=R2 => /2 */
#define BAT_ADC_VREF_MV          3300   /* VDDA 标称值，运行时用 VREFINT 校准 */
#define BAT_ADC_MAX_CODE         4095
#define BAT_VREFINT_MV           1210   /* 数据手册典型值 */
#define BAT_VREFINT_CAL_CODE     1520   /* 出厂校准值（0x1FFF7A2A 读回） */
#define BAT_ADC_OVERSAMPLE       16u    /* 过采样次数，等效 +2bit */

#endif /* BAND_CONFIG_H */
