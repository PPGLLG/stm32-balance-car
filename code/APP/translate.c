#include "translate.h"
#include "ir_nec.h"          /* IR_NEC_GetKey */
#include "control.h"         /* control_set_*（WiFi 实时调参）*/
#include "usart.h"           /* huart2（ESP01）*/
#include "FreeRTOS.h"        /* pdMS_TO_TICKS */
#include "task.h"            /* vTaskDelay */

/* ================= 红外：键码 → 命令 ================= */

/* 命令 ID */
typedef enum {
    CMD_NONE = 0,     /* 未映射/无效 */
    CMD_FORWARD,      /* 前进 */
    CMD_BACK,         /* 后退 */
    CMD_LEFT,         /* 左转 */
    CMD_RIGHT,        /* 右转 */
    CMD_STOP,         /* 停止 */
    CMD_SPEED_UP,     /* 加速档 */
    CMD_SPEED_DOWN,   /* 减速档 */
} ir_cmd_t;

/* 离散遥控器没有比例速度，用"速度档位"调速（数值等上板实测再调）*/
static uint8_t ir_speed_level = 1;    /* 当前档位 1~5 */
#define IR_SPEED_STEP   100.0f        /* 每档速度增量，初始档位 1 → 初始速度 100 */
#define IR_SPEED_MAX    5             /* 最高档 */
#define IR_TURN_VALUE   15.0f         /* 转向固定值 */

/* 键码(0~255) → 命令ID 映射表（C99 指定初始化）
 * 等遥控器到了，把真实键码填进来，例如：
 *   ir_key_map[0x45] = CMD_FORWARD;
 * 没填的默认 0（CMD_NONE），按了忽略。 */
static const uint8_t ir_key_map[256] = {
    /* [0x45] = CMD_FORWARD,  ← 示例，拿到遥控器后删掉注释填真的 */
    [0x18] = CMD_FORWARD,     /* 2  前进 */
    [0x10] = CMD_LEFT,        /* 4  左转 */
    [0x5A] = CMD_RIGHT,       /* 6  右转 */
    [0x4A] = CMD_BACK,        /* 8  后退 */
    [0x38] = CMD_STOP,        /* 5  停止 */
    [0x90] = CMD_SPEED_UP,    /* vol+ 加速档 */
    [0xA8] = CMD_SPEED_DOWN,  /* vol- 减速档 */
};

uint8_t translate_ir(RemoteData_t *data)
{
    uint8_t key = IR_NEC_GetKey();      /* 拿走键码（读一次自动清）*/
    uint8_t cmd = ir_key_map[key];      /* 查表：键码 → 命令ID */

    switch (cmd) {
        case CMD_FORWARD:               /* 前进 = 直走 */
            data->speed = (float)ir_speed_level * IR_SPEED_STEP;
            data->turn  = 0.0f;
            return 1;
        case CMD_BACK:
            data->speed = -(float)ir_speed_level * IR_SPEED_STEP;
            data->turn  = 0.0f;
            return 1;
        case CMD_LEFT:                  /* 左转 = 原地转 */
            data->speed = 0.0f;
            data->turn  = IR_TURN_VALUE;
            return 1;
        case CMD_RIGHT:
            data->speed = 0.0f;
            data->turn  = -IR_TURN_VALUE;
            return 1;
        case CMD_STOP:
            data->speed = 0.0f;
            data->turn  = 0.0f;
            return 1;
        case CMD_SPEED_UP:              /* 只调档，不产生运动命令 */
            if (ir_speed_level < IR_SPEED_MAX) ir_speed_level++;
            return 0;
        case CMD_SPEED_DOWN:
            if (ir_speed_level > 1) ir_speed_level--;
            return 0;
        default:
            return 0;                   /* 未映射键，忽略 */
    }
}

/* ================= WiFi：攒行 → 解析 ================= */

#define WIFI_LINE_MAX   48   /* 留足 +IPD 帧头 + 命令的空间 */

static char    wifi_line[WIFI_LINE_MAX];   /* 行缓冲 */
static uint8_t wifi_len;                   /* 已攒字节数 */

/* 极简浮点解析：支持前导空格、负号和小数点，如 " 5"、" -3.25"、"100"（不依赖 libc）*/
static float parse_float(const char *s)
{
    float sign = 1.0f, val = 0.0f, frac = 0.1f;

    while (*s == ' ' || *s == '\t') s++;   /* 跳过前导空格 */
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (float)(*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            val += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }
    return sign * val;
}

/* 剥 +IPD 壳：+IPD,0,4:S100 → 返回 "S100"（第一个冒号后就是数据）
 * 不是 +IPD 帧（没有冒号）→ 整行原样返回（兼容裸文本）*/
static char *extract_payload(char *line)
{
    char *p;
    for (p = line; *p; p++) {
        if (*p == ':') return p + 1;   /* 冒号后 = 数据 */
    }
    return line;   /* 没有冒号 = 普通命令 */
}

/* 判断一个字符是否可能作为数字开头 */
static uint8_t is_num_start(char c)
{
    return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.';
}

/* 解析一行：先剥 +IPD 壳，再按首字母解析
 *   运动命令：S<速度> / T<转向> / A<平衡角> / Z（全停）
 *   调参命令（WiFi 实时改 PID，直接写控制参数）：
 *     ap<值> = 角度环P(angle_kp)   ad<值> = 角度环D(angle_kd)
 *     sp<值> = 速度环P(speed_kp)   si<值> = 速度环I(speed_ki)
 *     tp<值> = 转向环P(turn_kp)    td<值> = 转向环D(turn_kd)
 *   例：手机发 "ap-0.8\n" 就把 angle_kp 设为 -0.8
 *
 * 注意：A/S/T 必须要求后面跟数字，否则 ESP01 的 AT 回显
 * （如 AT+CIPSEND、SEND OK）会被误当成运动命令。 */
static uint8_t parse_line(RemoteData_t *data, char *line)
{
    char *cmd = extract_payload(line);   /* 剥壳得命令 */
    /* 运动命令按首字母：S/T/A/Z */
    switch (cmd[0]) {
        case 'S':
            if (!is_num_start(cmd[1])) return 0;
            data->speed = parse_float(&cmd[1]);
            return 1;
        case 'T':
            if (!is_num_start(cmd[1])) return 0;
            data->turn  = parse_float(&cmd[1]);
            return 1;
        case 'A':
            if (!is_num_start(cmd[1])) return 0;
            data->angle = parse_float(&cmd[1]);
            return 1;
        case 'Z':
            data->speed = 0.0f;
            data->turn  = 0.0f;
            return 1;
        /* —— 调参命令：前两字符匹配 ap/ad/sp/si/tp/td —— */
        case 'a':
            if (cmd[1] == 'p')      { control_set_angle_kp(parse_float(&cmd[2])); return 1; }
            if (cmd[1] == 'd')      { control_set_angle_kd(parse_float(&cmd[2])); return 1; }
            return 0;
        case 's':
            if (cmd[1] == 'p')      { control_set_speed_kp(parse_float(&cmd[2])); return 1; }
            if (cmd[1] == 'i')      { control_set_speed_ki(parse_float(&cmd[2])); return 1; }
            return 0;
        case 't':
            if (cmd[1] == 'p')      { control_set_turn_kp(parse_float(&cmd[2])); return 1; }
            if (cmd[1] == 'd')      { control_set_turn_kd(parse_float(&cmd[2])); return 1; }
            return 0;
        default:
            return 0;
    }
}

/* ============ WiFi 波形发送（Vofa 经 ESP01 TCP 无线看波形）============ */
volatile uint8_t  g_vofa_ready    = 0;
volatile uint16_t g_vofa_line_len = 0;
char g_vofa_line[96];
static uint8_t g_wifi_clients = 0;   /* bit0~4 = 当前在线 TCP 客户端（最多5个）*/

/* ESP01 多连接模式下，客户端连上/断开会输出 "0,CONNECT" / "0,CLOSED" 事件行 */
static void handle_connect_event(const char *line)
{
    if (line[0] < '0' || line[0] > '4' || line[1] != ',') return;
    uint8_t id = (uint8_t)(line[0] - '0');
    if (line[2] == 'C' && line[3] == 'O' && line[4] == 'N')      g_wifi_clients |=  (uint8_t)(1 << id);
    else if (line[2] == 'C' && line[3] == 'L')                   g_wifi_clients &= (uint8_t)~(1 << id);
}

/* 把波形数据广播发给所有已连接的 WiFi 客户端（Vofa / 手机）。
 * 流程：AT+CIPSEND=<id>,<len>\r\n → 等 ESP 回 ">" → 发数据。
 * 简化版：发 AT 后延时 20ms 再发数据（不检查响应，偶尔丢帧可接受）。
 * ⚠️ 只能在非控制任务（remoteTask）调用：阻塞约 25ms/客户端，
 *    在控制任务里调用会破坏 10ms 控制节拍！ */
void wifi_vofa_send(const char *data, uint16_t len)
{
    uint8_t id;

    for (id = 0; id < 5; id++) {
        if (!(g_wifi_clients & (1 << id))) continue;

        char at[24];
        uint16_t n = 0;
        const char *hdr = "AT+CIPSEND=";
        while (*hdr) at[n++] = *hdr++;
        at[n++] = (char)('0' + id);
        at[n++] = ',';
        if (len >= 100) { at[n++] = (char)('0' + len / 100); }
        if (len >= 10)  { at[n++] = (char)('0' + (len / 10) % 10); }
        at[n++] = (char)('0' + len % 10);
        at[n++] = '\r';
        at[n++] = '\n';

        HAL_UART_Transmit(&huart2, (uint8_t *)at, n, 100);
        vTaskDelay(pdMS_TO_TICKS(20));          /* 等 ESP 回 ">" */
        HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
    }
}

/* 通用攒行解析：从环形缓冲读字节，遇换行解析一条命令。 */
static uint8_t translate_lines(RemoteData_t *data, RingBuf_t *rb,
                               char *line, uint8_t *len)
{
    uint8_t b;
    uint8_t produced = 0;

    while (RingBuf_Read(rb, &b)) {
        if (b == '\n') {                        /* 一行结束 */
            line[*len] = '\0';
            handle_connect_event(line);         /* WiFi 会用来维护客户端列表 */
            uint8_t r = parse_line(data, line); /* 再解析命令 */
            if (r) produced = r;                /* 成功才更新，垃圾/空行不覆盖 */
            *len = 0;
        } else if (b != '\r') {                 /* 丢 \r，其余攒进 */
            if (*len < WIFI_LINE_MAX - 1) {
                line[(*len)++] = (char)b;
            }
        }
    }
    return produced;
}

/* 读 WiFi 环形缓冲攒行，遇换行解析一条命令。
 * 一次可能攒多条命令，返回最后一条的结果。 */
uint8_t translate_wifi(RemoteData_t *data, RingBuf_t *rb)
{
    return translate_lines(data, rb, wifi_line, &wifi_len);
}
