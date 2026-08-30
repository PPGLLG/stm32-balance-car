#ifndef _RING_BUF_H
#define _RING_BUF_H

#include <stdint.h>

/* ============ 环形缓冲（单生产者单消费者，无锁）============
 * 用途：ISR 写字节，任务读字节（比如 WiFi 串口接收）
 *
 * 结构：数组 + 写指针 wr + 读指针 rd
 *   - wr 只被 ISR 动，rd 只被任务动；另一侧只"读"对方的指针
 *   - 两个指针都 volatile：跨中断/任务共享，防编译器缓存旧值
 *   - 牺牲一格判空满（size 个字节实际只用 size-1）：
 *       rd == wr              → 空
 *       (wr+1) % size == rd   → 满
 *
 * 为什么无锁：单写单读，指针读写是原子的；"留一格"保证写者永远
 * 不会覆盖读者还没读的区域，读者也不会越过写者，天然不冲突。
 */
typedef struct {
    uint8_t  *buf;             /* 数据区（外部传入的数组）*/
    uint16_t  size;            /* 缓冲区长度（实际可用 size-1）*/
    volatile uint16_t rd;      /* 读指针（任务动，ISR 读它判满）*/
    volatile uint16_t wr;      /* 写指针（ISR 动，任务读它判空/快照）*/
} RingBuf_t;

void     RingBuf_Init(RingBuf_t *rb, uint8_t *buf, uint16_t size);
uint8_t  RingBuf_Write(RingBuf_t *rb, uint8_t byte);  /* 1=写入成功 0=满了丢弃 */
uint8_t  RingBuf_Read (RingBuf_t *rb, uint8_t *out);  /* 1=读到 0=空 */
uint8_t  RingBuf_IsEmpty(RingBuf_t *rb);              /* 1=空 */
uint16_t RingBuf_Count(RingBuf_t *rb);                /* 当前未读字节数 */

#endif /* _RING_BUF_H */
