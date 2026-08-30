#include "ring_buf.h"

void RingBuf_Init(RingBuf_t *rb, uint8_t *buf, uint16_t size)
{
    rb->buf  = buf;
    rb->size = size;
    rb->rd   = 0;
    rb->wr   = 0;
}

uint8_t RingBuf_Write(RingBuf_t *rb, uint8_t byte)
{
    uint16_t next;

    if (rb->size == 0) return 0;

    /* 下一个写入位置（到顶绕回）*/
    next = rb->wr + 1;
    if (next >= rb->size) next = 0;

    /* 满：wr 将追上 rd（牺牲一格），丢弃这个字节 */
    if (next == rb->rd) return 0;

    /* 注意顺序：先写数据，再发布 wr（读侧看到新 wr 时数据一定已就绪）*/
    rb->buf[rb->wr] = byte;
    rb->wr = next;
    return 1;
}

uint8_t RingBuf_Read(RingBuf_t *rb, uint8_t *out)
{
    if (rb->rd == rb->wr) return 0;   /* 空 */

    *out = rb->buf[rb->rd];
    rb->rd++;
    if (rb->rd >= rb->size) rb->rd = 0;
    return 1;
}

uint8_t RingBuf_IsEmpty(RingBuf_t *rb)
{
    return (rb->rd == rb->wr) ? 1 : 0;
}

uint16_t RingBuf_Count(RingBuf_t *rb)
{
    if (rb->wr >= rb->rd)
        return (uint16_t)(rb->wr - rb->rd);
    return (uint16_t)(rb->size - rb->rd + rb->wr);
}
