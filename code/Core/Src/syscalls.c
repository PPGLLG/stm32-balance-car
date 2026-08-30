/* Bare-metal system call stubs.
 * Replaces -specs=nosys.specs when the toolchain doesn't ship libnosys/newlib
 * (e.g. Homebrew arm-none-eabi-gcc >= 16.x).
 * Works on macOS / Windows / Linux — pure C, no OS dependencies.
 */

#include "usart.h"        /* huart1（printf 重定向到调试串口）*/

typedef unsigned int size_t;
typedef long off_t;

struct stat {
    unsigned short st_mode;
    unsigned short st_ino;
    unsigned int   st_size;
    unsigned int   st_blksize;
};

#define S_IFCHR 0020000

/* ------------------------------------------------------------------ */
/* Init array — called by startup.s before main()                     */
/* ------------------------------------------------------------------ */
void __libc_init_array(void) { }

/* ------------------------------------------------------------------ */
/* Heap — sbrk for malloc() family                                    */
/* ------------------------------------------------------------------ */
extern char _end;   /* defined by linker script */

void *_sbrk(int incr) {
    static char *heap_end;
    char *prev;
    if (heap_end == 0) heap_end = &_end;
    prev = heap_end;
    heap_end += incr;
    return (void *)prev;
}

/* ------------------------------------------------------------------ */
/* File-descriptor stubs (used by printf / fopen etc.)                */
/* ------------------------------------------------------------------ */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    /* printf 重定向到 USART1（115200 调试串口，Vofa+ 看波形）*/
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
    return len;
}

int _read(int fd, char *ptr, int len) {
    (void)fd; (void)ptr; (void)len;
    return -1;
}

int _close(int fd) { (void)fd; return -1; }

off_t _lseek(int fd, off_t pos, int whence) {
    (void)fd; (void)pos; (void)whence;
    return (off_t)-1;
}

int _fstat(int fd, struct stat *st) {
    (void)fd;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd) { (void)fd; return 1; }

/* ------------------------------------------------------------------ */
/* Memory utils — needed by HAL (explicit calls, not compiler builtins) */
/* ------------------------------------------------------------------ */
void *memset(void *s, int c, size_t n) {
    unsigned char *p = (unsigned char *)s;
    while (n--) *p++ = (unsigned char)c;
    return s;
}

void *memcpy(void *dest, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dest;
    const unsigned char *s = (const unsigned char *)src;
    while (n--) *d++ = *s++;
    return dest;
}

void *memmove(void *dest, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dest;
    const unsigned char *s = (const unsigned char *)src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        d += n; s += n;
        while (n--) *--d = *--s;
    }
    return dest;
}

int memcmp(const void *a, const void *b, size_t n) {
    const unsigned char *pa = (const unsigned char *)a;
    const unsigned char *pb = (const unsigned char *)b;
    while (n--) {
        if (*pa != *pb) return *pa - *pb;
        pa++; pb++;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Process stubs                                                      */
/* ------------------------------------------------------------------ */
void _exit(int status) { (void)status; while (1) __asm__("nop"); }

int _kill(int pid, int sig) {
    (void)pid; (void)sig;
    return -1;
}

int _getpid(void) { return 1; }
