/* buf.c - growable byte buffer + string table (mirrors studiomdl's
 * FileBuf + strings section handling from write.cpp). */
#include "mdlc.h"

/* declared here, defined in buf.c, consumed by mdlwrite.c */
long  str_table_size(void);
void *str_table_data(long *len);

static void *xrealloc_p(void *p, size_t n) {
    void *q = realloc(p, n ? n : 1);
    if (!q) { ERR("out of memory"); exit(2); }
    return q;
}

void buf_init(buf_t *b) {
    b->cap = 65536;
    b->size = 0;
    b->data = (unsigned char *)xrealloc_p(NULL, b->cap);
}

void buf_free(buf_t *b) { free(b->data); b->data = NULL; b->size = b->cap = 0; }

static void buf_need(buf_t *b, long n) {
    while (b->size + n > b->cap) {
        b->cap *= 2;
        b->data = (unsigned char *)xrealloc_p(b->data, b->cap);
    }
}

long buf_pad(buf_t *b, int align) {
    long off = b->size;
    long rem = off % align;
    if (rem) {
        static const unsigned char zeros[16] = {0};
        buf_need(b, align - rem);
        buf_put(b, zeros, align - rem);
    }
    return b->size;
}

void buf_put(buf_t *b, const void *p, long n) {
    buf_need(b, n);
    memcpy(b->data + b->size, p, n);
    b->size += n;
}

int buf_putf(buf_t *b, float f) {
    int off = (int)b->size;
    buf_put(b, &f, 4);
    return off;
}

int buf_puti(buf_t *b, int v) {
    int off = (int)b->size;
    buf_put(b, &v, 4);
    return off;
}

int buf_putvec(buf_t *b, vec3d_t v) {
    int off = (int)b->size;
    buf_put(b, &v, sizeof(v));
    return off;
}

/* ------------------------------------------------------------- strtab */
void str_init(strtab_t *st, long cap) {
    memset(st, 0, sizeof(*st));          /* strings live in g_stage below;
                                            st->b stays empty and buf_free
                                            on it is a harmless no-op. */
    if (cap < 1024) cap = 1024;
    st->cap = cap;
    st->used = 0;
    st->strings_start = 0;
}

/* Strings are staged in their own malloc'd arena, appended to file later. */
typedef struct { char *arena; long used, cap; } stage_t;
static stage_t g_stage = {NULL, 0, 0};

int str_add(strtab_t *st, const char *s) {
    long len = strlen(s) + 1;
    if (!g_stage.arena) { g_stage.cap = 8192; g_stage.arena = (char*)xrealloc_p(NULL, g_stage.cap); g_stage.used = 0; }
    while (g_stage.used + len > g_stage.cap) {
        g_stage.cap *= 2;
        g_stage.arena = (char *)xrealloc_p(g_stage.arena, g_stage.cap);
    }
    memcpy(g_stage.arena + g_stage.used, s, len);
    /* index is 1-based offset into string table (offset 0 reserved) */
    int idx = (int)g_stage.used + 1;
    g_stage.used += len;
    (void)st;
    return idx;
}

void str_write(strtab_t *st, FILE *f) {
    long pad = (16 - (g_stage.used % 16)) % 16;
    static const char zeros[16] = {0};
    fwrite(g_stage.arena, 1, g_stage.used, f);
    if (pad) fwrite(zeros, 1, pad, f);
    st->used = g_stage.used + pad;
}

long str_table_size(void) {
    long pad = (16 - (g_stage.used % 16)) % 16;
    return g_stage.used + pad;
}

void *str_table_data(long *len) {
    long pad = (16 - (g_stage.used % 16)) % 16;
    if (pad) {
        static const char zeros[16] = {0};
        g_stage.arena = (char *)xrealloc_p(g_stage.arena, g_stage.used + pad);
        memcpy(g_stage.arena + g_stage.used, zeros, pad);
        g_stage.used += pad;
    }
    *len = g_stage.used;
    return g_stage.arena;
}
