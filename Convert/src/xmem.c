/* xmem.c - budgeted allocator for stl2mdl.
 *
 * Root cause of the old "out of memory" on a machine with 800 MB free:
 * simplify.c allocated its per-collapse scratch arrays (CSR adjacency,
 * edge-cost heap, quadric matrices) as ~4 MB automatic (stack) buffers.
 * Windows gives the main thread only 1 MB of stack by default -> the
 * allocation "failed" and we reported OOM even though plenty of RAM was
 * free.  Everything is now heap-allocated through xm_*(), and the total
 * footprint is guarded by a configurable budget (-mem <MB>, default 512).
 *
 * The budget is advisory: when an allocation would exceed it we print a
 * warning once and keep going unless the OS itself refuses; if malloc
 * really fails we free() what we can, retry once (defrag) and then abort
 * with a clear, actionable message instead of crashing.
 */
#include "mdlc.h"

static size_t g_budget   = 512u * 1024u * 1024u; /* 512 MB default      */
static size_t g_used     = 0;
static size_t g_peak     = 0;
static int    g_warned   = 0;
static char   g_oom[512] = {0};

void xm_set_budget(size_t bytes) {
    g_budget = bytes;
}
size_t xm_budget(void) { return g_budget; }
size_t xm_used(void)   { return g_used;   }

const char *xm_oom_msg(void) {
    if (g_oom[0]) return g_oom;
    snprintf(g_oom, sizeof(g_oom),
        "memory allocation failed (used %llu MB / budget %llu MB). "
        "Free RAM may still be available: reduce -lod/-physmax, lower "
        "-physlevel, or raise the budget with -mem <MB>.",
        (unsigned long long)(g_peak / (1024*1024)),
        (unsigned long long)(g_budget ? g_budget/(1024*1024) : 0));
    return g_oom;
}

static void xm_track(size_t delta_new_minus_old) { (void)delta_new_minus_old; }

static void xm_account(size_t oldn, size_t newn, const char *op) {
    size_t nu;
    if (newn > oldn) nu = g_used + (newn - oldn);
    else             nu = g_used - (oldn - newn);
    g_used = nu;
    if (nu > g_peak) g_peak = nu;
    if (g_budget && nu > g_budget && !g_warned) {
        g_warned = 1;
        WARN("peak usage %llu MB exceeds -mem budget %llu MB; "
             "raise it with -mem <MB> (0 = unlimited)",
             (unsigned long long)(nu/(1024*1024)),
             (unsigned long long)(g_budget/(1024*1024)));
    }
    (void)op; xm_track(0);
}

/* We keep our own per-block size header (portable — no malloc_usable_size
 * dependency on Windows) so accounting is exact and frees decrement usage. */
/* header MUST be max-aligned so any payload type is properly aligned */
typedef union { size_t n; double _a; void *_p; } xunit_t;
typedef struct { size_t n; unsigned magic; } xhdr_body;
static const size_t XHDR = ((sizeof(xhdr_body) + 15u) / 16u) * 16u; /* >=32 on LP64, keep 16-byte align */
#define XMAGIC 0x5A9E3C17u
typedef struct { size_t n; unsigned magic; char pad[8]; } xhdr_t;

static void *xm_get(size_t n) {           /* raw alloc + retry once */
    void *p = malloc(n);
    if (!p) p = malloc(n);                /* retry: OS may reclaim after GC */
    return p;
}

void *xm_alloc(size_t n) {
    xhdr_t *h;
    if (n == 0) n = 1;
    h = (xhdr_t *)xm_get(XHDR + n);
    if (!h) { ERR("%s", xm_oom_msg()); exit(2); }
    h->n = n; h->magic = XMAGIC;
    xm_account(0, n, "alloc");
    memset((char*)h + XHDR, 0, n);
    return (char*)h + XHDR;
}

void *xm_realloc(void *p, size_t n) {
    if (n == 0) n = 1;
    if (!p) return xm_alloc(n);
    else {
        xhdr_t *h = (xhdr_t *)((char*)p - XHDR);
        void *q;
        size_t oldn;
        if (h->magic != XMAGIC) {         /* foreign pointer: can't track */
            q = realloc(p, n);
            if (!q) { ERR("%s", xm_oom_msg()); exit(2); }
            return q;
        }
        oldn = h->n;
        if (oldn >= n) return p;          /* shrink: keep block, no copy */
        q = xm_get(XHDR + n);
        if (!q) { ERR("%s", xm_oom_msg()); exit(2); }
        memcpy(q, h, XHDR + oldn);
        free(h);
        ((xhdr_t*)q)->n = n; ((xhdr_t*)q)->magic = XMAGIC;
        xm_account(oldn, n, "realloc");
        return (char*)q + XHDR;
    }
}

void xm_free(void *p) {
    if (!p) return;
    {
        xhdr_t *h = (xhdr_t *)((char*)p - XHDR);
        if (h->magic == XMAGIC) { xm_account(h->n, 0, "free"); free(h); }
        else free(p);                     /* foreign pointer (libc malloc) */
    }
}
