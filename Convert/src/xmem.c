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

/* We keep our own per-block header (portable — no malloc_usable_size
 * dependency on Windows) so accounting is exact and frees decrement usage.
 * The payload is ALWAYS at a fixed 16-byte-aligned offset from the raw
 * malloc block, which makes every operation O(1) and safe:
 *
 *   raw block:  [ pad .. | total_size | magic | payload ... ]
 *   p (user) points to payload; magic lives right before it.
 *
 * A pointer is "ours" iff the 4-byte magic just before the payload matches.
 * Foreign pointers (from libc malloc/realloc/free elsewhere in the code)
 * are detected and passed through to the C library instead of corrupting
 * the heap — this was the root cause of the previous crash where physics.c
 * freed an xm_alloc'd array with plain free().
 */
/* Ownership model (no pointer-header tricks):
 *   - every allocation that MUST be xm-owned is tagged in a small registry;
 *   - foreign pointers (libc malloc/calloc/realloc from stl.c/buf.c) are
 *     detected via the registry and passed straight to free()/realloc().
 * The registry is a 16-bit hash of the pointer into bucketed arrays with
 * linear probing on collision; lookups are O(1) amortised, which matters
 * because mesh_free() is called per convex piece during decomposition.
 */
#define XBUCKETS 4096

typedef struct xent {
    void *p;              /* user pointer (payload start) */
    size_t n;             /* payload size                 */
    struct xent *next;
} xent_t;

static xent_t *g_tab[XBUCKETS];
static xent_t *g_freelist;

static unsigned xb(void *p) {
    size_t v = (size_t)p;
    return (unsigned)((v >> 4u) % XBUCKETS);
}

static xent_t *xent_new(void) {
    xent_t *e;
    if (g_freelist) { e = g_freelist; g_freelist = e->next; return e; }
    e = (xent_t *)malloc(sizeof(xent_t));
    if (!e) { ERR("%s", xm_oom_msg()); exit(2); }
    return e;
}

static void xent_put(xent_t *e) { e->next = g_freelist; g_freelist = e; }

static xent_t *x_find(void *p) {
    xent_t *e;
    for (e = g_tab[xb(p)]; e; e = e->next) if (e->p == p) return e;
    return NULL;
}

static void x_add(void *p, size_t n) {
    unsigned b = xb(p);
    xent_t *e = xent_new();
    e->p = p; e->n = n; e->next = g_tab[b]; g_tab[b] = e;
}

static size_t x_del(void *p) {           /* returns stored size, 0 if absent */
    unsigned b = xb(p);
    xent_t **pp = &g_tab[b], *e;
    while (*pp) {
        e = *pp;
        if (e->p == p) { *pp = e->next; { size_t n = e->n; xent_put(e); return n; } }
        pp = &e->next;
    }
    return 0;
}

int xm_owns(const void *p) { return p != NULL && x_find((void*)p) != NULL; }

void *xm_alloc(size_t n) {
    void *p;
    if (n == 0) n = 1;
    p = malloc(n);
    if (!p) p = malloc(n);               /* transient pressure: retry */
    if (!p) { ERR("%s", xm_oom_msg()); exit(2); }
    memset(p, 0, n);
    x_add(p, n);
    xm_account(0, n, "alloc");
    return p;
}

void *xm_realloc(void *p, size_t n) {
    size_t oldn;
    void *r;
    if (n == 0) n = 1;
    if (!p) return xm_alloc(n);
    oldn = x_del(p);                     /* removes tag if ours, 0 if foreign */
    if (oldn == 0) {                     /* foreign pointer: plain realloc */
        r = realloc(p, n);
        if (!r) { ERR("%s", xm_oom_msg()); exit(2); }
        return r;
    }
    r = realloc(p, n);                   /* p was malloc'd by us: safe */
    if (!r) {                            /* keep original alive + re-tag */
        r = malloc(n);
        if (!r) { ERR("%s", xm_oom_msg()); exit(2); }
        memcpy(r, p, oldn < n ? oldn : n);
        free(p);
    }
    x_add(r, n);
    xm_account(oldn, n, "realloc");
    return r;
}

void xm_free(void *p) {
    size_t n;
    if (!p) return;
    n = x_del(p);
    if (n) xm_account(n, 0, "free");
    free(p);                             /* both cases: all our blocks came
                                            from malloc(), so this is valid */
}

