/* mdlc.h - shared types/helpers for stl2mdl: a self-contained C compiler of
 * Source (2007, studiohdr v48 / VVD v4) models from STL meshes.
 *
 * Implements, from scratch (logic modeled on Valve's studiomdl write.cpp /
 * collisionmodel.cpp / simplify.cpp):
 *   - STL (binary + ASCII) reader with vertex welding and smooth normals
 *   - progressive-decimation mesh downgrade (visual LODs + mandatory physics
 *     decimation so the physics hull is ALWAYS cheaper than the source mesh)
 *   - convex-piece decomposition of the collision mesh (VHACD-style greedy
 *     splitting), mass properties (volume, center of mass, inertia tensor)
 *   - MDL (studiohdr 48) + VVD (v4) writers, byte-identical layout to the SDK
 */
#ifndef MDLC_H
#define MDLC_H

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "studio_c.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --------------------------------------------------------------- vectors */
static __inline__ void v_sub(const vec3d_t a, const vec3d_t b, vec3d_t *o) {
    o->x = a.x - b.x; o->y = a.y - b.y; o->z = a.z - b.z;
}
static __inline__ void v_add(const vec3d_t a, const vec3d_t b, vec3d_t *o) {
    o->x = a.x + b.x; o->y = a.y + b.y; o->z = a.z + b.z;
}
static __inline__ void v_scale(vec3d_t *v, float s) {
    v->x *= s; v->y *= s; v->z *= s;
}
static __inline__ float v_dot(const vec3d_t a, const vec3d_t b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
static __inline__ void v_cross(const vec3d_t a, const vec3d_t b, vec3d_t *o) {
    o->x = a.y*b.z - a.z*b.y;
    o->y = a.z*b.x - a.x*b.z;
    o->z = a.x*b.y - a.y*b.x;
}
static __inline__ float v_len(const vec3d_t a) { return sqrtf(v_dot(a,a)); }
static __inline__ void v_norm(vec3d_t *v) {
    float l = v_len(*v);
    if (l > 1e-12f) { v->x/=l; v->y/=l; v->z/=l; }
}

/* ----------------------------------------------------------------- mesh */
typedef struct {
    vec3d_t *v;          /* positions                     */
    vec3d_t *n;          /* per-vertex normals            */
    unsigned int *tri;   /* triangle indices, 3 per face  */
    unsigned int nv, nt;
} mesh_t;

typedef struct {
    /* options */
    char    src[512];
    char    out[512];        /* basename without extension              */
    char    material[128];   /* $material name written into textureline */
    float   scale;           /* STL units -> Source inches              */
    int     lodtris[6];      /* target tri counts per visual LOD        */
    int     nlods;
    int     physlevel;       /* 0..5, physics decimation strength       */
    int     physmaxtris;     /* hard cap for collision tris             */
    int     physpieces;      /* max convex pieces                       */
    float   mass;
    float   upgradeweights;  /* keep original weights (unused knob)     */
    int     nomerge;         /* disable vertex welding                  */
    int     verbose;
    int     forcephys;       /* always downgrade physics even w/o flag  */
    size_t  membytes;        /* memory budget for the allocator pool    */
} opts_t;

/* ------------------------------------------------------------- xmemory */
/* All big allocations go through xm*() helpers: a single realloc-based
 * arena with a configurable budget (-mem <MB>, default 512 MB).  When the
 * budget would be exceeded we free() and retry once (defragment), then
 * fail with a clear message instead of an opaque crash. */
void  *xm_alloc(size_t n);           /* calloc-like, zeroed            */
void  *xm_realloc(void *p, size_t n);/* realloc that never returns NULL silently */
void   xm_free(void *p);
size_t xm_used(void);
size_t xm_budget(void);
void   xm_set_budget(size_t bytes);  /* 0 = unlimited                  */
const char *xm_oom_msg(void);        /* human-readable OOM hint        */

/* ----------------------------------------------------------- strtab/buf */
typedef struct {
    unsigned char *data;
    long size, cap;
} buf_t;

typedef struct {
    buf_t b;
    long  strings_start;   /* offset of string section inside b.data     */
    long  used;            /* bytes used in string section               */
    long  cap;             /* capacity of string section                 */
} strtab_t;

void  buf_init(buf_t *b);
void  buf_free(buf_t *b);
long  buf_pad(buf_t *b, int align);           /* returns new offset      */
void  buf_put(buf_t *b, const void *p, long n);
int   buf_putf(buf_t *b, float f);            /* returns offset          */
int   buf_puti(buf_t *b, int v);
int   buf_putvec(buf_t *b, vec3d_t v);

void  str_init(strtab_t *st, long cap);
int   str_add(strtab_t *st, const char *s);   /* returns index (>=1)     */
void  str_write(strtab_t *st, FILE *f);

/* ------------------------------------------------------------ mesh utils */
mesh_t *mesh_new(void);
void    mesh_free(mesh_t *m);
int     mesh_load_stl(const char *path, mesh_t *m, int *was_ascii, float dims[3]);
void    mesh_weld(mesh_t *m, float eps);
void    mesh_normals(mesh_t *m);
void    mesh_bounds(const mesh_t *m, vec3d_t *mn, vec3d_t *mx);
float   mesh_area(const mesh_t *m);
void    mesh_transform(mesh_t *m, float (*rot)[3], const vec3d_t *offset, float scale);

/* progressive decimation: collapse edges until nt <= target. Returns new
 * mesh (original untouched). target==0 means no reduction. */
mesh_t *mesh_decimate(const mesh_t *src, unsigned int target);

/* convex hull (incremental, brute-force over faces) - returns hull mesh */
mesh_t *mesh_convex_hull(const mesh_t *m);

/* decompose collision mesh into convex pieces (greedy plane split) */
int  mesh_decompose_convex(const mesh_t *m, int max_pieces, float trim_eps,
                           mesh_t ***out_pieces, int *out_count);

/* mass properties of a closed triangle soup (signed-volume method) */
void mesh_massprops(const mesh_t *m, float density,
                    float volume_out[1], vec3d_t *com, float inertia[6]);

/* ------------------------------------------------------------- physics */
typedef struct {
    mesh_t  **pieces;      /* convex pieces                              */
    int       npieces;
    vec3d_t   com;
    float     vol;
    float     inertia[6];  /* xx yy zz xy xz yz about COM (unit mass)    */
    float     mass;
} phys_t;

void phys_build(const mesh_t *coll, const opts_t *o, phys_t *p);
void phys_free(phys_t *p);

/* --------------------------------------------------------------- output */
int  write_mdl_vvd(const char *basepath, const mesh_t *vis, mesh_t *lods[],
                   int nlods, const phys_t *phys, const opts_t *o);
int  vtx_write(const char *basepath, const mesh_t *vis, mesh_t *lods[],
               int nlods, const opts_t *o);   /* .vtx (VVP) - required by engine */

#define MSG(o, ...) do { if ((o)->verbose) { printf("[stl2mdl] "); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define INFO(...)  do { printf("[stl2mdl] "); printf(__VA_ARGS__); printf("\n"); } while (0)
#define ERR(...)   do { fprintf(stderr, "[stl2mdl:error] "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while (0)
#define WARN(...)  do { printf("[stl2mdl:warn] "); printf(__VA_ARGS__); printf("\n"); } while (0)

#endif /* MDLC_H */
