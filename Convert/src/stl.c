/* stl.c - STL reader (binary + ASCII), vertex welding, smooth normals.
 * Coordinate conversion Z-up (typical CAD) -> Source Y-up is done in main
 * via mesh_transform(); here we only load raw geometry.
 */
#include "mdlc.h"

mesh_t *mesh_new(void) {
    mesh_t *m = (mesh_t *)calloc(1, sizeof(mesh_t));
    return m;
}

void mesh_free(mesh_t *m) {
    if (!m) return;
    free(m->v); free(m->n); free(m->tri); free(m);
}

static void *xrealloc(void *p, size_t n) {
    void *q = realloc(p, n ? n : 1);
    if (!q) { ERR("out of memory"); exit(2); }
    return q;
}

/* ---------------------------------------------------------- binary STL */
static int stl_load_binary(FILE *f, mesh_t *m) {
    unsigned char hdr[80];
    unsigned int  ntri, i;
    if (fread(hdr, 1, 80, f) != 80) return 0;
    if (fread(&ntri, 4, 1, f) != 1) return 0;
    if (ntri == 0 || ntri > 50000000u) return 0;

    long cur = ftell(f);
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, cur, SEEK_SET);
    if ((unsigned long)(sz - cur) < (unsigned long)ntri * 50u) return 0;

    m->nt = ntri;
    m->nv = ntri * 3;
    m->v   = (vec3d_t *)xrealloc(NULL, sizeof(vec3d_t) * m->nv);
    m->n   = (vec3d_t *)xrealloc(NULL, sizeof(vec3d_t) * m->nv);
    m->tri = (unsigned int *)xrealloc(NULL, sizeof(unsigned int) * m->nv);

    for (i = 0; i < ntri; i++) {
        float rec[12]; /* normal(3) v0(3) v1(3) v2(3) attrib(2 bytes packed) */
        unsigned short attr;
        if (fread(rec, 4, 12, f) != 12) { ERR("truncated binary STL"); return 0; }
        if (fread(&attr, 2, 1, f) != 1) { ERR("truncated binary STL"); return 0; }
        m->tri[i*3+0] = i*3+0;
        m->tri[i*3+1] = i*3+1;
        m->tri[i*3+2] = i*3+2;
        m->v[i*3+0].x = rec[3];  m->v[i*3+0].y = rec[4];  m->v[i*3+0].z = rec[5];
        m->v[i*3+1].x = rec[6];  m->v[i*3+1].y = rec[7];  m->v[i*3+1].z = rec[8];
        m->v[i*3+2].x = rec[9];  m->v[i*3+2].y = rec[10]; m->v[i*3+2].z = rec[11];
    }
    return 1;
}

/* ------------------------------------------------------------ ASCII STL */
static int stl_load_ascii(FILE *f, mesh_t *m) {
    char line[512];
    unsigned int cap = 4096, nvmax = cap;
    vec3d_t *vs = (vec3d_t *)xrealloc(NULL, sizeof(vec3d_t)*nvmax);
    unsigned int nv = 0, nt = 0;
    rewind(f);

    while (fgets(line, sizeof(line), f)) {
        float x, y, z;
        if (sscanf(line, " vertex %f %f %f", &x, &y, &z) == 3 ||
            sscanf(line, "vertex %f %f %f", &x, &y, &z) == 3) {
            if (nv >= nvmax) {
                nvmax *= 2;
                vs = (vec3d_t *)xrealloc(vs, sizeof(vec3d_t)*nvmax);
            }
            vs[nv].x = x; vs[nv].y = y; vs[nv].z = z;
            nv++;
            if (nv % 3 == 0) nt++;
        }
    }
    if (nv < 3 || nv % 3 != 0) { free(vs); return 0; }
    m->nv = nv; m->nt = nt;
    m->v = vs;
    m->n = (vec3d_t *)xrealloc(NULL, sizeof(vec3d_t)*nv);
    m->tri = (unsigned int *)xrealloc(NULL, sizeof(unsigned int)*nv);
    {
        unsigned int i;
        for (i = 0; i < nv; i++) m->tri[i] = i;
    }
    return 1;
}

/* ---------------------------------------------------------------- weld */
/* hash-grid welding: merges coincident vertices within eps */
typedef struct { unsigned int *head; int mask; } weldgrid_t;

static unsigned int wkey(unsigned int ix, unsigned int iy, unsigned int iz) {
    /* 32-bit spatial hash */
    unsigned int h = ix * 1640531527u ^ iy * 2654435761u ^ iz * 2246822519u;
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15;
    return h;
}

void mesh_weld(mesh_t *m, float eps) {
    unsigned int i, n = m->nv, mask = 1;
    unsigned int *next, *head, *remap;
    if (eps <= 0) eps = 1e-5f;
    while (mask < n * 2u) mask <<= 1;
    mask -= 1;
    head = (unsigned int *)xrealloc(NULL, sizeof(unsigned int)*(mask+1));
    next = (unsigned int *)xrealloc(NULL, sizeof(unsigned int)*n);
    remap= (unsigned int *)xrealloc(NULL, sizeof(unsigned int)*n);
    for (i = 0; i <= mask; i++) head[i] = 0xffffffffu;

    vec3d_t *cv = (vec3d_t *)xrealloc(NULL, sizeof(vec3d_t)*n);
    unsigned int nc = 0;
    float inv = 1.0f / eps;

    for (i = 0; i < n; i++) {
        int gx = (int)floorf(m->v[i].x * inv), gy = (int)floorf(m->v[i].y * inv),
            gz = (int)floorf(m->v[i].z * inv);
        unsigned int h = wkey((unsigned)gx, (unsigned)gy, (unsigned)gz) & mask;
        unsigned int found = 0xffffffffu, k;
        for (k = head[h]; k != 0xffffffffu; k = next[k]) {
            vec3d_t d; v_sub(m->v[i], cv[k], &d);
            if (fabsf(d.x) <= eps && fabsf(d.y) <= eps && fabsf(d.z) <= eps) {
                found = k; break;
            }
        }
        if (found == 0xffffffffu) {
            found = nc;
            cv[nc++] = m->v[i];
            next[found] = head[h];
            head[h] = found;
        }
        remap[i] = found;
    }

    /* rebuild triangles with dedup (drop degenerate) */
    {
        unsigned int *tri2 = (unsigned int *)xrealloc(NULL, sizeof(unsigned int)*m->nt*3);
        unsigned int nt2 = 0;
        for (i = 0; i < m->nt; i++) {
            unsigned int a = remap[m->tri[i*3+0]],
                         b = remap[m->tri[i*3+1]],
                         c = remap[m->tri[i*3+2]];
            if (a==b || b==c || a==c) continue;
            tri2[nt2*3+0]=a; tri2[nt2*3+1]=b; tri2[nt2*3+2]=c; nt2++;
        }
        free(m->tri); free(m->v); free(m->n);
        m->v = cv; m->tri = tri2;
        m->nv = nc; m->nt = nt2;
        m->n = (vec3d_t *)xrealloc(NULL, sizeof(vec3d_t)*(nc?nc:1));
    }
    free(head); free(next); free(remap);
}

/* ------------------------------------------------------------ normals */
void mesh_normals(mesh_t *m) {
    unsigned int i;
    memset(m->n, 0, sizeof(vec3d_t)*m->nv);
    for (i = 0; i < m->nt; i++) {
        vec3d_t e1, e2, fn;
        unsigned int a = m->tri[i*3], b = m->tri[i*3+1], c = m->tri[i*3+2];
        v_sub(m->v[b], m->v[a], &e1);
        v_sub(m->v[c], m->v[a], &e2);
        v_cross(e1, e2, &fn);
        /* weight by area (unnormalized cross length) => standard */
        m->n[a].x += fn.x; m->n[a].y += fn.y; m->n[a].z += fn.z;
        m->n[b].x += fn.x; m->n[b].y += fn.y; m->n[b].z += fn.z;
        m->n[c].x += fn.x; m->n[c].y += fn.y; m->n[c].z += fn.z;
    }
    for (i = 0; i < m->nv; i++) v_norm(&m->n[i]);
}

void mesh_bounds(const mesh_t *m, vec3d_t *mn, vec3d_t *mx) {
    unsigned int i;
    mn->x = mn->y = mn->z =  1e30f;
    mx->x = mx->y = mx->z = -1e30f;
    for (i = 0; i < m->nv; i++) {
        if (m->v[i].x < mn->x) mn->x = m->v[i].x;
        if (m->v[i].y < mn->y) mn->y = m->v[i].y;
        if (m->v[i].z < mn->z) mn->z = m->v[i].z;
        if (m->v[i].x > mx->x) mx->x = m->v[i].x;
        if (m->v[i].y > mx->y) mx->y = m->v[i].y;
        if (m->v[i].z > mx->z) mx->z = m->v[i].z;
    }
}

float mesh_area(const mesh_t *m) {
    unsigned int i; float s = 0;
    for (i = 0; i < m->nt; i++) {
        vec3d_t e1, e2, cr;
        v_sub(m->v[m->tri[i*3+1]], m->v[m->tri[i*3]], &e1);
        v_sub(m->v[m->tri[i*3+2]], m->v[m->tri[i*3]], &e2);
        v_cross(e1, e2, &cr);
        s += 0.5f * v_len(cr);
    }
    return s;
}

void mesh_transform(mesh_t *m, float (*rot)[3], const vec3d_t *offset, float scale) {
    unsigned int i;
    for (i = 0; i < m->nv; i++) {
        vec3d_t p = m->v[i], o;
        o.x = (rot[0][0]*p.x + rot[0][1]*p.y + rot[0][2]*p.z) * scale + offset->x;
        o.y = (rot[1][0]*p.x + rot[1][1]*p.y + rot[1][2]*p.z) * scale + offset->y;
        o.z = (rot[2][0]*p.x + rot[2][1]*p.y + rot[2][2]*p.z) * scale + offset->z;
        m->v[i] = o;
        if (m->n) {
            vec3d_t nn = m->n[i], no;
            no.x = rot[0][0]*nn.x + rot[0][1]*nn.y + rot[0][2]*nn.z;
            no.y = rot[1][0]*nn.x + rot[1][1]*nn.y + rot[1][2]*nn.z;
            no.z = rot[2][0]*nn.x + rot[2][1]*nn.y + rot[2][2]*nn.z;
            v_norm(&no);
            m->n[i] = no;
        }
    }
}

/* ---------------------------------------------------------------- load */
int mesh_load_stl(const char *path, mesh_t *m, int *was_ascii, float dims[3]) {
    FILE *f = fopen(path, "rb");
    char probe[6] = {0};
    *was_ascii = 0;
    if (!f) { ERR("cannot open '%s'", path); return 0; }
    if (fread(probe, 1, 5, f) != 5) { fclose(f); ERR("file too small: %s", path); return 0; }
    rewind(f);
    if (memcmp(probe, "solid", 5) == 0) {
        /* could still be binary named solid...; try ASCII first */
        if (stl_load_ascii(f, m)) {
            *was_ascii = 1;
        } else {
            rewind(f);
            if (!stl_load_binary(f, m)) { fclose(f); ERR("failed to parse STL: %s", path); return 0; }
        }
    } else {
        if (!stl_load_binary(f, m)) { fclose(f); ERR("failed to parse STL: %s", path); return 0; }
    }
    fclose(f);
    {
        vec3d_t mn, mx;
        mesh_bounds(m, &mn, &mx);
        dims[0] = mx.x - mn.x; dims[1] = mx.y - mn.y; dims[2] = mx.z - mn.z;
    }
    return m->nt > 0;
}
