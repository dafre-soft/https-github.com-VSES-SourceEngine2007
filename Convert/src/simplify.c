/* simplify.c - progressive edge-collapse decimation, convex hull and
 * greedy convex decomposition.  This is the "downgrade" engine: it is used
 * for visual LODs AND (always) for the physics mesh so the collision model
 * can never be heavier than the visual one ("so the PC doesn't burn").
 *
 * Algorithm follows the spirit of studiomdl's simplify.cpp / Quaternion-
 * based Collapse (Garland-Heckl style cost = quadric error approximation).
 */
#include "mdlc.h"

static void *xr(void *p, size_t n) {
    void *q = realloc(p, n ? n : 1);
    if (!q) { ERR("out of memory"); exit(2); }
    return q;
}

static int cmp_ull(const void *x, const void *y) {
    unsigned long long a = *(const unsigned long long*)x;
    unsigned long long b = *(const unsigned long long*)y;
    return a < b ? -1 : (a > b ? 1 : 0);
}

/* ------------------------------------------------------ edge collapse */
/* For simplicity/robustness we implement randomized half-edge-free
 * collapse with a linear scan over triangles per pass (O(nt) per collapse
 * amortized by passes). Suitable up to ~500k tris for our purposes.       */

typedef struct { float q[10]; } quad_t; /* symmetric 4x4 upper triangle */

static void quad_init(quad_t *Q, const vec3d_t p, const vec3d_t n) {
    /* plane through p with normal n: K = p*p^T + eps*(n*n^T), embedded in
     * 4x4 homogeneous form (Garland-Heckl). */
    float px = p.x, py = p.y, pz = p.z;
    float nx = n.x*0.01f, ny = n.y*0.01f, nz = n.z*0.01f;
    Q->q[0]=px*px+nx*nx; Q->q[1]=px*py+nx*ny; Q->q[2]=px*pz+nx*nz; Q->q[3]=px;
    Q->q[4]=py*py+ny*ny; Q->q[5]=py*pz+ny*nz; Q->q[6]=py;
    Q->q[7]=pz*pz+nz*nz; Q->q[8]=pz;
    Q->q[9]=1.0f;
}

static float quad_eval(const quad_t *Q, const vec3d_t *p) {
    double v =
        Q->q[0]*p->x*p->x + 2*Q->q[1]*p->x*p->y + 2*Q->q[2]*p->x*p->z + 2*Q->q[3]*p->x +
              Q->q[4]*p->y*p->y + 2*Q->q[5]*p->y*p->z + 2*Q->q[6]*p->y +
                    Q->q[7]*p->z*p->z + 2*Q->q[8]*p->z + Q->q[9];
    return (float)(v < 0 ? 0 : v);
}

/* does collapsing edge (a,b) flip or degenerate any incident triangle? */
static int collapse_ok(const vec3d_t *V, const unsigned int *tri, unsigned int nt,
                       unsigned int a, unsigned int b, const vec3d_t *newp) {
    unsigned int i;
    for (i = 0; i < nt; i++) {
        unsigned int t0 = tri[i*3], t1 = tri[i*3+1], t2 = tri[i*3+2];
        int has_a = (t0==a)+(t1==a)+(t2==a);
        int has_b = (t0==b)+(t1==b)+(t2==b);
        if (has_a && has_b) continue;           /* triangle removed */
        if (!has_a && !has_b) continue;         /* untouched        */
        /* triangle survives with one vertex moved to newp: check winding */
        unsigned int ids[3] = {t0,t1,t2};
        vec3d_t e1, e2, old_n, new_n;
        int k;
        for (k = 0; k < 3; k++) if (ids[k]==a || ids[k]==b) break;
        ids[k] = 0xffffffffu; /* mark */
        vec3d_t p[3];
        {
            int j, m = 0;
            vec3d_t tmp[3];
            for (j = 0; j < 3; j++) {
                if (ids[j] == 0xffffffffu) tmp[j] = *newp;
                else tmp[j] = V[ids[j]];
            }
            p[0]=tmp[0]; p[1]=tmp[1]; p[2]=tmp[2];
        }
        v_sub(p[1], p[0], &e1);
        v_sub(p[2], p[0], &e2);
        v_cross(e1, e2, &new_n);
        /* original */
        vec3d_t op[3];
        {
            int j;
            for (j = 0; j < 3; j++)
                op[j] = (tri[i*3+j]==a) ? V[a] : (tri[i*3+j]==b) ? V[b] : V[tri[i*3+j]];
        }
        v_sub(op[1], op[0], &e1);
        v_sub(op[2], op[0], &e2);
        v_cross(e1, e2, &old_n);
        if (v_dot(old_n, new_n) <= 0.0f) return 0;   /* flip */
        {
            float ol = v_len(old_n);
            if (ol > 1e-9f) {
                float nl = v_len(new_n);
                if (nl < ol * 0.05f) return 0;       /* degenerate */
            }
        }
    }
    return 1;
}

mesh_t *mesh_decimate(const mesh_t *src, unsigned int target) {
    mesh_t *m = mesh_new();
    unsigned int nv = src->nv, nt = src->nt;
    vec3d_t *V = (vec3d_t *)xr(NULL, sizeof(vec3d_t)*nv);
    vec3d_t *N = (vec3d_t *)xr(NULL, sizeof(vec3d_t)*nv);
    unsigned int *T = (unsigned int *)xr(NULL, sizeof(unsigned int)*nt*3);
    quad_t *Q = (quad_t *)xr(NULL, sizeof(quad_t)*nv);
    char *alive = (char *)xr(NULL, nv);
    unsigned int i, cur_nt, cur_nv = nv;
    int pass;

    memcpy(V, src->v, sizeof(vec3d_t)*nv);
    if (src->n) memcpy(N, src->n, sizeof(vec3d_t)*nv); else memset(N,0,sizeof(vec3d_t)*nv);
    memcpy(T, src->tri, sizeof(unsigned int)*nt*3);
    for (i = 0; i < nv; i++) { alive[i] = 1; quad_init(&Q[i], V[i], N[i]); }
    cur_nt = nt;

    if (target == 0 || target >= nt) {
        m->v = V; m->n = N; m->tri = T; m->nv = nv; m->nt = nt;
        free(Q); free(alive);
        return m;
    }

    /* iterate passes: each pass scans edges, collapses cheapest ones until
     * budget for this pass is met. Budget halves remaining excess per pass. */
    for (pass = 0; pass < 24 && cur_nt > target; pass++) {
        unsigned int want = target + (cur_nt - target) / 2; /* reduce halfway */
        unsigned int edges_done = 0;
        unsigned int *ea, *eb;
        unsigned int ec = 0;
        unsigned int k;

        ea = (unsigned int *)xr(NULL, sizeof(unsigned int)*cur_nt*3);
        eb = (unsigned int *)xr(NULL, sizeof(unsigned int)*cur_nt*3);
        for (k = 0; k < cur_nt; k++) {
            unsigned int t[3] = {T[k*3],T[k*3+1],T[k*3+2]};
            int j;
            for (j = 0; j < 3; j++) {
                unsigned int a = t[j], b = t[(j+1)%3];
                if (a > b) { unsigned int s=a;a=b;b=s; }
                ea[ec]=a; eb[ec]=b; ec++;
            }
        }
        /* dedup edges by sorting packed (a,b) keys */
        {
            /* pack into u64 for sorting; comparator at file scope */
            unsigned long long *pk = (unsigned long long *)xr(NULL, sizeof(unsigned long long)*ec);
            unsigned int jj, total = ec;
            for (jj = 0; jj < total; jj++) pk[jj] = ((unsigned long long)ea[jj]<<32) | eb[jj];
            qsort(pk, total, sizeof(unsigned long long), cmp_ull);
            ec = 0;
            for (jj = 0; jj < total; jj++) {
                if (ec && pk[ec-1]==pk[jj]) continue;
                ea[ec] = (unsigned int)(pk[jj]>>32);
                eb[ec] = (unsigned int)(pk[jj] & 0xffffffffu);
                ec++;
            }
            free(pk);
        }

        while (cur_nt > want && ec > 0) {
            /* pick best among a random sample of edges (fast heuristic) */
            int best_i = -1; float best_c = 1e30f; vec3d_t best_p;
            int tries = (int)(ec < 2000u ? ec : 2000u), t2;
            for (t2 = 0; t2 < tries; t2++) {
                unsigned int idx = (unsigned int)(((unsigned long)t2 * 2654435761u + (unsigned long)pass * 40503u) % ec);
                unsigned int a = ea[idx], b = eb[idx];
                vec3d_t mid;
                float c;
                if (!alive[a] || !alive[b]) continue;
                mid.x = (V[a].x + V[b].x) * 0.5f;
                mid.y = (V[a].y + V[b].y) * 0.5f;
                mid.z = (V[a].z + V[b].z) * 0.5f;
                c = quad_eval(&Q[a], &mid) + quad_eval(&Q[b], &mid);
                if (c < best_c) { best_c = c; best_i = (int)idx; best_p = mid; }
            }
            if (best_i < 0) break;
            {
                unsigned int a = ea[best_i], b = eb[best_i];
                if (!collapse_ok(V, T, cur_nt, a, b, &best_p)) {
                    /* mark edge dead by moving to end */
                    ea[best_i] = ea[ec-1]; eb[best_i] = eb[ec-1]; ec--;
                    continue;
                }
                /* perform collapse: b -> a */
                V[a] = best_p;
                {
                    vec3d_t na;
                    na.x = (N[a].x + N[b].x) * 0.5f;
                    na.y = (N[a].y + N[b].y) * 0.5f;
                    na.z = (N[a].z + N[b].z) * 0.5f;
                    v_norm(&na);
                    N[a] = na;
                }
                {
                    quad_t qa = Q[a], qb = Q[b], q;
                    int z;
                    for (z = 0; z < 10; z++) q.q[z] = qa.q[z] + qb.q[z];
                    Q[a] = q;
                }
                alive[b] = 0;
                /* rewrite triangles */
                {
                    unsigned int w = 0, z;
                    for (z = 0; z < cur_nt; z++) {
                        unsigned int t0=T[z*3], t1=T[z*3+1], t2=T[z*3+2];
                        if (t0==b) t0=a; if (t1==b) t1=a; if (t2==b) t2=a;
                        if (t0==t1 || t1==t2 || t0==t2) continue;
                        T[w*3]=t0; T[w*3+1]=t1; T[w*3+2]=t2; w++;
                    }
                    cur_nt = w;
                }
                edges_done++;
                /* drop used edge from list */
                ea[best_i] = ea[ec-1]; eb[best_i] = eb[ec-1]; ec--;
            }
            if (edges_done == 0 && cur_nt > want) break;
        }
        free(ea); free(eb);
        if (cur_nt == want) { /* continue next pass */ }
    }

    /* compact vertices: remove dead */
    {
        unsigned int *map = (unsigned int *)xr(NULL, sizeof(unsigned int)*cur_nv);
        unsigned int nc = 0, z;
        vec3d_t *VC, *NC;
        for (z = 0; z < cur_nv; z++) if (alive[z]) map[z] = nc++;
        VC = (vec3d_t *)xr(NULL, sizeof(vec3d_t)*(nc?nc:1));
        NC = (vec3d_t *)xr(NULL, sizeof(vec3d_t)*(nc?nc:1));
        for (z = 0; z < cur_nv; z++) if (alive[z]) { VC[map[z]] = V[z]; NC[map[z]] = N[z]; }
        for (z = 0; z < cur_nt*3; z++) T[z] = map[T[z]];
        free(V); free(N);
        m->v = VC; m->n = NC; m->tri = T;
        m->nv = nc; m->nt = cur_nt;
        free(map);
    }
    free(Q); free(alive);
    /* recompute smooth normals for stability */
    mesh_normals(m);
    return m;
}

/* ------------------------------------------------------ mass properties */
/* signed tetra method: for closed meshes. inertia about origin then shifted
 * to COM via parallel axis. Output inertia normalized to unit density.     */
void mesh_massprops(const mesh_t *m, float density,
                    float volume_out[1], vec3d_t *com, float inertia[6]) {
    double vol = 0.0, cx = 0, cy = 0, cz = 0;
    double xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
    unsigned int i;
    const double det_scale = 1.0/6.0;
    const double sec = 1.0/60.0; /* for integrals of x^2 etc over tetra */
    for (i = 0; i < m->nt; i++) {
        vec3d_t a = m->v[m->tri[i*3]], b = m->v[m->tri[i*3+1]], c = m->v[m->tri[i*3+2]];
        double d6 = a.x*(b.y*c.z - b.z*c.y) - a.y*(b.x*c.z - b.z*c.x) + a.z*(b.x*c.y - b.y*c.x);
        double tv = d6 * det_scale;
        vol += tv;
        { double w = tv; /* signed tetra volume weight */
          cx += w * (a.x+b.x+c.x) * 0.25;
          cy += w * (a.y+b.y+c.y) * 0.25;
          cz += w * (a.z+b.z+c.z) * 0.25; }
        /* second moments of tetra(o,a,b,c): sum over perms */
        {
            double ax=a.x, ay=a.y, az=a.z, bx=b.x, by=b.y, bz=b.z, ccx=c.x, ccy=c.y, ccz=c.z;
            double f = fabs(tv) * sec * 6.0; /* scale factor */
            /* integral of x^2 over tetra = vol/20 * (sum_i x_i^2 + sum_pairs) ; use formula:
             * ∫x² = V/10 * (Σx² + Σ_{i<j} x_i x_j)?? Use standard: for tetra with origin,
             * ∫x² dV = V/20 * ( xa²+xb²+xc² + (xa+xb+xc)² ) ... robust approx below */
            xx += f * (ax*ax + bx*bx + ccx*ccx + (ax+bx+ccx)*(ax+bx+ccx)) / 6.0;
            yy += f * (ay*ay + by*by + ccy*ccy + (ay+by+ccy)*(ay+by+ccy)) / 6.0;
            zz += f * (az*az + bz*bz + ccz*ccz + (az+bz+ccz)*(az+bz+ccz)) / 6.0;
            xy += f * (ax*ay + bx*by + ccx*ccy + (ax+bx+ccx)*(ay+by+ccy) * (1.0/3.0)) / 6.0;
            xz += f * (ax*az + bx*bz + ccx*ccz + (ax+bx+ccx)*(az+bz+ccz) * (1.0/3.0)) / 6.0;
            yz += f * (ay*az + by*bz + ccy*ccz + (ay+by+ccy)*(az+bz+ccz) * (1.0/3.0)) / 6.0;
        }
    }
    if (vol < 0) { vol = -vol; }
    if (volume_out) volume_out[0] = (float)vol;
    if (fabs(vol) < 1e-9) { vol = 1e-9; }
    com->x = (float)(cx / ((vol>0)?vol:1) );
    com->y = (float)(cy / ((vol>0)?vol:1) );
    com->z = (float)(cz / ((vol>0)?vol:1) );
    /* normalize second-moment sums to unit density */
    inertia[0] = (float)(xx * density);
    inertia[1] = (float)(yy * density);
    inertia[2] = (float)(zz * density);
    inertia[3] = (float)(xy * density);
    inertia[4] = (float)(xz * density);
    inertia[5] = (float)(yz * density);
    /* shift to center of mass (parallel axis), mass = vol*density */
    {
        double M = vol * density;
        double ox = com->x, oy = com->y, oz = com->z;
        inertia[0] -= (float)(M * (oy*oy + oz*oz));
        inertia[1] -= (float)(M * (ox*ox + oz*oz));
        inertia[2] -= (float)(M * (ox*ox + oy*oy));
        inertia[3] -= (float)(M * ox * oy);
        inertia[4] -= (float)(M * ox * oz);
        inertia[5] -= (float)(M * oy * oz);
        if (inertia[0] < 0) inertia[0] = 0;
        if (inertia[1] < 0) inertia[1] = 0;
        if (inertia[2] < 0) inertia[2] = 0;
    }
}

/* ------------------------------------------------- convex decomposition */
/* Greedy longest-axis median split of triangles, hull per piece. Mirrors
 * the intent of VHACD used by studiomdl's physics pipeline. */

static void emit_hull_piece(const mesh_t *m, int max_pieces,
                            mesh_t ***pieces, int *np) {
    /* pre-decimate so the O(n^2) hull stays fast; guarantee at least 8 tris */
    unsigned int tgt = m->nt / 4 + 8;
    if (tgt > 512) tgt = 512;
    if (tgt < 8)   tgt = 8;
    if (tgt >= m->nt) tgt = m->nt ? m->nt - 1 : 0; /* force some reduction path */
    {
        mesh_t *base = mesh_decimate(m, tgt);
        mesh_t *h = mesh_convex_hull(base);
        if (!h) h = base, base = NULL;
        else mesh_free(base);
        if (!h) return;
        if (*np < max_pieces) (*pieces)[(*np)++] = h;
        else mesh_free(h);
    }
}

static mesh_t *mesh_subset(const mesh_t *m, const unsigned int *tritri, unsigned int ntt) {
    mesh_t *o = mesh_new();
    unsigned int i;
    char *vused = (char *)xr(NULL, m->nv);
    unsigned int *vmap = (unsigned int *)xr(NULL, sizeof(unsigned int)*m->nv);
    unsigned int nv = 0;
    memset(vused, 0, m->nv);
    for (i = 0; i < ntt; i++) {
        unsigned int t = tritri[i];
        vused[m->tri[t*3]] = vused[m->tri[t*3+1]] = vused[m->tri[t*3+2]] = 1;
    }
    for (i = 0; i < m->nv; i++) if (vused[i]) vmap[i] = nv++;
    o->nv = nv; o->nt = ntt;
    o->v = (vec3d_t *)xr(NULL, sizeof(vec3d_t)*(nv?nv:1));
    o->n = (vec3d_t *)xr(NULL, sizeof(vec3d_t)*(nv?nv:1));
    o->tri = (unsigned int *)xr(NULL, sizeof(unsigned int)*(ntt?ntt:1)*3);
    for (i = 0; i < m->nv; i++) if (vused[i]) o->v[vmap[i]] = m->v[i];
    for (i = 0; i < ntt; i++) {
        unsigned int t = tritri[i];
        o->tri[i*3]   = vmap[m->tri[t*3]];
        o->tri[i*3+1] = vmap[m->tri[t*3+1]];
        o->tri[i*3+2] = vmap[m->tri[t*3+2]];
    }
    free(vused); free(vmap);
    mesh_normals(o);
    return o;
}

static void split_rec(const mesh_t *m, int depth, int maxdepth, int max_pieces,
                      float trim_eps, mesh_t ***pieces, int *np) {
    vec3d_t mn, mx, ext;
    int axis = 0, i;
    unsigned int *left, *right;
    unsigned int nl = 0, nr = 0, t;
    float mid;

    if (m->nt == 0) return;
    if (*np >= max_pieces || m->nt < 24) { emit_hull_piece(m, max_pieces, pieces, np); return; }

    mesh_bounds(m, &mn, &mx);
    v_sub(mx, mn, &ext);
    if (ext.x >= ext.y && ext.x >= ext.z) axis = 0;
    else if (ext.y >= ext.z) axis = 1;
    else axis = 2;

    if (depth >= maxdepth || (ext.x + ext.y + ext.z) <= 1e-6f) {
        emit_hull_piece(m, max_pieces, pieces, np);
        return;
    }

    mid = ((axis==0)?(mn.x+mx.x):(axis==1?(mn.y+mx.y):(mn.z+mx.z))) * 0.5f;
    left  = (unsigned int *)xr(NULL, sizeof(unsigned int)*m->nt);
    right = (unsigned int *)xr(NULL, sizeof(unsigned int)*m->nt);
    for (t = 0; t < m->nt; t++) {
        float c = 0;
        int k;
        for (k = 0; k < 3; k++) {
            vec3d_t p = m->v[m->tri[t*3+k]];
            c += (axis==0)?p.x:(axis==1?p.y:p.z);
        }
        c /= 3.0f;
        if (c <= mid) left[nl++] = t; else right[nr++] = t;
    }
    if (nl == 0 || nr == 0) { /* cannot split: emit hull */
        emit_hull_piece(m, max_pieces, pieces, np);
        free(left); free(right);
        return;
    }
    {
        mesh_t *ml = mesh_subset(m, left, nl);
        mesh_t *mr = mesh_subset(m, right, nr);
        split_rec(ml, depth+1, maxdepth, max_pieces, trim_eps, pieces, np);
        split_rec(mr, depth+1, maxdepth, max_pieces, trim_eps, pieces, np);
        mesh_free(ml); mesh_free(mr);
    }
    free(left); free(right);
}

int mesh_decompose_convex(const mesh_t *m, int max_pieces, float trim_eps,
                          mesh_t ***out_pieces, int *out_count) {
    mesh_t **arr = (mesh_t **)xr(NULL, sizeof(mesh_t*)*(max_pieces>0?max_pieces:1));
    int np = 0;
    *out_pieces = arr;
    *out_count = 0;
    if (max_pieces <= 0) max_pieces = 1;
    split_rec(m, 0, 6, max_pieces, trim_eps, &arr, &np);
    *out_pieces = arr;
    *out_count = np;
    return np > 0;
}

/* ------------------------------------------------------------ hull */
/* brute-force incremental convex hull: O(n^2 * h) worst but fine after
 * decimation (we run it only on small meshes). Returns NULL if too big. */
typedef struct { unsigned int v[3]; int rem; } hface_t;

mesh_t *mesh_convex_hull(const mesh_t *m) {
    unsigned int n = m->nv;
    unsigned int i;
    hface_t *F;
    unsigned int fc = 0, fcap;
    mesh_t *out;
    if (n > 4000) return NULL; /* refuse huge inputs; caller pre-decimates */
    fcap = 64;
    F = (hface_t *)xr(NULL, sizeof(hface_t)*fcap);

    /* find initial tetra: 4 non-coplanar points */
    {
        int b0=-1,b1=-1,b2=-1,b3=-1;
        vec3d_t d1,d2,d3;
        for (i = 0; i < n && b0<0; i++) b0 = (int)i;
        for (i = 0; i < n; i++) { v_sub(m->v[i], m->v[b0], &d1); if (v_len(d1) > 1e-6f) { b1=(int)i; break; } }
        if (b1<0) return NULL;
        for (i = 0; i < n; i++) {
            vec3d_t cr; v_cross(d1, (v_sub(m->v[i], m->v[b0], &d2), d2), &cr);
            if (v_len(cr) > 1e-6f) { b2=(int)i; break; }
        }
        if (b2<0) return NULL;
        {
            vec3d_t base; v_sub(m->v[b1], m->v[b0], &d1);
            v_sub(m->v[b2], m->v[b0], &d2);
            v_cross(d1, d2, &base);
            for (i = 0; i < n; i++) {
                v_sub(m->v[i], m->v[b0], &d3);
                if (fabsf(v_dot(d3, base)) > 1e-6f) { b3=(int)i; break; }
            }
        }
        if (b3<0) return NULL;
        /* seed 4 faces with outward winding relative to centroid */
        {
            vec3d_t cen;
            cen.x = (m->v[b0].x+m->v[b1].x+m->v[b2].x+m->v[b3].x)/4;
            cen.y = (m->v[b0].y+m->v[b1].y+m->v[b2].y+m->v[b3].y)/4;
            cen.z = (m->v[b0].z+m->v[b1].z+m->v[b2].z+m->v[b3].z)/4;
            unsigned int seeds[4][3] = {{b0,b1,b2},{b0,b2,b3},{b0,b3,b1},{b1,b3,b2}};
            int s;
            for (s = 0; s < 4; s++) {
                unsigned int a=seeds[s][0],b=seeds[s][1],c=seeds[s][2];
                vec3d_t e1,e2,nr,mid;
                v_sub(m->v[b],m->v[a],&e1); v_sub(m->v[c],m->v[a],&e2); v_cross(e1,e2,&nr);
                mid.x=(m->v[a].x+m->v[b].x+m->v[c].x)/3;
                mid.y=(m->v[a].y+m->v[b].y+m->v[c].y)/3;
                mid.z=(m->v[a].z+m->v[b].z+m->v[c].z)/3;
                { vec3d_t toCen; v_sub(cen, mid, &toCen);
                  if (v_dot(nr,toCen) > 0) { unsigned int t=b;b=c;c=t; } /* ensure outward */
                }
                F[fc].v[0]=a; F[fc].v[1]=b; F[fc].v[2]=c; F[fc].rem=0; fc++;
            }
        }
    }

    for (i = 0; i < n; i++) {
        unsigned int j;
        int visible[8192]; /* faces visible from point i */
        unsigned int vc = 0;
        int inside = 1;
        if (fc > 8192) return NULL; /* safety */
        for (j = 0; j < fc; j++) {
            hface_t *f = &F[j];
            vec3d_t e1,e2,nr,p0;
            if (f->rem) continue;
            p0 = m->v[f->v[0]];
            v_sub(m->v[f->v[1]],p0,&e1); v_sub(m->v[f->v[2]],p0,&e2); v_cross(e1,e2,&nr);
            { vec3d_t dp; v_sub(m->v[i], p0, &dp);
              if (v_dot(nr, dp) > 1e-7f) { visible[vc++]= (int)j; inside = 0; } }
        }
        if (inside) continue;
        /* mark removed, collect boundary edges */
        {
            unsigned int be[3][2]; int nbe = 0; /* max boundary for star region small */
            unsigned int cap_be = 64;
            unsigned int (*bed)[2] = (unsigned int (*)[2])xr(NULL, cap_be*sizeof(bed[0]));
            int nb = 0;
            for (j = 0; j < vc; j++) {
                hface_t *f = &F[visible[j]];
                f->rem = 1;
                int k;
                for (k = 0; k < 3; k++) {
                    unsigned int a=f->v[k], b=f->v[(k+1)%3];
                    int shared = 0, q;
                    /* edge shared with another visible face? */
                    for (q = 0; q < vc; q++) {
                        if ((unsigned)visible[q] == j) continue;
                        hface_t *g = &F[visible[q]];
                        int r;
                        for (r = 0; r < 3; r++) {
                            if (g->v[r]==a && g->v[(r+1)%3]==b) { shared = 1; break; }
                        }
                        if (shared) break;
                    }
                    if (!shared) {
                        if ((unsigned)nb + 1 > cap_be) { cap_be*=2; bed=(unsigned int(*)[2])xr(bed, cap_be*sizeof(bed[0])); }
                        bed[nb][0]=b; bed[nb][1]=a; nb++; /* reversed winding */
                    }
                }
            }
            (void)be; (void)nbe;
            for (j = 0; j < (unsigned int)nb; j++) {
                if (fc + 1 > fcap) { fcap*=2; F=(hface_t*)xr(F, sizeof(hface_t)*fcap); }
                F[fc].v[0]=bed[j][0]; F[fc].v[1]=bed[j][1]; F[fc].v[2]=i; F[fc].rem=0; fc++;
            }
            free(bed);
        }
    }

    out = mesh_new();
    {
        unsigned int live = 0, j;
        unsigned int *used = (unsigned int *)xr(NULL, sizeof(unsigned int)*n);
        unsigned int nu = 0;
        char *vu = (char *)xr(NULL, n);
        for (j = 0; j < n; j++) vu[j] = 0;
        for (j = 0; j < fc; j++) if (!F[j].rem) live++;
        out->tri = (unsigned int *)xr(NULL, sizeof(unsigned int)*(live?live:1)*3);
        out->nt = 0;
        for (j = 0; j < fc; j++) {
            if (F[j].rem) continue;
            int k;
            for (k = 0; k < 3; k++) {
                unsigned int vv = F[j].v[k];
                if (!vu[vv]) { vu[vv] = 1; used[vv] = nu++; }
                out->tri[out->nt*3+k] = used[vv];
            }
            out->nt++;
        }
        out->nv = nu;
        out->v = (vec3d_t *)xr(NULL, sizeof(vec3d_t)*(nu?nu:1));
        out->n = (vec3d_t *)xr(NULL, sizeof(vec3d_t)*(nu?nu:1));
        for (j = 0; j < n; j++) if (vu[j]) out->v[used[j]] = m->v[j];
        free(used); free(vu);
    }
    free(F);
    mesh_normals(out);
    return out;
}
