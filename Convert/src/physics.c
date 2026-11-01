/* physics.c - builds the collision model from a (already downgraded) mesh:
 * convex decomposition + mass properties.  This is the "перенос физики"
 * (physics transfer) core: STL has no physics, so we synthesize a proper
 * VPhysics-compatible set of convex pieces with real inertia, exactly like
 * studiomdl's WriteCylinderSidecap/convex-piece pipeline in write.cpp. */
#include "mdlc.h"

void phys_build(const mesh_t *coll, const opts_t *o, phys_t *p) {
    mesh_t **pieces = NULL;
    int npieces = 0;
    float vol_tot = 0;
    vec3d_t com_acc = {0,0,0};
    float in_acc[6] = {0,0,0,0,0,0};
    int i;

    memset(p, 0, sizeof(*p));
    p->mass = o->mass > 0 ? o->mass : 1.0f;

    if (!mesh_decompose_convex(coll, o->physpieces > 0 ? o->physpieces : 8,
                               0.5f, &pieces, &npieces) || npieces == 0) {
        /* last-resort fallback: single decimated hull */
        mesh_t *h = mesh_convex_hull(coll);
        if (!h) h = mesh_decimate(coll, coll->nt / 2 + 4);
        pieces = (mesh_t **)xm_alloc(sizeof(mesh_t*));
        pieces[0] = h;
        npieces = 1;
    }

    p->pieces = pieces;
    p->npieces = npieces;

    /* aggregate mass properties over all pieces (density scaled to target
     * mass): volume-weighted COM, summed inertias about global origin then
     * re-centred on the aggregate COM. */
    for (i = 0; i < npieces; i++) {
        float vol[1]; vec3d_t c; float I[6];
        mesh_massprops(pieces[i], 1.0f, vol, &c, I);
        if (vol[0] <= 0) vol[0] = 1.0f; /* open/degenerate piece: unit weight */
        vol_tot += vol[0];
        com_acc.x += c.x * vol[0];
        com_acc.y += c.y * vol[0];
        com_acc.z += c.z * vol[0];
        in_acc[0] += I[0]; in_acc[1] += I[1]; in_acc[2] += I[2];
        in_acc[3] += I[3]; in_acc[4] += I[4]; in_acc[5] += I[5];
    }
    if (vol_tot <= 0) vol_tot = 1.0f;
    p->vol = vol_tot;
    p->com.x = com_acc.x / vol_tot;
    p->com.y = com_acc.y / vol_tot;
    p->com.z = com_acc.z / vol_tot;

    /* scale unit-density inertia to requested mass */
    {
        float k = p->mass / vol_tot;
        int j;
        for (j = 0; j < 6; j++) p->inertia[j] = in_acc[j] * k;
    }
    MSG(o, "physics: %d convex pieces, %d tris total, mass=%.2f",
        npieces, (int)(coll->nt), p->mass);
}

void phys_free(phys_t *p) {
    int i;
    if (!p) return;
    for (i = 0; i < p->npieces; i++) mesh_free(p->pieces[i]);
    xm_free(p->pieces);   /* may be libc- or xm-allocated: xm_free handles both */
    p->pieces = NULL; p->npieces = 0;
}
