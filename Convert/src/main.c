/* main.c - stl2mdl: console STL -> Source MDL converter (studiohdr v48/VVD v4)
 * with full physics transfer and downgrade.  Builds on Windows (.exe) and
 * Linux (.elf).
 *
 * Downgrade semantics (as requested):
 *   - visual mesh: reduced only when the user asks (-lod/-lodpct), producing
 *     real LOD chain entries in MDL+VVD;
 *   - physics: ALWAYS downgraded (decimated + convex-decomposed + capped),
 *     so collision cost can never exceed the visual one ("so the PC doesn't
 *     burn").
 */
#include "mdlc.h"

static char g_pctspec[256];

static void usage(void) {
    printf(
"stl2mdl - STL to Source 2007 MDL converter (self-contained studiomdl-style compiler)\n"
"Usage: stl2mdl [options] input.stl [output_basename]\n"
"\n"
"  -o <name>          output basename (default: input name); writes .mdl/.vvd\n"
"  -scale <f>         STL units -> Source inches (default 0.4)\n"
"  -unit <mm|cm|m|in> set scale automatically for the given STL unit\n"
"  -mat <path>        $basetexture material path written into the model\n"
"  -lod <t1,t2,...>   visual downgrade targets (triangles per extra LOD)\n"
"                     Example: -lod 5000,1500,400\n"
"  -lodpct <p1,p2,..> same, as %% of source triangles (e.g. 50,25,10)\n"
"  -physlevel <0-5>   physics downgrade strength (default 3). Physics is\n"
"                     ALWAYS decimated regardless of visual options.\n"
"  -physmax <tris>    hard cap on total collision triangles (default 6000)\n"
"  -physpieces <n>    max convex pieces (default 6)\n"
"  -mass <kg>         physical mass (default 50)\n"
"  -rot <ax><deg>     rotate mesh (x/y/z + degrees), repeatable\n"
"  -swapyz            swap Y/Z axes (CAD Z-up -> Source Y-up)\n"
"  -nomerge           disable vertex welding (keep faceted STL verts)\n"
"  -mem <MB>          memory budget for the converter (default 512 MB;\n"
"                     0 = unlimited). All scratch data is heap-allocated,\n"
"                     so this never touches the (1 MB) Windows stack.\n"
"  -q                 quiet\n"
"  -h, --help         this help\n"
"\n"
"Physics transfer: STL carries no physics, so a VPhysics-compatible collision\n"
"model is synthesized: decimate -> greedy convex decomposition -> per-piece\n"
"volume / center of mass / inertia tensor -> mstudiocollisionmodel lump.\n");
}

static int rot_apply(float (*rot)[3], char ax, float deg) {
    float r = deg * (float)M_PI / 180.0f, c = cosf(r), sn = sinf(r);
    float m[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    float acc[3][3];
    int i, j, k;
    if (ax=='x'||ax=='X') { m[1][1]=c; m[1][2]=-sn; m[2][1]=sn; m[2][2]=c; }
    else if (ax=='y'||ax=='Y') { m[0][0]=c; m[0][2]=sn; m[2][0]=-sn; m[2][2]=c; }
    else if (ax=='z'||ax=='Z') { m[0][0]=c; m[0][1]=-sn; m[1][0]=sn; m[1][1]=c; }
    else return 0;
    for (i=0;i<3;i++) for (j=0;j<3;j++){ acc[i][j]=0; for(k=0;k<3;k++) acc[i][j]+=m[i][k]*rot[k][j]; }
    for (i=0;i<3;i++) for (j=0;j<3;j++) rot[i][j]=acc[i][j];
    return 1;
}

int main(int argc, char **argv) {
    opts_t o;
    mesh_t *vis = NULL;
    mesh_t *lods[6] = {0};
    mesh_t *colldec = NULL;
    phys_t phys;
    int ai, was_ascii, have_pct = 0;
    float dims[3];
    float rot[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    int rot_used = 0;
    vec3d_t zero = {0,0,0};
    char outbase[512];
    const char *infile = NULL;

    memset(&o, 0, sizeof(o));
    memset(&phys, 0, sizeof(phys));
    o.scale = 0.4f;
    o.mass = 50.0f;
    o.physlevel = 3;
    o.physmaxtris = 6000;
    o.physpieces = 6;
    o.verbose = 1;

    for (ai = 1; ai < argc; ai++) {
        const char *a = argv[ai];
        if (!strcmp(a,"-h") || !strcmp(a,"--help")) { usage(); return 0; }
        else if (!strcmp(a,"-o") && ai+1<argc) snprintf(o.out,sizeof(o.out),"%s",argv[++ai]);
        else if (!strcmp(a,"-scale") && ai+1<argc) o.scale=(float)atof(argv[++ai]);
        else if (!strcmp(a,"-unit") && ai+1<argc) {
            const char *u=argv[++ai];
            if(!strcmp(u,"mm")) o.scale=0.03937f;
            else if(!strcmp(u,"cm")) o.scale=0.3937f;
            else if(!strcmp(u,"m")) o.scale=39.37f;
            else if(!strcmp(u,"in")) o.scale=1.0f;
            else { ERR("unknown unit '%s'", u); return 1; }
        }
        else if (!strcmp(a,"-mat") && ai+1<argc) snprintf(o.material,sizeof(o.material),"%s",argv[++ai]);
        else if (!strcmp(a,"-lod") && ai+1<argc) {
            char *t=strdup(argv[++ai]), *p=t;
            while (p && *p && o.nlods<6) { o.lodtris[o.nlods++]=atoi(p); p=strchr(p,','); if(p)p++; }
            free(t);
        }
        else if (!strcmp(a,"-lodpct") && ai+1<argc) {
            snprintf(g_pctspec,sizeof(g_pctspec),"%s",argv[++ai]);
            have_pct = 1;
        }
        else if (!strcmp(a,"-physlevel") && ai+1<argc) { o.physlevel=atoi(argv[++ai]); if(o.physlevel<0)o.physlevel=0; if(o.physlevel>5)o.physlevel=5; }
        else if (!strcmp(a,"-physmax") && ai+1<argc) o.physmaxtris=atoi(argv[++ai]);
        else if (!strcmp(a,"-physpieces") && ai+1<argc) o.physpieces=atoi(argv[++ai]);
        else if (!strcmp(a,"-mass") && ai+1<argc) o.mass=(float)atof(argv[++ai]);
        else if (!strcmp(a,"-rot") && ai+1<argc) {
            const char *s = argv[++ai];
            if (!rot_apply(rot, s[0], (float)atof(s+1))) {
                ERR("bad -rot '%s' (use x/y/z + degrees)", s); return 1;
            }
            rot_used = 1;
        }
        else if (!strcmp(a,"-swapyz")) {
            /* Source: Y up. CAD STL: usually Z up. Map (x,y,z)->(x,z,-y)?
             * Classic studiomdl axis fix is swap Y/Z keeping handedness: */
            static float sw[3][3]={{1,0,0},{0,0,1},{0,-1,0}};
            float acc[3][3]; int i,j,k;
            for(i=0;i<3;i++)for(j=0;j<3;j++){acc[i][j]=0;for(k=0;k<3;k++)acc[i][j]+=sw[i][k]*rot[k][j];}
            for(i=0;i<3;i++)for(j=0;j<3;j++)rot[i][j]=acc[i][j];
            rot_used = 1;
        }
        else if (!strcmp(a,"-nomerge")) o.nomerge = 1;
        else if ((!strcmp(a,"-mem") || !strcmp(a,"--memory") || !strcmp(a,"-memory")) && ai+1<argc) {
            double mb = atof(argv[++ai]);
            if (mb < 0) mb = 0;
            o.membytes = (size_t)(mb * 1024.0 * 1024.0);
            xm_set_budget(o.membytes);
        }
        else if (!strcmp(a,"-q")) o.verbose = 0;
        else if (a[0]=='-' && a[1]) { ERR("unknown option '%s'", a); usage(); return 1; }
        else {
            if (!infile) { snprintf(o.src,sizeof(o.src),"%s",a); infile=a; }
            else if (!o.out[0]) snprintf(o.out,sizeof(o.out),"%s",a);
        }
    }
    if (!infile) { usage(); return 1; }
    if (!o.out[0]) {
        char *dot, *slash, *bs, *last;
        snprintf(outbase,sizeof(outbase),"%s",infile);
        dot=strrchr(outbase,'.'); if(dot)*dot=0;
        slash=strrchr(outbase,'/'); bs=strrchr(outbase,'\\');
        last = (slash > bs) ? slash : bs;
        if (last) memmove(outbase,last+1,strlen(last)+1);
        snprintf(o.out,sizeof(o.out),"%s",outbase);
    } else {
        /* strip a user-supplied .mdl extension so we never write "foo.mdl.mdl" */
        char *d = o.out + strlen(o.out);
        if (d - o.out >= 4 && !strcmp(d-4, ".mdl")) d[0] = 0;
    }

    INFO("reading %s ...", infile);
    vis = mesh_new();
    if (!mesh_load_stl(infile, vis, &was_ascii, dims)) {
        ERR("failed to read STL '%s'", infile);
        return 1;
    }
    INFO("%s STL: %u tris / %u verts (raw)", was_ascii?"ASCII":"binary", vis->nt, vis->nv);

    if (!o.nomerge) {
        mesh_weld(vis, 1e-4f);
        INFO("welded: %u tris / %u verts", vis->nt, vis->nv);
    }
    mesh_normals(vis);

    if (o.scale != 1.0f || rot_used)
        mesh_transform(vis, rot, &zero, o.scale);

    /* resolve percent LOD specs now that source tri count is known */
    if (have_pct) {
        char *t=strdup(g_pctspec), *p=t;
        o.nlods=0;
        while (p && *p && o.nlods<6) {
            float pct=(float)atof(p);
            long tgt=(long)(vis->nt*pct/100.0f);
            o.lodtris[o.nlods++]=(int)(tgt>3?tgt:3);
            p=strchr(p,','); if(p)p++;
        }
        free(t);
    }

    /* ---- visual downgrade (only on request) ---- */
    {
        int i;
        unsigned int prev = vis->nt;
        for (i = 0; i < o.nlods; i++) {
            unsigned int tgt = (unsigned int)o.lodtris[i];
            if (tgt == 0 || tgt >= prev) break;
            lods[i] = mesh_decimate(vis, tgt);
            if (!lods[i] || lods[i]->nt >= prev) { mesh_free(lods[i]); lods[i]=NULL; break; }
            INFO("visual LOD%d: %u tris (%.1f%% of source)", i+1, lods[i]->nt,
                 100.0f*lods[i]->nt/(float)vis->nt);
            prev = lods[i]->nt;
        }
        o.nlods = i;
    }

    /* ---- physics: ALWAYS downgraded ---- */
    {
        unsigned int phystarget;
        static const unsigned int lvl_base[6] = {24, 48, 96, 192, 384, 768};
        phystarget = lvl_base[o.physlevel];
        if ((int)phystarget > o.physmaxtris) phystarget = (unsigned int)o.physmaxtris;
        if (phystarget > vis->nt) phystarget = vis->nt;   /* never heavier than visual */
        if (phystarget < 4) phystarget = 4;
        colldec = mesh_decimate(vis, phystarget);
        if (!colldec || colldec->nt > vis->nt) {
            if (colldec) mesh_free(colldec);
            colldec = mesh_convex_hull(vis);
        }
        if (!colldec) { ERR("physics decimation failed"); return 1; }
        INFO("physics mesh: %u tris (downgrade level %d, ALWAYS applied)",
             colldec->nt, o.physlevel);
    }

    phys_build(colldec, &o, &phys);
    INFO("physics: %d convex piece(s), volume %.1f in^3, mass %.2f kg, COM (%.1f,%.1f,%.1f)",
         phys.npieces, phys.vol, phys.mass, phys.com.x, phys.com.y, phys.com.z);
    if (o.verbose)
        INFO("memory: peak %llu MB of %llu MB budget (-mem to change)",
             (unsigned long long)(xm_used()/(1024*1024)),
             (unsigned long long)(xm_budget()?(xm_budget()/(1024*1024)):0));

    if (write_mdl_vvd(o.out, vis, lods, o.nlods, &phys, &o) != 0) return 1;
    if (vtx_write(o.out, vis, lods, o.nlods, &o) != 0) return 1;

    { int i; for (i=0;i<6;i++) mesh_free(lods[i]); }
    if (colldec != vis) mesh_free(colldec);
    phys_free(&phys);
    mesh_free(vis);
    INFO("done.");
    return 0;
}
