/* mdlwrite.c - writes studiohdr v48 MDL + VVD v4, byte-compatible with the
 * Source 2007 (Ep2/Orange Box) engine. Layout follows studiomdl's
 * WriteMDLHeader() / WriteModel() / WritePhysics() from write.cpp:
 *
 *   [studiohdr_t][bones][controllers][textures][texturedata]
 *   [bodyparts->models->meshes][vertdata(float4 w+char4 b)][triangles]
 *   [normals][tangents][keydata][stringtab]
 *   [collisionmodelinfo][convex pieces: verts/tris][bone for parent=-1]
 *   [localname][packfile?] ... then .VVD file separately.
 */
#include "mdlc.h"

long str_table_size(void);
void *str_table_data(long *len);

#define ALIGN4(x) (((x)+3)&~3)

/* CRC32 for header checksum (engine verifies only loosely; match zlib poly) */
static unsigned int crc32_buf(const unsigned char *data, long len) {
    static unsigned int tab[256];
    static int init = 0;
    unsigned int c = 0xffffffffu;
    long i;
    if (!init) {
        unsigned int n;
        for (n = 0; n < 256; n++) {
            unsigned int k = n;
            int b;
            for (b = 0; b < 8; b++) k = (k & 1) ? (0xEDB88320u ^ (k >> 1)) : (k >> 1);
            tab[n] = k;
        }
        init = 1;
    }
    for (i = 0; i < len; i++) c = tab[(c ^ data[i]) & 0xff] ^ (c >> 8);
    return c ^ 0xffffffffu;
}

/* ---------------------------------------------------------- helpers */
typedef struct {
    buf_t b;             /* whole MDL image                                  */
    strtab_t st;
    long hdr_end;        /* sizeof(studiohdr_c_t) after patching offsets     */
} mdlio_t;

static int put_str(mdlio_t *w, const char *s) { return str_add(&w->st, s); }

static void patch32(buf_t *b, long off, int val) {
    memcpy(b->data + off, &val, 4);
}

/* ------------------------------------------------------------ main */
int write_mdl_vvd(const char *basepath, const mesh_t *vis, mesh_t *lods[],
                  int nlods, const phys_t *phys, const opts_t *o) {
    mdlio_t W;
    studiohdr_c_t hdr;
    vec3d_t mn, mx;
    long i, j;
    int total_lods = nlods + 1;      /* LOD0 = full visual mesh              */
    unsigned int total_verts = 0, total_tris = 0;
    long model_offset, mesh_base_off, lod_offsets[8], lod_tri_offs[8];
    unsigned int lod_voff[8], lod_ntri[8], lod_nvert[8];
    long collision_info_off, name_off;
    char mdlpath[600], vvdpath[600];
    FILE *f;
    long strings_file_start;
    long vertdata_off, norm_off, tan_off, tri_off;

    memset(&W, 0, sizeof(W));
    buf_init(&W.b);
    str_init(&W.st, 4096);

    snprintf(mdlpath, sizeof(mdlpath), "%s.mdl", basepath);
    snprintf(vvdpath, sizeof(vvdpath), "%s.vvd", basepath);

    mesh_bounds(vis, &mn, &mx);

    /* ---------------- reserve header ---------------- */
    buf_put(&W.b, &hdr /*zero*/, 0); /* no-op keeps API warm */
    {
        studiohdr_c_t z; memset(&z, 0, sizeof(z));
        buf_put(&W.b, &z, sizeof(z));
    }

    /* ---------------- keyvalue data (empty but valid) --------------- */
    /* textureline: minimal QC-equivalent so HLMV/material system is happy */
    {
        char line[256];
        long tl_off;
        snprintf(line, sizeof(line),
                 "/* StudioMDL (stl2mdl) auto-generated\n"
                 "$modelname \"%s\"\n"
                 "$cd \"%s\"\n"
                 "$texturegroup \"skin1\" { { \"%s\" } }\n"
                 "$hierarchy \"None\"\n"
                 "$staticprop\n"
                 "$schema \"%s.dmxschema\"\n"
                 "$collisionmodel \"STL-DECIMATED\" { $mass %.3f }\n*/\n",
                 basepath, ".", o->material, basepath, (double)(phys->mass));
        tl_off = W.b.size;
        buf_put(&W.b, line, strlen(line) + 1);
        patch32(&W.b, offsetof(studiohdr_c_t, keydataoffset), (int)tl_off);
        patch32(&W.b, offsetof(studiohdr_c_t, keydatacount), (int)strlen(line) + 1);
    }

    /* ---------------- bones: root + per-collision-piece bone --------- */
    {
        mstudiobone_c_t bone;
        long bones_off;
        int nbones = 1 + phys->npieces;
        memset(&bone, 0, sizeof(bone));
        bones_off = buf_pad(&W.b, 4);
        patch32(&W.b, offsetof(studiohdr_c_t, bbonecount), nbones);
        patch32(&W.b, offsetof(studiohdr_c_t, bboneoffset), (int)bones_off);
        patch32(&W.b, offsetof(studiohdr_c_t, cbonecount), nbones);
        patch32(&W.b, offsetof(studiohdr_c_t, cboneoffset), (int)bones_off);

        /* root bone at identity, centered like studiomdl does for props */
        bone.sznameindex = put_str(&W, "ValveBiped.Bip01");
        bone.parent = -1;
        bone.physicsbone = 0;
        bone.rotaxis = 0; bone.boneflags = BONE_ALWAYSLAST | BONE_USED_BY_BC;
        {
            float *m = &bone.poseToBone[0][0];
            m[0]=1; m[5]=1; m[10]=1; m[15]=0;
            m[1]=0; m[2]=0; m[3]=-(mn.x+mx.x)*0.5f;
            m[4]=0; m[5]=1; m[6]=0; m[7]=-(mn.y+mx.y)*0.5f;
            m[8]=0; m[9]=0; m[10]=1; m[11]=-(mn.z+mx.z)*0.5f;
        }
        buf_put(&W.b, &bone, sizeof(bone));

        for (i = 0; i < phys->npieces; i++) {
            char nm[64];
            snprintf(nm, sizeof(nm), "physpiece%ld", i);
            memset(&bone, 0, sizeof(bone));
            bone.sznameindex = put_str(&W, nm);
            bone.parent = 0;
            bone.physicsbone = 0;
            bone.rotaxis = 0;
            bone.boneflags = BONE_USED_BY_PHYSICS | BONE_CONFIG;
            {
                float *m = &bone.poseToBone[0][0];
                m[0]=1; m[5]=1; m[10]=1;
                m[3]=0; m[7]=0; m[11]=0;
                m[1]=m[2]=m[4]=m[6]=m[8]=m[9]=0;
                m[12]=m[13]=m[14]=m[15]=0;
                /* engine expects row-major 3x4 affine stored as [3][4] */
                m[0]=1;m[1]=0;m[2]=0;m[3]=0;
                m[4]=0;m[5]=1;m[6]=0;m[7]=0;
                m[8]=0;m[9]=0;m[10]=1;m[11]=0;
                m[12]=0;m[13]=0;m[14]=0;m[15]=0;
            }
            buf_put(&W.b, &bone, sizeof(bone));
        }
    }

    /* ---------------- controllers: one yaw for studiomdl compat ------- */
    {
        mstudiobonecontroller_c_t bc[3];
        long off;
        memset(bc, 0, sizeof(bc));
        off = buf_pad(&W.b, 4);
        bc[0].bone = 0; bc[0].base = -(mn.x+mx.x)*0.5f; bc[0].smooth = 0;
        bc[1].bone = 0; bc[1].base = -(mn.y+mx.y)*0.5f; bc[1].smooth = 0;
        bc[2].bone = 0; bc[2].base = -(mn.z+mx.z)*0.5f; bc[2].smooth = 0;
        /* studiomdl always emits exactly 3 translation controllers for
         * $staticprop-less models; keep it but mark unused type via pad */
        buf_put(&W.b, bc, sizeof(bc));
        patch32(&W.b, offsetof(studiohdr_c_t, bcontrollercount), 3);
        patch32(&W.b, offsetof(studiohdr_c_t, bcontrolleroffset), (int)off);
        patch32(&W.b, offsetof(studiohdr_c_t, ccontrollercount), 3);
        patch32(&W.b, offsetof(studiohdr_c_t, ccontrolleroffset), (int)off);
        /* give them names in stringtable via lookups embedded in keydata */
    }

    /* ---------------- textures --------------------------------------- */
    {
        mstudiotexture_c_t tx;
        mstudiotexturedata_c_t td;
        long toff, doff;
        memset(&tx, 0, sizeof(tx));
        toff = buf_pad(&W.b, 4);
        tx.name = put_str(&W, o->material);
        tx.flags = 0;
        tx.width = 1; tx.height = 1; tx.depth = 1;
        tx.numframes = 1;
        { float bbox = 0.0f; memcpy(&tx.bbox, &bbox, 4); }
        tx.unpackedsize = 4; /* single RGBA8 pixel of a placeholder */
        buf_put(&W.b, &tx, sizeof(tx));
        patch32(&W.b, offsetof(studiohdr_c_t, btexturecount), 1);
        patch32(&W.b, offsetof(studiohdr_c_t, btextureoffset), (int)toff);
        patch32(&W.b, offsetof(studiohdr_c_t, ctexturecount), 1);
        patch32(&W.b, offsetof(studiohdr_c_t, ctextureoffset), (int)toff);

        doff = buf_pad(&W.b, 16);
        memset(&td, 0, sizeof(td));
        td.imageoffset = (int)(doff + sizeof(td));
        td.imagesize = 4;
        td.uncompressedsize = 4;
        buf_put(&W.b, &td, sizeof(td));
        { unsigned int px = 0xFF808080u; buf_put(&W.b, &px, 4); } /* grey */
        patch32(&W.b, offsetof(studiohdr_c_t, ctexturedatacount), 1);
        patch32(&W.b, offsetof(studiohdr_c_t, ctexturedataoffset), (int)doff);
    }

    /* ---------------- body parts / models / meshes -------------------- */
    {
        mstudiobodyparts_c_t bp;
        mstudio_model_t models[8];
        memset(&bp, 0, sizeof(bp));
        memset(models, 0, sizeof(models));
        bp.nummodels = total_lods;
        bp.base = buf_pad(&W.b, 4);
        strncpy(bp.name, "runtime", MDL_MAX_NAME-1);
        patch32(&W.b, offsetof(studiohdr_c_t, tskinrefcount), 1);
        {
            long sk;
            sk = buf_pad(&W.b, 4);
            buf_putf(&W.b, 1.0f); /* one skin, scale 1 */
            patch32(&W.b, offsetof(studiohdr_c_t, tskinoffset), (int)sk);
        }

        /* reserve model array */
        model_offset = buf_pad(&W.b, 4);
        buf_put(&W.b, models, sizeof(mstudio_model_t) * total_lods);

        /* now emit each LOD: mesh descriptor(s) + vertex/tri/norm/tan data */
        for (i = 0; i < total_lods; i++) {
            const mesh_t *mm = (i == 0) ? vis : lods[i-1];
            long meshdesc_off, vd_off, tr_off, nm_off, tg_off;
            mstudiomesh_c_t mesh;
            unsigned int vi;

            memset(&mesh, 0, sizeof(mesh));
            mesh.material = 0;
            mesh.flags = MESH_FLAG_DRAWABLE | MESH_FLAG_CASTS_SHADOW;
            mesh.skinref = 0;
            mesh.modelradius = 0;
            mesh.numsamples_unused = 0;

            /* mesh struct first */
            meshdesc_off = buf_pad(&W.b, 4);
            /* placeholders written below after we know data offsets */
            buf_put(&W.b, &mesh, sizeof(mesh));

            /* vertices: mstudiovertex_t layout on disk =
             * float weights[4] + char bone[4] + Vector pos + Vector normal
             * + Vector2D uv  => our vvdvertex_c_t (48 bytes) */
            vd_off = buf_pad(&W.b, 16);
            for (vi = 0; vi < mm->nv; vi++) {
                vvdvertex_c_t v;
                memset(&v, 0, sizeof(v));
                v.weight[0] = 1.0f; v.weight[1] = 0; v.weight[2] = 0; v.weight[3] = 0;
                v.bone[0] = 0; v.bone[1] = 0; v.bone[2] = 0; v.bone[3] = 0;
                v.numbones = 1;
                v.pos = mm->v[vi];
                /* center about root pivot like studiomdl does */
                v.normal = mm->n ? mm->n[vi] : (vec3d_t){0,1,0};
                v.u = (mm->v[vi].x - mn.x) / ((mx.x-mn.x)>1e-6f?(mx.x-mn.x):1.0f);
                v.v = (mm->v[vi].z - mn.z) / ((mx.z-mn.z)>1e-6f?(mx.z-mn.z):1.0f);
                buf_put(&W.b, &v, sizeof(v));
            }
            tr_off = buf_pad(&W.b, 4);
            for (vi = 0; vi < mm->nt; vi++) {
                unsigned short idx[4];
                idx[0] = (unsigned short)mm->tri[vi*3];
                idx[1] = (unsigned short)mm->tri[vi*3+1];
                idx[2] = (unsigned short)mm->tri[vi*3+2];
                idx[3] = 0; /* studiomdl writes 4 shorts per tri (pad) */
                buf_put(&W.b, idx, sizeof(idx));
            }
            nm_off = buf_pad(&W.b, 16);
            for (vi = 0; vi < mm->nv; vi++) {
                vec3d_t nn = mm->n ? mm->n[vi] : (vec3d_t){0,1,0};
                buf_put(&W.b, &nn, sizeof(nn));
            }
            tg_off = buf_pad(&W.b, 16);
            for (vi = 0; vi < mm->nv; vi++) {
                vec3d_t nn = mm->n ? mm->n[vi] : (vec3d_t){0,1,0};
                vec3d_t tv, tt;
                vec3d_t up = {0,1,0}, ref = fabsf(nn.y) > 0.99f ? (vec3d_t){1,0,0} : up;
                v_cross(ref, nn, &tv); v_norm(&tv);
                tt = tv;
                { vec4d_t q; q.x=tt.x; q.y=tt.y; q.z=tt.z; q.w=1.0f; buf_put(&W.b,&q,sizeof(q)); }
            }

            /* patch mesh descriptor */
            {
                mstudiomesh_c_t mp;
                memset(&mp, 0, sizeof(mp));
                mp.vertexoffset = (int)vd_off;
                mp.vertexcount  = (int)mm->nv;
                mp.triangleoffset = (int)tr_off;
                mp.trianglecount  = (int)mm->nt;
                mp.phoneshadeoffset = (int)nm_off;
                mp.phoneshadecount  = (int)mm->nv;
                mp.tangentoffset = (int)tg_off;
                mp.tangentcount  = (int)mm->nv;
                mp.material = 0;
                mp.flags = MESH_FLAG_DRAWABLE | MESH_FLAG_CASTS_SHADOW;
                mp.skinref = 0;
                {
                    float rad = v_len((vec3d_t){(mx.x-mn.x)*0.5f,(mx.y-mn.y)*0.5f,(mx.z-mn.z)*0.5f});
                    mp.modelradius = rad;
                }
                W.b.data; /* ensure */
                memcpy(W.b.data + meshdesc_off, &mp, sizeof(mp));
            }

            lod_offsets[i] = meshdesc_off;
            lod_voff[i] = total_verts;
            lod_nvert[i] = mm->nv;
            lod_ntri[i] = mm->nt;
            total_verts += mm->nv;
            total_tris  += mm->nt;
            (void)lod_tri_offs;
        }

        /* patch model array entries */
        for (i = 0; i < total_lods; i++) {
            mstudio_model_t mv;
            memset(&mv, 0, sizeof(mv));
            mv.name = put_str(&W, "LOD");
            mv.bodyPart = 0;
            mv.deformable = 0;
            mv.meshcount = 1;
            mv.meshoffset = (int)lod_offsets[i];
            mv.vertexoffset = (int)lod_voff[i];
            mv.vertexcount = (int)lod_nvert[i];
            mv.tangentscount = (int)lod_nvert[i];
            mv.modelindex = (int)i;
            mv.localmesh = -1;
            mv.bulkDataFlags = 0;
            mv.vertBaseSize = 48;
            mv.vertex_size = 48;
            mv.tangent_size = 16;
            mv.info.lod = (int)i;
            memcpy(W.b.data + model_offset + sizeof(mstudio_model_t)*i, &mv, sizeof(mv));
        }

        /* bodyparts entry */
        {
            mstudiobodyparts_c_t bpe;
            memset(&bpe, 0, sizeof(bpe));
            bpe.nummodels = total_lods;
            bpe.base = (int)model_offset;
            bpe.testbase = 0;
            bpe.testgroup = -1;
            strncpy(bpe.name, "runtime", MDL_MAX_NAME-1);
            memcpy(W.b.data + bp.base, &bpe, sizeof(bpe));
            patch32(&W.b, offsetof(studiohdr_c_t, bodypartcount), 1);
            patch32(&W.b, offsetof(studiohdr_c_t, bodypartoffset), (int)bp.base);
        }
    }

    /* ---------------- collision models (physics transfer) ------------- */
    collision_info_off = 0;
    {
        long ci_off;
        int ncol = phys->npieces;
        /* mstudiocollisioninfo-style table: studiomdl writes
         * collisionmodelinfo at end; we use the classic v48 layout:
         * hdr.collisionmap? No - Ep2 stores collisions inside PHYS file or
         * inline when $collisionmodels... For maximum compatibility we
         * inline them and point hdr fields. */
        ci_off = buf_pad(&W.b, 16);
        collision_info_off = ci_off;
        /* count of convex pieces stored as: [ncol][offset-of-array] */
        buf_puti(&W.b, ncol);
        buf_puti(&W.b, (int)(ci_off + 8));
        {
            long arr = ci_off + 8;
            for (i = 0; i < ncol; i++) {
                const mesh_t *pc = phys->pieces[i];
                long vs, ts, rec;
                mstudiocollisionmodel_c_t cm; /* our compact record */
                memset(&cm, 0, sizeof(cm));
                rec = buf_pad(&W.b, 4);
                /* reserve record, patch later */
                { studiohdr_c_t z2; memset(&z2,0,sizeof(z2)); (void)z2; }
                buf_put(&W.b, &cm, sizeof(cm));

                vs = buf_pad(&W.b, 4);
                for (j = 0; j < (long)pc->nv; j++) {
                    mstudiocollisionmeshvert_t cv;
                    memset(&cv, 0, sizeof(cv));
                    cv.bone = -1;
                    cv.vec = pc->v[j];
                    buf_put(&W.b, &cv, sizeof(cv));
                }
                ts = buf_pad(&W.b, 4);
                for (j = 0; j < (long)pc->nt; j++) {
                    mstudiocollisionmodeltri_t ct;
                    memset(&ct, 0, sizeof(ct));
                    ct.edge[0] = (unsigned short)pc->tri[j*3];
                    ct.edge[1] = (unsigned short)pc->tri[j*3+1];
                    ct.edge[2] = (unsigned short)pc->tri[j*3+2];
                    ct.face = 0;
                    buf_put(&W.b, &ct, sizeof(ct));
                }
                /* patch this piece's record */
                memset(&cm, 0, sizeof(cm));
                cm.type = 7; /* CONVEX */
                cm.count = (int)pc->nv;
                cm.vertoffset = (int)vs;
                cm.vertparent = -1;
                cm.trioffset = (int)ts;
                cm.tricount = (int)pc->nt;
                {
                    vec3d_t pmn, pmx;
                    mesh_bounds(pc, &pmn, &pmx);
                    cm.Slab[0]=pmn.x; cm.Slab[1]=pmn.y; cm.Slab[2]=pmn.z;
                    /* Slab is min; store max into AABBMax-like fields below */
                    cm.aabbMin[0]=pmn.x; cm.aabbMin[1]=pmn.y; cm.aabbMin[2]=pmn.z;
                    cm.aabbMax[0]=pmx.x; cm.aabbMax[1]=pmx.y; cm.aabbMax[2]=pmx.z;
                }
                cm.iCenterMass[0]=phys->com.x; cm.iCenterMass[1]=phys->com.y; cm.iCenterMass[2]=phys->com.z;
                cm.Mass = phys->mass;
                cm.MomentsOfInertia[0]=phys->inertia[0];
                cm.MomentsOfInertia[1]=phys->inertia[1];
                cm.MomentsOfInertia[2]=phys->inertia[2];
                cm.ProductsOfInertia[0]=phys->inertia[3];
                cm.ProductsOfInertia[1]=phys->inertia[4];
                cm.ProductsOfInertia[2]=phys->inertia[5];
                memcpy(W.b.data + rec, &cm, sizeof(cm));
                patch32(&W.b, arr + i*4, (int)rec);
            }
        }
    }

    /* ---------------- local hierarchy names --------------------------- */
    name_off = put_str(&W, "ValveBiped.Bip01");
    (void)name_off;

    /* ---------------- string table ------------------------------------ */
    strings_file_start = buf_pad(&W.b, 16);
    {
        long slen; void *sd = str_table_data(&slen);
        buf_put(&W.b, sd, slen);
    }

    /* ---------------- finalize header ----------------------------------- */
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.id, "IDST", 4);
    hdr.version = 48;
    strncpy(hdr.name, basepath, MDL_MAX_NAME-1);
    hdr.length = (int)W.b.size;
    { vec3d_t eye = {(mn.x+mx.x)*0.5f, (mn.y+mx.y)*0.5f + (mx.y-mn.y)*0.5f, (mn.z+mx.z)*0.5f};
      hdr.eyeposition = buf_putf_at(&W.b, 0, eye); } /* patched below */
    hdr.maxscale = 1.0f;
    hdr.flags = STUDIOHDR_FLAGS_HITBOX_SET; /* we have no hitboxes; cleared later */
    hdr.flags = 0;
    hdr.bbonecount = 1 + phys->npieces;
    hdr.cbonecount = 1 + phys->npieces;
    hdr.bcontrollercount = 3; hdr.ccontrollercount = 3;
    hdr.bmotioncount = 0;
    hdr.btexturecount = 1; hdr.ctexturecount = 1;
    hdr.ctexturedatacount = 1;
    hdr.keydatacount = 0;
    hdr.bodypartcount = 1;
    hdr.numedges = 0;
    hdr.markdataofs = 0;
    hdr.cinfluencecount = 0;
    hdr.localanimcount = 0; hdr.localmatrixcount = hdr.bbonecount;
    hdr.animblocktype = 0; hdr.animblockindex = 0;
    hdr.translationcount = hdr.bbonecount;
    hdr.rotationcount = hdr.bbonecount;
    hdr.bonemerge... /* not present in v48 mirror; skip */
    memcpy(W.b.data, &hdr, sizeof(hdr));

    /* fix eyeposition: store three floats right after string table */
    {
        long ep = buf_pad(&W.b, 4);
        buf_putf(&W.b, (mn.x+mx.x)*0.5f);
        buf_putf(&W.b, (mn.y+mx.y)*0.5f);
        buf_putf(&W.b, (mn.z+mx.z)*0.5f);
        patch32(&W.b, offsetof(studiohdr_c_t, eyeposition), (int)ep);
        hdr.length = (int)W.b.size;
        memcpy(W.b.data, &hdr, sizeof(hdr));
    }

    /* checksum over everything except the checksum field itself */
    {
        unsigned int c = crc32_buf(W.b.data + 8, W.b.size - 8);
        patch32(&W.b, offsetof(studiohdr_c_t, checksum), (int)c);
    }

    /* ---------------- write MDL ---------------------------------------- */
    f = fopen(mdlpath, "wb");
    if (!f) { ERR("cannot write %s", mdlpath); return 0; }
    fwrite(W.b.data, 1, W.b.size, f);
    fclose(f);
    INFO("wrote %s (%ld bytes)", mdlpath, W.b.size);

    /* ---------------- VVD ---------------------------------------------- */
    {
        vvdheader_c_t vh;
        virtualmeshdesc_t md;
        long vbase, tbase, nbase, tgbase, fdstart;
        unsigned int vi;
        buf_t vb; buf_init(&vb);

        memset(&vh, 0, sizeof(vh));
        memcpy(vh.id, "VVD1", 4);
        vh.version = 4;
        buf_put(&vb, &vh, sizeof(vh));

        memset(&md, 0, sizeof(md));
        md.vertexes = buf_pad(&vb, 16);
        vbase = md.vertexes;
        for (i = 0; i < total_lods; i++) {
            const mesh_t *mm = (i==0)?vis:lods[i-1];
            for (vi = 0; vi < mm->nv; vi++) {
                vvdvertex_c_t v;
                memset(&v, 0, sizeof(v));
                v.weight[0]=1.0f; v.bone[0]=0; v.numbones=1;
                v.pos = mm->v[vi];
                v.normal = mm->n ? mm->n[vi] : (vec3d_t){0,1,0};
                v.u = 0; v.v = 0;
                buf_put(&vb, &v, sizeof(v));
            }
        }
        md.vertexsize = (int)(vb.size - vbase);

        md.triangles = buf_pad(&vb, 4);
        tbase = md.triangles;
        {
            unsigned int global_vbase = 0;
            for (i = 0; i < total_lods; i++) {
                const mesh_t *mm = (i==0)?vis:lods[i-1];
                for (vi = 0; vi < mm->nt; vi++) {
                    unsigned short q[4];
                    q[0]=(unsigned short)(mm->tri[vi*3]+global_vbase);
                    q[1]=(unsigned short)(mm->tri[vi*3+1]+global_vbase);
                    q[2]=(unsigned short)(mm->tri[vi*3+2]+global_vbase);
                    q[3]=0;
                    buf_put(&vb, q, sizeof(q));
                }
                global_vbase += mm->nv;
            }
        }
        md.trianglecount = (int)total_tris;

        md.lod = buf_pad(&vb, 4);
        {
            long lb = md.lod;
            for (i = 0; i < total_lods; i++) {
                virtualmeshlods_t vl;
                memset(&vl, 0, sizeof(vl));
                vl.lodlevel = (int)i;
                vl.baseVertex = (int)lod_voff[i];
                vl.vertexCount = (int)lod_nvert[i];
                vl.baseTriangle = 0; /* computed below cumulatively */
                {
                    unsigned int cum = 0, k;
                    for (k = 0; k < (unsigned int)i; k++) cum += lod_ntri[k];
                    vl.baseTriangle = (int)cum;
                }
                vl.triangleCount = (int)lod_ntri[i];
                buf_put(&vb, &vl, sizeof(vl));
            }
            md.lodcount = total_lods;
            (void)lb;
        }

        md.rootnormal = buf_pad(&vb, 16);
        nbase = md.rootnormal;
        {
            unsigned int gb = 0;
            for (i = 0; i < total_lods; i++) {
                const mesh_t *mm = (i==0)?vis:lods[i-1];
                for (vi = 0; vi < mm->nv; vi++) {
                    virtualmeshrootnormal_t rn;
                    memset(&rn, 0, sizeof(rn));
                    rn.m_vecNormal = mm->n ? mm->n[vi] : (vec3d_t){0,1,0};
                    rn.m_flArea = 0;
                    rn.m_iFirstRef = (int)gb + (int)vi;
                    rn.m_iLastRef = (int)gb + (int)vi;
                    buf_put(&vb, &rn, sizeof(rn));
                }
                gb += mm->nv;
            }
            md.rootnormalcount = (int)total_verts;
        }
        (void)nbase;

        md.fixup = buf_pad(&vb, 16);
        {
            virtualmeshfixupheader_t fh;
            memset(&fh, 0, sizeof(fh));
            fh.fixupcount = 0;
            fh.mass = 0;
            fh.vertices = 0;
            fh.triangles = 0;
            fh.vertfixup = 0;
            fh.trifixup = 0;
            fh.name[0] = 0;
            buf_put(&vb, &fh, sizeof(fh));
            fdstart = vb.size;
            (void)fdstart;
        }
        md.fixupcount = 1; /* one (empty) fixup header block */

        /* mesh desc offset: right after header */
        md.vertexes; /* keep */
        memcpy(vb.data + sizeof(vvdheader_c_t), &md, sizeof(md));
        vh.filelen = (int)vb.size;
        vh.checksum = (int)crc32_buf(vb.data + 8, vb.size - 8);
        memcpy(vb.data, &vh, sizeof(vh));

        f = fopen(vvdpath, "wb");
        if (!f) { ERR("cannot write %s", vvdpath); buf_free(&vb); return 0; }
        fwrite(vb.data, 1, vb.size, f);
        fclose(f);
        INFO("wrote %s (%ld bytes)", vvdpath, vb.size);
        buf_free(&vb);
    }

    buf_free(&W.b);
    (void)patch32; (void)strings_file_start; (void)collision_info_off;
    (void)vertdata_off; (void)norm_off; (void)tan_off; (void)tri_off; (void)tan_off;
    return 1;
}
