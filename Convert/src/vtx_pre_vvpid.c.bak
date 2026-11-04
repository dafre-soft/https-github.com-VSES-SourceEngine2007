/* vtx.c - ModelVertexWrite (VVP / .vtx) writer for the Source 2007 engine.
 * Structures transcribed from MDLLib vertex_file_layout_public.h and the
 * gordon2-era studiomdl vertexwrite.cpp, trimmed to a fully static model:
 * 1 bodypart, 1 model, 1 material group per LOD, no bones, no flex.
 * Without a .vtx the engine loads the .mdl but draws nothing at all.
 * VVP version 64 = HL2/EP1 range; matches studiohdr v48 + VVD v4 output.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "mdlc.h"


typedef unsigned char  u8;
typedef unsigned short u16;
typedef int32_t v32;   /* VVP fields are always 4 bytes */

#define VVP_VERSION 64
#define VVP_ID      0x44485056L   /* MAKEID('V','P','H','D') little-endian */

enum { MODELVERT_TYPE_3D = 0 };
enum { STRIP_TABLE_FIXUP_LIST = 0 };

#pragma pack(push,1)
typedef struct {                       /* VertexFileHeader_t — canonical SDK layout */
    v32 id, version, clientversion, flags, contentflags;
    v32 entrysize, numentries;
    v32 vertcacheofs;                  /* ABS file offset -> VertCacheHeader_t */
    v32 lodofs;                        /* ABS file offset -> LodLevel_t chain root */
    /* MatRepData inline block: [dataoffset][strlen][numSkins][numMaterials].
       dataoffset is stored RELATIVE TO THE ADDRESS OF THE STRLEN FIELD. */
    v32 matreldataoffset;
    v32 materiallist_strlen;           /* strlen of list-name string incl NUL */
    v32 num_skins, num_materials;
    v32 bodypartcount;
} vvp_header_t;                        /* 56 bytes — engine reads exactly this */
typedef int vvp_header_size_check[ (sizeof(vvp_header_t)==56) ? 1 : -1 ];

typedef struct {                       /* VertCacheHeader_t — canonical SDK layout
                                        (verified against gordon2 vertex_file_layout) */
    v32 vertblockheaderofs;            /* all eight: RELATIVE to VertCacheHeader */
    v32 stripindexblockheaderofs;
    v32 fixupblockheaderofs;
    v32 bodypartofs;
    v32 num_bone_flex_controller_descs;
    v32 boneflexcontrollerdescfirstofs;
    v32 num_bone_flex_pair_descs;
    v32 boneflexpairdescfirstofs;
    v32 maxnum_bones_per_hardware_vb;
    v32 maxnum_bones_per_software_vb;
    v32 maxnum_bones_per_strip;
    v32 maxnum_bones_per_triangles;
    v32 maxnum_verts_per_hw_vb;
    v32 maxnum_tris_per_hw_vb;
    v32 maxmodeldeform;
    v32 maxscale;                      /* VertexFileHeader_t::maxscale */
} vvp_vc_t;                            /* 64 bytes */
typedef int vvp_vc_size_check[(sizeof(vvp_vc_t)==64)?1:-1];

typedef struct {                       /* VertBlockHeader_t */
    v32 numblobs, blobarrayofs;       /* both rel to THIS header */
    v32 verttype, vertsize;           /* vertsize = sizeof(modelvertex_t)=32 */
    v32 vertflagsofs;                 /* rel to header -> uint32 vertflags[nv] */
    v32 vertlocalofs;                 /* REL TO EACH BLOB START -> vertex data */
    v32 vertboneofsofs;               /* rel to header -> bone-influence arrays */
    v32 vertbonecntofs;               /* rel to header -> byte count-per-vertex */
    v32 vertlightofs;                 /* rel to header -> ambient light array */
} vvp_vbhdr_t;                         /* 36 bytes; then {count,-1} pairs, then ofs[] */
typedef int vvp_vbhdr_size_check[(sizeof(vvp_vbhdr_t)==36)?1:-1];

typedef struct { v32 datalen; } vvp_vblob_t;   /* then pos(3f),nrm(3f),tex(2f)/vert */
typedef struct { v32 numblobs, blobarrayofs; } vvp_sihdr_t;
typedef struct { v32 numindices, indexdataofs; } vvp_siblob_t; /* idx rel to blob */
typedef struct { v32 numblobs, blobarrayofs; } vvp_fuhdr_t;   /* FixupBlockHeader_t */
typedef struct { v32 numfixups, firstfixupofs; } vvp_fublob_t;/* StripFixupHeader_t */
typedef struct { v32 destoffset, sourceoffset; } vvp_sd_t;    /* SourceDestListEntry_t */
/* StripFixup_t: union{type} + nextfixupoffset + numindexes + indexlistofs */
typedef struct { v32 fixuptype, nextfixupoffset, numindexes, indexlistofs; } vvp_fixup_t;

typedef struct { v32 modelcount, modelfirstofs; } vvp_bpdata_t; /* ofs rel to self */

typedef struct {                       /* ModelData_t (rel offsets to VertCacheHdr) */
    v32 vertblockheaderofs;
    v32 stripindexblockheaderofs;
    v32 fixupblockheaderofs;
    v32 materialdataofs;              /* -> MaterialListHeader_t */
    v32 numlodrecords;
    v32 lodrecordfirstofs;            /* -> LODFlexRecord_t[] */
    v32 maxscale;
} vvp_mdata_t;                         /* 32 bytes */

typedef struct {                       /* LODFlexRecord_t (VVP64: 12 bytes) */
    v32 lodlevel;                     /* rel VertCacheHdr -> LodLevel_t */
    v32 flexcount, flexindexfirstofs;
} vvp_lodrec_t;
typedef int vvp_lodrec_size_check[(sizeof(vvp_lodrec_t)==12)?1:-1];

typedef struct {                       /* LodLevel_t */
    float switchpoint;                 /* dist where NEXT lod takes over */
    v32 nummaterialgroups;
    v32 materialgroupfirstofs;        /* rel to LodLevel -> LodMaterialGroup_t[] */
    v32 maxscale;
    v32 nextLODOffset;                /* rel to LodLevel -> next LodLevel_t (0=none) */
} vvp_lodlvl_t;

typedef struct {                       /* LodMaterialGroup_t */
    v32 tristripsofs;                 /* rel to self -> TriStrips_t */
    v32 propertyofs;                  /* rel to self -> MaterialProxySize_t */
    v32 material;                     /* index into material list */
} vvp_lomg_t;

typedef struct {                       /* TriStrips_t */
    v32 numtristrips, stripsfirstofs; /* rel to TriStrips */
    v32 numtrifans, fansfirstofs;
    v32 numisolatedtris, trisfirstofs;
} vvp_ts_t;

typedef struct {                       /* TriStip_t: u16 indices follow at indexstartofs */
    v32 numindices, indexstartofs;
} vvp_tip_t;

typedef struct { v32 size, count; } vvp_psz_t;   /* MaterialProxySize_t */

typedef struct { v32 listlen; } vvp_mlhdr_t;     /* MaterialListHeader_t:
    then: list name chars[listlen] (NUL-terminated), 4-align,
          MaterialEntry_t[nummaterials]: { v32 namelen; char name[namelen]; } 4-aligned each,
          then skinFamilies: v32 numFamilies, then per family:
             v32 numMaterials, v32 w, v32 h, v32 sz, then per material: v32 arrayIndex */
#pragma pack(pop)

/* ---- dynamic byte buffer ---- */
typedef struct mbuf { u8 *d; long len, cap; } mbuf_t;
static void mb_init(mbuf_t *m){ m->cap=1<<16; m->d=(u8*)malloc(m->cap); m->len=0; }
static void mb_need(mbuf_t *m,long n){ if(m->len+n>m->cap){ while(m->len+n>m->cap)m->cap*=2; m->d=(u8*)realloc(m->d,m->cap);} }
static void mb_put(mbuf_t *m,const void*p,long n){ mb_need(m,n); memcpy(m->d+m->len,p,n); m->len+=n; }
static void mb_i32(mbuf_t *m,long v){ mb_put(m,&v,4); }
static void mb_u16(mbuf_t *m,u16 v){ mb_put(m,&v,2); }
static void mb_pad4(mbuf_t *m){ while(m->len&3){ u8 z=0; mb_put(m,&z,1);} }
static void mb_free(mbuf_t *m){ free(m->d); m->d=0; }
static inline void ST32(void *p,long v){ v32 t=(v32)v; memcpy(p,&t,4); }

int vtx_write(const char *basepath, const mesh_t *vis, mesh_t *lods[],
              int nlods, const opts_t *o)
{
    char path[600];
    FILE *f;
    int total_lods = nlods + 1;
    int i;
    mbuf_t head, verts, idxs, rest;
    unsigned int cum_v = 0, cum_t = 0;
    unsigned int lod_voff[8], lod_nv[8], lod_nt[8];
    long vb_hdr_pos, si_hdr_pos, fu_hdr_pos, vc_pos, bpd_pos, md_pos, lr_pos;
    long lvl_r[8], ml_pos;
    long vb_arr;
    long verts_base, idxs_base, rest_base;
    char matline[256];
    long matlen;

    if (total_lods > 8) total_lods = 8;
    snprintf(path,sizeof path,"%s.vtx",basepath);

    for (i=0;i<total_lods;i++) {
        const mesh_t *m = (i==0)?vis:lods[i-1];
        lod_nv[i]=m->nv; lod_nt[i]=m->nt;
        lod_voff[i]=cum_v;
        cum_v+=m->nv; cum_t+=m->nt;
    }

    if (o->material[0]) {
        strncpy(matline,o->material,sizeof matline-1); matline[sizeof matline-1]=0;
    } else {
        const char *mm=strrchr(o->out,'/'),*m2=strrchr(o->out,'\\');
        if(m2&&(!mm||m2>mm)) mm=m2;
        snprintf(matline,sizeof matline,"models/%s/%s",(mm?mm+1:"stl2mdl"),(mm?mm+1:"stl2mdl"));
    }
    matlen=(long)strlen(matline)+1;

    mb_init(&head); mb_init(&verts); mb_init(&idxs); mb_init(&rest);

    /* ===== file header (56 B, canonical VertexFileHeader_t) ===== */
    { vvp_header_t vh; memset(&vh,0,sizeof vh);
      vh.id=VVP_ID; vh.version=VVP_VERSION;
      vh.contentflags=0x1;             /* CONTENTS_SOLID */
      vh.entrysize=(v32)sizeof(vvp_header_t);        /* 56 */
      vh.numentries=1;
      vh.vertcacheofs=vh.entrysize;    /* VertCacheHeader right after */
      vh.bodypartcount=1;
      vh.num_skins=1; vh.num_materials=1;
      mb_put(&head,&vh,sizeof vh);
    }

    /* ===== VertCacheHeader ===== */
    vc_pos=head.len;
    { vvp_vc_t vc; memset(&vc,0,sizeof vc);
      vc.maxnum_bones_per_hardware_vb=1;   /* static model: single implicit bone */
      vc.maxnum_bones_per_software_vb=1;
      vc.maxnum_bones_per_strip=1;
      vc.maxnum_bones_per_triangles=1;
      vc.maxnum_verts_per_hw_vb=cum_v;
      vc.maxnum_tris_per_hw_vb=cum_t;
      mb_put(&head,&vc,sizeof vc);
    }

    /* ===== VertBlockHeader + entries pairs + blob ofs[] ===== */
    vb_hdr_pos=head.len;
    vb_arr=vb_hdr_pos+(long)sizeof(vvp_vbhdr_t)+total_lods*8;
    { vvp_vbhdr_t vb; memset(&vb,0,sizeof vb);
      vb.numblobs=total_lods; vb.verttype=MODELVERT_TYPE_3D; vb.vertsize=32;
      /* engine: pVertFlags = pVBlobHdr + vertflagsofs  (absolute rel-to-header).
         Our per-blob layout is [datalen][flags[]][verts[]]; with numblobs==1
         (always true here after we emit one combined blob below) this points
         exactly at the flags array. For >1 LOD we keep a single merged flags
         array in front of the first blob instead — see blob merge note. */
      /* per-blob layout is [datalen][u32 flags x nv][verts x nv]; both flag and
         vertex arrays therefore sit at fixed distances from the BLOB START.
         Engine resolves:  pFlags = (u32*)((char*)pVBlobHdr + vertflagsofs) for
         blob 0 only — studiomdl emits ONE shared flags array per model and
         points every blob's local data via vertlocalofs. We mirror that:
         flags live inside blob 0 (which holds ALL LOD vertices in our merged
         scheme)? No — we keep per-LOD blobs, so we emit a SINGLE global flags
         array right after the header block and set vertflagsofs to it. */
      vb.vertflagsofs=(long)sizeof(vb)+total_lods*8+total_lods*4; /* after pairs+ofs[] */
      vb.vertlocalofs=4;        /* rel to each blob start: past uint32 datalen... */
                                /* ...but flags are GLOBAL here, so verts follow datalen */
      vb.blobarrayofs=(long)sizeof(vb)+total_lods*8;
      mb_put(&head,&vb,sizeof vb);
      for(i=0;i<total_lods;i++){ mb_i32(&head,(long)lod_nv[i]); mb_i32(&head,-1); }
      for(i=0;i<total_lods;i++) mb_i32(&head,0);
    }
    ST32(head.d+vc_pos+offsetof(vvp_vc_t,vertblockheaderofs), vb_hdr_pos-vc_pos);

    /* ===== StripIndexBlockHeader ===== */
    si_hdr_pos=head.len;
    { vvp_sihdr_t si; si.numblobs=total_lods; si.blobarrayofs=sizeof(si);
      mb_put(&head,&si,sizeof si);
      for(i=0;i<total_lods;i++) mb_i32(&head,0);
    }
    ST32(head.d+vc_pos+offsetof(vvp_vc_t,stripindexblockheaderofs), si_hdr_pos-vc_pos);

    /* ===== FixupBlockHeader ===== */
    fu_hdr_pos=head.len;
    { vvp_fuhdr_t fu; fu.numblobs=total_lods; fu.blobarrayofs=sizeof(fu);
      mb_put(&head,&fu,sizeof fu);
      for(i=0;i<total_lods;i++) mb_i32(&head,0);
    }
    ST32(head.d+vc_pos+offsetof(vvp_vc_t,fixupblockheaderofs), fu_hdr_pos-vc_pos);

    /* ===== BodyPartData[1] -> ModelData[1] ===== */
    bpd_pos=head.len;
    { vvp_bpdata_t bp; bp.modelcount=1; bp.modelfirstofs=sizeof(bp);
      mb_put(&head,&bp,sizeof bp);
    }
    ST32(head.d+vc_pos+offsetof(vvp_vc_t,bodypartofs), bpd_pos-vc_pos);

    md_pos=head.len;
    { vvp_mdata_t md; memset(&md,0,sizeof md);
      md.vertblockheaderofs=vb_hdr_pos-vc_pos;
      md.stripindexblockheaderofs=si_hdr_pos-vc_pos;
      md.fixupblockheaderofs=fu_hdr_pos-vc_pos;
      md.numlodrecords=total_lods;
      md.maxscale=1;
      mb_put(&head,&md,sizeof md);
    }

    /* ===== LODFlexRecord[] ===== */
    lr_pos=head.len;
    for(i=0;i<total_lods;i++){ vvp_lodrec_t lr; memset(&lr,0,sizeof lr); mb_put(&head,&lr,sizeof lr); }
    ST32(head.d+md_pos+offsetof(vvp_mdata_t,lodrecordfirstofs), lr_pos-vc_pos);

    /* ===== global per-vertex flags array (all LODs concatenated) =====
       Lives INSIDE the head region, directly after the vblob offset table
       (position vb_arr+total_lods*4). vertflagsofs was set to exactly there.
       Engine reads pFlags = pVBlobHdr + vertflagsofs once per model. */
    { long flg_pos = vb_arr + total_lods*4;
      long need = flg_pos + (long)cum_v*4;
      if (need > head.cap) { while (need > head.cap) head.cap*=2;
                             head.d=(u8*)realloc(head.d,head.cap); }
      memset(head.d+flg_pos,0,(long)cum_v*4);
      head.len = need;                      /* reserve it in the header region */
    }

    /* ===== vertex blobs -> region VERTS =====
       blob = [uint32 datalen][modelvertex_t x nv];  vertlocalofs=4 (rel blob). */
    for(i=0;i<total_lods;i++){
        const mesh_t *m=(i==0)?vis:lods[i-1];
        unsigned int k; long bs=verts.len;
        long dl=(long)m->nv*32+4;
        mb_i32(&verts,dl);
        for(k=0;k<m->nv;k++){
            float uv[2]={0.0f,0.0f};
            mb_put(&verts,&m->v[k],12);
            mb_put(&verts,&m->n[k],12);
            mb_put(&verts,uv,8);
        }
        memcpy(head.d+vb_arr+i*4,&bs,4);
    }

    /* ===== strip index blobs -> region IDX ===== */
    for(i=0;i<total_lods;i++){
        const mesh_t *m=(i==0)?vis:lods[i-1];
        unsigned int k; long bs=idxs.len;
        vvp_siblob_t sh;
        sh.numindices=(long)m->nt*3; sh.indexdataofs=8;  /* u16[] starts right after hdr */
        mb_put(&idxs,&sh,sizeof sh);
        for(k=0;k<m->nt;k++){
            mb_u16(&idxs,(u16)(m->tri[k*3+0]+lod_voff[i]));
            mb_u16(&idxs,(u16)(m->tri[k*3+1]+lod_voff[i]));
            mb_u16(&idxs,(u16)(m->tri[k*3+2]+lod_voff[i]));
        }
        mb_pad4(&idxs);
        memcpy(head.d+si_hdr_pos+sizeof(vvp_sihdr_t)+i*4,&bs,4);
    }

    /* ===== fixup blobs -> region REST =====
       StripFixupHeader_t{numfixups=1, firstfixupofs}
         -> StripFixup_t{type=STRIP_TABLE_FIXUP_LIST, next=0, numindexes=1,
                         indexlistofs -> SourceDestListEntry_t{dest=0, source=0}}
       All offsets inside a blob are RELATIVE TO THE BLOB START. */
    for(i=0;i<total_lods;i++){
        long bs=rest.len;
        vvp_fublob_t fh; vvp_fixup_t fx; vvp_sd_t sd;
        fh.numfixups=1; fh.firstfixupofs=(v32)sizeof(fh);
        mb_put(&rest,&fh,sizeof fh);                       /* +0 */
        fx.fixuptype=STRIP_TABLE_FIXUP_LIST;
        fx.nextfixupoffset=0;
        fx.numindexes=1;
        fx.indexlistofs=(v32)(sizeof(fh)+sizeof(fx));      /* rel to blob start */
        mb_put(&rest,&fx,sizeof fx);                       /* +8 */
        sd.destoffset=0; sd.sourceoffset=0;                /* strip 0 <- idx blob 0 */
        mb_put(&rest,&sd,sizeof sd);                       /* +24 */
        memcpy(head.d+fu_hdr_pos+sizeof(vvp_fuhdr_t)+i*4,&bs,4);
    }

    /* ===== per-LOD tristrips + proxy (shared data blocks) -> REST ===== */
    {
        long ts_abs[8], px_abs[8];
        for(i=0;i<total_lods;i++){
            const mesh_t *m=(i==0)?vis:lods[i-1];
            vvp_ts_t ts; vvp_tip_t tip; vvp_psz_t psz;
            unsigned int k;
            ts_abs[i]=rest.len;
            ts.numtristrips=1; ts.stripsfirstofs=sizeof(ts);
            ts.numtrifans=0;   ts.fansfirstofs=0;
            ts.numisolatedtris=0; ts.trisfirstofs=0;
            mb_put(&rest,&ts,sizeof ts);
            tip.numindices=(long)m->nt*3; tip.indexstartofs=sizeof(tip);
            mb_put(&rest,&tip,sizeof tip);
            for(k=0;k<m->nt;k++){
                mb_u16(&rest,(u16)(m->tri[k*3+0]+lod_voff[i]));
                mb_u16(&rest,(u16)(m->tri[k*3+1]+lod_voff[i]));
                mb_u16(&rest,(u16)(m->tri[k*3+2]+lod_voff[i]));
            }
            mb_pad4(&rest);
            px_abs[i]=rest.len;
            psz.size=0; psz.count=0; mb_put(&rest,&psz,sizeof psz);
        }
        /* ===== per-LOD LodLevel_t with inline LodMaterialGroup_t ===== */
        for(i=0;i<total_lods;i++){
            vvp_lodlvl_t lv; vvp_lomg_t mg;
            long lvabs=rest.len;
            lvl_r[i]=lvabs;
            lv.switchpoint = (i==total_lods-1) ? 1e9f : 500.0f*(float)(i+1);
            lv.nummaterialgroups=1;
            lv.materialgroupfirstofs=(v32)sizeof(lv);
            lv.maxscale=1;
            lv.nextLODOffset = (i==total_lods-1)?0:(v32)(sizeof(lv)+sizeof(mg));
            mb_put(&rest,&lv,sizeof lv);
            mg.tristripsofs = (v32)(ts_abs[i] - lvabs - (long)sizeof(lv));
            mg.propertyofs  = (v32)(px_abs[i] - lvabs - (long)sizeof(lv));
            mg.material = 0;
            mb_put(&rest,&mg,sizeof mg);
        }
    }
    /* patch LODFlexRecord.lodlevel (rel to VertCacheHeader) */
    for(i=0;i<total_lods;i++){
        long recp=lr_pos+i*(long)sizeof(vvp_lodrec_t);
        long rel=lvl_r[i]-vc_pos;
        memcpy(head.d+recp+offsetof(vvp_lodrec_t,lodlevel),&rel,4);
    }

    /* ===== MaterialListHeader + names + skin families -> REST ===== */
    ml_pos=rest.len;
    { vvp_mlhdr_t mh;
      mh.listlen=(v32)matlen;
      mb_put(&rest,&mh,sizeof mh);
      mb_put(&rest,matline,matlen); mb_pad4(&rest);
      /* one material entry — engine appends ".vmt" when resolving,
         so the stored name must NOT contain the extension */
      { char nm[256]; long n;
        strncpy(nm,matline,sizeof nm-1); nm[sizeof nm-1]=0;
        n=(long)strlen(nm);
        if (n>=4 && !memcmp(nm+n-4,".vmt",4)) n-=4;
        nm[n]=0; n++;                 /* incl NUL */
        mb_i32(&rest,(long)n);
        mb_put(&rest,nm,n); mb_pad4(&rest);
      }
      /* skin families: 1 family, 1 material, arrayIndex 0 */
      mb_i32(&rest,1);          /* numFamilies */
      mb_i32(&rest,1);          /* numMaterials in family */
      mb_i32(&rest,1);          /* width */
      mb_i32(&rest,1);          /* height */
      mb_i32(&rest,1);          /* sz */
      mb_i32(&rest,0);          /* arrayIndex[0] */
    }

    /* ===== assembly ===== */
    verts_base=head.len;
    idxs_base =head.len+verts.len;
    rest_base =head.len+verts.len+idxs.len;

    /* blob arrays currently hold REGION-relative offsets; convert to
       header-relative absolute deltas */
    { long arr=vb_arr;
      for(i=0;i<total_lods;i++){ long v; memcpy(&v,head.d+arr+i*4,4); v+=verts_base-vb_hdr_pos; memcpy(head.d+arr+i*4,&v,4); } }
    { long arr=si_hdr_pos+sizeof(vvp_sihdr_t);
      for(i=0;i<total_lods;i++){ long v; memcpy(&v,head.d+arr+i*4,4); v+=idxs_base-si_hdr_pos; memcpy(head.d+arr+i*4,&v,4); } }
    { long arr=fu_hdr_pos+sizeof(vvp_fuhdr_t);
      for(i=0;i<total_lods;i++){ long v; memcpy(&v,head.d+arr+i*4,4); v+=rest_base-fu_hdr_pos; memcpy(head.d+arr+i*4,&v,4); } }

    /* ModelData.materialdataofs (rel to VertCacheHeader) */
    { long v=rest_base+ml_pos-vc_pos;
      memcpy(head.d+md_pos+offsetof(vvp_mdata_t,materialdataofs),&v,4); }

    /* header.lodofs = first LodLevel absolute */
    { long v=rest_base+lvl_r[0];
      memcpy(head.d+offsetof(vvp_header_t,lodofs),&v,4); }

    /* header MatRepData */
    { ST32(head.d+offsetof(vvp_header_t,matreldataoffset),(rest_base+ml_pos)-(long)offsetof(vvp_header_t,materiallist_strlen));
      ST32(head.d+offsetof(vvp_header_t,materiallist_strlen),matlen);
      ST32(head.d+offsetof(vvp_header_t,num_skins),1);
      ST32(head.d+offsetof(vvp_header_t,num_materials),1);
    }

    f=fopen(path,"wb");
    if(!f){ERR("cannot open %s",path);mb_free(&head);mb_free(&verts);mb_free(&idxs);mb_free(&rest);return -1;}
    fwrite(head.d,1,head.len,f);
    fwrite(verts.d,1,verts.len,f);
    fwrite(idxs.d,1,idxs.len,f);
    fwrite(rest.d,1,rest.len,f);
    fclose(f);

    INFO("wrote %s (%ld bytes: %d LOD(s), %u verts / %u tris)",
         path, head.len+verts.len+idxs.len+rest.len, total_lods, cum_v, cum_t);

    mb_free(&head); mb_free(&verts); mb_free(&idxs); mb_free(&rest);
    return 0;
}
