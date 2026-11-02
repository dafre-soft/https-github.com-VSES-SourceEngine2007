/* vvp_layout.h - VertexFile structures for Source 2007 engine (.vtx / VVP format)
 * Transcribed from MDLLib vertex_file_layout_public.h and studiomdl gordon2 source.
 * Version 64 corresponds to HL2/Ep1 range; matches studiohdr v48 + VVD v4 output.
 */
#ifndef VVP_LAYOUT_H
#define VVP_LAYOUT_H

#include <stdint.h>

typedef int32_t  v32;   /* All VVP offsets/sizes are 4 bytes */
typedef uint16_t u16;
typedef uint8_t  u8;

#define VVP_ID       0x50564844L  /* MAKEID('V','P','H','D') */
#define VVP_VERSION  64

enum { MODELVERT_TYPE_3D = 0 };
enum { STRIP_TABLE_FIXUP_LIST = 0 };

#pragma pack(push, 1)

/* ===== VertexFileHeader_t (56 bytes) ===== */
typedef struct {
    v32 id;                    /* 'VPHD' */
    v32 version;               /* VVP_VERSION */
    v32 clientversion;         /* always 0 */
    v32 flags;                 /* always 0 */
    v32 contentflags;          /* CONTENTS_SOLID = 1 */
    
    v32 entrysize;             /* sizeof(VertexFileHeader_t) + 16 = 72 */
    v32 numentries;            /* always 1 */
    
    v32 vertcacheofs;          /* abs -> VertCacheHeader_t */
    v32 lodofs;                /* abs -> first LodLevel_t */
    
    /* MatRepData inline: [dataoffset][strlen][skins][materials] */
    v32 matreldataoffset;      /* rel to &materiallist_strlen */
    v32 materiallist_strlen;   /* strlen of material list name incl NUL */
    v32 skins;                 /* num skin families */
    v32 materials;             /* num materials */
    
    v32 bodypartcount;         /* always 1 */
} vvp_header_t;

typedef char _check_vvp_header[(sizeof(vvp_header_t)==56)?1:-1];

/* ===== VertCacheHeader_t (48 bytes) ===== */
typedef struct {
    v32 maxnum_bones_per_hardware_vb;
    v32 maxnum_bones_per_software_vb;
    v32 maxnum_bones_per_strip;
    v32 maxnum_bones_per_triangles;
    v32 maxnum_verts_per_hw_vb;
    v32 maxnum_tris_per_hw_vb;
    v32 maxmodeldeform;        /* always 0 */
    v32 maxscale;              /* VertexFileHeader::maxscale */
    
    v32 vertblockheaderofs;    /* rel to VertCacheHeader */
    v32 stripindexblockheaderofs;
    v32 fixupblockheaderofs;
    v32 bodypartofs;           /* rel to VertCacheHeader -> BodyPartData[0] */
} vvp_vc_t;

typedef char _check_vvp_vc[(sizeof(vvp_vc_t)==48)?1:-1];

/* ===== VertBlockHeader_t (36 bytes) ===== */
typedef struct {
    v32 numblobs;
    v32 blobarrayofs;          /* rel to this header */
    v32 verttype;              /* MODELVERT_TYPE_3D */
    v32 vertsize;              /* bytes per vertex (pos+nrm+uv = 32) */
    
    v32 vertflagsofs;          /* 0 if no flags */
    v32 vertlocalofs;          /* rel to each blob start */
    v32 vertboneofsofs;        /* 0 if static */
    v32 vertbonecntofs;        /* 0 if static */
    v32 vertlightofs;          /* 0 if no light */
} vvp_vbhdr_t;

typedef char _check_vvp_vbhdr[(sizeof(vvp_vbhdr_t)==36)?1:-1];

/* ===== VertBlob_t: datalen followed by vertices ===== */
typedef struct {
    v32 datalen;               /* numverts * vertsize */
    /* then: pos(3f), nrm(3f), uv(2f) per vertex */
} vvp_vblob_t;

/* ===== StripIndexBlockHeader_t (8 bytes) ===== */
typedef struct {
    v32 numblobs;
    v32 blobarrayofs;          /* rel to this header */
} vvp_sihdr_t;

/* ===== StripIndexBlob_t: datalen, indexstartrel, then u16 indices ===== */
typedef struct {
    v32 datalen;               /* numindices * 2 */
    v32 indexstartofs;         /* rel to this blob start = 8 */
    /* then: u16 indices[] */
} vvp_siblob_t;

/* ===== FixupBlockHeader_t (8 bytes) ===== */
typedef struct {
    v32 numblobs;
    v32 blobarrayofs;          /* rel to this header */
} vvp_fuhdr_t;

/* ===== FixupBlobHeader_t ===== */
typedef struct {
    v32 numfixups;
    v32 fixupsofs;             /* rel to this header */
} vvp_fublob_t;

/* ===== FixupSourceDest_t ===== */
typedef struct {
    v32 destofs;               /* rel to TriStip index array */
    v32 sourceofs;             /* rel to TriStip index array */
} vvp_sd_t;

/* ===== Fixup_t ===== */
typedef struct {
    v32 fixuptype;             /* STRIP_TABLE_FIXUP_LIST */
    v32 numsources;
    v32 nextoffset;            /* 0 if last */
    v32 firstsourcedestofs;    /* rel to this fixup */
} vvp_fixup_t;

/* ===== BodyPartData_t (8 bytes) ===== */
typedef struct {
    v32 modelcount;
    v32 modelfirstofs;         /* rel to this header */
} vvp_bpdata_t;

/* ===== ModelData_t (32 bytes) ===== */
typedef struct {
    v32 vertblockheaderofs;    /* rel to VertCacheHeader */
    v32 stripindexblockheaderofs;
    v32 fixupblockheaderofs;
    v32 materialdataofs;       /* rel to VertCacheHeader -> LodMaterialGroup_t[0] */
    v32 numlodrecords;
    v32 lodrecordfirstofs;     /* rel to VertCacheHeader -> LODFlexRecord_t[] */
    v32 maxscale;
} vvp_mdata_t;

typedef char _check_vvp_mdata[(sizeof(vvp_mdata_t)==32)?1:-1];

/* ===== LODFlexRecord_t (16 bytes) ===== */
typedef struct {
    v32 flexcount;
    v32 flexindexfirstofs;
    v32 lodlevel;              /* rel to VertCacheHeader -> LodLevel_t */
    v32 numswitchvertices;
    v32 switchvertexfirstofs;
} vvp_lodrec_t;

typedef char _check_vvp_lodrec[(sizeof(vvp_lodrec_t)==20)?1:-1];

/* ===== LodLevel_t (20 bytes) ===== */
typedef struct {
    float switchpoint;         /* dist where NEXT lod takes over */
    v32 nummaterialgroups;
    v32 materialgroupfirstofs; /* rel to LodLevel -> LodMaterialGroup_t[] */
    v32 maxscale;
    v32 nextLODOffset;         /* rel to LodLevel -> next LodLevel_t (0=none) */
} vvp_lodlvl_t;

typedef char _check_vvp_lodlvl[(sizeof(vvp_lodlvl_t)==20)?1:-1];

/* ===== LodMaterialGroup_t (12 bytes) ===== */
typedef struct {
    v32 tristripsofs;          /* rel to self -> TriStrips_t */
    v32 propertyofs;           /* rel to self -> MaterialProxySize_t */
    v32 material;              /* index into material list */
} vvp_lomg_t;

typedef char _check_vvp_lomg[(sizeof(vvp_lomg_t)==12)?1:-1];

/* ===== TriStrips_t (20 bytes) ===== */
typedef struct {
    v32 numtristrips;
    v32 stripsfirstofs;        /* rel to TriStrips */
    v32 numtrifans;
    v32 fansfirstofs;
    v32 numisolatedtris;
    v32 trisfirstofs;
} vvp_ts_t;

typedef char _check_vvp_ts[(sizeof(vvp_ts_t)==24)?1:-1];

/* ===== TriStip_t (8 bytes): u16 indices follow at indexstartofs ===== */
typedef struct {
    v32 numindices;
    v32 indexstartofs;         /* rel to TriStip start = 8 */
} vvp_tip_t;

/* ===== MaterialProxySize_t (8 bytes) ===== */
typedef struct {
    v32 size;                  /* 0 if no proxy */
    v32 count;                 /* 0 if no proxy */
} vvp_psz_t;

#pragma pack(pop)

#endif /* VVP_LAYOUT_H */
