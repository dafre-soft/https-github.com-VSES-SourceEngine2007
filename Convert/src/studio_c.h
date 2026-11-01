/* studio_c.h - Plain-C mirror of Valve's Source (2007) SDK structures from
 * src_main/public/studio.h.  Field names, types and layout are kept identical
 * to the original C++ headers so that this self-contained compiler produces
 * byte-compatible MDL/VVD files for the Episode Two / Orange Box era engine
 * (studiohdr version 48, vvd version 4).
 */
#ifndef STUDIO_C_H
#define STUDIO_C_H

#include <stdint.h>

typedef int32_t  intu;      /* "int" in the original headers               */
typedef uint32_t uintu;
typedef float    fltim;

#ifndef VECTOR_H
typedef struct { fltim x, y, z; } vec3d_t;
#endif
typedef struct { fltim x, y, z, w; } vec4d_t;

/* Four-component packed vertex blendweight (byte4 in the SDK) */
typedef struct { unsigned char b0, b1, b2, b3; } blendweight_c_t;

#define MDL_MAX_NAME     32
#define MDL_MAX_PATH     64
#define MDL_MAX_BONES    128
#define MDL_MAX_BLEND    8
#define MDL_MAX_SEQCTRL  2
#define MDC_HITBOXMAX    8
#define MDC_HULLGROUPMAX 4
#define BONE_MASS        1.0f

/* ------------------------------------------------------------------ flags */
/* studiohdr_t surfaceFlags / flags */
#define STUDIOHDR_FLAGS_MONOLITHIC_LBONE   0x00000001
#define STUDIOHDR_FLAGS_HITBOX_SET         0x00000002
#define STUDIOHDR_FLAGS_ONLY_SHADOW        0x00000004
#define STUDIOHDR_FLAGS_NO_FORCED_FADE     0x00000010
#define STUDIOHDR_FLAGS_FORCE_OPACITY      0x00000020
#define STUDIOHDR_FLAGS_HITBOX_COLLIDE     0x00000040
/* studiomdl propcompile flags (studiohdr "flags" field, EP2 era):
 * the engine ONLY generates . vtx on load when PROPCOMPILE_VTX is set;
 * without it a static prop has no vertex layer and renders as nothing. */
#define PROPCOMPILE_STATIC_PROP            0x00000001
#define PROPCOMPILE_VERTEX_PROCESSING_INFO 0x00000002
#define PROPCOMPILE_LIGHTMAP_SINGLE_PASS   0x00000004
#define PROPCOMPILE_NO_CACHE               0x00000008
#define PROPCOMPILE_FULL_OPTIMIZATION      0x00000010
#define PROPCOMPILE_NO_FIXUP_CRACKS        0x00000020
#define PROPCOMPILE_KEEP_SPLITTING         0x00000040
#define PROPCOMPILE_KEEP_INDEX             0x00000080
#define PROPCOMPILE_ONLY_SHADOWS           0x00000100
#define PROPCOMPILE_NO_MERGE               0x00000200
#define PROPCOMPILE_FORCE_NOobsidian       0x00000400
#define PROPCOMPILE_FORCE_HARDOBSIDIAN     0x00000800
#define PROPCOMPILE_RUN_STUDIODUMPPROPS    0x00001000
#define PROPCOMPILE_LAZY_EXTRACT           0x00002000
#define PROPCOMPILE_DONT_GENERATE_VTX      0x00004000
#define PROPCOMPILE_VTX                    0x00008000
#define TRANSLUCENT                        0x00000001
#define MODELTRANSPARENCY_NONE             0
#define MODELTRANSPARENCY_ALPHA            1

/* mstudiobone_t flags */
#define BONE_NEVERLAST                     0x00000001
#define BONE_ALWAYSLAST                    0x00000002
#define BONE_USED_BY_HITBOX                0x00000100
#define BONE_USED_BY_ATTACHMENT            0x00000200
#define BONE_USED_BY_MASKBONES             0x00000400
#define BONE_USED_BY_BC                    0x00000800
#define BONE_USED_BY_KINECT                0x00001000
#define BONE_PROCEDURAL                    0x00002000
#define BONE_USED_BY_PHYSICS               0x00008000
#define BONE_BOX                           0x00010000
#define BONE_ANIMATION                     0x00020000
#define BONE_CONFIG                        0x00040000
#define BONE_JIGGLE                        0x00080000
#define BONE_FLEX                          0x00100000
#define BONE_SURFACE                       0x00200000
#define BONE_WORKFLOW_CONTROL              0x00400000
#define BONE_RENDER_TO_TEXTURE             0x00800000
#define BONE_ADDITIONAL_WORLDSPACE_PARENT  0x01000000
#define BONE_USED_BY_VEHICLE_CLIMBABLE_WALLS 0x02000000
#define BONE_USED_BY_CHANGE_EYEPODS        0x04000000
#define BONE_USED_BY_CREATION_ENGINE_AUDIO 0x08000000

/* mesh flags */
#define MESH_FLAG_DRAWABLE                 0x01
#define MESH_FLAG_CASTS_SHADOW             0x02

/* model flags */
#define MODELFLAG_AUTOSCALE_TEXTURES       0x01
#define MODELFLAGS_SPLITBONECONTAINER      0x02

/* sequence flags */
#define SEQ_FLAG_LOOPING                   0x00000001
#define SEQ_FLAG_HITBOXSET                 0x00000002
#define SEQ_FLAG_TRANSMIT                   0x00000004
#define SEQ_FLAG_UNKNOWN_INTERPOLATION     0x00000008
#define SEQ_FLAG_AUTO_MERGE_LOOPS          0x00000010
#define SEQ_FLAG_NOACTIVITYEVENTS          0x00000020
#define SEQ_FLAG_ALTERNATE_SEQUENCE        0x00000040
#define SEQ_FLAG_PRIMARY_SEQUENCE          0x00000080

/* physics collision flags */
#define SOLID_BBOX                         1
#define SOLID_CUSTOM                       5
#define COLLISION_ONLY_DYNAMIC             0x00000001
#define COLLISION_ONLY_STATIC              0x00000002
#define CONTENTS_SOLID                     0x00000001
#define MASK_PLAYERSOLID                   0x00000001

/* ---------------------------------------------------------------- header */
#define IDSTUDIOHEADER   "IDST"
#define IDSTUDIOBONES    "ISTU"
#define STUDIOHDR_VERSION 48
#define VVD_VERSION       4

/* texture flags (mstudiotexture_t.flags) */
#define EX_PERSISTENT_LOWRES     0x1
#define EX_PERSISTENT_MEDIUMRES  0x2
#define EX_PERSISTENT_HIGHRES    0x4
#define EX_RENDER_TARGET         0x8
#define EX_NO_LIGHTMAP           0x10
#define EX_PERSISTENT_NORMALMAP  0x20

/* collision model types */
#define CONCAVE_TILEMODE          1
#define CONCAVE_OCTREEMODE        2
#define CONCAVE_STATICMODEL       3
#define CONCAVE_TYPESPIECEMODE    7

#pragma pack(push, 4)

typedef struct {
    char        id[4];            /* "IDST"                                    */
    intu        version;          /* 48                                        */
    intu        checksum;
    char        szname[64];       /* studiohdr_t::szname — bytes[64], model name */
    intu        length;
    intu        eyeposition;      /* vec3d offset                             */
    intu        unused1;          /* (v48: unused)                              */
    intu        hitboxsetoffset;  /* mstudiohitboxset_t lump                    */
    intu        simplemodelinfo;  /* mstudiesimplemodelinfo_t lump              */
    intu        unused2;          /* (v48: unused)                              */
    intu        maxscale;
    intu        flags;
    intu        usedcount;
    intu        bbonecount;       /* number of bones                            */
    intu        bboneoffset;
    intu        cbonecount;       /* count of runtime needed bones              */
    intu        cboneoffset;
    intu        bcontrollercount;
    intu        bcontrolleroffset;
    intu        ccontrollercount;
    intu        ccontrolleroffset;
    intu        bmotioncount;
    intu        bmotionoffset;
    intu        btexturecount;
    intu        btextureoffset;
    intu        ctexturecount;
    intu        ctextureoffset;
    intu        tskinrefcount;
    intu        tskinoffset;
    intu        tlocalordercount;
    intu        tlocalorderoffset;
    intu        hostablecount;
    intu        hostableoffset;
    intu        collisionmodelinfo_count;
    intu        collisionmodelinfo_offset;
    intu        ctexturedatacount;
    intu        ctexturedataoffset;
    intu        skinrefcount_unused;
    intu        skinrefoffset_unused;
    intu        keydatacount;
    intu        keydataoffset;
    intu        cinfluencecount;
    intu        cinfluenceoffset;
    intu        boneMergeBonesCount;
    intu        boneMergeBonesOffset;
    intu        _dummy;
    intu        boneMergeStartBoneIndex;
    intu        camPathCount;
    intu        camPathOffset;
    intu        boneRotationCount;
    intu        boneRotationOffset;
} studiohdr_c_t;

typedef struct {
    intu        sznameindex;
    intu        parent;
    intu        bonecontrollerparent;
    intu        physicsbone;
    fltim       poseToBone[3][4];
    vec3d_t     qAlignment;
    intu        rotaxis;
    vec3d_t     pos;
    vec3d_t     quat;             /* stored as x,y,z,w (quat_t is a union)     */
    vec3d_t     rotation;
    vec3d_t     translation;
    intu        boneflags;
    intu        bonecontrollers;
    intu        localToTranslational;
    intu        localToRotational;
} mstudiobone_c_t;

typedef struct {
    intu        bone;
    intu        base;
    fltim       smooth;
    intu        _unused;
} mstudiobonecontroller_c_t;

typedef struct {
    intu        lookupType;
    intu        lookupName;
    intu        params[MDL_MAX_SEQCTRL];
    intu        count[MDL_MAX_SEQCTRL];
    intu        pad[4];
} mstudiomovement_c_t;

typedef struct {
    intu        texture;
    intu        flags;
    intu        u;
    intu        v;
} mstudyuvanim_t;


typedef struct {
    intu        bodypart;
    fltim       deformscale;
    intu        model;
    intu        pad;
} mstudio_model_t;

typedef struct {
    intu        nummodels;
    intu        base;
    intu        testbase;
    intu        testgroup;
    char        name[MDL_MAX_NAME];
    intu        pad;
} mstudiobodyparts_c_t;

/* mstudiomodel_t - studiohdr v48 layout (write.cpp: WriteModel) */
typedef struct {
    char        name[MDL_MAX_NAME];
    intu        type;
    intu        bonecontrollercount;      /* always 0 in v48 */
    intu        bonecontrolleroffset;     /* always 0 in v48 */
    intu        keyvalueindex;            /* fltim-equivalent offset into modelkeyvalue data */
    intu        meshcount;
    intu        meshoffset;
    intu        modelvertexcount;
    intu        vertexoffset;             /* lump of mstudiovertex_t (legacy) */
    intu        trianglecount;
    intu        triangleoffset;           /* lump of mstudiotriangle_t */
    intu        skinrefcount;
    intu        skinoffset;
    intu        lodcount;
    intu        lodoffset;
    intu        bgmcount;                 /* studiohdr >= 41 */
    intu        bgmoffset;
    intu        cgmcount;
    intu        cgmoffset;
    intu        vertexdata;               /* ModelVertexData_t start */
    /* ModelVertexData_t (inline): */
    intu        vertdeformcount;
    intu        vertdeformoffset;
    intu        normalscount;
    intu        normoffset;
    intu        tangentscount;
    intu        tangentoffset;
    fltim       vertdeformstarttime;
    fltim       vertdeformfinaltime;
    intu        uvcount;                  /* studiohdr >= 43: mstudiovertexuv_t array */
    intu        uvoffset;
} mstudiomodel_c_t;

typedef struct {
    unsigned short texels[3];
    short          lightmapofs[3];
} mstudiovertexuv_t;

typedef struct {
    intu        count;                    /* always 3 */
    intu        index[3];
    intu        flags;
    intu        id;
} mstudiotriangle_t;

typedef struct {
    intu        firstlevel;
    intu        remaplevel[8];
    intu        force_material;
} mstudiomodel_lod_t;

typedef struct {
    intu        lod;
    intu        city;
    intu        _pad[2];
} mstudiesimplemodelinfo_t;

typedef struct {
    intu        numbones;
    intu        boneoffset;
    intu        numbonecontrollers;
    intu        bonecontrolloffset;
    intu        hitboxset;
    intu        hitboxoffset;
    intu        hitboxcount;
    intu        localoffset;
    intu        namingoffset;
    intu        keyvalueindex;
    intu        source;
    intu        ids;
    intu        pad[4];
} mstudioboneproxy_c_t;

typedef struct {
    intu        bone;
    intu        group;
    vec3d_t     origin;
    vec3d_t     normal;
    vec3d_t     vert;
    fltim       dist;
    intu        flags;
    intu        _pad;
} mstudiocollisionmeshvert_t;

typedef struct {
    vec3d_t     center;
    fltim       radius;
    intu        vertical;
    intu        _pad;
} mstudiocollisionmeshcapsulevert_t;

typedef struct {
    intu        index;
    intu        _pad[3];
} mstudiocollisionedgetrim_t;

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        parent;
    fltim       pos[3];
    fltim       quat[4];
    intu        radius;
    intu        length;
    intu        how;
    intu        fixed;
    intu        mass;             /* fltim                                     */
    intu        pads;
    intu        _pad[2];
} mstudioradialedge_t;

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        parent;
    fltim       pos[3];
    fltim       quat[4];
    intu        radius;
    intu        length;
    intu        how;
    intu        fixed;
    intu        mass;
    intu        pads;
    intu        _pad[2];
} mstudioradialedge2_t;

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        parent;
    fltim       pos[3];
    fltim       quat[4];
    intu        radius;
    intu        length;
    intu        how;
    intu        fixed;
    intu        mass;
    intu        pads;
    intu        _pad[2];
} mstudiojointballsocket_t;

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        parent;
    fltim       pos[3];
    fltim       quat[4];
    intu        radius;
    intu        length;
    intu        how;
    intu        fixed;
    intu        mass;
    intu        pads;
    intu        _pad[2];
} mstudiojointaxis_t;

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        parent;
    fltim       pos[3];
    fltim       quat[4];
    intu        radius;
    intu        length;
    intu        how;
    intu        fixed;
    intu        mass;
    intu        pads;
    intu        _pad[2];
} mstudiojoinslider_t;

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        parent;
    fltim       pos[3];
    fltim       quat[4];
    intu        radius;
    intu        length;
    intu        how;
    intu        fixed;
    intu        mass;
    intu        pads;
    intu        _pad[2];
} mstudiojointuniversal_t;

/* ------------------------------------------------------------- mesh data */
typedef struct {
    char        name[MDL_MAX_NAME];             /* 64 */
    intu        material;
    intu        meshvertexdata;                 /* MeshVertexData_t start */
    /* MeshVertexData_t inline: */
    intu        vertexoffset;                   /* legacy mstudiovertex_t */
    intu        vertexcount;
    intu        triangleoffset;
    intu        trianglecount;
    intu        wedgecount;
    intu        wedgeoffset;
    intu        fixupcount;
    intu        fixupoffset;
    intu        bonetablecount;
    intu        bonetableoffset;
    intu        normcount;
    intu        normoffset;
    intu        tangentcount;
    intu        tangentoffset;
    intu        uvcount;
    intu        uvoffset;
    intu        verts;                          /* == vertexoffset */
    fltim       vertdeformstarttime;
    fltim       vertdeformfinaltime;
} mstudiomesh_c_t;

typedef struct {
    vec3d_t     pos;
    fltim       weight[4];
    intu        bone[4];
    intu        _pad;
} mstudiovertex_t;

/* EXACT 48-byte on-disk VVD vertex, identical layout to mstudiovertex_t:
 * float weights[4] (16B) + char bone[4] (4B) + pad(4B)? -- see studio.h:
 * mstudioboneweight_t = float weight[4]; char bone[4]; byte numbones; +pad
 * then Vector pos, Vector normal, Vector2D uv. */
typedef struct {
    fltim       weight[4];
    unsigned char bone[4];
    unsigned char numbones;
    unsigned char pad[3];
    vec3d_t     pos;
    vec3d_t     normal;
    fltim       u, v;
} vvdvertex_c_t;

typedef struct {
    intu        vertexoffset;
    intu        vertexcount;
    intu        triangleoffset;
    intu        trianglecount;
    intu        material;
    intu        flags;
} virtualmeshlods_t;

typedef struct {
    intu        lodlevel;
    intu        loderror;
} virtualmeshtransition_t;

typedef struct {
    intu        vertexes;
    intu        vertexsize;
    intu        triangles;
    intu        trianglecount;
    intu        lod;
    intu        lodcount;
    intu        transition;
    intu        transitioncount;
    intu        mesh;
    intu        meshcount;
    intu        rootnormal;
    intu        rootnormalcount;
    intu        fixup;
    intu        fixupcount;
    char        name[MDL_MAX_NAME];
    intu        source;
    intu        _pad[3];
} virtualmeshdesc_t;

typedef struct {
    char        id[4];            /* "VVD1"                                   */
    intu        version;          /* 4                                         */
    intu        checksum;
    intu        filelen;
    virtualmeshdesc_t meshdesc;
} vvdheader_c_t;

/* EXACT SDK layout (vphysics/vmodelinfo): 280 bytes, no padding holes. */
typedef struct {
    intu        size;                                   /* = sizeof this struct      */
    intu        sizefixup;                              /* = sizeof fixup vertex rec */
    char        name[MDL_MAX_NAME];                     /* "vFixupHeader"            */
    vec3d_t     origin;                                 /* model origin              */
    float       scale;                                  /* model scale               */
    intu        maxweightcount;                         /* 1 for static props        */
    intu        verttablesize;                          /* # fixup vertices          */
    intu        triangletablesize;                      /* # fixup triangles         */
    intu        fixupcacheoffset;                       /* -> vFixupCache            */
    intu        vertexidstableoffset;                   /* -> vVertexIDS             */
    intu        triangleidstableoffset;                 /* -> vTriangleIDS           */
    intu        vertextransformsoffset;                 /* -> vTransforms            */
    intu        vertextransformscount;                  /* == verttablesize          */
    intu        triangletransformsoffset;               /* -> vTransforms            */
    intu        triangletransformscount;                /* == triangletablesize      */
    intu        _pad[47];                               /* pad to 280 bytes          */
} virtualmeshfixupheader_t;

/* EXACT SDK layout (virtualmeshfixupvertex_t): 40 bytes. */
typedef struct {
    intu            vertexid;
    intu            triangleid;
    unsigned short  vertexfixup;
    unsigned short  trianglefixup;
    vec3d_t         pos;
    float           flWeight[4];
    unsigned char   unBone[4];
} virtualmeshfixupvertex_t;

/* vFixupCache record: 16 bytes */
typedef struct {
    intu        vertindexstart;
    intu        vertindexcount;
    intu        triindexstart;
    intu        triindexcount;
} virtualmeshfixupcache_t;

/* ------------------------------------------------------------ collision */
typedef struct {
    intu        type;             /* 7 = convex                                */
    intu        count;            /* triangle count                            */
    intu        vertcount;
    intu        vertoffset;
    intu        vertparent;
    intu        edgetrimoffset;
    intu        edgetrimcount;
    intu        faceoffset;
    intu        facecount;
    intu        boxoffset;
    intu        edgemargin;
    intu        facemargin;
    intu        spherecenter;
    intu        sphereradius;
    intu        surfacenameIndex;
    intu        surfaceprop;
    intu        surfaceflags;
    intu        iCenterMass;
    intu        iMass;
    intu        iPad;
    fltim       Slab[3];
    fltim       CenterOfMass[3];
    fltim       MomentsOfInertia[3];
    fltim       ProductsOfInertia[3];
    fltim       AABBMin[3];
    fltim       AABBMax[3];
    intu        _pad[8];
} mstudiocollisionmodel_c_t;

typedef struct {
    intu        index;
    intu        normal;
    intu        vertindex;
    intu        parent;
    intu        next;
    intu        _pad[3];
} mstudiocollisionmodeltri_t;

/* --------------------------------------------------------------- hitbox */
typedef struct {
    intu        bone;
    intu        group;
    vec3d_t     bbmin;
    vec3d_t     bbmax;
    intu        sznameindex;
    intu        solid;
    intu        _pad[2];
} mstudiobonebox_c_t;

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        hitboxcount;
    intu        hitboxoffset;
} mstudiohitboxset_c_t;

typedef struct {
    intu        material;
    intu        flags;
} mstudiomaterialmod_t;

/* -------------------------------------------------------------- textures */
typedef struct {
    intu        flags;            /* persistent group hdr (unused -> zeros)   */
    intu        _pad[3];
} mstudiopersistentgrouphdr_c_t;

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        flags;
    intu        usage;
    fltim       bbox[2];
    intu        texdatacount;
    intu        texdataoffset;
    fltim       absorb[3];
    fltim       scattered[3];
    intu        material;         /* string table index of full material path */
    intu        _pad[2];
} mstudiotexture_c_t;

typedef struct {
    intu        imageoffset;
    intu        imagesize;
    intu        uncompressedsize;
    intu        _pad;
} mstudiotexturedata_c_t;

typedef struct {
    intu        flags;
    intu        totalUnpackedSize;
    intu        maxFrame;
    intu        maxScale;
    intu        pad[4];
} mstudiopackedtex_flex_t; /* not used */

typedef struct {
    char        name[MDL_MAX_NAME];
    intu        parent;
    intu        flags;
    intu        numchannels;
    intu        channelarray;
    intu        channels;
    intu        pad;
} mstudioanimrec_c_t;

typedef struct {
    intu        channelvalue;
    intu        channelindex;
    intu        next;
    intu        _pad;
} mstudioanimlist_t;

typedef struct {
    intu        channeltype;
    intu        startframe;
    intu        endframe;
    intu        next;
    intu        _pad;
} mstudioanimvalue_desc_t;

typedef struct {
    intu        channelvalue;
    intu        channelindex;
    intu        next;
    intu        _pad;
} mstudioanimvaluelist_t;

/* -------------------------------------------------------------- sequence */
typedef struct {
    char        label[32];
    intu        duration;
    intu        time;
    intu        numlines;
    intu        weight;
    intu        numcontrols;
    intu        controlptr;
    intu        interactioncount;
    intu        interactionoffset;
    intu        motioncount;
    intu        motionoffset;
    intu        animblocktype;
    intu        animblockindex;
    intu        sectionunknown;
    intu        movementcount;
    intu        movementoffset;
    fltim       previewreference;
    intu        automovementindex;
    intu        collisionindex;
    intu        eventsindex;
    intu        eventscount;
    intu        framerate;
    intu        entrynode;
    intu        exitnode;
    intu        nodelookahead;
    intu        seqgroup;
    char        inactsequence[32];
    intu        activity;
    fltim       actweight;
    intu        numacttrans;
    intu        acttransoffset;
    intu        numiklocks;
    intu        iklocksoffset;
    intu        keyvalueindex;
    intu        alpha;
    intu        beta;
    intu        interpout;
    intu        interpin;
    fltim       poseparamstart;
    fltim       poseparamend;
    fltim       cycle;
    fltim       cycletoduration;
    intu        flags;
    intu        activitypauseweight;
    intu        weightlist;
    intu        numweights;
    intu        pad[4];
} mstudioseqdesc_c_t;

typedef struct {
    intu        controller;
    intu        master;
    fltim       lo;
    fltim       hi;
    fltim        loBlend;
    fltim        hiBlend;
    fltim       SplineMasterValue;
} mstudioseqctrl_t;

typedef struct {
    intu        szNameIndex;
    intu        hldam;
    intu        ldam;
    intu        ldamp;
    intu        ldt;
    intu        ldfreq;
    intu        rdfdam;
    intu        rdam;
    intu        rdamp;
    intu        rdt;
    intu        rdfreq;
    intu        rdir;
    intu        _pad;
} mstudiojigglebone_t;

#pragma pack(pop)

#endif /* STUDIO_C_H */
