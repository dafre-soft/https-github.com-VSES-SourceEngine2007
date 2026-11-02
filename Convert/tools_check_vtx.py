#!/usr/bin/env python3
"""Structural validator for .vtx (VVP) produced by stl2mdl, per Source 2007
(vertex_file_layout_public.h). Walks every pointer the engine dereferences on
load and fails loudly on any OOB / mismatch — this is exactly what crashes
the game when wrong."""
import struct, sys

def main(path):
    d = open(path,'rb').read(); n=len(d); errs=[]
    def u32(o):
        if o+4>n: raise Exception(f"OOB read at {o} (file {n})")
        return struct.unpack_from('<I',d,o)[0]
    def i32(o): return struct.unpack_from('<i',d,o)[0]
    def E(m): errs.append(m); print("!!",m)

    if u32(0)!=0x44485056: E("bad ID"); return errs
    ver=i32(4)
    vc=u32(28); lodofs=u32(32)
    print(f"VVP ver={ver} size={n} vertcacheofs={vc} lodofs={lodofs}")
    # VertCacheHeader_t = 64 bytes; block offsets live at +36..+48, RELATIVE to vc
    vbh=vc+i32(vc+36); sih=vc+i32(vc+40); fuh=vc+i32(vc+44); bpd=vc+i32(vc+48)
    nb=u32(vbh); blobarr=vbh+u32(vbh+4)
    nsi=u32(sih); siarr=sih+u32(sih+4)
    nfu=u32(fuh); fuarr=fuh+u32(fuh+4)
    if not(0<nb<64): E("numblobs")
    nvb=[]
    for i in range(nb):
        blob=vbh+u32(blobarr+i*4); dl=u32(blob); nv=(dl-4)//u32(vbh+12)
        nvb.append(nv)
        if blob+dl>n: E(f"vblob{i} OOB")
    cum=0
    for i in range(nsi):
        blob=sih+u32(siarr+i*4); ni=u32(blob); io=blob+u32(blob+4)
        if io+ni*2>n: E(f"siblob{i} OOB"); continue
        idxs=struct.unpack_from('<%dH'%ni,d,io)
        cum+=nvb[i] if i<len(nvb) else 0
        if max(idxs)>=cum: E(f"siblob{i} index {max(idxs)} >= cumverts {cum}")
    md=bpd+u32(bpd+4)
    if i32(md)!=vbh-vc or i32(md+4)!=sih-vc or i32(md+8)!=fuh-vc: E("ModelData offsets")
    nl=u32(md+16); lr=vc+i32(md+20)
    lvl=lodofs; visited=0; chain=[]
    while lvl and visited<64:
        nxt=i32(lvl+16); mg=lvl+i32(lvl+8)
        ts=mg+i32(mg); strip=ts+i32(ts+4); ni=u32(strip)
        if strip+i32(strip+4)+ni*2>n: E(f"LOD{visited} strip OOB"); break
        chain.append(lvl); visited+=1
        lvl=lvl+nxt if nxt else None
    if visited!=nl: E(f"LOD chain {visited} != records {nl}")
    for i in range(min(nl,len(chain))):
        if vc+i32(lr+i*12)!=chain[i]: E(f"LODFlexRecord[{i}] bad lodlevel")
    ml=vc+i32(md+12)
    listlen=u32(ml)
    off=(ml+4+listlen+3)&~3
    mnamelen=u32(off); off2=(off+4+mnamelen+3)&~3
    if off2+24>n: E("skin family header OOB")
    matrel=u32(36)
    if 40+matrel!=ml: E("MatRepData -> wrong target")
    if u32(40)!=listlen: E("header strlen mismatch")
    if not(u32(44)==1 and u32(48)==1 and u32(52)==1): E("skins/materials/bodyparts")
    name=d[ml+4:ml+4+listlen].split(b'\0')[0].decode(errors='replace')
    mname=d[off+4:off+4+mnamelen].split(b'\0')[0].decode(errors='replace')
    print(f"matlist '{name}' entry '{mname}' LODs={nl} blobs={nb}")
    print("RESULT:", "FAIL" if errs else "ALL CHECKS PASSED")
    return errs

if __name__=="__main__":
    sys.exit(1 if main(sys.argv[1]) else 0)
