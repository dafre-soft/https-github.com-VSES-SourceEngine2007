import struct, sys
fn=sys.argv[1] if len(sys.argv)>1 else 'build/out/vtest.vtx'
d=open(fn,'rb').read()
ok=True
def chk(c,msg):
    global ok
    print(("  OK  " if c else "  BAD ")+msg)
    if not c: ok=False
print("file",fn,"size",len(d))
id,ver,cv,fl,cf=struct.unpack_from('<5i',d,0)
chk(id==0x44485056 or id==0x50564844,"ID VPHD = %08X"%id)
chk(ver==64,"version %d"%ver)
es,ne=struct.unpack_from('<2i',d,20); vco,lofofs=struct.unpack_from('<2i',d,28)
mro,mls,sk,mt,bpc=struct.unpack_from('<5i',d,36)
print("entrysize %d numentries %d vertcacheofs %d lodofs %d"%(es,ne,vco,lofofs))
chk(es==72 and ne==1,"header entry fields")
vc=vco; chk(0<vc<len(d),"vertcacheofs in range")
vbrel,sirel,furel,bprel=struct.unpack_from('<4i',d,vc+32)
maxv,maxt=struct.unpack_from('<2i',d,vc+16)
print("VC rel: vb=%d si=%d fu=%d bp=%d  maxverts=%d maxtris=%d"%(vbrel,sirel,furel,bprel,maxv,maxt))
vbh=vc+vbrel
nblob,barr,vt,vsz=struct.unpack_from('<4i',d,vbh)
chk(nblob<=8 and vsz==32 and vt==0,"VertBlockHeader blobs=%d verttype=%d vertsize=%d"%(nblob,vt,vsz))
ents=[struct.unpack_from('<2i',d,vbh+36+i*8) for i in range(nblob)]
tot=sum(e[0] for e in ents)
chk(tot==maxv,"sum blob verts %d == maxnum_verts %d"%(tot,maxv))
varr=[struct.unpack_from('<i',d,vbh+barr+i*4)[0] for i in range(nblob)]
for i,a in enumerate(varr):
    bs=vbh+a; dl=struct.unpack_from('<i',d,bs)[0]
    chk(dl==ents[i][0]*vsz,"vertblob[%d] abs=%d datalen %d == count*%d"%(i,bs,dl,vsz))
    vloc=struct.unpack_from('<i',d,vbh+24)[0]  # vertlocalofs
    chk(bs+vloc+dl<=len(d),"vertblob[%d] data fits file"%(i,))
sih=vc+sirel
nb2,barr2=struct.unpack_from('<2i',d,sih)
tisum=0
for i in range(nb2):
    a=struct.unpack_from('<i',d,sih+barr2+i*4)[0]
    bs=sih+a
    dl,st=struct.unpack_from('<2i',d,bs)
    tisum+=dl//2
    chk(st==8 and dl%6==0 and bs+st+dl<=len(d),"stripidx blob[%d] datalen=%d idxstart=%d"%(i,dl,st))
chk(tisum==maxt*3,"sum strip indices %d == maxtris*3 %d"%(tisum,maxt*3))
fuh=vc+furel
nb3,barr3=struct.unpack_from('<2i',d,fuh)
for i in range(nb3):
    a=struct.unpack_from('<i',d,fuh+barr3+i*4)[0]
    bs=fuh+a
    nf,fo=struct.unpack_from('<2i',d,bs)
    fx=struct.unpack_from('<4i',d,bs+fo)
    sd=struct.unpack_from('<2i',d,bs+fo+fx[3])
    chk(fx[0]==0 and sd==(0,0),"fixup blob[%d] type=%d sd=%s"%(i,fx[0],sd))
bpd=vc+bprel
mc,mfo=struct.unpack_from('<2i',d,bpd)
chk(mc==1,"bodypart modelcount %d"%mc)
md=bpd+mfo
mvb,msi,mfu,matdo,nlr,lroff,maxsc=struct.unpack_from('<7i',d,md)
chk((mvb,msi,mfu)==(vbrel,sirel,furel),"ModelData block rel offs match VC ones")
chk(nlr==nblob,"numlodrecords %d == numblobs %d"%(nlr,nblob))
lr=[struct.unpack_from('<3i',d,vc+lroff+i*12) for i in range(nlr)]  # LODFlexRecord_t = 12 bytes
prev=None
for i,r in enumerate(lr):
    lvl=vc+r[2]
    nmg,mgofs,maxs,nextl=struct.unpack_from('<4i',d,lvl+4); sp=struct.unpack_from('<f',d,lvl)[0]
    chk(lvl<len(d) and nmg==1,"LOD%d level abs=%d switch=%.0f nummg=%d"%(i,lvl,sp,nmg))
    mgabs=lvl+mgofs
    tsrel,pxrel,mati=struct.unpack_from('<3i',d,mgabs)
    chk(mati==0 and pxrel>=12,"LOD%d LodMatGroup tsrel=%d pxrel=%d mat=%d"%(i,tsrel,pxrel,mati))
    ts=mgabs+tsrel
    nstrip,sfo,nfan,ffo,niso,tfo=struct.unpack_from('<6i',d,ts)
    tip=struct.unpack_from('<2i',d,ts+sfo)
    chk(nstrip==1 and tip[1]==8,"LOD%d TriStrips n=%d TriStip numindices=%d startofs=%d"%(i,nstrip,tip[0],tip[1]))
    idxabs=ts+sfo+tip[1]
    chk(idxabs+tip[0]*2<=len(d),"LOD%d index array fits (%d u16)"%(i,tip[0]))
    idxs=struct.unpack_from('<%dH'%tip[0],d,idxabs)
    nv=ents[i][0] if i<nblob else 0
    off= sum(e[0] for e in ents[:i])
    bad=[k for k,v in enumerate(idxs) if v>=off+nv and not (off<=v<off+nv)] if False else [k for k,v in enumerate(idxs) if not (off<=v<off+nv)]
    chk(not bad,"LOD%d all %d indices in vertex range [%d,%d) (bad=%d)"%(i,tip[0],off,off+nv,len(bad)))
    px=mgabs+pxrel
    psz=struct.unpack_from('<2i',d,px)
    chk(psz==(0,0),"LOD%d MaterialProxySize %s"%(i,psz))
    if prev is not None:
        chk(prev==lvl,"LOD chain: next of %d -> %d"%(i-1,lvl))
    prev=lvl+nextl if nextl else None
chk(lofofs==vc+lr[0][2],"header.lodofs %d == first LODLevel abs %d"%(lofofs,vc+lr[0][2]))
# material data at header-relative mro from &materiallist field (offset 40)
p=40+mro
end=d.index(b'\x00',p)
print("matlist name:",d[p:end].decode(errors='replace'))
p=end+1
if p%4:p+=4-(p%4)
ln=struct.unpack_from('<i',d,p)[0]; p+=4
name=d[p:p+ln-1].decode(errors='replace')
p+=ln
if p%4:p+=4-(p%4)
fam=struct.unpack_from('<i',d,p)[0]
chk(sk==1 and mt==1 and ln>0,"matrep skins=%d materials=%d namelen=%d name=%s fam=%d"%(sk,mt,ln,name,fam))
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
