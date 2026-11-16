// Direct TileCache calls against pinned, unmodified RecastNavigation. Test only.
#include "DetourTileCacheBuilder.h"
#include "DetourAlloc.h"
#include "DetourTileCache.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include <sstream>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include "../vendor/recastnavigation/DetourTileCache/Source/DetourTileCacheBuilder.cpp"

static float number() {
    unsigned bits;std::cin>>bits;float value;std::memcpy(&value,&bits,4);return value;
}
static void point(float* p) {for(int i=0;i<3;++i)p[i]=number();}
static void bytes(const std::vector<unsigned char>& data) {
    std::cout<<'[';for(size_t i=0;i<data.size();++i){if(i)std::cout<<',';std::cout<<(unsigned)data[i];}std::cout<<']';
}
static void mark() {
    int w,h;std::cin>>w>>h;float orig[3];point(orig);float cs=number(),ch=number();
    std::vector<unsigned char> heights(w*h),areas(w*h),cons(w*h);
    for(auto* v:{&heights,&areas,&cons})for(auto& a:*v){unsigned x;std::cin>>x;a=(unsigned char)x;}
    dtTileCacheLayerHeader header={};header.width=(unsigned char)w;header.height=(unsigned char)h;
    dtTileCacheLayer layer={};layer.header=&header;layer.heights=heights.data();layer.areas=areas.data();layer.cons=cons.data();
    auto snapshot=[&](dtStatus status){std::cout<<'['<<status<<',';bytes(heights);std::cout<<',';bytes(areas);std::cout<<',';bytes(cons);std::cout<<']';};
    std::cout<<'[';snapshot(DT_SUCCESS);int count;std::cin>>count;
    for(int i=0;i<count;++i){
        std::string op;std::cin>>op;float a[3],b[3];point(a);dtStatus s;
        if(op=="C") {
            float radius=number(),height=number();unsigned area;std::cin>>area;
            s=dtMarkCylinderArea(layer,orig,cs,ch,a,radius,height,(unsigned char)area);
        }else if(op=="O") {
            point(b);float rotation[2]={number(),number()};unsigned area;std::cin>>area;
            s=dtMarkBoxArea(layer,orig,cs,ch,a,b,rotation,(unsigned char)area);
        }else {
            point(b);unsigned area;std::cin>>area;
            s=dtMarkBoxArea(layer,orig,cs,ch,a,b,(unsigned char)area);
        }
        std::cout<<',';snapshot(s);
    }
    std::cout<<']';
}
struct FixtureCompressor : dtTileCacheCompressor {
    int mode,extra,limit,report;unsigned compressStatus,decompressStatus;
    std::vector<long long> log;
    FixtureCompressor(){std::cin>>mode>>extra>>limit>>report>>compressStatus>>decompressStatus;}
    void record(int kind,const unsigned char* input,int size,const unsigned char* output,int capacity){
        log.insert(log.end(),{kind,size,capacity});
        log.insert(log.end(),input,input+size);log.insert(log.end(),output,output+capacity);
    }
    int maxCompressedSize(int size) override {log.insert(log.end(),{0,size,size+extra});return size+extra;}
    dtStatus compress(const unsigned char* input,int size,unsigned char* output,int capacity,int* outSize) override {
        record(1,input,size,output,capacity);int n=std::min(size,capacity);
        for(int i=0;i<n;++i)output[i]=mode?input[size-1-i]^0xa5:input[i];
        *outSize=report<0?n:report;return compressStatus;
    }
    dtStatus decompress(const unsigned char* input,int size,unsigned char* output,int capacity,int* outSize) override {
        record(2,input,size,output,capacity);int n=std::min(limit<0?capacity:limit,mode==2?capacity:std::min(size,capacity));
        for(int i=0;i<n;++i)output[i]=mode==2?(unsigned char)(i*17+3):(mode?input[size-1-i]^0xa5:input[i]);
        *outSize=report<0?n:report;return decompressStatus;
    }
    void outputLog(){std::cout<<'[';for(size_t i=0;i<log.size();++i){if(i)std::cout<<',';std::cout<<log[i];}std::cout<<']';}
};
static void rawBytes(const unsigned char* data,int size){std::cout<<'[';for(int i=0;i<size;++i){if(i)std::cout<<',';std::cout<<(unsigned)data[i];}std::cout<<']';}
static std::vector<unsigned char> inputBytes(){int n;std::cin>>n;std::vector<unsigned char> bytes(n);for(auto& b:bytes){unsigned x;std::cin>>x;b=(unsigned char)x;}return bytes;}
static void build(){
    static_assert(sizeof(dtTileCacheLayerHeader)==56,"native header layout");
    FixtureCompressor compressor;dtTileCacheLayerHeader header;
    unsigned pad0,pad1;std::cin>>pad0>>pad1;std::memset(&header,(unsigned char)pad0,sizeof(header));((unsigned char*)&header)[sizeof(header)-1]=(unsigned char)pad1;
    unsigned magic,version;std::cin>>magic>>version>>header.tx>>header.ty>>header.tlayer;header.magic=(int)magic;header.version=(int)version;
    point(header.bmin);point(header.bmax);unsigned hmin,hmax,w,h,minx,maxx,miny,maxy;
    std::cin>>hmin>>hmax>>w>>h>>minx>>maxx>>miny>>maxy;
    header.hmin=(unsigned short)hmin;header.hmax=(unsigned short)hmax;header.width=(unsigned char)w;header.height=(unsigned char)h;
    header.minx=(unsigned char)minx;header.maxx=(unsigned char)maxx;header.miny=(unsigned char)miny;header.maxy=(unsigned char)maxy;
    auto heights=inputBytes(),areas=inputBytes(),cons=inputBytes();unsigned char* data=0;int size=0;
    dtStatus s=dtBuildTileCacheLayer(&compressor,&header,heights.data(),areas.data(),cons.data(),&data,&size);
    std::cout<<'['<<s<<',';if(data)rawBytes(data,size);else std::cout<<"null";std::cout<<',';compressor.outputLog();std::cout<<']';dtFree(data);
}
static void decompress(){
    FixtureCompressor compressor;auto data=inputBytes();dtTileCacheAlloc alloc;dtTileCacheLayer* layer=0;
    dtStatus s=dtDecompressTileCacheLayer(&alloc,&compressor,data.data(),(int)data.size(),&layer);
    std::cout<<'['<<s<<',';
    if(layer){
        std::cout<<(unsigned)layer->regCount<<',';rawBytes((unsigned char*)layer->header,sizeof(dtTileCacheLayerHeader));std::cout<<",[";
        int n=(int)layer->header->width*layer->header->height;
        rawBytes(layer->heights,n);std::cout<<',';rawBytes(layer->areas,n);std::cout<<',';rawBytes(layer->cons,n);std::cout<<',';rawBytes(layer->regs,n);std::cout<<"],";
    }else std::cout<<"null,";
    compressor.outputLog();std::cout<<']';dtFreeTileCacheLayer(&alloc,layer);
}
static void endian(){auto data=inputBytes();bool ok=dtTileCacheHeaderSwapEndian(data.data(),(int)data.size());std::cout<<'['<<ok<<',';bytes(data);std::cout<<']';}
static void meshBuild(){
    int w,h;std::cin>>w>>h;dtTileCacheLayerHeader header={};header.width=(unsigned char)w;header.height=(unsigned char)h;
    point(header.bmin);point(header.bmax);float cs=number(),ch=number();(void)cs;(void)ch;int climb;std::cin>>climb;float error=number();
    int n=w*h;std::vector<unsigned char> heights(std::max(n,1)),areas(std::max(n,1)),cons(std::max(n,1)),regs(std::max(n,1));
    for(auto* grid:{&heights,&areas,&cons})for(int i=0;i<n;++i){unsigned x;std::cin>>x;(*grid)[i]=(unsigned char)x;}
    dtTileCacheLayer layer={};layer.header=&header;layer.heights=heights.data();layer.areas=areas.data();layer.cons=cons.data();layer.regs=regs.data();
    dtTileCacheAlloc alloc;dtTileCacheContourSet* contours=0;dtTileCachePolyMesh* mesh=0;
    dtStatus status=dtBuildTileCacheRegions(&alloc,layer,climb);
    if(!dtStatusFailed(status)){contours=dtAllocTileCacheContourSet(&alloc);status=dtBuildTileCacheContours(&alloc,layer,climb,error,*contours);}
    if(!dtStatusFailed(status)){mesh=dtAllocTileCachePolyMesh(&alloc);status=dtBuildTileCachePolyMesh(&alloc,*contours,*mesh);}
    std::cout<<'['<<status<<',';
    if(dtStatusFailed(status))std::cout<<"null";
    else{
        std::cout<<mesh->nvp<<','<<(unsigned)layer.regCount<<',';rawBytes(regs.data(),n);std::cout<<",[";
        for(int i=0;i<contours->nconts;++i){if(i)std::cout<<',';const auto& c=contours->conts[i];std::cout<<'['<<(unsigned)c.reg<<','<<(unsigned)c.area<<",[";for(int j=0;j<c.nverts;++j){if(j)std::cout<<',';rawBytes(c.verts+j*4,4);}std::cout<<"]]";}
        std::cout<<"],[";for(int i=0;i<mesh->nverts;++i){if(i)std::cout<<',';std::cout<<'['<<mesh->verts[i*3]<<','<<mesh->verts[i*3+1]<<','<<mesh->verts[i*3+2]<<']';}
        std::cout<<"],[";for(int i=0;i<mesh->npolys;++i){if(i)std::cout<<',';std::cout<<'[';for(int j=0;j<12;++j){if(j)std::cout<<',';std::cout<<mesh->polys[i*12+j];}std::cout<<']';}
        std::cout<<"],";rawBytes(mesh->areas,mesh->npolys);std::cout<<",[";for(int i=0;i<mesh->npolys;++i){if(i)std::cout<<',';std::cout<<mesh->flags[i];}std::cout<<']';
    }
    std::cout<<']';dtFreeTileCachePolyMesh(&alloc,mesh);dtFreeTileCacheContourSet(&alloc,contours);
}

static void stages(){
    std::string mode;int w,h,climb;unsigned initial;std::cin>>mode>>w>>h>>climb;float error=number();std::cin>>initial;
    auto heights=inputBytes(),areas=inputBytes(),cons=inputBytes(),regs=inputBytes();
    dtTileCacheLayerHeader header={};header.width=(unsigned char)w;header.height=(unsigned char)h;
    dtTileCacheLayer layer={};layer.header=&header;layer.heights=heights.data();layer.areas=areas.data();layer.cons=cons.data();layer.regs=regs.data();layer.regCount=(unsigned char)initial;
    dtTileCacheAlloc alloc;dtTileCacheContourSet* contours=0;dtStatus status;
    if(mode=="R")status=dtBuildTileCacheRegions(&alloc,layer,climb);
    else{contours=dtAllocTileCacheContourSet(&alloc);status=dtBuildTileCacheContours(&alloc,layer,climb,error,*contours);}
    std::cout<<'['<<status<<','<<(unsigned)layer.regCount<<',';bytes(regs);std::cout<<',';
    if(contours){std::cout<<'[';for(int i=0;i<contours->nconts;++i){if(i)std::cout<<',';auto& c=contours->conts[i];std::cout<<'['<<(unsigned)c.reg<<','<<(unsigned)c.area<<",[";for(int j=0;j<c.nverts;++j){if(j)std::cout<<',';rawBytes(c.verts+j*4,4);}std::cout<<"]]";}std::cout<<']';}else std::cout<<"null";
    std::cout<<']';dtFreeTileCacheContourSet(&alloc,contours);
}
static void polyBuild(bool partial=false){
    int count;std::cin>>count;std::vector<dtTileCacheContour> contours(count);std::vector<std::vector<unsigned char>> vertices(count);
    for(int i=0;i<count;++i){unsigned reg,area;std::cin>>contours[i].nverts>>reg>>area;contours[i].reg=(unsigned char)reg;contours[i].area=(unsigned char)area;
        vertices[i].resize(contours[i].nverts*4);for(auto& v:vertices[i]){unsigned n;std::cin>>n;v=(unsigned char)n;}contours[i].verts=vertices[i].data();}
    dtTileCacheContourSet set={};set.nconts=count;set.conts=contours.data();dtTileCacheAlloc alloc;auto* mesh=dtAllocTileCachePolyMesh(&alloc);
    dtStatus status=dtBuildTileCachePolyMesh(&alloc,set,*mesh);std::cout<<'['<<status<<',';
    if(dtStatusFailed(status)&&!partial)std::cout<<"null";
    else{
        std::cout<<'[';for(int i=0;i<mesh->nverts;++i){if(i)std::cout<<',';std::cout<<'['<<mesh->verts[i*3]<<','<<mesh->verts[i*3+1]<<','<<mesh->verts[i*3+2]<<']';}
        std::cout<<"],[";for(int i=0;i<mesh->npolys;++i){if(i)std::cout<<',';std::cout<<'[';for(int j=0;j<12;++j){if(j)std::cout<<',';std::cout<<mesh->polys[i*12+j];}std::cout<<']';}
        std::cout<<"],";rawBytes(mesh->areas,mesh->npolys);std::cout<<",[";for(int i=0;i<mesh->npolys;++i){if(i)std::cout<<',';std::cout<<mesh->flags[i];}std::cout<<']';
    }
    std::cout<<']';dtFreeTileCachePolyMesh(&alloc,mesh);
}

static void floatBits(std::ostream& out,const float* p,int n){out<<'[';for(int i=0;i<n;++i){unsigned b;std::memcpy(&b,p+i,4);if(i)out<<',';out<<b;}out<<']';}
struct ObjectProcess : dtTileCacheMeshProcess {
    int mode;std::vector<std::string> log;
    float connection[6]={.5f,0,.5f,3.5f,0,3.5f},radius=.75f;
    unsigned short flags=7;unsigned char area=5,dir=1;unsigned user=0xfedcba98;
    void process(dtNavMeshCreateParams* p,unsigned char* areas,unsigned short* polyFlags) override {
        std::ostringstream out;out<<'['<<mode<<','<<p->vertCount<<','<<p->polyCount<<','<<p->nvp<<",["<<p->tileX<<','<<p->tileY<<','<<p->tileLayer<<"],";
        floatBits(out,p->bmin,3);out<<',';floatBits(out,p->bmax,3);out<<",[";
        for(int i=0;i<p->polyCount;++i){if(i)out<<',';out<<'['<<(unsigned)areas[i]<<','<<polyFlags[i]<<']';}out<<"]]";log.push_back(out.str());
        if(mode==0)return;
        for(int i=0;i<p->polyCount;++i){areas[i]=(unsigned char)((areas[i]+mode)%64);polyFlags[i]=(unsigned short)((i+1)*3);}
        if(mode==2)p->polyCount=0;
        if(mode==3)p->tileX++;
        if(mode==4){p->userId=0x87654321;p->buildBvTree=true;}
        if(mode==5){p->offMeshConVerts=connection;p->offMeshConRad=&radius;p->offMeshConFlags=&flags;p->offMeshConAreas=&area;p->offMeshConDir=&dir;p->offMeshConUserID=&user;p->offMeshConCount=1;}
    }
};
static dtTileCacheParams objectParams(){
    dtTileCacheParams p={};point(p.orig);p.cs=number();p.ch=number();std::cin>>p.width>>p.height;
    p.walkableHeight=number();p.walkableRadius=number();p.walkableClimb=number();p.maxSimplificationError=number();std::cin>>p.maxTiles>>p.maxObstacles;return p;
}
static void object(){
    auto p=objectParams();int navTiles,navPolys,processMode;std::cin>>navTiles>>navPolys>>processMode;
    FixtureCompressor comp;ObjectProcess process;process.mode=processMode;
    dtTileCacheAlloc alloc;dtTileCache cache;dtStatus init=cache.init(&p,&alloc,&comp,processMode<0?0:&process);
    dtNavMeshParams np={};std::memcpy(np.orig,p.orig,12);np.tileWidth=p.width*p.cs;np.tileHeight=p.height*p.cs;np.maxTiles=navTiles;np.maxPolys=navPolys;
    dtNavMesh nav;dtStatus navInit=nav.init(&np);if(dtStatusFailed(init)||dtStatusFailed(navInit)){std::cout<<"null";return;}
    std::vector<unsigned char*> borrowed;
    auto snap=[&](){
        std::cout<<"[[";
        for(int i=0;i<cache.getTileCount();++i){auto* t=cache.getTile(i);if(i)std::cout<<',';std::cout<<'['<<t->salt<<',';
            if(t->header)rawBytes((unsigned char*)t->header,sizeof(dtTileCacheLayerHeader));else std::cout<<"null";
            std::cout<<',';rawBytes(t->data,t->dataSize);std::cout<<','<<t->flags<<',';
            if(t->next)std::cout<<(t->next-cache.getTile(0));else std::cout<<"null";std::cout<<']';}
        std::cout<<"],[";
        for(int i=0;i<cache.getObstacleCount();++i){auto* o=cache.getObstacle(i);if(i)std::cout<<',';std::cout<<'[';float shape[8];std::memcpy(shape,&o->orientedBox,sizeof(shape));floatBits(std::cout,shape,8);
            for(auto* a:{o->touched,o->pending}){std::cout<<",[";for(int j=0;j<8;++j){if(j)std::cout<<',';std::cout<<a[j];}std::cout<<']';}
            std::cout<<','<<o->salt<<','<<(unsigned)o->type<<','<<(unsigned)o->state<<','<<(unsigned)o->ntouched<<','<<(unsigned)o->npending<<',';
            if(o->next)std::cout<<(o->next-cache.getObstacle(0));else std::cout<<"null";
            float lo[3],hi[3];cache.getObstacleBounds(o,lo,hi);std::cout<<',';floatBits(std::cout,lo,3);std::cout<<',';floatBits(std::cout,hi,3);std::cout<<']';}
        std::cout<<"],[";
        for(int i=0;i<nav.getMaxTiles();++i){auto* t=((const dtNavMesh&)nav).getTile(i);if(i)std::cout<<',';std::cout<<'['<<nav.getTileRef(t)<<',';
            if(t->header)rawBytes(t->data,t->dataSize);else std::cout<<"null";std::cout<<','<<t->flags<<','<<t->dataSize<<','<<t->linksFreeList<<']';}
        std::cout<<"],";comp.outputLog();comp.log.clear();std::cout<<",[";for(size_t i=0;i<process.log.size();++i){if(i)std::cout<<',';std::cout<<process.log[i];}process.log.clear();std::cout<<"]]";
    };
    std::cout<<"[["<<init<<",";snap();std::cout<<']';int count;std::cin>>count;
    for(int step=0;step<count;++step){std::string op;std::cin>>op;std::cout<<",[";
        if(op=="A"){unsigned flags;std::cin>>flags;auto data=inputBytes();auto* raw=(unsigned char*)dtAlloc(data.size(),DT_ALLOC_PERM);std::memcpy(raw,data.data(),data.size());dtCompressedTileRef r=0xabcdef;
            dtStatus s=cache.addTile(raw,(int)data.size(),(unsigned char)flags,&r);if(dtStatusFailed(s))dtFree(raw);else if(!(flags&1))borrowed.push_back(raw);std::cout<<'['<<s<<','<<r<<']';}
        else if(op=="R"){unsigned r;std::cin>>r;unsigned char* data=0;int size=-1;auto s=cache.removeTile(r,&data,&size);std::cout<<'['<<s<<',';if(data)rawBytes(data,size);else std::cout<<"null";std::cout<<']';}
        else if(op=="C"){float pos[3];point(pos);float radius=number(),height=number();dtObstacleRef r=0xabcdef;auto s=cache.addObstacle(pos,radius,height,&r);std::cout<<'['<<s<<','<<r<<']';}
        else if(op=="B"){float lo[3],hi[3];point(lo);point(hi);dtObstacleRef r=0xabcdef;auto s=cache.addBoxObstacle(lo,hi,&r);std::cout<<'['<<s<<','<<r<<']';}
        else if(op=="O"){float c[3],h[3];point(c);point(h);float angle=number();dtObstacleRef r=0xabcdef;auto s=cache.addBoxObstacle(c,h,angle,&r);std::cout<<'['<<s<<','<<r<<']';}
        else if(op=="X"){unsigned r;std::cin>>r;std::cout<<cache.removeObstacle(r);}
        else if(op=="U"){bool done=false;float dt=number();auto s=cache.update(dt,&nav,&done);std::cout<<'['<<s<<','<<done<<']';}
        else if(op=="T"){unsigned r;std::cin>>r;std::cout<<cache.buildNavMeshTile(r,&nav);}
        else if(op=="V"){int x,y;std::cin>>x>>y;std::cout<<cache.buildNavMeshTilesAt(x,y,&nav);}
        else if(op=="Q"){float lo[3],hi[3];point(lo);point(hi);int cap;std::cin>>cap;std::vector<dtCompressedTileRef> refs(std::max(cap,1));int n=-1;auto s=cache.queryTiles(lo,hi,refs.data(),&n,cap);std::cout<<'['<<s<<",[";for(int i=0;i<n;++i){if(i)std::cout<<',';std::cout<<refs[i];}std::cout<<"]]";}
        else if(op=="G"){int x,y,cap;std::cin>>x>>y>>cap;std::vector<dtCompressedTileRef> refs(std::max(cap,1));int n=cache.getTilesAt(x,y,refs.data(),cap);std::cout<<'[';for(int i=0;i<n;++i){if(i)std::cout<<',';std::cout<<refs[i];}std::cout<<']';}
        else if(op=="H"){unsigned tr,ob;std::cin>>tr>>ob;auto* t=cache.getTileByRef(tr);auto* o=cache.getObstacleByRef(ob);std::cout<<'[';if(t)std::cout<<(t-cache.getTile(0));else std::cout<<"null";std::cout<<',';if(o)std::cout<<(o-cache.getObstacle(0));else std::cout<<"null";std::cout<<']';}
        else if(op=="L"){int i;std::cin>>i;float lo[3],hi[3];cache.calcTightTileBounds(cache.getTile(i)->header,lo,hi);std::cout<<'[';floatBits(std::cout,lo,3);std::cout<<',';floatBits(std::cout,hi,3);std::cout<<']';}
        else if(op=="E"){unsigned salt,tile;std::cin>>salt>>tile;unsigned r=cache.encodeTileId(salt,tile),o=cache.encodeObstacleId(salt,tile);std::cout<<'['<<r<<','<<cache.decodeTileIdSalt(r)<<','<<cache.decodeTileIdTile(r)<<','<<o<<','<<cache.decodeObstacleIdSalt(o)<<','<<cache.decodeObstacleIdObstacle(o)<<']';}
        else if(op=="D"){std::cin>>comp.decompressStatus;std::cout<<DT_SUCCESS;}
        else if(op=="P"){std::cin>>process.mode;std::cout<<DT_SUCCESS;}
        else if(op=="I"){p=objectParams();std::cout<<cache.init(&p,&alloc,&comp,processMode<0?0:&process);}
        else return;
        std::cout<<',';snap();std::cout<<']';
    }
    std::cout<<']';
    // Borrowed data remains owned by this fixture. Native cache does not free it.
    for(auto* b:borrowed)dtFree(b);
}
int main() {
    std::string mode;if(!(std::cin>>mode))return 2;
    if(mode=="distance") {
        int n;std::cin>>n;std::cout<<'[';for(int i=0;i<n;++i){int p[6];for(int& v:p)std::cin>>v;float d=distancePtSeg(p[0],p[1],p[2],p[3],p[4],p[5]);unsigned bits;std::memcpy(&bits,&d,4);if(i)std::cout<<',';std::cout<<bits;}std::cout<<']';
    }else if(mode=="stages")stages();else if(mode=="object")object();else if(mode=="poly_into")polyBuild(true);else if(mode=="poly")polyBuild();else if(mode=="mesh")meshBuild();else if(mode=="mark")mark();else if(mode=="build")build();else if(mode=="decompress")decompress();else if(mode=="endian")endian();else return 3;
}
