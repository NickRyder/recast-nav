// Matched optimized benchmark. Pinned library translation units are unmodified.
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourCrowd.h"
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static void require(bool ok) { if (!ok) throw std::runtime_error("benchmark operation failed"); }
struct Hash {
    uint64_t value = UINT64_C(0xcbf29ce484222325);
    void word(uint64_t n) { for (int i=0;i<8;++i) { value=(value^(n&255))*UINT64_C(0x100000001b3); n>>=8; } }
    void bytes(const unsigned char* data,size_t count){for(size_t i=0;i<count;++i)value=(value^data[i])*UINT64_C(0x100000001b3);}
    void point(const float* p) { for (int i=0;i<3;++i) { uint32_t n;std::memcpy(&n,p+i,4);word(n); } }
};
static std::vector<unsigned char> read(const std::string& path) {
    std::ifstream in(path,std::ios::binary);require(bool(in));
    return std::vector<unsigned char>((std::istreambuf_iterator<char>(in)),std::istreambuf_iterator<char>());
}
struct Geometry { std::vector<float> vertices;std::vector<int> triangles; };
static Geometry geometry(const std::string& dir,const std::string& file="geometry.bin") {
    auto data=read(dir+"/"+file);require(data.size()>=8);uint32_t counts[2];std::memcpy(counts,data.data(),8);
    Geometry g;g.vertices.resize(counts[0]*3);g.triangles.resize(counts[1]*3);require(data.size()==8+(g.vertices.size()+g.triangles.size())*4);
    std::memcpy(g.vertices.data(),data.data()+8,g.vertices.size()*4);std::memcpy(g.triangles.data(),data.data()+8+g.vertices.size()*4,g.triangles.size()*4);return g;
}
static uint64_t recast(const Geometry& g,int partition=0) {
    rcContext ctx(false);rcConfig c={};c.bmax[0]=c.bmax[2]=24;c.bmin[1]=-2;c.bmax[1]=8;c.cs=.25f;c.ch=.125f;
    c.walkableSlopeAngle=45;c.walkableHeight=12;c.walkableClimb=4;c.walkableRadius=2;c.minRegionArea=8;c.mergeRegionArea=20;
    c.maxEdgeLen=48;c.maxSimplificationError=1.3f;c.maxVertsPerPoly=6;c.detailSampleDist=1.5f;c.detailSampleMaxError=.125f;
    rcCalcGridSize(c.bmin,c.bmax,c.cs,&c.width,&c.height);int nv=int(g.vertices.size()/3),nt=int(g.triangles.size()/3);
    std::vector<unsigned char> areas(nt,0);rcMarkWalkableTriangles(&ctx,c.walkableSlopeAngle,g.vertices.data(),nv,g.triangles.data(),nt,areas.data());
    rcHeightfield solid;require(rcCreateHeightfield(&ctx,solid,c.width,c.height,c.bmin,c.bmax,c.cs,c.ch));
    require(rcRasterizeTriangles(&ctx,g.vertices.data(),nv,g.triangles.data(),areas.data(),nt,solid,c.walkableClimb));
    rcFilterLowHangingWalkableObstacles(&ctx,c.walkableClimb,solid);rcFilterLedgeSpans(&ctx,c.walkableHeight,c.walkableClimb,solid);rcFilterWalkableLowHeightSpans(&ctx,c.walkableHeight,solid);
    rcCompactHeightfield compact;require(rcBuildCompactHeightfield(&ctx,c.walkableHeight,c.walkableClimb,solid,compact));require(rcErodeWalkableArea(&ctx,c.walkableRadius,compact));
    rcHeightfieldLayerSet layers;require(rcBuildHeightfieldLayers(&ctx,compact,0,c.walkableHeight,layers));
    // Match the Rust observed pipeline: distance is an exposed stage for every strategy.
    require(rcBuildDistanceField(&ctx,compact));
    if(partition==0)require(rcBuildRegions(&ctx,compact,0,c.minRegionArea,c.mergeRegionArea));
    else if(partition==1)require(rcBuildRegionsMonotone(&ctx,compact,0,c.minRegionArea,c.mergeRegionArea));
    else require(rcBuildLayerRegions(&ctx,compact,0,c.minRegionArea));
    rcContourSet contours;require(rcBuildContours(&ctx,compact,c.maxSimplificationError,c.maxEdgeLen,contours,RC_CONTOUR_TESS_WALL_EDGES));
    rcPolyMesh p;require(rcBuildPolyMesh(&ctx,contours,c.maxVertsPerPoly,p));rcPolyMeshDetail d;require(rcBuildPolyMeshDetail(&ctx,p,compact,c.detailSampleDist,c.detailSampleMaxError,d));
    Hash h;h.word(p.nverts);h.word(p.npolys);h.word(p.nvp);
    for(int i=0;i<p.nverts*3;++i)h.word(p.verts[i]);
    for(int half=0;half<2;++half)for(int i=0;i<p.npolys;++i)for(int j=0;j<p.nvp;++j)h.word(p.polys[i*p.nvp*2+half*p.nvp+j]);
    for(int i=0;i<p.npolys;++i){h.word(p.regs[i]);h.word(p.areas[i]);h.word(p.flags[i]);}
    h.word(d.nmeshes);h.word(d.nverts);h.word(d.ntris);
    for(int i=0;i<d.nmeshes*4;++i)h.word(d.meshes[i]);for(int i=0;i<d.nverts;++i)h.point(d.verts+i*3);for(int i=0;i<d.ntris*4;++i)h.word(d.tris[i]);
    h.word(layers.nlayers);for(int i=0;i<layers.nlayers;++i){const auto& l=layers.layers[i];h.word(l.width);h.word(l.height);int n=l.width*l.height;for(int j=0;j<n;++j)h.word(l.heights[j]);for(int j=0;j<n;++j)h.word(l.areas[j]);for(int j=0;j<n;++j)h.word(l.cons[j]);}
    return h.value;
}
static void buildRecast(const Geometry& g,rcCompactHeightfield& compact,rcHeightfieldLayerSet& layers,rcPolyMesh& p,rcPolyMeshDetail& d,int partition,int nvp,bool stacked,std::vector<unsigned char>* initialAreas=nullptr,bool layerCapacity=false) {
    rcContext ctx(false);rcConfig c={};c.bmax[0]=c.bmax[2]=stacked?64:24;c.bmin[1]=-2;c.bmax[1]=stacked?10:8;c.cs=.25f;c.ch=.125f;
    c.walkableSlopeAngle=45;c.walkableHeight=12;c.walkableClimb=4;c.walkableRadius=2;c.minRegionArea=8;c.mergeRegionArea=20;
    c.maxEdgeLen=48;c.maxSimplificationError=1.3f;c.maxVertsPerPoly=nvp;c.detailSampleDist=1.5f;c.detailSampleMaxError=.125f;
    rcCalcGridSize(c.bmin,c.bmax,c.cs,&c.width,&c.height);int nv=int(g.vertices.size()/3),nt=int(g.triangles.size()/3);
    std::vector<unsigned char> areas(nt,0);rcMarkWalkableTriangles(&ctx,c.walkableSlopeAngle,g.vertices.data(),nv,g.triangles.data(),nt,areas.data());
    rcHeightfield solid;require(rcCreateHeightfield(&ctx,solid,c.width,c.height,c.bmin,c.bmax,c.cs,c.ch));
    require(rcRasterizeTriangles(&ctx,g.vertices.data(),nv,g.triangles.data(),areas.data(),nt,solid,c.walkableClimb));
    rcFilterLowHangingWalkableObstacles(&ctx,c.walkableClimb,solid);rcFilterLedgeSpans(&ctx,c.walkableHeight,c.walkableClimb,solid);rcFilterWalkableLowHeightSpans(&ctx,c.walkableHeight,solid);
    require(rcBuildCompactHeightfield(&ctx,c.walkableHeight,c.walkableClimb,solid,compact));require(rcErodeWalkableArea(&ctx,c.walkableRadius,compact));
    if(initialAreas)initialAreas->assign(compact.areas,compact.areas+compact.spanCount);
    bool layersOk=rcBuildHeightfieldLayers(&ctx,compact,0,c.walkableHeight,layers);
    if(layerCapacity){require(!layersOk);return;}require(layersOk);
    // Match the Rust observed pipeline: distance is an exposed stage for every strategy.
    require(rcBuildDistanceField(&ctx,compact));
    if(partition==0)require(rcBuildRegions(&ctx,compact,0,c.minRegionArea,c.mergeRegionArea));
    else if(partition==1)require(rcBuildRegionsMonotone(&ctx,compact,0,c.minRegionArea,c.mergeRegionArea));
    else require(rcBuildLayerRegions(&ctx,compact,0,c.minRegionArea));
    rcContourSet contours;require(rcBuildContours(&ctx,compact,c.maxSimplificationError,c.maxEdgeLen,contours,RC_CONTOUR_TESS_WALL_EDGES));
    require(rcBuildPolyMesh(&ctx,contours,c.maxVertsPerPoly,p));require(rcBuildPolyMeshDetail(&ctx,p,compact,c.detailSampleDist,c.detailSampleMaxError,d));
}
static uint64_t recastWide(const Geometry& g,int partition,int nvp,bool stacked) {
    rcCompactHeightfield compact;rcHeightfieldLayerSet layers;rcPolyMesh p;rcPolyMeshDetail d;
    buildRecast(g,compact,layers,p,d,partition,nvp,stacked);
    Hash h;h.word(p.nverts);h.word(p.npolys);h.word(p.nvp);
    for(int i=0;i<p.nverts*3;++i)h.word(p.verts[i]);
    for(int half=0;half<2;++half)for(int i=0;i<p.npolys;++i)for(int j=0;j<p.nvp;++j)h.word(p.polys[i*p.nvp*2+half*p.nvp+j]);
    for(int i=0;i<p.npolys;++i){h.word(p.regs[i]);h.word(p.areas[i]);h.word(p.flags[i]);}
    h.word(d.nmeshes);h.word(d.nverts);h.word(d.ntris);
    for(int i=0;i<d.nmeshes*4;++i)h.word(d.meshes[i]);for(int i=0;i<d.nverts;++i)h.point(d.verts+i*3);for(int i=0;i<d.ntris*4;++i)h.word(d.tris[i]);
    h.word(layers.nlayers);for(int i=0;i<layers.nlayers;++i){const auto& l=layers.layers[i];h.word(l.width);h.word(l.height);int n=l.width*l.height;for(int j=0;j<n;++j)h.word(l.heights[j]);for(int j=0;j<n;++j)h.word(l.areas[j]);for(int j=0;j<n;++j)h.word(l.cons[j]);}
    return h.value;
}
static void load(dtNavMesh& nav,const std::string& dir, const std::string& file="nav.bin") {
    auto bytes=read(dir+"/"+file);auto* data=static_cast<unsigned char*>(dtAlloc(bytes.size(),DT_ALLOC_PERM));require(data!=nullptr);std::memcpy(data,bytes.data(),bytes.size());require(dtStatusSucceed(nav.init(data,int(bytes.size()),DT_TILE_FREE_DATA)));
}
static void point(uint32_t i,float* p){p[0]=float(i%32)+.5f;p[1]=0;p[2]=float(i/32)+.5f;}
struct DetourBuffers {
    dtPolyRef path[256],refs[256];float points[256*3];unsigned char flags[256];
};
static uint64_t detour(const dtNavMesh& nav,dtNavMeshQuery& q,DetourBuffers& output) {
    auto& path=output.path;auto& points=output.points;auto& flags=output.flags;auto& refs=output.refs;
    Hash h;uint32_t seed=0x187531b1;dtQueryFilter filter;dtPolyRef base=nav.getPolyRefBase(nav.getTile(0));
    for(int i=0;i<128;++i){seed=seed*1664525+1013904223;uint32_t a=(seed>>16)%1024;seed=seed*1664525+1013904223;uint32_t b=(seed>>16)%1024;float start[3],end[3];point(a,start);point(b,end);
        int n=0;dtStatus status=q.findPath(base|a,base|b,start,end,&filter,path,&n,256);h.word(status);h.word(n);for(int j=0;j<n;++j)h.word(path[j]);
        int ns=0;status=q.findStraightPath(start,end,path,n,points,flags,refs,&ns,256,0);h.word(status);h.word(ns);for(int j=0;j<ns;++j){h.point(points+j*3);h.word(flags[j]);h.word(refs[j]);}
    }return h.value;
}
static uint64_t crowd(dtNavMesh& nav) {
    dtCrowd c;require(c.init(64,.25f,&nav));dtCrowdAgentParams p={};p.radius=.25f;p.height=2;p.maxAcceleration=8;p.maxSpeed=3.5f;p.collisionQueryRange=3;p.pathOptimizationRange=7.5f;p.separationWeight=2;p.updateFlags=31;
    dtPolyRef base=nav.getPolyRefBase(static_cast<const dtNavMesh&>(nav).getTile(0));
    for(int i=0;i<64;++i){float pos[]={.5f+float(i%8)*4,0,.5f+float(i/8)*4};int index=c.addAgent(pos,&p);require(index==i);float end[]={32-pos[0],0,32-pos[2]};int poly=int(end[2])*32+int(end[0]);require(c.requestMoveTarget(index,base|poly,end));}
    Hash h;for(int i=0;i<120;++i){c.update(.05f,nullptr);h.word(c.getVelocitySampleCount());}
    for(int i=0;i<64;++i){const auto* a=c.getAgent(i);h.word(a->state);h.word(a->targetState);h.point(a->npos);h.point(a->disp);h.point(a->dvel);h.point(a->nvel);h.point(a->vel);uint32_t bits;std::memcpy(&bits,&a->desiredSpeed,4);h.word(bits);h.word(a->corridor.getPathCount());for(int j=0;j<a->corridor.getPathCount();++j)h.word(a->corridor.getPath()[j]);}return h.value;
}
struct Identity: dtTileCacheCompressor {
    int maxCompressedSize(int n) override{return n;}
    dtStatus compress(const unsigned char* input,int n,unsigned char* out,int cap,int* size) override {if(n>cap)return DT_FAILURE|DT_BUFFER_TOO_SMALL;std::memcpy(out,input,n);*size=n;return DT_SUCCESS;}
    dtStatus decompress(const unsigned char* input,int n,unsigned char* out,int cap,int* size) override{return compress(input,n,out,cap,size);}
};
struct Flags: dtTileCacheMeshProcess {void process(dtNavMeshCreateParams* p,unsigned char*,unsigned short* flags) override {for(int i=0;i<p->polyCount;++i)flags[i]=1;}};
static uint64_t tilecache(const std::vector<unsigned char>& bytes,bool allShapes=false) {
    dtTileCacheParams p={};p.cs=.5f;p.ch=.125f;p.width=p.height=32;p.walkableHeight=2;p.walkableRadius=.25f;p.walkableClimb=.5f;p.maxSimplificationError=1.3f;p.maxTiles=4;p.maxObstacles=32;
    dtTileCacheAlloc alloc;Identity compressor;Flags process;dtTileCache c;require(dtStatusSucceed(c.init(&p,&alloc,&compressor,&process)));
    dtNavMesh nav;dtNavMeshParams np={};np.tileWidth=np.tileHeight=16;np.maxTiles=4;np.maxPolys=2048;require(dtStatusSucceed(nav.init(&np)));
    auto* data=static_cast<unsigned char*>(dtAlloc(bytes.size(),DT_ALLOC_PERM));require(data!=nullptr);std::memcpy(data,bytes.data(),bytes.size());dtCompressedTileRef ref;require(dtStatusSucceed(c.addTile(data,int(bytes.size()),DT_COMPRESSEDTILE_FREE_DATA,&ref)));require(dtStatusSucceed(c.buildNavMeshTile(ref,&nav)));
    Hash h;for(int episode=0;episode<4;++episode){dtObstacleRef obstacles[8];for(int i=0;i<8;++i){float pos[]={2+float(i%4)*3,0,3+float(i/4)*6+float(episode)*.125f};if(!allShapes||i%3==0)require(dtStatusSucceed(c.addObstacle(pos,.75f,2,&obstacles[i])));
        else if(i%3==1){float lo[]={pos[0]-.75f,-.5f,pos[2]-.5f},hi[]={pos[0]+.75f,2,pos[2]+.5f};require(dtStatusSucceed(c.addBoxObstacle(lo,hi,&obstacles[i])));}
        else {float half[]={.75f,2,.5f};require(dtStatusSucceed(c.addBoxObstacle(pos,half,.375f,&obstacles[i])));}}
        for(int remove=0;remove<2;++remove){if(remove)for(auto r:obstacles)h.word(c.removeObstacle(r));bool done=false;do{h.word(c.update(.05f,&nav,&done));h.word(done);}while(!done);
            auto r=nav.getTileRefAt(0,0,0);h.word(r);const auto* t=nav.getTileByRef(r);require(t!=nullptr);h.word(t->dataSize);for(int j=0;j<t->dataSize;++j)h.word(t->data[j]);}
    }return h.value;
}
#include "library_bench_extended.h"
#include "library_bench_crowd.h"
int main(int argc,char** argv) {
    try {require(argc==4);std::string name=argv[1],dir=argv[2];int iterations=std::stoi(argv[3]);require(iterations>=0);
        Geometry g;dtNavMesh nav;dtNavMeshQuery query;std::vector<unsigned char> layer,lifecycleData[2];
        if(name.find("recast")==0)g=geometry(dir,name=="recast-layer-capacity"?"geometry-layer-capacity.bin":name.find("recast-stacked")==0?"geometry-stacked.bin":"geometry.bin");if(name=="detour"||name=="detour-spatial"||name=="detour-search"||name=="detour-sliced"||name=="crowd"||name=="crowd-small"||name=="crowd-dense"||name=="crowd-offmesh"||name=="crowd-corridor"||name=="crowd-path-queue")load(nav,dir,name=="crowd-offmesh"?"nav-crowd-link.bin":"nav.bin");if(name=="detour"||name=="detour-spatial"||name=="detour-search"||name=="detour-sliced")require(dtStatusSucceed(query.init(&nav,2048)));if(name.find("tilecache")==0)layer=read(dir+"/layer.bin");
        if(name=="detour-lifecycle"){lifecycleData[0]=read(dir+"/nav-left.bin");lifecycleData[1]=read(dir+"/nav-right.bin");}
        RecastUtilities recastUtilities;if(name=="recast-utilities")recastUtilities.prepare(g);DetourBuffers output;SpatialBuffers spatialOutput;SearchBuffers searchOutput;CacheStreamBuffers cacheStreamOutput(name=="tilecache-stream");LifecycleBuffers lifecycleOutput(lifecycleData[0].size()>lifecycleData[1].size()?lifecycleData[0].size():lifecycleData[1].size());CrowdComponentBuffers componentOutput;Hash hash;auto start=std::chrono::steady_clock::now();for(int i=0;i<iterations;++i){uint64_t value;if(name=="recast-layer-capacity")value=recastLayerCapacity(g);else if(name=="recast-utilities")value=recastUtilities.run(g);else if(name=="recast-stacked")value=recastWide(g,0,6,true);else if(name=="recast-stacked-monotone")value=recastWide(g,1,3,true);else if(name=="recast-stacked-layers")value=recastWide(g,2,12,true);else if(name=="recast")value=recast(g);else if(name=="recast-monotone")value=recast(g,1);else if(name=="recast-layers")value=recast(g,2);else if(name=="detour-spatial")value=spatial(nav,query,spatialOutput);else if(name=="detour-search")value=search(nav,query,searchOutput);else if(name=="detour-sliced")value=sliced(nav,query,searchOutput);else if(name=="detour-lifecycle")value=lifecycle(lifecycleData,lifecycleOutput);else if(name=="proximity-grid")value=proximity();else if(name=="avoidance-grid")value=avoidance(false);else if(name=="avoidance-adaptive")value=avoidance(true);else if(name=="tilecache-codec")value=cacheCodec();else if(name=="tilecache-builder")value=cacheBuilder();else if(name=="tilecache-stream")value=cacheStream(cacheStreamOutput);else if(name=="tilecache-shapes")value=tilecache(layer,true);else if(name=="detour")value=detour(nav,query,output);else if(name=="crowd")value=crowd(nav);else if(name=="crowd-corridor")value=crowdCorridor(nav,componentOutput);else if(name=="crowd-path-queue")value=crowdQueue(nav,componentOutput);else if(name=="crowd-small"||name=="crowd-dense"||name=="crowd-offmesh")value=crowdCases(nav,name=="crowd-dense",name=="crowd-offmesh");else if(name=="tilecache")value=tilecache(layer);else throw std::runtime_error("unknown workload");hash.word(value);}
        auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start).count();
        std::cout<<"{\"case\":\""<<name<<"\",\"iterations\":"<<iterations<<",\"checksum\":\""<<std::hex;std::cout.width(16);std::cout.fill('0');std::cout<<hash.value<<std::dec<<"\",\"elapsed_ns\":"<<ns<<"}\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
