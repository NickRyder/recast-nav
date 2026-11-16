// Additional matched workloads. Pinned library sources remain unmodified.
#include "DetourObstacleAvoidance.h"
#include "DetourProximityGrid.h"

static void real(Hash& h,float value) { uint32_t bits;std::memcpy(&bits,&value,4);h.word(bits); }
struct SpatialBuffers {
    dtPolyRef rayPath[256],visited[256],refs[64],parents[64],wallRefs[18];
    float wallSegments[18*6];
    dtRaycastHit ray={};
    SpatialBuffers(){ray.path=rayPath;ray.maxPath=256;}
};
static uint32_t randomSeed;
static float draw(){randomSeed=randomSeed*1664525+1013904223;return float(randomSeed>>8)*(1.f/16777216.f);}
static uint64_t spatial(const dtNavMesh& nav,dtNavMeshQuery& q,SpatialBuffers& out) {
    dtQueryFilter filter;Hash h;randomSeed=0x187531b1;const dtPolyRef base=nav.getPolyRefBase(nav.getTile(0));
    for(uint32_t i=0;i<32;++i){uint32_t a=(i*37)%1024,b=(i*113+17)%1024;float start[3],end[3];point(a,start);point(b,end);dtPolyRef ref=base|a;
        h.word(q.raycast(ref,start,end,&filter,DT_RAYCAST_USE_COSTS,&out.ray,0));
        real(h,out.ray.t);h.point(out.ray.hitNormal);h.word(uint32_t(out.ray.hitEdgeIndex));real(h,out.ray.pathCost);h.word(out.ray.pathCount);for(int j=0;j<out.ray.pathCount;++j)h.word(out.ray.path[j]);
        int n=0;float moved[3]={};h.word(q.moveAlongSurface(ref,start,end,&filter,moved,out.visited,&n,256));h.point(moved);h.word(n);for(int j=0;j<n;++j)h.word(out.visited[j]);
        h.word(q.findLocalNeighbourhood(ref,start,4,&filter,out.refs,out.parents,&n,64));h.word(n);for(int j=0;j<n;++j){h.word(out.refs[j]);h.word(out.parents[j]);}
        h.word(q.getPolyWallSegments(ref,&filter,out.wallSegments,out.wallRefs,&n,18));h.word(n);for(int j=0;j<n;++j){h.point(out.wallSegments+j*6);h.point(out.wallSegments+j*6+3);h.word(out.wallRefs[j]);}
        float distance=0,position[3]={},normal[3]={};h.word(q.findDistanceToWall(ref,start,6,&filter,&distance,position,normal));real(h,distance);h.point(position);h.point(normal);
        const float extents[]={2,4,2};dtPolyRef nearest=0;bool over=false;float nearestPoint[3]={};h.word(q.findNearestPoly(start,extents,&filter,&nearest,nearestPoint,&over));h.word(nearest);h.point(nearestPoint);h.word(over);
        dtPolyRef selected=0;float selectedPoint[3]={};h.word(q.findRandomPoint(&filter,draw,&selected,selectedPoint));h.word(selected);h.point(selectedPoint);
        h.word(q.findRandomPointAroundCircle(ref,start,5,&filter,draw,&selected,selectedPoint));h.word(selected);h.point(selectedPoint);h.word(randomSeed);
    }return h.value;
}
static uint64_t proximity() {
    dtProximityGrid grid;require(grid.init(4096,1));unsigned short ids[128]={};Hash h;
    for(int batch=0;batch<4;++batch){grid.clear();for(int i=0;i<128;++i){float x=float(i%16)*.375f-3,z=float(i/16)*.375f-1.5f+float(batch)*.125f;unsigned short id=(i%5==0)?60000+i:i%64;grid.addItem(id,x-.35f,z-.35f,x+.35f,z+.35f);}
        for(int i=0;i<4;++i)h.word(uint32_t(grid.getBounds()[i]));
        for(int i=0;i<256;++i){float x=float(i%16)*.5f-4,z=float(i/16)*.25f-2;const int caps[]={0,8,32,128};int n=grid.queryItems(x-1,z-1,x+1,z+1,ids,caps[i%4]);h.word(n);for(int j=0;j<n;++j)h.word(ids[j]);h.word(grid.getItemCountAt(i%8-4,i/32-4));}
    }return h.value;
}
static uint64_t avoidance(bool adaptive) {
    dtObstacleAvoidanceQuery q;require(q.init(16,8));dtObstacleAvoidanceDebugData debug;require(debug.init(2048));
    dtObstacleAvoidanceParams p={};p.velBias=.4f;p.weightDesVel=2;p.weightCurVel=.75f;p.weightSide=.75f;p.weightToi=2.5f;p.horizTime=2.5f;p.gridSize=23;p.adaptiveDivs=7;p.adaptiveRings=2;p.adaptiveDepth=3;Hash h;
    for(int i=0;i<32;++i){q.reset();for(int j=0;j<8;++j){float pos[]={float(j%4)*1.25f-1.5f,0,float(j/4)*1.5f+1},vel[]={float(j)*.125f-.5f,0,.25f},desired[]={.5f,0,-.25f};q.addCircle(pos,.3f,vel,desired);}
        const float corners[][3]={{-4,0,-4},{-4,0,4},{4,0,4},{4,0,-4}};for(int j=0;j<4;++j)q.addSegment(corners[j],corners[(j+1)%4]);
        const float pos[]={.125f,0,.25f},vel[]={float(i)*.03125f-.5f,0,.25f},desired[]={1.5f-float(i%4)*.5f,0,-.75f+float(i%3)*.5f};float result[3]={};bool record=i%8==0;
        int count=adaptive?q.sampleVelocityAdaptive(pos,.3f,3,vel,desired,result,&p,record?&debug:nullptr):q.sampleVelocityGrid(pos,.3f,3,vel,desired,result,&p,record?&debug:nullptr);h.word(count);h.point(result);
        if(record){debug.normalizeSamples();h.word(debug.getSampleCount());for(int j=0;j<debug.getSampleCount();++j){h.point(debug.getSampleVelocity(j));real(h,debug.getSampleSize(j));real(h,debug.getSamplePenalty(j));real(h,debug.getSampleDesiredVelocityPenalty(j));real(h,debug.getSampleCurrentVelocityPenalty(j));real(h,debug.getSamplePreferredSidePenalty(j));real(h,debug.getSampleCollisionTimePenalty(j));}}
        for(int j=0;j<q.getObstacleCircleCount();++j){const auto* c=q.getObstacleCircle(j);h.point(c->dp);real(h,c->np[0]);real(h,c->np[2]);}for(int j=0;j<q.getObstacleSegmentCount();++j)h.word(q.getObstacleSegment(j)->touch);
    }return h.value;
}

struct LifecycleBuffers {
    DetourBuffers query;
    std::vector<unsigned char> exported;
    unsigned char state[8192];
    explicit LifecycleBuffers(size_t size){exported.reserve(size);}
};
static void exportTile(const dtNavMesh& nav,dtTileRef ref,std::vector<unsigned char>& output,Hash& h) {
    const dtMeshTile* tile=nav.getTileByRef(ref);require(tile!=nullptr);
    output.assign(tile->data,tile->data+tile->dataSize);h.word(output.size());h.bytes(output.data(),output.size());
}
static uint64_t lifecycle(const std::vector<unsigned char> (&data)[2],LifecycleBuffers& out) {
    dtNavMesh nav;dtNavMeshParams p={};p.tileWidth=p.tileHeight=32;p.maxTiles=4;p.maxPolys=2048;require(dtStatusSucceed(nav.init(&p)));
    dtNavMeshQuery query;require(dtStatusSucceed(query.init(&nav,4096)));dtQueryFilter filter;dtTileRef references[2]={};Hash h;
    for(int cycle=0;cycle<4;++cycle){for(int step=0;step<2;++step){int tile=(cycle+step)%2;auto* bytes=static_cast<unsigned char*>(dtAlloc(data[tile].size(),DT_ALLOC_PERM));require(bytes!=nullptr);std::memcpy(bytes,data[tile].data(),data[tile].size());
            h.word(nav.addTile(bytes,int(data[tile].size()),DT_TILE_FREE_DATA,cycle%2==0?0:references[tile],&references[tile]));require(references[tile]!=0);h.word(references[tile]);}
        for(auto ref:references)exportTile(nav,ref,out.exported,h);
        const auto* tile=nav.getTileByRef(references[0]);int stateSize=nav.getTileStateSize(tile);require(stateSize<=int(sizeof(out.state)));std::memset(out.state,0xa5,stateSize);h.word(nav.storeTileState(tile,out.state,stateSize));h.word(stateSize);h.bytes(out.state,stateSize);
        for(int j=0;j<32;++j){auto ref=references[0]|(j*13);h.word(nav.setPolyFlags(ref,2+j%3));h.word(nav.setPolyArea(ref,(cycle+j)%64));unsigned short flags=0;unsigned char area=0;require(dtStatusSucceed(nav.getPolyFlags(ref,&flags)));require(dtStatusSucceed(nav.getPolyArea(ref,&area)));h.word(flags);h.word(area);}
        exportTile(nav,references[0],out.exported,h);h.word(nav.restoreTileState(const_cast<dtMeshTile*>(nav.getTileByRef(references[0])),out.state,stateSize));
        const float start[]={30.5f,0,16.5f},end[]={33.5f,0,16.5f};int n=0;dtStatus status=query.findPath(references[0]|542,references[1]|513,start,end,&filter,out.query.path,&n,256);h.word(status);h.word(n);bool offmesh=false;for(int j=0;j<n;++j){h.word(out.query.path[j]);offmesh|=out.query.path[j]==(references[0]|1024);}require(status==DT_SUCCESS&&offmesh);
        int ns=0;h.word(query.findStraightPath(start,end,out.query.path,n,out.query.points,out.query.flags,out.query.refs,&ns,256,DT_STRAIGHTPATH_ALL_CROSSINGS));h.word(ns);for(int j=0;j<ns;++j){h.point(out.query.points+j*3);h.word(out.query.flags[j]);h.word(out.query.refs[j]);}
        unsigned char* removed=nullptr;int removedSize=0;h.word(nav.removeTile(references[1],&removed,&removedSize));h.word(removed!=nullptr);h.word(uint32_t(removedSize));h.word(nav.isValidPolyRef(references[1]|1));
        exportTile(nav,references[0],out.exported,h);h.word(nav.removeTile(references[0],&removed,&removedSize));h.word(removed!=nullptr);h.word(uint32_t(removedSize));
    }return h.value;
}

struct SearchBuffers {
    dtPolyRef refs[1024],parents[1024],path[1025],existing[1025];
    float costs[1024];
};
struct HashPolygons: dtPolyQuery {
    Hash& hash;
    explicit HashPolygons(Hash& h):hash(h){}
    void process(const dtMeshTile*,dtPoly** polys,dtPolyRef* refs,int count) override {
        hash.word(count);
        for(int i=0;i<count;++i){hash.word(refs[i]);hash.word(polys[i]->flags);hash.word(polys[i]->getArea());}
    }
};
static uint64_t search(const dtNavMesh& nav,dtNavMeshQuery& query,SearchBuffers& out) {
    dtQueryFilter filter;Hash h;
    const dtPolyRef base=nav.getPolyRefBase(nav.getTile(0));
    for(uint32_t i=0;i<32;++i){uint32_t a=i*37%1024;float center[3];point(a,center);dtPolyRef start=base|a;
        const int caps[]={0,8,128,1024};int cap=caps[i%4];
        for(int shape=0;shape<2;++shape){int n=0;dtStatus status;
            if(shape){float x=center[0],y=center[1],z=center[2];float vertices[]={x-3.25f,y,z-3.25f,x-3.25f,y,z+3.25f,x+3.25f,y,z+3.25f,x+3.25f,y,z-3.25f};
                status=query.findPolysAroundShape(start,vertices,4,&filter,out.refs,out.parents,out.costs,&n,cap);
            }else status=query.findPolysAroundCircle(start,center,4.25f,&filter,out.refs,out.parents,out.costs,&n,cap);
            h.word(status);h.word(n);
            for(int j=0;j<n;++j){h.word(out.refs[j]);h.word(out.parents[j]);real(h,out.costs[j]);}
            uint32_t indices[]={a,(a+33)%1024,(i*113+17)%1024};
            for(auto index:indices){dtPolyRef end=base|index;h.word(query.isInClosedList(end));int count=0;h.word(query.getPathFromDijkstraSearch(end,out.path,&count,16));h.word(count);for(int j=0;j<count;++j)h.word(out.path[j]);}
        }
        HashPolygons callback(h);const float extents[]={4,2,4};h.word(query.queryPolygons(center,extents,&filter,&callback));
    }return h.value;
}
static uint64_t sliced(const dtNavMesh& nav,dtNavMeshQuery& query,SearchBuffers& out) {
    dtQueryFilter filter;Hash h;const dtPolyRef base=nav.getPolyRefBase(nav.getTile(0));
    for(uint32_t i=0;i<32;++i){uint32_t a=i*37%1024,b=(i*113+17)%1024;float start[3],end[3];point(a,start);point(b,end);bool partial=i%4>=2;int existingCount=0;
        if(partial)h.word(query.findPath(base|a,base|b,start,end,&filter,out.existing,&existingCount,256));
        dtStatus status=query.initSlicedFindPath(base|a,base|b,start,end,&filter,i%2==0?0:DT_FINDPATH_ANY_ANGLE);h.word(status);
        while(dtStatusInProgress(status)){int done=0;status=query.updateSlicedFindPath(partial?int(i%7+1):8,&done);h.word(status);h.word(done);if(partial)break;}
        const int caps[]={1,3,16,256};int cap=caps[(i/4)%4],n=0;
        if(partial)status=query.finalizeSlicedFindPathPartial(out.existing,existingCount,out.path,&n,cap);
        else status=query.finalizeSlicedFindPath(out.path,&n,cap);
        h.word(status);h.word(n);for(int j=0;j<n;++j)h.word(out.path[j]);
    }return h.value;
}

struct BenchCodec: dtTileCacheCompressor {
    unsigned char mask;int fail=0;Hash log;
    explicit BenchCodec(unsigned char m):mask(m){}
    int maxCompressedSize(int n) override {log.word(0);log.word(n);return n+8;}
    dtStatus transfer(int tag,const unsigned char* input,int n,unsigned char* output,int cap,int* size){
        log.word(tag);log.word(n);log.word(cap);bool failed=fail==tag;
        dtStatus status=failed?DT_FAILURE|DT_INVALID_PARAM:DT_SUCCESS|DT_BUFFER_TOO_SMALL;log.word(status);
        int count=failed?n/2:n;require(count<=cap);for(int i=0;i<count;++i)output[i]=input[i]^mask;*size=count;return status;
    }
    dtStatus compress(const unsigned char* input,int n,unsigned char* output,int cap,int* size) override {return transfer(1,input,n,output,cap,size);}
    dtStatus decompress(const unsigned char* input,int n,unsigned char* output,int cap,int* size) override {return transfer(2,input,n,output,cap,size);}
};
struct CacheGrids {
    std::vector<unsigned char> heights,areas,cons;
    CacheGrids(int width,int batch,bool varied):heights(width*width,0),areas(width*width,63),cons(width*width,0){
        for(int z=0;z<width;++z)for(int x=0;x<width;++x){int i=x+z*width;
            if(varied){heights[i]=(x/8+z/8)%3;areas[i]=(x>=8&&x<12&&z>=10&&z<18)?0:1+(x/8+z/8+batch)%3;}
            cons[i]=(x>0)|((z+1<width)<<1)|((x+1<width)<<2)|((z>0)<<3);
        }
    }
};
static dtTileCacheLayerHeader cacheHeader(int width,int x,int z,int level){
    dtTileCacheLayerHeader h={};h.magic=DT_TILECACHE_MAGIC;h.version=DT_TILECACHE_VERSION;h.tx=x;h.ty=z;h.tlayer=level;
    h.bmin[0]=float(x)*16;h.bmin[1]=float(level)*4;h.bmin[2]=float(z)*16;
    h.bmax[0]=float(x)*16+float(width)*.5f;h.bmax[1]=float(level)*4+2;h.bmax[2]=float(z)*16+float(width)*.5f;
    h.width=h.height=width;h.maxx=h.maxy=width-1;return h;
}
static uint64_t cacheCodec(){
    Hash h;dtTileCacheAlloc alloc;
    for(int batch=0;batch<16;++batch){int width=16+batch%3*8;auto header=cacheHeader(width,batch%2,0,0);CacheGrids grids(width,batch,true);BenchCodec codec(batch*17);
        unsigned char* data=nullptr;int size=0;dtStatus status=dtBuildTileCacheLayer(&codec,&header,grids.heights.data(),grids.areas.data(),grids.cons.data(),&data,&size);require(status==DT_SUCCESS);
        h.word(status);h.word(size);h.bytes(data,size);h.word(dtTileCacheHeaderSwapEndian(data,size));h.bytes(data,size);h.word(dtTileCacheHeaderSwapEndian(data,size));h.bytes(data,size);
        dtTileCacheLayer* layer=nullptr;status=dtDecompressTileCacheLayer(&alloc,&codec,data,size,&layer);require(status==DT_SUCCESS);h.word(status);
        for(const auto* grid:{layer->heights,layer->areas,layer->cons,layer->regs})h.bytes(grid,width*width);dtFreeTileCacheLayer(&alloc,layer);
        codec.fail=1;unsigned char* rejected=nullptr;int rejectedSize=0;h.word(dtBuildTileCacheLayer(&codec,&header,grids.heights.data(),grids.areas.data(),grids.cons.data(),&rejected,&rejectedSize));require(rejected==nullptr);
        codec.fail=2;h.word(dtDecompressTileCacheLayer(&alloc,&codec,data,size,&layer));require(layer==nullptr);codec.fail=0;
        for(int byte:{0,4}){data[byte]^=1;h.word(dtDecompressTileCacheLayer(&alloc,&codec,data,size,&layer));require(layer==nullptr);data[byte]^=1;}
        h.word(codec.log.value);dtFree(data);
    }return h.value;
}
static uint64_t cacheBuilder(){
    Hash h;dtTileCacheAlloc alloc;
    for(int batch=0;batch<8;++batch){CacheGrids grids(32,batch,true);auto header=cacheHeader(32,0,0,0);std::vector<unsigned char> regs(1024,0);
        dtTileCacheLayer layer={};layer.header=&header;layer.heights=grids.heights.data();layer.areas=grids.areas.data();layer.cons=grids.cons.data();layer.regs=regs.data();
        float pos[]={2+float(batch)*.25f,0,4},lo[]={8,0,7},hi[]={9.5f,2,9},center[]={12,0,12},half[]={1,2,.5f},rotation[]={.5f,.5f};
        h.word(dtMarkCylinderArea(layer,header.bmin,.5f,.125f,pos,.75f,2,0));h.word(dtMarkBoxArea(layer,header.bmin,.5f,.125f,lo,hi,0));h.word(dtMarkBoxArea(layer,header.bmin,.5f,.125f,center,half,rotation,0));
        auto* contours=dtAllocTileCacheContourSet(&alloc);auto* mesh=dtAllocTileCachePolyMesh(&alloc);require(contours&&mesh);
        require(dtBuildTileCacheRegions(&alloc,layer,4)==DT_SUCCESS);require(dtBuildTileCacheContours(&alloc,layer,4,1.3f,*contours)==DT_SUCCESS);require(dtBuildTileCachePolyMesh(&alloc,*contours,*mesh)==DT_SUCCESS);
        h.word(DT_SUCCESS);h.word(mesh->nvp);h.word(layer.regCount);h.bytes(layer.regs,1024);h.word(contours->nconts);
        for(int i=0;i<contours->nconts;++i){const auto& c=contours->conts[i];h.word(c.reg);h.word(c.area);h.word(c.nverts);h.bytes(c.verts,c.nverts*4);}
        h.word(mesh->nverts);h.word(mesh->npolys);for(int i=0;i<mesh->nverts*3;++i)h.word(mesh->verts[i]);for(int i=0;i<mesh->npolys*12;++i)h.word(mesh->polys[i]);for(int i=0;i<mesh->npolys;++i)h.word(mesh->areas[i]);for(int i=0;i<mesh->npolys;++i)h.word(mesh->flags[i]);
        dtFreeTileCachePolyMesh(&alloc,mesh);dtFreeTileCacheContourSet(&alloc,contours);
    }return h.value;
}
struct CacheProcess: dtTileCacheMeshProcess {
    Hash log;
    void process(dtNavMeshCreateParams* p,unsigned char* areas,unsigned short* flags) override {
        log.word(p->vertCount);log.word(p->polyCount);log.word(uint32_t(p->tileX));log.word(uint32_t(p->tileY));log.word(uint32_t(p->tileLayer));
        for(int i=0;i<p->polyCount;++i){log.word(areas[i]);log.word(flags[i]);areas[i]=(areas[i]+i%4)%7;flags[i]=1+areas[i]%3;}
        p->userId=0x17600000|(uint32_t(p->tileX)<<12)|(uint32_t(p->tileY)<<4)|uint32_t(p->tileLayer);
    }
};
struct CacheStreamBuffers {std::vector<unsigned char> exported;explicit CacheStreamBuffers(bool enabled){if(enabled)exported.reserve(65536);}};
static void cacheSnapshot(const dtTileCache& cache,const dtNavMesh& nav,CacheStreamBuffers& out,Hash& h){
    for(int z=0;z<2;++z)for(int x=0;x<2;++x)for(int level=0;level<2;++level){dtTileRef ref=nav.getTileRefAt(x,z,level);require(ref!=0);h.word(ref);exportTile(nav,ref,out.exported,h);}
    for(int i=0;i<cache.getObstacleCount();++i){const auto* ob=cache.getObstacle(i);h.word(ob->state);h.word(ob->type);h.word(ob->salt);h.word(ob->ntouched);h.word(ob->npending);for(int j=0;j<ob->ntouched;++j)h.word(ob->touched[j]);for(int j=0;j<ob->npending;++j)h.word(ob->pending[j]);}
}
static uint64_t cacheStream(CacheStreamBuffers& out){
    dtTileCacheAlloc alloc;BenchCodec codec(0x5a);CacheProcess process;dtTileCache cache;dtTileCacheParams p={};p.cs=.5f;p.ch=.125f;p.width=p.height=32;p.walkableHeight=2;p.walkableRadius=.25f;p.walkableClimb=.5f;p.maxSimplificationError=1.3f;p.maxTiles=16;p.maxObstacles=32;require(dtStatusSucceed(cache.init(&p,&alloc,&codec,&process)));
    dtNavMesh nav;dtNavMeshParams np={};np.tileWidth=np.tileHeight=16;np.maxTiles=16;np.maxPolys=2048;require(dtStatusSucceed(nav.init(&np)));dtCompressedTileRef references[8]={};Hash h;
    for(int z=0;z<2;++z)for(int x=0;x<2;++x)for(int level=0;level<2;++level){auto header=cacheHeader(32,x,z,level);CacheGrids grids(32,0,false);unsigned char* data=nullptr;int size=0;
        require(dtBuildTileCacheLayer(&codec,&header,grids.heights.data(),grids.areas.data(),grids.cons.data(),&data,&size)==DT_SUCCESS);dtCompressedTileRef r=0;h.word(cache.addTile(data,size,DT_COMPRESSEDTILE_FREE_DATA,&r));references[z*4+x*2+level]=r;h.word(r);h.word(cache.buildNavMeshTile(r,&nav));}
    cacheSnapshot(cache,nav,out,h);
    for(int batch=0;batch<2;++batch){dtObstacleRef obstacles[8];for(int i=0;i<8;++i){float pos[]={14+float(i%4)*1.25f,-1,14+float(i/4)*3.25f+float(batch)*.125f};dtStatus status;
        if(i%3==0)status=cache.addObstacle(pos,1.25f,9,&obstacles[i]);
        else if(i%3==1){float lo[]={pos[0]-1.25f,-1,pos[2]-1},hi[]={pos[0]+1.25f,8,pos[2]+1};status=cache.addBoxObstacle(lo,hi,&obstacles[i]);}
        else {float center[]={pos[0],3,pos[2]},half[]={1.25f,4,1};status=cache.addBoxObstacle(center,half,.375f,&obstacles[i]);}h.word(status);h.word(obstacles[i]);}
        for(int remove=0;remove<2;++remove){if(remove)for(auto r:obstacles)h.word(cache.removeObstacle(r));bool done=false;do{dtStatus status=cache.update(.05f,&nav,&done);h.word(status);h.word(done);require(!dtStatusFailed(status));}while(!done);cacheSnapshot(cache,nav,out,h);}
    }
    codec.fail=2;h.word(cache.buildNavMeshTile(references[0],&nav));cacheSnapshot(cache,nav,out,h);codec.fail=0;h.word(cache.buildNavMeshTile(references[0],&nav));
    unsigned char* removed=nullptr;int size=0;h.word(cache.removeTile(references[3],&removed,&size));h.word(removed!=nullptr);h.word(cache.buildNavMeshTile(references[3],&nav));
    auto header=cacheHeader(32,1,0,1);CacheGrids grids(32,0,false);unsigned char* data=nullptr;require(dtBuildTileCacheLayer(&codec,&header,grids.heights.data(),grids.areas.data(),grids.cons.data(),&data,&size)==DT_SUCCESS);dtCompressedTileRef r=0;h.word(cache.addTile(data,size,DT_COMPRESSEDTILE_FREE_DATA,&r));h.word(r);h.word(cache.buildNavMeshTile(r,&nav));h.word(cache.buildNavMeshTilesAt(2,2,&nav));cacheSnapshot(cache,nav,out,h);h.word(codec.log.value);h.word(process.log.value);return h.value;
}

static void hashRecastPolygons(Hash& h,const rcPolyMesh& p){
    h.word(p.nverts);h.word(p.npolys);h.word(p.nvp);h.word(p.maxpolys);h.point(p.bmin);h.point(p.bmax);real(h,p.cs);real(h,p.ch);h.word(uint32_t(p.borderSize));real(h,p.maxEdgeError);
    for(int i=0;i<p.nverts*3;++i)h.word(p.verts[i]);for(int half=0;half<2;++half)for(int i=0;i<p.npolys;++i)for(int j=0;j<p.nvp;++j)h.word(p.polys[i*p.nvp*2+half*p.nvp+j]);for(int i=0;i<p.npolys;++i){h.word(p.regs[i]);h.word(p.areas[i]);h.word(p.flags[i]);}
}
static void hashRecastDetail(Hash& h,const rcPolyMeshDetail& d){
    h.word(d.nmeshes);h.word(d.nverts);h.word(d.ntris);for(int i=0;i<d.nmeshes*4;++i)h.word(d.meshes[i]);for(int i=0;i<d.nverts;++i)h.point(d.verts+i*3);for(int i=0;i<d.ntris*4;++i)h.word(d.tris[i]);
}
struct RecastUtilities {
    rcCompactHeightfield compact;rcHeightfieldLayerSet layers;rcPolyMesh p;rcPolyMeshDetail d;
    std::vector<unsigned char> initialAreas,triangleAreas;float output[32*3];
    void prepare(const Geometry& g){buildRecast(g,compact,layers,p,d,0,12,false,&initialAreas);triangleAreas.resize(g.triangles.size()/3,17);for(int i=0;i<p.npolys;++i)p.flags[i]=(i*13+7);}
    uint64_t run(const Geometry& g){rcContext ctx(false);Hash h;h.word(compact.width);h.word(compact.height);h.word(compact.spanCount);
        for(int batch=0;batch<16;++batch){std::memcpy(compact.areas,initialAreas.data(),initialAreas.size());float x=2+float(batch)*.125f;
            float lo[]={x,-1,2},hi[]={x+4,4,6},cylinder[]={12,0,12},polygon[]={5,0,7,5,0,11,9,0,11,9,0,7};
            rcMarkBoxArea(&ctx,lo,hi,21,compact);rcMarkCylinderArea(&ctx,cylinder,2.25f,4,7,compact);rcMarkConvexPolyArea(&ctx,polygon,4,-1,4,3,compact);require(rcMedianFilterWalkableArea(&ctx,compact));for(int i=0;i<compact.spanCount;++i)h.word(compact.areas[i]);
            float bmin[3],bmax[3];rcCalcBounds(g.vertices.data(),g.vertices.size()/3,bmin,bmax);h.point(bmin);h.point(bmax);int width,height;rcCalcGridSize(bmin,bmax,.3f,&width,&height);h.word(uint32_t(width));h.word(uint32_t(height));
            std::fill(triangleAreas.begin(),triangleAreas.end(),17);rcMarkWalkableTriangles(&ctx,30+float(batch)*.25f,g.vertices.data(),g.vertices.size()/3,g.triangles.data(),g.triangles.size()/3,triangleAreas.data());rcClearUnwalkableTriangles(&ctx,15,g.vertices.data(),g.vertices.size()/3,g.triangles.data(),g.triangles.size()/3,triangleAreas.data());for(auto area:triangleAreas)h.word(area);
            for(int i=0;i<32;++i){output[i*3]=-17;output[i*3+1]=.25f;output[i*3+2]=31;}const int caps[]={0,4,16,32};const float offsets[]={-.5f,0,.75f,1.25f};int count=rcOffsetPoly(polygon,4,offsets[batch%4],output,caps[batch%4]);h.word(count);for(int i=0;i<32;++i)h.point(output+i*3);
            rcPolyMesh copy;require(rcCopyPolyMesh(&ctx,p,copy));hashRecastPolygons(h,copy);rcPolyMesh* sources[]={&p,&copy};rcPolyMesh merged;require(rcMergePolyMeshes(&ctx,sources,2,merged));hashRecastPolygons(h,merged);rcPolyMeshDetail* details[]={&d,nullptr,&d};rcPolyMeshDetail combined;require(rcMergePolyMeshDetails(&ctx,details,3,combined));hashRecastDetail(h,combined);
        }return h.value;
    }
};

static uint64_t recastLayerCapacity(const Geometry& g){
    rcCompactHeightfield compact;rcHeightfieldLayerSet layers;rcPolyMesh p;rcPolyMeshDetail d;
    buildRecast(g,compact,layers,p,d,0,6,true,nullptr,true);Hash h;h.word(compact.width);h.word(compact.height);h.word(compact.spanCount);for(int i=0;i<compact.spanCount;++i)h.word(compact.areas[i]);h.word(1);return h.value;
}
