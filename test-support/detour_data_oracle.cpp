// Calls the unmodified pinned Detour tile builder, endian utilities and loader.
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourAlloc.h"
#include "DetourCommon.h"
#include "detour_node_snapshot.h"
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <memory>
#include <map>
#include <iostream>
#include <string>
#include <vector>

template<class T> static void read(std::vector<T>& v, int n) {
    v.resize(n);
    for (int i = 0; i < n; ++i) { unsigned long long x; std::cin >> x; v[i] = (T)x; }
}
static bool floatBits=false;
static void inputFloat(float& value) {
    if(floatBits){unsigned bits;std::cin>>bits;std::memcpy(&value,&bits,4);}
    else std::cin>>value;
}
static void floats(std::vector<float>& v, int n) {
    v.resize(n); for (float& x : v) inputFloat(x);
}
static void bytes(const unsigned char* p, int n) {
    std::cout << '[';
    for (int i = 0; i < n; ++i) { if (i) std::cout << ','; std::cout << (unsigned)p[i]; }
    std::cout << ']';
}
static void point(float* p) {
    for(int i=0;i<3;++i) { unsigned bits;std::cin>>bits;std::memcpy(p+i,&bits,4); }
}
static void pointBits(const float* p) {
    for(int i=0;i<3;++i) { unsigned bits;std::memcpy(&bits,p+i,4);std::cout<<','<<bits; }
}
struct CallbackFilter : dtQueryFilter {
    unsigned blocked,ignoreFlags;
    mutable std::vector<std::int64_t> log;
    explicit CallbackFilter(bool input=true):blocked(255),ignoreFlags(0) {
        if(!input)return;
        unsigned include,exclude;std::cin>>include>>exclude>>blocked>>ignoreFlags;
        setIncludeFlags((unsigned short)include);setExcludeFlags((unsigned short)exclude);
        for(int i=0;i<64;++i) { unsigned bits;std::cin>>bits;float f;std::memcpy(&f,&bits,4);setAreaCost(i,f); }
    }
    bool passFilter(dtPolyRef r,const dtMeshTile* tile,const dtPoly* poly) const override {
        // Fixture-only area sentinel mirrors the stateful Rust test filter.
        const bool rejectFirst=blocked==254 && log.empty();
        log.insert(log.end(),{0,(std::int64_t)r,poly->flags,poly->getArea(),tile->header->x,tile->header->y,tile->header->layer,tile->header->userId});
        return !rejectFirst && (ignoreFlags || dtQueryFilter::passFilter(r,tile,poly)) && poly->getArea()!=blocked;
    }
    float getCost(const float* a,const float* b,dtPolyRef pr,const dtMeshTile* pt,const dtPoly* pp,
        dtPolyRef cr,const dtMeshTile* ct,const dtPoly* cp,dtPolyRef nr,const dtMeshTile* nt,const dtPoly* np) const override {
        log.insert(log.end(),{1,(std::int64_t)pr,(std::int64_t)cr,(std::int64_t)nr,pp?pp->getArea():255,cp->getArea(),np?np->getArea():255});
        const dtMeshTile* tiles[]={pt,ct,nt};
        for(const dtMeshTile* tile:tiles) {
            log.push_back(tile?tile->header->x:0);log.push_back(tile?tile->header->y:0);log.push_back(tile?tile->header->layer:0);
            log.push_back(tile?tile->header->userId:0);
        }
        for(int i=0;i<3;++i) { unsigned bits;std::memcpy(&bits,a+i,4);log.push_back(bits); }
        for(int i=0;i<3;++i) { unsigned bits;std::memcpy(&bits,b+i,4);log.push_back(bits); }
        float cost=dtQueryFilter::getCost(a,b,pr,pt,pp,cr,ct,cp,nr,nt,np);
        if(pp)cost+=(float)pp->getArea()*0.125f;
        if(np)cost+=(float)np->getArea()*0.0625f;
        cost+=(float)ct->header->x*0.25f;
        return cost;
    }
    void outputLog() const {
        std::cout<<'[';for(size_t i=0;i<log.size();++i) { if(i)std::cout<<',';std::cout<<log[i]; }std::cout<<']';
    }
};
struct RecordingQuery : dtPolyQuery {
    CallbackFilter& filter;
    RecordingQuery(CallbackFilter& f):filter(f) {}
    void process(const dtMeshTile* tile,dtPoly** polys,dtPolyRef* refs,int count) override {
        filter.log.insert(filter.log.end(),{2,tile->header->x,tile->header->y,tile->header->layer,tile->header->userId,count});
        for(int i=0;i<count;++i) { filter.log.push_back((std::int64_t)refs[i]);filter.log.push_back(polys[i]->getArea()); }
    }
};
static std::vector<unsigned> randomValues;
static size_t randomCursor;
static CallbackFilter* randomFilter;
static float nextRandom() {
    unsigned bits=randomValues[randomCursor++%randomValues.size()];float value;std::memcpy(&value,&bits,4);
    randomFilter->log.insert(randomFilter->log.end(),{3,bits});return value;
}
static void runtime() {
    dtNavMeshParams p={};
    for(float& v:p.orig) std::cin>>v;
    std::cin>>p.tileWidth>>p.tileHeight>>p.maxTiles>>p.maxPolys;
    int initialNodes;std::cin>>initialNodes;
    std::vector<std::vector<unsigned char> > buffers;
    std::map<const unsigned char*,int> actualSizes;
    dtNavMesh mesh;std::cout<<'['<<mesh.init(&p);
    dtNavMeshQuery persistent;persistent.init(&mesh,initialNodes);
    std::unique_ptr<CallbackFilter> slicedFilter(new CallbackFilter(false));
    std::string op;
    while(std::cin>>op) {
        std::cout<<',';
        if(op=="r" || op=="c") {
            CallbackFilter filter;int n;std::cin>>n;read(randomValues,n);randomCursor=0;randomFilter=&filter;
            dtPolyRef ref=123;float position[3]={-31,17,3};dtStatus s;
            if(op=="c") {
                unsigned long long start;std::cin>>start;float center[3],radius;point(center);unsigned bits;std::cin>>bits;std::memcpy(&radius,&bits,4);
                s=persistent.findRandomPointAroundCircle((dtPolyRef)start,center,radius,&filter,nextRandom,&ref,position);
            }else s=persistent.findRandomPoint(&filter,nextRandom,&ref,position);
            std::cout<<'['<<s<<','<<ref;pointBits(position);std::cout<<','<<randomCursor<<',';filter.outputLog();std::cout<<']';randomFilter=0;
        } else if(op=="t") {
            CallbackFilter filter;unsigned long long start;int capacity;std::cin>>start>>capacity;
            float center[3],radius;point(center);unsigned bits;std::cin>>bits;std::memcpy(&radius,&bits,4);
            std::vector<dtPolyRef> refs(std::max(capacity,1)),parents(std::max(capacity,1));int count=0;
            dtStatus s=persistent.findLocalNeighbourhood((dtPolyRef)start,center,radius,&filter,refs.data(),parents.data(),&count,capacity);
            std::cout<<'['<<s<<",[";for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<'['<<refs[i]<<','<<parents[i]<<']';}
            std::cout<<"],";filter.outputLog();std::cout<<']';
        } else if(op=="w") {
            CallbackFilter filter;unsigned long long ref;int capacity,portals;std::cin>>ref>>capacity>>portals;
            std::vector<float> verts(std::max(capacity,1)*6);std::vector<dtPolyRef> refs(std::max(capacity,1));int count=0;
            dtStatus s=persistent.getPolyWallSegments((dtPolyRef)ref,&filter,verts.data(),portals?refs.data():0,&count,capacity);
            std::cout<<'['<<s<<",[";for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<'['<<refs[i];pointBits(&verts[i*6]);pointBits(&verts[i*6+3]);std::cout<<']';}
            std::cout<<"],";filter.outputLog();std::cout<<']';
        } else if(op=="v") {
            CallbackFilter filter;unsigned long long start;std::cin>>start;
            float center[3],radius;point(center);unsigned bits;std::cin>>bits;std::memcpy(&radius,&bits,4);
            float distance=-19,position[3]={-31,17,3},normal[3]={2,3,5};
            dtStatus s=persistent.findDistanceToWall((dtPolyRef)start,center,radius,&filter,&distance,position,normal);
            std::memcpy(&bits,&distance,4);std::cout<<'['<<s<<','<<bits;pointBits(position);pointBits(normal);
            std::cout<<',';filter.outputLog();std::cout<<']';
        } else if(op=="G" || op=="B") {
            CallbackFilter filter;unsigned long long start;int capacity;std::cin>>start>>capacity;
            std::vector<dtPolyRef> refs(std::max(capacity,1)),parents(std::max(capacity,1));
            std::vector<float> costs(std::max(capacity,1));int count=0;dtStatus s;
            if(op=="G"){
                float center[3],radius;point(center);unsigned bits;std::cin>>bits;std::memcpy(&radius,&bits,4);
                s=persistent.findPolysAroundCircle((dtPolyRef)start,center,radius,&filter,refs.data(),parents.data(),costs.data(),&count,capacity);
            }else{
                int n;std::cin>>n;std::vector<float> shape(std::max(n,1)*3);for(int i=0;i<n;++i)point(&shape[i*3]);
                s=persistent.findPolysAroundShape((dtPolyRef)start,shape.data(),n,&filter,refs.data(),parents.data(),costs.data(),&count,capacity);
            }
            std::cout<<'['<<s<<",[";for(int i=0;i<count;++i){if(i)std::cout<<',';unsigned bits;std::memcpy(&bits,&costs[i],4);std::cout<<'['<<refs[i]<<','<<parents[i]<<','<<bits<<']';}
            std::cout<<"],";filter.outputLog();std::cout<<']';
        } else if(op=="a") {
            const dtNavMesh& view=mesh;int x,y,layer,capacity,index;unsigned long long ref;std::cin>>x>>y>>layer>>capacity>>index>>ref;float position[3];point(position);
            const dtNavMeshParams* params=mesh.getParams();std::cout<<'['<<mesh.getMaxTiles()<<','<<params->maxPolys;pointBits(params->orig);std::cout<<',';float size[]={params->tileWidth,params->tileHeight};for(int k=0;k<2;++k){unsigned bits;std::memcpy(&bits,size+k,4);std::cout<<bits<<',';}
            int tx,ty;mesh.calcTileLoc(position,&tx,&ty);std::cout<<tx<<','<<ty<<",[";
            std::vector<const dtMeshTile*> found(std::max(capacity,1));int n=mesh.getTilesAt(x,y,found.data(),capacity);for(int k=0;k<n;++k){if(k)std::cout<<',';std::cout<<(found[k]-view.getTile(0));}std::cout<<"],";
            const dtMeshTile* at=mesh.getTileAt(x,y,layer);std::cout<<(at?at-view.getTile(0):-1)<<','<<mesh.getTileRefAt(x,y,layer)<<','<<mesh.getTileRef(at)<<','<<mesh.getPolyRefBase(at);
            for(const dtMeshTile* tile:{index>=0 && index<mesh.getMaxTiles()?view.getTile(index):0,mesh.getTileByRef((dtTileRef)ref)}){std::cout<<",[";if(tile)std::cout<<(tile-view.getTile(0))<<','<<tile->salt<<','<<(tile->header?1:0)<<','<<mesh.getTileRef(tile)<<','<<mesh.getPolyRefBase(tile);std::cout<<']';}
            const dtMeshTile* tile=0;const dtPoly* poly=0;dtStatus st=mesh.getTileAndPolyByRef((dtPolyRef)ref,&tile,&poly);std::cout<<",["<<st<<','<<mesh.isValidPolyRef((dtPolyRef)ref)<<','<<(tile?tile-view.getTile(0):-1)<<','<<(poly?poly-tile->polys:-1)<<"],[";
            unsigned it=mesh.decodePolyIdTile((dtPolyRef)ref),ip=mesh.decodePolyIdPoly((dtPolyRef)ref);if(it<(unsigned)mesh.getMaxTiles() && view.getTile(it)->header && ip<(unsigned)view.getTile(it)->header->polyCount){mesh.getTileAndPolyByRefUnsafe((dtPolyRef)ref,&tile,&poly);std::cout<<(tile-view.getTile(0))<<','<<(poly-tile->polys);}
            std::cout<<"]]";
        } else if(op=="g") {
            unsigned long long end;int capacity,count;std::cin>>end>>capacity>>count;
            std::vector<dtPolyRef> path(std::max(std::max(capacity,count),1));
            for(int i=0;i<count;++i){unsigned long long ref;std::cin>>ref;path[i]=(dtPolyRef)ref;}
            dtStatus s=persistent.getPathFromDijkstraSearch((dtPolyRef)end,path.data(),&count,capacity);
            std::cout<<'['<<s<<",[";for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<path[i];}std::cout<<"]]";
        } else if(op=="M") {
            CallbackFilter filter;unsigned long long start;int capacity;std::cin>>start>>capacity;
            float a[3],b[3],position[3]={-31,17,3};point(a);point(b);
            std::vector<dtPolyRef> visited(std::max(capacity,1));int count=0;
            dtStatus s=persistent.moveAlongSurface((dtPolyRef)start,a,b,&filter,position,visited.data(),&count,capacity);
            std::cout<<'['<<s;pointBits(position);std::cout<<",[";
            for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<visited[i];}
            std::cout<<"],";filter.outputLog();std::cout<<']';
        } else if(op=="b") {
            slicedFilter.reset(new CallbackFilter());unsigned long long start,end;unsigned options;
            std::cin>>start>>end>>options;float a[3],b[3];point(a);point(b);
            dtStatus s=persistent.initSlicedFindPath((dtPolyRef)start,(dtPolyRef)end,a,b,slicedFilter.get(),options);
            std::cout<<'['<<s<<',';slicedFilter->outputLog();std::cout<<']';
        } else if(op=="u") {
            int iterations,hasDone,done=-17;std::cin>>iterations>>hasDone;slicedFilter->log.clear();
            dtStatus s=persistent.updateSlicedFindPath(iterations,hasDone?&done:0);
            std::cout<<'['<<s<<','<<done<<',';slicedFilter->outputLog();std::cout<<']';
        } else if(op=="f" || op=="p") {
            int capacity;std::cin>>capacity;std::vector<dtPolyRef> existing;
            if(op=="p"){int n;std::cin>>n;read(existing,n);}
            // One initialized spare slot makes native's exact-ray-budget final
            // append observable without accessing beyond allocated memory.
            std::vector<dtPolyRef> path(std::max(capacity,1)+1);int count=0;slicedFilter->log.clear();
            dtStatus s=op=="f"?persistent.finalizeSlicedFindPath(path.data(),&count,capacity)
                :persistent.finalizeSlicedFindPathPartial(existing.data(),(int)existing.size(),path.data(),&count,capacity);
            std::cout<<'['<<s<<",[";for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<path[i];}
            std::cout<<"],";slicedFilter->outputLog();std::cout<<']';
        } else if(op=="e") {
            CallbackFilter updated;*slicedFilter=updated;std::cout<<"[]";
        } else if(op=="n") {
            float pos[3];point(pos);dtNodePool* pool=persistent.getNodePool();pool->clear();
            for(int i=0;i<pool->getMaxNodes();++i){dtNode* node=pool->getNode((dtPolyRef)(i+1));std::memcpy(node->pos,pos,12);}
            pool->clear();std::cout<<"[]";
        } else if(op=="Y" || op=="Z") {
            CallbackFilter filter;unsigned long long start,previous=0;unsigned options=0;int capacity;
            std::cin>>start;if(op=="Y")std::cin>>previous>>options;std::cin>>capacity;
            float a[3],b[3];point(a);point(b);std::vector<dtPolyRef> path(std::max(capacity,1));
            if(op=="Y") {
                dtRaycastHit hit={};hit.t=-9;hit.hitNormal[0]=-0.f;hit.hitNormal[1]=42;hit.hitNormal[2]=-3;
                hit.hitEdgeIndex=-77;hit.pathCost=-13;hit.path=path.data();hit.maxPath=capacity;
                dtStatus s=persistent.raycast((dtPolyRef)start,a,b,&filter,options,&hit,(dtPolyRef)previous);
                unsigned t,cost;std::memcpy(&t,&hit.t,4);std::memcpy(&cost,&hit.pathCost,4);
                std::cout<<'['<<s<<','<<t;pointBits(hit.hitNormal);std::cout<<','<<hit.hitEdgeIndex<<','<<cost<<",[";
                for(int i=0;i<hit.pathCount;++i){if(i)std::cout<<',';std::cout<<path[i];}std::cout<<"],";filter.outputLog();std::cout<<']';
            } else {
                float t,normal[3];int count=0;dtStatus s=persistent.raycast((dtPolyRef)start,a,b,&filter,&t,normal,path.data(),&count,capacity);
                unsigned bits;std::memcpy(&bits,&t,4);std::cout<<'['<<s<<','<<bits;pointBits(normal);std::cout<<",[";
                for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<path[i];}std::cout<<"],";filter.outputLog();std::cout<<']';
            }
        } else if(op=="I") {
            int n;std::cin>>n;dtStatus s=persistent.init(&mesh,n);
            std::cout<<'['<<s<<','<<persistent.getNodePool()->getMaxNodes()<<','<<persistent.getNodePool()->getHashSize()<<','<<persistent.getNodePool()->getNodeCount()<<']';
        } else if(op=="O") {std::cout<<'[';nodePoolContents(*persistent.getNodePool());std::cout<<']';}
        else if(op=="L") {
            int n;std::cin>>n;std::cout<<'[';
            for(int i=0;i<n;++i) {unsigned long long r;std::cin>>r;if(i)std::cout<<',';std::cout<<persistent.isInClosedList((dtPolyRef)r);}
            std::cout<<']';
        } else if(op=="D" || op=="d") {
            dtQueryFilter filter;
            if(op=="d") {
                unsigned include,exclude;std::cin>>include>>exclude;
                filter.setIncludeFlags((unsigned short)include);filter.setExcludeFlags((unsigned short)exclude);
                for(int i=0;i<64;++i){unsigned bits;float cost;std::cin>>bits;std::memcpy(&cost,&bits,4);filter.setAreaCost(i,cost);}
            }
            unsigned long long a,b;std::cin>>a>>b;float start[3],end[3];point(start);point(end);int capacity;std::cin>>capacity;
            std::vector<dtPolyRef> path(std::max(capacity,1));int count=0;
            dtStatus s=persistent.findPath((dtPolyRef)a,(dtPolyRef)b,start,end,&filter,path.data(),&count,capacity);
            std::cout<<'['<<s<<",[";for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<path[i];}std::cout<<"]]";
        } else if(op=="W") {
            CallbackFilter filter;unsigned long long ref;std::cin>>ref;
            std::cout<<'['<<persistent.isValidPolyRef((dtPolyRef)ref,&filter)<<',';filter.outputLog();std::cout<<']';
        } else if(op=="header_counts") {
            unsigned long long ref;std::cin>>ref;
            const dtMeshTile* tile=mesh.getTileByRef((dtTileRef)ref);
            for(int* count:{&tile->header->polyCount,&tile->header->vertCount,
                &tile->header->maxLinkCount,&tile->header->detailMeshCount,
                &tile->header->detailVertCount,&tile->header->detailTriCount,
                &tile->header->bvNodeCount,&tile->header->offMeshConCount}) {
                std::uint32_t word;std::cin>>word;std::memcpy(count,&word,sizeof(word));
            }
            bytes(tile->data,actualSizes[tile->data]);
        } else if(op=="polygon_record") {
            unsigned long long ref; unsigned index,word;
            std::cin>>ref>>index;
            const dtMeshTile* tile=mesh.getTileByRef((dtTileRef)ref);
            dtPoly& record=tile->polys[index];
            std::cin>>record.firstLink;
            for(int i=0;i<DT_VERTS_PER_POLYGON;++i){std::cin>>word;record.verts[i]=(unsigned short)word;}
            for(int i=0;i<DT_VERTS_PER_POLYGON;++i){std::cin>>word;record.neis[i]=(unsigned short)word;}
            std::cin>>word;record.flags=(unsigned short)word;
            std::cin>>word;record.vertCount=(unsigned char)word;
            std::cin>>word;record.areaAndtype=(unsigned char)word;
            bytes(tile->data,actualSizes[tile->data]);
        } else if(op=="vertex_record" || op=="detail_vertex_record") {
            unsigned long long ref; unsigned index;std::cin>>ref>>index;
            const dtMeshTile* tile=mesh.getTileByRef((dtTileRef)ref);
            point((op=="vertex_record" ? tile->verts : tile->detailVerts)+index*3);
            bytes(tile->data,actualSizes[tile->data]);
        } else if(op=="detail_record") {
            unsigned long long ref; unsigned index,vbase,tbase,nv,nt;
            std::cin>>ref>>index>>vbase>>tbase>>nv>>nt;
            const dtMeshTile* tile=mesh.getTileByRef((dtTileRef)ref);
            dtPolyDetail& record=tile->detailMeshes[index];
            record.vertBase=vbase;record.triBase=tbase;record.vertCount=(unsigned char)nv;record.triCount=(unsigned char)nt;
            bytes(tile->data,actualSizes[tile->data]);
        } else if(op=="off_mesh_record") {
            unsigned long long ref;unsigned index,radius,poly,flags,side,user;
            std::cin>>ref>>index;
            const dtMeshTile* tile=mesh.getTileByRef((dtTileRef)ref);
            dtOffMeshConnection& record=tile->offMeshCons[index];
            point(record.pos);point(record.pos+3);
            std::cin>>radius>>poly>>flags>>side>>user;
            std::memcpy(&record.rad,&radius,4);record.poly=(unsigned short)poly;
            record.flags=(unsigned char)flags;record.side=(unsigned char)side;record.userId=user;
            bytes(tile->data,actualSizes[tile->data]);
        } else if(op=="triangle_record") {
            unsigned long long ref; unsigned index, a, b, c, flags;
            std::cin>>ref>>index>>a>>b>>c>>flags;
            const dtMeshTile* tile=mesh.getTileByRef((dtTileRef)ref);
            unsigned char* record=tile->detailTris+index*4;
            record[0]=(unsigned char)a;record[1]=(unsigned char)b;
            record[2]=(unsigned char)c;record[3]=(unsigned char)flags;
            bytes(tile->data,actualSizes[tile->data]);
        } else if(op=="bv_record") {
            unsigned long long ref; unsigned index, lo[3], hi[3]; int value;
            std::cin>>ref>>index;
            for(unsigned& x:lo)std::cin>>x;
            for(unsigned& x:hi)std::cin>>x;
            std::cin>>value;
            const dtMeshTile* tile=mesh.getTileByRef((dtTileRef)ref);
            dtBVNode& node=tile->bvTree[index];
            for(int i=0;i<3;++i){node.bmin[i]=(unsigned short)lo[i];node.bmax[i]=(unsigned short)hi[i];}
            node.i=value;
            bytes(tile->data,actualSizes[tile->data]);
        } else if(op=="link_record") {
            unsigned long long ref, value; unsigned index, next, edge, side, lo, hi;
            std::cin>>ref>>index>>value>>next>>edge>>side>>lo>>hi;
            const dtMeshTile* tile=mesh.getTileByRef((dtTileRef)ref);
            dtLink& link=tile->links[index];
            link.ref=(dtPolyRef)value;link.next=next;link.edge=(unsigned char)edge;
            link.side=(unsigned char)side;link.bmin=(unsigned char)lo;link.bmax=(unsigned char)hi;
            bytes(tile->data,actualSizes[tile->data]);
        } else if(op=="A") {
            unsigned long long last;int n;std::cin>>last>>n;
            buffers.emplace_back();read(buffers.back(),n);actualSizes[buffers.back().data()]=n;
            dtTileRef ref=0;dtStatus s=mesh.addTile(buffers.back().data(),n,0,(dtTileRef)last,&ref);
            std::cout<<'['<<s<<','<<ref<<']';
        } else if(op=="k") {
            int flags,size,n;unsigned long long last;std::cin>>flags>>size>>last>>n;std::vector<unsigned char> data;read(data,n);unsigned char* bytes;
            if(flags & DT_TILE_FREE_DATA){bytes=(unsigned char*)dtAlloc(n,DT_ALLOC_PERM);std::memcpy(bytes,data.data(),n);}else{buffers.push_back(std::move(data));bytes=buffers.back().data();}
            dtTileRef ref=0;dtStatus st=mesh.addTile(bytes,size,flags,(dtTileRef)last,&ref);if(dtStatusFailed(st) && (flags & DT_TILE_FREE_DATA))dtFree(bytes);if(dtStatusSucceed(st))actualSizes[bytes]=n;std::cout<<'['<<st<<','<<ref<<']';
        } else if(op=="o") {
            unsigned long long ref;std::cin>>ref;const auto* tile=mesh.getTileByRef((dtTileRef)ref);int n=tile && tile->header ? actualSizes[tile->data] : 0;
            unsigned char sentinel[3]={0xa5,0xa6,0xa7};unsigned char* data=sentinel;int size=-17;dtStatus st=mesh.removeTile((dtTileRef)ref,&data,&size);
            std::cout<<'['<<st<<','<<(data?1:0)<<','<<size<<",[";if(data){int count=dtStatusFailed(st)?3:n;for(int j=0;j<count;++j){if(j)std::cout<<',';std::cout<<(unsigned)data[j];}}std::cout<<"]]";
        } else if(op=="h") {
            const dtNavMesh& view=mesh;std::cout<<'[';for(int i=0;i<mesh.getMaxTiles();++i){if(i)std::cout<<',';const auto* tile=view.getTile(i);std::cout<<'['<<i<<','<<tile->salt<<','<<(tile->header?1:0)<<','<<tile->flags<<','<<tile->dataSize<<','<<tile->linksFreeList<<',';if(tile->data){std::cout<<'[';int n=actualSizes[tile->data];for(int j=0;j<n;++j){if(j)std::cout<<',';std::cout<<(unsigned)tile->data[j];}std::cout<<']';}else std::cout<<"null";std::cout<<']';}std::cout<<']';
        } else if(op=="R") {
            unsigned long long r;std::cin>>r;unsigned char* data=0;int n=0;
            dtStatus s=mesh.removeTile((dtTileRef)r,&data,&n);
            std::cout<<'['<<s<<',';bytes(data,n);std::cout<<']';
        } else if(op=="S") {
            std::cout<<'[';
            for(int i=0;i<mesh.getMaxTiles();++i) {
                if(i)std::cout<<',';const dtMeshTile* t=static_cast<const dtNavMesh&>(mesh).getTile(i);
                if(!t->header)std::cout<<"null";
                else {std::cout<<'['<<mesh.getTileRef(t)<<','<<t->linksFreeList<<',';bytes(t->data,t->dataSize);std::cout<<']';}
            }std::cout<<']';
        } else if(op=="V" || op=="Q") {
            CallbackFilter filter;float pos[3],ext[3];point(pos);point(ext);dtNavMeshQuery q;q.init(&mesh,64);
            if(op=="V") {
                RecordingQuery query(filter);dtStatus s=q.queryPolygons(pos,ext,&filter,&query);
                std::cout<<'['<<s<<',';filter.outputLog();std::cout<<']';
            } else {
                int capacity;std::cin>>capacity;std::vector<dtPolyRef> refs(std::max(capacity,1));int count=0;
                dtStatus s=q.queryPolygons(pos,ext,&filter,refs.data(),&count,capacity);
                std::cout<<'['<<s<<",[";for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<refs[i];}
                std::cout<<"],";filter.outputLog();std::cout<<']';
            }
        } else if(op=="J") {
            CallbackFilter filter;float pos[3],ext[3],near[3]={};point(pos);point(ext);
            dtNavMeshQuery q;q.init(&mesh,64);dtPolyRef ref=0;bool over=false;
            dtStatus s=q.findNearestPoly(pos,ext,&filter,&ref,near,&over);
            std::cout<<'['<<s<<','<<ref;pointBits(near);std::cout<<','<<over<<',';filter.outputLog();std::cout<<']';
        } else if(op=="K") {
            CallbackFilter filter;unsigned long long a,b;std::cin>>a>>b;float start[3],end[3];point(start);point(end);int capacity;std::cin>>capacity;
            dtNavMeshQuery q;q.init(&mesh,64);std::vector<dtPolyRef> path(capacity);int count=0;
            dtStatus s=q.findPath((dtPolyRef)a,(dtPolyRef)b,start,end,&filter,path.data(),&count,capacity);
            std::cout<<'['<<s<<",[";for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<path[i];}std::cout<<"],";filter.outputLog();std::cout<<",[";nodePoolContents(*q.getNodePool());std::cout<<"]]";
        } else if(op=="N") {
            float pos[3],ext[3],near[3]={};point(pos);point(ext);
            unsigned include,exclude;std::cin>>include>>exclude;
            dtQueryFilter filter;filter.setIncludeFlags((unsigned short)include);filter.setExcludeFlags((unsigned short)exclude);
            dtNavMeshQuery q;q.init(&mesh,64);dtPolyRef ref=0;
            dtStatus s=q.findNearestPoly(pos,ext,&filter,&ref,near);
            std::cout<<'['<<s<<','<<ref;pointBits(near);std::cout<<']';
        } else if(op=="C") {
            unsigned long long r;std::cin>>r;float pos[3];point(pos);
            float cp[3]={-0.f,42.f,-3.f},bp[3]={-0.f,42.f,-3.f},height=-123.f;bool over=false;
            dtNavMeshQuery q;q.init(&mesh,64);
            dtStatus cs=q.closestPointOnPoly((dtPolyRef)r,pos,cp,&over);
            dtStatus bs=q.closestPointOnPolyBoundary((dtPolyRef)r,pos,bp);
            dtStatus hs=q.getPolyHeight((dtPolyRef)r,pos,&height);unsigned bits;std::memcpy(&bits,&height,4);
            std::cout<<'['<<cs<<','<<over;pointBits(cp);std::cout<<','<<bs;pointBits(bp);std::cout<<','<<hs<<','<<bits<<']';
        } else if(op=="H") {
            float start[3],end[3];point(start);point(end);int capacity,options,n;std::cin>>capacity>>options>>n;
            std::vector<dtPolyRef> path;read(path,n);
            // Partial-path recovery can append a final vertex after portal
            // insertion fills the advertised capacity. Retain a spare slot.
            int allocated=std::max(capacity,1)+1;
            std::vector<float> pts(allocated*3);std::vector<unsigned char> flags(allocated);std::vector<dtPolyRef> refs(allocated);int count=0;
            dtNavMeshQuery q;q.init(&mesh,64);
            dtStatus s=q.findStraightPath(start,end,path.data(),n,pts.data(),flags.data(),refs.data(),&count,capacity,options);
            std::cout<<'['<<s<<",[";
            for(int i=0;i<count;++i){if(i)std::cout<<',';std::cout<<'['<<(unsigned)flags[i]<<','<<refs[i];pointBits(&pts[i*3]);std::cout<<']';}
            std::cout<<"]]";
        } else if(op=="P") {
            unsigned long long a,b;std::cin>>a>>b;float start[3],end[3];point(start);point(end);
            int nodes,capacity,corners;std::cin>>nodes>>capacity>>corners;
            dtNavMeshQuery q;dtStatus init=q.init(&mesh,nodes);dtQueryFilter filter;
            std::vector<dtPolyRef> path(capacity);int count=0;
            dtStatus s=q.findPath((dtPolyRef)a,(dtPolyRef)b,start,end,&filter,path.data(),&count,capacity);
            std::cout<<'['<<init<<','<<s<<",[";
            for(int i=0;i<count;++i) {if(i)std::cout<<',';std::cout<<path[i];}std::cout<<"],";
            std::vector<float> pts(corners*3);std::vector<unsigned char> flags(corners);std::vector<dtPolyRef> refs(corners);int nc=0;
            s=q.findStraightPath(start,end,path.data(),count,pts.data(),flags.data(),refs.data(),&nc,corners);
            std::cout<<s<<",[";
            for(int i=0;i<nc;++i) {if(i)std::cout<<',';std::cout<<'['<<(unsigned)flags[i]<<','<<refs[i];pointBits(&pts[i*3]);std::cout<<']';}
            std::cout<<"]]";
        } else if(op=="E") {
            unsigned long long prev,r;std::cin>>prev>>r;float a[3]={},b[3]={};
            dtStatus s=mesh.getOffMeshConnectionPolyEndPoints((dtPolyRef)prev,(dtPolyRef)r,a,b);
            std::cout<<'['<<s;pointBits(a);pointBits(b);std::cout<<',';
            const dtOffMeshConnection* con=mesh.getOffMeshConnectionByRef((dtPolyRef)r);
            if(con) std::cout<<'['<<con->poly<<','<<(unsigned)con->flags<<','<<(unsigned)con->side<<','<<con->userId<<']';
            else std::cout<<"null";
            std::cout<<']';
        } else if(op=="F") {
            unsigned long long r;unsigned flags,area;std::cin>>r>>flags>>area;
            dtStatus fs=mesh.setPolyFlags((dtPolyRef)r,(unsigned short)flags),as=mesh.setPolyArea((dtPolyRef)r,(unsigned char)area);
            unsigned short f=0;unsigned char a=0;dtStatus fg=mesh.getPolyFlags((dtPolyRef)r,&f),ag=mesh.getPolyArea((dtPolyRef)r,&a);
            std::cout<<'['<<fs<<','<<as<<','<<fg<<','<<f<<','<<ag<<','<<(unsigned)a<<']';
        } else if(op=="T") {
            unsigned long long r;int n;std::cin>>r>>n;const dtMeshTile* t=mesh.getTileByRef((dtTileRef)r);
            std::vector<unsigned char> out(n,0xa5);
            std::cout<<'['<<mesh.getTileStateSize(t)<<','<<mesh.storeTileState(t,out.data(),n)<<',';bytes(out.data(),n);std::cout<<']';
        } else if(op=="U") {
            unsigned long long r;int n;std::cin>>r>>n;std::vector<unsigned char> in;read(in,n);
            dtMeshTile* t=const_cast<dtMeshTile*>(mesh.getTileByRef((dtTileRef)r));
            std::cout<<'['<<mesh.restoreTileState(t,in.data(),n)<<']';
        } else return;
    }
    std::cout<<']';
}
static void initialization() {
    int count;std::cin>>count;std::cout<<'[';
    for(int i=0;i<count;++i) {
        dtNavMeshParams p={};auto value=[](float& v){unsigned bits;std::cin>>bits;std::memcpy(&v,&bits,4);};
        for(float& v:p.orig)value(v);value(p.tileWidth);value(p.tileHeight);std::cin>>p.maxTiles>>p.maxPolys;
        // This fixture tests bit-budget metadata, not large allocations. Native
        // init allocates only maxTiles records and its small tile lookup table.
        if(p.maxTiles<0 || p.maxTiles>8)std::exit(5);
        dtNavMesh mesh;dtStatus status=mesh.init(&p);if(i)std::cout<<',';std::cout<<'['<<status;
        if(dtStatusSucceed(status)) {
            const auto* stored=mesh.getParams();pointBits(stored->orig);
            for(float v:{stored->tileWidth,stored->tileHeight}){unsigned bits;std::memcpy(&bits,&v,4);std::cout<<','<<bits;}
            std::cout<<','<<stored->maxTiles<<','<<stored->maxPolys<<",[";
            const dtNavMesh& view=mesh;for(int j=0;j<mesh.getMaxTiles();++j){if(j)std::cout<<',';const auto* t=view.getTile(j);std::cout<<'['<<t->salt<<','<<t->flags<<','<<t->dataSize<<','<<(t->header?1:0)<<']';}std::cout<<"],";
#ifdef DT_POLYREF64
            const unsigned bits=DT_TILE_BITS+DT_POLY_BITS;
#else
            const unsigned bits=dtIlog2(dtNextPow2(p.maxTiles))+dtIlog2(dtNextPow2(p.maxPolys));
#endif
            if(bits<sizeof(dtPolyRef)*8) {
                std::cout<<'[';for(unsigned word:{0u,1u,0xffffffffu}){if(word)std::cout<<',';const auto ref=mesh.encodePolyId(word,word,word);unsigned s,t,p;mesh.decodePolyId(ref,s,t,p);std::cout<<'['<<ref<<','<<s<<','<<t<<','<<p<<']';}std::cout<<']';
            } else std::cout<<"null";
        }
        std::cout<<']';
    }
    std::cout<<"]\n";
}

int main() {
    std::string mode; std::cin >> mode;
    if (mode == "init") { initialization(); }
    else if (mode == "runtime") { runtime(); }
    else if (mode == "helpers") {
        int count;std::cin>>count;std::cout<<'[';
        for(int i=0;i<count;++i){unsigned flags,status,detail;int index;std::cin>>flags>>index>>status>>detail;
            if(i)std::cout<<',';std::cout<<'['<<dtGetDetailTriEdgeFlags((unsigned char)flags,index)<<','
                <<dtStatusSucceed(status)<<','<<dtStatusFailed(status)<<','<<dtStatusInProgress(status)<<','<<dtStatusDetail(status,detail)<<']';
        }
        std::cout<<"]\n";
    }
    else if (mode == "status") {
        const unsigned int codes[] = { DT_SUCCESS, DT_FAILURE, DT_IN_PROGRESS, DT_STATUS_DETAIL_MASK,
            DT_WRONG_MAGIC, DT_WRONG_VERSION, DT_OUT_OF_MEMORY, DT_INVALID_PARAM, DT_BUFFER_TOO_SMALL,
            DT_OUT_OF_NODES, DT_PARTIAL_RESULT, DT_ALREADY_OCCUPIED };
        std::cout << '[';
        for (int i = 0; i < 12; ++i) { if (i) std::cout << ','; std::cout << codes[i]; }
        std::cout << ']';
    } else if (mode == "create" || mode == "create_bits") {
        floatBits=mode=="create_bits";
        dtNavMeshCreateParams p = {};
        int details, ids, bv;
        std::cin >> p.vertCount >> p.polyCount >> p.nvp >> details
            >> p.detailVertsCount >> p.detailTriCount >> p.offMeshConCount >> ids
            >> p.tileX >> p.tileY >> p.tileLayer >> p.userId;
        for(float* value:{&p.walkableHeight,&p.walkableRadius,&p.walkableClimb,&p.cs,&p.ch})inputFloat(*value);
        std::cin>>bv;
        p.buildBvTree = bv;
        for (int i = 0; i < 3; ++i) inputFloat(p.bmin[i]);
        for (int i = 0; i < 3; ++i) inputFloat(p.bmax[i]);
        std::vector<unsigned short> verts, polys, flags, offFlags;
        std::vector<unsigned char> areas, tris, offAreas, offDirs;
        std::vector<unsigned int> meshes, offIds;
        std::vector<float> dv, offVerts, offRadius;
        read(verts, p.vertCount * 3); read(polys, p.polyCount * p.nvp * 2);
        read(flags, p.polyCount); read(areas, p.polyCount);
        if (details) read(meshes, p.polyCount * 4);
        floats(dv, p.detailVertsCount * 3); read(tris, p.detailTriCount * 4);
        floats(offVerts, p.offMeshConCount * 6); floats(offRadius, p.offMeshConCount);
        read(offFlags, p.offMeshConCount); read(offAreas, p.offMeshConCount); read(offDirs, p.offMeshConCount);
        if (ids) read(offIds, p.offMeshConCount);
        // A zero-width polygon array still has caller-owned non-null backing.
        if(polys.empty())polys.push_back(0);
        p.verts = verts.data(); p.polys = polys.data(); p.polyFlags = flags.data(); p.polyAreas = areas.data();
        p.detailMeshes = details ? meshes.data() : 0; p.detailVerts = dv.data(); p.detailTris = tris.data();
        p.offMeshConVerts = offVerts.data(); p.offMeshConRad = offRadius.data(); p.offMeshConFlags = offFlags.data();
        p.offMeshConAreas = offAreas.data(); p.offMeshConDir = offDirs.data(); p.offMeshConUserID = ids ? offIds.data() : 0;
        unsigned char* data = 0; int size = 0;
        bool ok = dtCreateNavMeshData(&p, &data, &size);
        std::cout << "{\"ok\":" << (ok ? "true" : "false") << ",\"bytes\":";
        bytes(data, size); std::cout << '}'; dtFree(data);
    } else {
        int n; std::cin >> n;
        std::vector<unsigned char> data; read(data, n);
        if(mode=="load_flags") {
            int flags,size;std::cin>>flags>>size;unsigned char* owned=0;unsigned char* ptr=data.data();if(flags & DT_TILE_FREE_DATA){owned=(unsigned char*)dtAlloc(n,DT_ALLOC_PERM);std::memcpy(owned,ptr,n);ptr=owned;}
            dtNavMesh mesh;dtStatus status=mesh.init(ptr,size,flags);const auto* t=((const dtNavMesh&)mesh).getTile(0);const auto* p=mesh.getParams();
            std::cout<<'['<<status<<','<<t->flags<<','<<t->dataSize<<','<<mesh.getPolyRefBase(t);pointBits(p->orig);float dims[]={p->tileWidth,p->tileHeight};for(float v:dims){unsigned bits;std::memcpy(&bits,&v,4);std::cout<<','<<bits;}std::cout<<','<<p->maxTiles<<','<<p->maxPolys<<',';bytes(ptr,n);std::cout<<']';
            if(dtStatusFailed(status) && owned)dtFree(owned);
        } else         if (mode == "load") {
            dtNavMesh mesh;
            dtStatus status = mesh.init(data.data(), n, 0);
            std::cout << "{\"status\":" << status << ",\"bytes\":";
            bytes(data.data(), n); std::cout << '}';
        } else {
            bool ok = mode == "header" ? dtNavMeshHeaderSwapEndian(data.data(), n)
                : dtNavMeshDataSwapEndian(data.data(), n);
            std::cout << "{\"ok\":" << (ok ? "true" : "false") << ",\"bytes\":";
            bytes(data.data(), n); std::cout << '}';
        }
    }
}
