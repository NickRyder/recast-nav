// Live test-only oracle using unmodified C++ Recast and Detour, including dtTileCache.
#include "Recast.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMeshBuilder.h"
#include <iostream>
#include <vector>
#include <map>
#include <cstring>
#include <cstdint>
#include <stdexcept>
#include <cmath>
#include <algorithm>
using Values=std::vector<int64_t>;
static int64_t bits(float f) {uint32_t u;std::memcpy(&u,&f,4);return u;}
static void require(bool ok,const char* label) {if(!ok)throw std::runtime_error(label);}
struct Compressor:dtTileCacheCompressor {
    int maxCompressedSize(int n) override {return n;}
    dtStatus compress(const unsigned char* in,int n,unsigned char* out,int cap,int* size) override {if(n>cap)return DT_FAILURE|DT_BUFFER_TOO_SMALL;std::memcpy(out,in,n);*size=n;return DT_SUCCESS;}
    dtStatus decompress(const unsigned char* in,int n,unsigned char* out,int cap,int* size) override {return compress(in,n,out,cap,size);}
};
struct Process:dtTileCacheMeshProcess {void process(dtNavMeshCreateParams* p,unsigned char*,unsigned short* flags) override {for(int i=0;i<p->polyCount;++i)flags[i]=1;}};
static void point(Values& out,const float* p) {for(int i=0;i<3;++i)out.push_back(bits(p[i]));}
struct Key {dtObstacleRef ref;float min[3],max[3];};
int main() {
    try {
        float orig[3],maxy,cs,ch,ah,ar,ac,slope;int txs,tzs,vox,maxTiles,maxPolys,maxObs,workers,nv,nt;
        for(float& v:orig)std::cin>>v;
        std::cin>>maxy>>txs>>tzs>>vox>>cs>>ch>>ah>>ar>>ac>>slope>>maxTiles>>maxPolys>>maxObs>>workers>>nv>>nt;
        std::vector<float> vertices(nv*3);std::vector<int> triangles(nt*3);
        for(auto& v:vertices)std::cin>>v;for(auto& v:triangles)std::cin>>v;
        require(!std::cin.fail(),"geometry input");
        dtTileCacheAlloc alloc;Compressor compressor;Process process;dtTileCache cache;dtNavMesh nav;dtNavMeshQuery query;
        dtTileCacheParams cp={};std::memcpy(cp.orig,orig,12);cp.cs=cs;cp.ch=ch;cp.width=cp.height=vox;cp.walkableHeight=ah;cp.walkableRadius=ar;cp.walkableClimb=ac;cp.maxSimplificationError=0;cp.maxTiles=maxTiles;cp.maxObstacles=maxObs;
        require(!dtStatusFailed(cache.init(&cp,&alloc,&compressor,&process)),"cache init");
        dtNavMeshParams np={};std::memcpy(np.orig,orig,12);np.tileWidth=np.tileHeight=vox*cs;np.maxTiles=maxTiles;np.maxPolys=maxPolys;
        require(!dtStatusFailed(nav.init(&np)),"nav init");
        const int height=int(std::ceil(ah/ch)),climb=int(std::ceil(ac/ch)),radius=int(std::ceil(ar/cs)),border=radius+3;
        for(int z=0;z<tzs;++z)for(int x=0;x<txs;++x) {
            float lo[]={orig[0]+x*np.tileWidth-border*cs,orig[1],orig[2]+z*np.tileHeight-border*cs};
            float hi[]={orig[0]+x*np.tileWidth+np.tileWidth+border*cs,maxy,orig[2]+z*np.tileHeight+np.tileHeight+border*cs};
            rcContext ctx;rcHeightfield solid;
            require(rcCreateHeightfield(&ctx,solid,vox+border*2,vox+border*2,lo,hi,cs,ch),"heightfield");
            std::vector<unsigned char> areas(nt,0);rcMarkWalkableTriangles(&ctx,slope,vertices.data(),nv,triangles.data(),nt,areas.data());
            require(rcRasterizeTriangles(&ctx,vertices.data(),nv,triangles.data(),areas.data(),nt,solid,climb),"rasterize");
            rcFilterLowHangingWalkableObstacles(&ctx,climb,solid);rcFilterLedgeSpans(&ctx,height,climb,solid);rcFilterWalkableLowHeightSpans(&ctx,height,solid);
            rcCompactHeightfield compact;require(rcBuildCompactHeightfield(&ctx,height,climb,solid,compact),"compact");require(rcErodeWalkableArea(&ctx,radius,compact),"erode");
            rcHeightfieldLayerSet layers;require(rcBuildHeightfieldLayers(&ctx,compact,border,height,layers),"layers");
            for(int id=0;id<layers.nlayers;++id) {
                const auto& l=layers.layers[id];dtTileCacheLayerHeader h={};h.magic=DT_TILECACHE_MAGIC;h.version=DT_TILECACHE_VERSION;h.tx=x;h.ty=z;h.tlayer=id;
                std::memcpy(h.bmin,l.bmin,12);std::memcpy(h.bmax,l.bmax,12);h.hmin=l.hmin;h.hmax=l.hmax;h.width=l.width;h.height=l.height;h.minx=l.minx;h.maxx=l.maxx;h.miny=l.miny;h.maxy=l.maxy;
                unsigned char* data=nullptr;int size=0;require(!dtStatusFailed(dtBuildTileCacheLayer(&compressor,&h,l.heights,l.areas,l.cons,&data,&size)),"layer bytes");
                require(!dtStatusFailed(cache.addTile(data,size,DT_COMPRESSEDTILE_FREE_DATA,nullptr)),"cache add tile");require(!dtStatusFailed(cache.buildNavMeshTilesAt(x,z,&nav)),"build nav tiles");
            }
        }
        require(!dtStatusFailed(query.init(&nav,4096)),"query init");
        std::map<uint64_t,Key> keys;uint64_t requested=0,revision=0;
        std::vector<Values> results;
        std::string op;
        while(std::cin>>op) {
            Values out;
            if(op=="S") {
                out.push_back(maxTiles);
                for(int i=0;i<maxTiles;++i) {
                    const dtMeshTile* tile=static_cast<const dtNavMesh&>(nav).getTile(i);out.push_back(tile->header?1:0);
                    if(!tile->header)continue;
                    const auto& h=*tile->header;out.insert(out.end(),{static_cast<int64_t>(nav.getPolyRefBase(tile)),h.x,h.y,h.layer,h.vertCount,h.polyCount});
                    out.insert(out.end(),{bits(h.walkableHeight),bits(h.walkableRadius),bits(h.walkableClimb)});
                    for(int j=0;j<h.vertCount*3;++j)out.push_back(bits(tile->verts[j]));
                    for(int j=0;j<h.polyCount;++j) {
                        const auto& p=tile->polys[j];out.insert(out.end(),{p.vertCount,p.flags,p.getArea()});
                        for(int k=0;k<p.vertCount;++k)out.push_back(p.verts[k]);for(int k=0;k<p.vertCount;++k)out.push_back(p.neis[k]);
                        int links=0;for(unsigned int k=p.firstLink;k!=DT_NULL_LINK;k=tile->links[k].next)++links;out.push_back(links);
                        for(unsigned int k=p.firstLink;k!=DT_NULL_LINK;k=tile->links[k].next) {const auto& l=tile->links[k];out.insert(out.end(),{static_cast<int64_t>(l.ref),l.edge,l.side,l.bmin,l.bmax});}
                    }
                }
                out.push_back(maxObs);
                for(int i=0;i<maxObs;++i) {const auto& ob=*cache.getObstacle(i);out.insert(out.end(),{ob.salt,ob.state,ob.ntouched,ob.npending});for(int j=0;j<ob.ntouched;++j)out.push_back(ob.touched[j]);for(int j=0;j<ob.npending;++j)out.push_back(ob.pending[j]);}
                out.push_back(keys.size());out.push_back(requested);
            } else if(op=="A" || op=="B") {
                uint64_t key=0;if(op=="A")std::cin>>key;Key value={};for(float& v:value.min)std::cin>>v;for(float& v:value.max)std::cin>>v;
                auto existing=keys.find(key);if(op=="A" && existing!=keys.end() && std::equal(value.min,value.min+3,existing->second.min) && std::equal(value.max,value.max+3,existing->second.max)) {out={DT_SUCCESS,existing->second.ref,0};}
                else {dtStatus status=DT_SUCCESS;if(op=="A" && existing!=keys.end())status=cache.removeObstacle(existing->second.ref);if(!dtStatusFailed(status))status=cache.addBoxObstacle(value.min,value.max,&value.ref);if(!dtStatusFailed(status) && op=="A") {keys[key]=value;++requested;}out={status,value.ref,1};}
            } else if(op=="D") {
                uint64_t key;std::cin>>key;auto existing=keys.find(key);dtStatus status=DT_SUCCESS;bool changed=existing!=keys.end();if(changed) {status=cache.removeObstacle(existing->second.ref);if(!dtStatusFailed(status)){keys.erase(existing);++requested;}}out={status,changed};
            } else if(op=="R") {dtObstacleRef ref;std::cin>>ref;out={cache.removeObstacle(ref)};
            } else if(op=="U" || op=="L") {
                int limit=100000;if(op=="L")std::cin>>limit;int calls=0;bool done=false;dtStatus status=DT_SUCCESS;
                while(calls<limit && !done) {auto before=nav.getMaxTiles();(void)before;status=cache.update(1,&nav,&done);++calls;if(dtStatusFailed(status))break;}
                // Count actual tile replacements using the nav snapshot below; revision is a Rust publication counter, not C++ API.
                out={status,calls,done};
            } else if(op=="P") {
                float p[3],ext[3],nearest[3]={};for(float& v:p)std::cin>>v;for(float& v:ext)std::cin>>v;dtQueryFilter filter;filter.setIncludeFlags(1);dtPolyRef ref=0;auto status=query.findNearestPoly(p,ext,&filter,&ref,nearest);out={status,static_cast<int64_t>(ref)};point(out,nearest);
            } else if(op=="V") {dtPolyRef ref;std::cin>>ref;out={nav.isValidPolyRef(ref)};
            } else if(op=="Q" || op=="H" || op=="G") {
                uint64_t key=0;int partial=0,nodes=4096,maxpath=256,maxstraight=256;
                if(op=="G")std::cin>>key;
                if(op=="H" || op=="G")std::cin>>partial;
                float start[3],end[3];for(float& v:start)std::cin>>v;for(float& v:end)std::cin>>v;
                if(op=="Q")std::cin>>nodes>>maxpath>>maxstraight;
                dtNavMeshQuery q;auto init=q.init(&nav,nodes);require(!dtStatusFailed(init),"query budget init");dtQueryFilter filter;filter.setIncludeFlags(1);
                float ext[]={1,100,1},ns[3]={},ne[3]={};dtPolyRef sr=0,er=0;
                if(op=="G" && keys.count(key)) {const auto& b=keys[key];float e[]={std::max(1.f,(b.max[0]-b.min[0])*.5f+.05f),100,std::max(1.f,(b.max[2]-b.min[2])*.5f+.05f)};q.findNearestPoly(start,e,&filter,&sr,ns);if(sr)std::memcpy(start,ns,12);else {out={0};results.push_back(out);continue;}}
                auto ss=q.findNearestPoly(start,ext,&filter,&sr,ns);auto es=q.findNearestPoly(end,ext,&filter,&er,ne);bool proxy=false;
                if(!er && partial) {proxy=true;float wide[]={4,100,4};es=q.findNearestPoly(end,wide,&filter,&er,ne);}
                if(op=="Q") {out={ss,es,static_cast<int64_t>(sr),static_cast<int64_t>(er)};point(out,ns);point(out,ne);}
                if(!sr || !er) {if(op!="Q")out={0};results.push_back(out);continue;}
                std::vector<dtPolyRef> path(maxpath);int npath=0;auto ps=q.findPath(sr,er,ns,ne,&filter,path.data(),&npath,maxpath);
                bool reaches=npath>0 && path[npath-1]==er && !proxy && !(ps&(DT_BUFFER_TOO_SMALL|DT_OUT_OF_NODES|DT_PARTIAL_RESULT));
                if(op!="Q" && !reaches && !partial) {out={0};results.push_back(out);continue;}
                std::vector<float> straight(maxstraight*3);std::vector<unsigned char> flags(maxstraight);std::vector<dtPolyRef> refs(maxstraight);int nstraight=0;
                auto fs=q.findStraightPath(ns,ne,path.data(),npath,straight.data(),flags.data(),refs.data(),&nstraight,maxstraight,0);
                if(op=="Q") {out.push_back(ps);out.push_back(npath);for(int i=0;i<npath;++i)out.push_back(path[i]);out.push_back(fs);out.push_back(nstraight);for(int i=0;i<nstraight;++i) {point(out,straight.data()+i*3);out.push_back(flags[i]);out.push_back(refs[i]);}}
                else {out={1,reaches && !(fs&(DT_BUFFER_TOO_SMALL|DT_PARTIAL_RESULT)),ps,fs,nstraight};for(int i=0;i<nstraight*3;++i)out.push_back(bits(straight[i]));}
            } else throw std::runtime_error("unknown command: "+op);
            results.push_back(out);
        }
        std::cout<<'[';for(size_t i=0;i<results.size();++i) {if(i)std::cout<<',';std::cout<<'[';for(size_t j=0;j<results[i].size();++j){if(j)std::cout<<',';std::cout<<results[i][j];}std::cout<<']';}std::cout<<"]\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
