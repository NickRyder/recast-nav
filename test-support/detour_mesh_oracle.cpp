// Direct mesh/reference calls against pinned, unmodified Detour. Test only.
#include <cstdlib>
#include <cstdint>
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "Recast.h"
#include "detour_node_snapshot.h"
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

template<class T> static void scalar_output(T value) {std::cout<<+value;}
static void scalar_output(bool value) {std::cout<<(value ? "true" : "false");}
template<class T> static void scalar_integers() {
    auto read=[]() {
        std::string text;std::cin>>text;
        if(text=="true")return (T)1;
        if(text=="false")return (T)0;
        return text[0]=='-' ? (T)std::stoll(text) : (T)std::stoull(text);
    };
    const T a=read(),b=read(),s=read();
    std::cout<<",[";bool comma=false;
    for(T value:{dtMin(a,b),dtMax(a,b),dtAbs(a),dtSqr(s),dtClamp(s,a,b),
                 rcMin(a,b),rcMax(a,b),rcAbs(a),rcSqr(s),rcClamp(s,a,b)}) {
        if(comma)std::cout<<',';comma=true;scalar_output(value);
    }
    std::cout<<']';
}

int main() {
    int tiles,polygons;
    if (!(std::cin >> tiles >> polygons)) return 2;
    std::string mode; std::cin >> mode;
    dtNavMesh mesh;
    dtNavMeshParams params={}; params.tileWidth=2; params.tileHeight=3;
    params.maxTiles=tiles; params.maxPolys=polygons;
    const dtStatus status=mesh.init(&params);
    std::cout << '[' << status;
    if (dtStatusSucceed(status) && mode=="poly_properties") {
        unsigned initial,area,type;
        while(std::cin>>initial>>area>>type) {
            dtPoly poly={};poly.areaAndtype=(unsigned char)initial;
            auto output=[&](){std::cout<<','<<(unsigned)poly.areaAndtype<<','<<(unsigned)poly.getArea()<<','<<(unsigned)poly.getType();};
            output();poly.setArea((unsigned char)area);output();poly.setType((unsigned char)type);output();
        }
    } else if (dtStatusSucceed(status) && mode=="intersection") {
        int count;std::cin>>count;float start[3],end[3];std::vector<float> verts(count*3);
        for(float* p:{start,end})for(int i=0;i<3;++i){unsigned bits;std::cin>>bits;std::memcpy(p+i,&bits,4);}
        for(float& v:verts){unsigned bits;std::cin>>bits;std::memcpy(&v,&bits,4);}
        float min,max;int enter,exit;
        bool hit=dtIntersectSegmentPoly2D(start,end,verts.data(),count,min,max,enter,exit);
        unsigned minBits,maxBits;std::memcpy(&minBits,&min,4);std::memcpy(&maxBits,&max,4);
        std::cout<<','<<hit<<','<<minBits<<','<<maxBits<<','<<enter<<','<<exit;
    } else if (dtStatusSucceed(status) && mode=="segments") {
        int count;std::cin>>count;
        for(int i=0;i<count;++i){
            float p[12];for(float& v:p){unsigned bits;std::cin>>bits;std::memcpy(&v,&bits,4);}
            float s=-17,t=19;bool hit=dtIntersectSegSeg2D(p,p+3,p+6,p+9,s,t);
            unsigned a,b;std::memcpy(&a,&s,4);std::memcpy(&b,&t,4);
            std::cout<<",["<<hit<<','<<a<<','<<b<<']';
        }
    } else if (dtStatusSucceed(status) && mode=="overlap") {
        int count;std::cin>>count;
        for(int i=0;i<count;++i){
            int na,nb;std::cin>>na>>nb;std::vector<float> a(na*3),b(nb*3);
            for(auto* v:{&a,&b})for(float& p:*v){unsigned bits;std::cin>>bits;std::memcpy(&p,&bits,4);}
            std::cout<<','<<dtOverlapPolyPoly2D(a.data(),na,b.data(),nb);
        }
    } else if (dtStatusSucceed(status) && mode=="convex_random") {
        int count;std::cin>>count;
        for(int i=0;i<count;++i){
            int n;std::cin>>n;float s,t;for(float* f:{&s,&t}){unsigned bits;std::cin>>bits;std::memcpy(f,&bits,4);}
            std::vector<float> vertices(n*3),areas(n+2);
            for(auto* v:{&vertices,&areas})for(float& p:*v){unsigned bits;std::cin>>bits;std::memcpy(&p,&bits,4);}
            float out[3];dtRandomPointInConvexPoly(vertices.data(),n,areas.data(),s,t,out);
            std::cout<<",[";bool comma=false;for(float f:out){unsigned bits;std::memcpy(&bits,&f,4);if(comma)std::cout<<',';comma=true;std::cout<<bits;}
            for(float f:areas){unsigned bits;std::memcpy(&bits,&f,4);std::cout<<','<<bits;}std::cout<<']';
        }
    } else if (dtStatusSucceed(status) && mode=="backend_math") {
        int count;std::cin>>count;
        for(int i=0;i<count;++i){unsigned words[2];float a,b;std::cin>>words[0]>>words[1];std::memcpy(&a,words,4);std::memcpy(&b,words+1,4);
            std::cout<<",[";bool comma=false;
            for(float value:{dtMathFabsf(a),dtMathSqrtf(a),dtMathFloorf(a),dtMathCeilf(a),dtMathCosf(a),dtMathSinf(a),dtMathAtan2f(a,b)}){
                unsigned bits;std::memcpy(&bits,&value,4);if(comma)std::cout<<',';comma=true;std::cout<<bits;
            }
            std::cout<<','<<dtMathIsfinite(a)<<']';
        }
    } else if (dtStatusSucceed(status) && mode=="scalar_templates") {
        int count;std::cin>>count;
        for(int i=0;i<count;++i) {
            std::string type;std::cin>>type;
            if(type=="i8")scalar_integers<signed char>();
            else if(type=="u8")scalar_integers<unsigned char>();
            else if(type=="i16")scalar_integers<short>();
            else if(type=="u16")scalar_integers<unsigned short>();
            else if(type=="i32")scalar_integers<int>();
            else if(type=="u32")scalar_integers<unsigned int>();
            else if(type=="i64")scalar_integers<long long>();
            else if(type=="u64")scalar_integers<unsigned long long>();
            else if(type=="isize")scalar_integers<intptr_t>();
            else if(type=="usize")scalar_integers<uintptr_t>();
            else if(type=="bool")scalar_integers<bool>();
            else if(type=="f64") {
                double a,b,s;
                for(double* value:{&a,&b,&s}){uint64_t bits;std::cin>>bits;std::memcpy(value,&bits,8);}
                std::cout<<",[";bool comma=false;
                for(double value:{dtMin(a,b),dtMax(a,b),dtAbs(a),dtSqr(s),dtClamp(s,a,b),
                                  rcMin(a,b),rcMax(a,b),rcAbs(a),rcSqr(s),rcClamp(s,a,b)}) {
                    uint64_t bits;std::memcpy(&bits,&value,8);if(comma)std::cout<<',';comma=true;std::cout<<bits;
                }
                std::cout<<']';
            } else return 2;
        }
    } else if (dtStatusSucceed(status) && mode=="math") {
        int count;std::cin>>count;
        auto read=[](float* p,int n){for(int i=0;i<n;++i){unsigned bits;std::cin>>bits;std::memcpy(p+i,&bits,4);}};
        auto floats=[](const float* p,int n){std::cout<<'[';for(int i=0;i<n;++i){if(i)std::cout<<',';unsigned bits;std::memcpy(&bits,p+i,4);std::cout<<bits;}std::cout<<']';};
        for(int i=0;i<count;++i){
            float a[3],b[3],c[3],s;read(a,3);read(b,3);read(c,3);read(&s,1);unsigned word;int side;std::cin>>word>>side;
            std::cout<<",[";float v[3];dtVcross(v,a,b);floats(v,3);
            std::cout<<',';dtVmad(v,a,b,s);floats(v,3);std::cout<<',';dtVlerp(v,a,b,s);floats(v,3);
            std::cout<<',';dtVadd(v,a,b);floats(v,3);std::cout<<',';dtVsub(v,a,b);floats(v,3);
            std::cout<<',';dtVscale(v,a,s);floats(v,3);std::cout<<',';dtVcopy(v,a);dtVmin(v,b);floats(v,3);
            std::cout<<',';dtVcopy(v,a);dtVmax(v,b);floats(v,3);std::cout<<',';dtVcopy(v,a);dtVnormalize(v);floats(v,3);
            float f[]={dtVdot(a,b),dtVlen(a),dtVlenSqr(a),dtVdist(a,b),dtVdistSqr(a,b),dtVdist2D(a,b),dtVdist2DSqr(a,b),dtVdot2D(a,b),dtVperp2D(a,b),dtTriArea2D(a,b,c),dtMin(a[0],b[0]),dtMax(a[0],b[0]),dtAbs(a[0]),dtSqr(s),dtClamp(s,a[0],b[0])};
            std::cout<<',';floats(f,15);
            unsigned short qa[3],qb[3],qc[3];for(int k=0;k<3;++k){unsigned x,y,z;std::memcpy(&x,a+k,4);std::memcpy(&y,b+k,4);std::memcpy(&z,c+k,4);qa[k]=(unsigned short)x;qb[k]=(unsigned short)y;qc[k]=(unsigned short)z;}
            std::cout<<",["<<dtVequal(a,b)<<','<<dtVisfinite(a)<<','<<dtVisfinite2D(a)<<','<<dtOverlapBounds(a,b,b,c)<<','<<dtOverlapQuantBounds(qa,qb,qb,qc)<<']';
            unsigned short u16=(unsigned short)word;short i16=(short)word;unsigned u32=word;int i32;std::memcpy(&i32,&word,4);float ff;std::memcpy(&ff,&word,4);
            dtSwapEndian(&u16);dtSwapEndian(&i16);dtSwapEndian(&u32);dtSwapEndian(&i32);dtSwapEndian(&ff);unsigned fb;std::memcpy(&fb,&ff,4);
            std::cout<<",["<<dtNextPow2(word)<<','<<dtIlog2(word)<<','<<dtAlign4(side)<<','<<dtOppositeTile(side)<<','<<u16<<','<<i16<<','<<u32<<','<<i32<<','<<fb<<"]]";
        }
    } else if (dtStatusSucceed(status) && mode=="geometry") {
        int count;std::cin>>count;
        auto read=[](float* p,int n){for(int i=0;i<n;++i){unsigned bits;std::cin>>bits;std::memcpy(p+i,&bits,4);}};
        auto floats=[](const float* p,int n){std::cout<<'[';for(int i=0;i<n;++i){if(i)std::cout<<',';unsigned bits;std::memcpy(&bits,p+i,4);std::cout<<bits;}std::cout<<']';};
        for(int i=0;i<count;++i){
            float p[3],a[3],b[3],c[3];read(p,3);read(a,3);read(b,3);read(c,3);int n,ni;std::cin>>n>>ni;
            std::vector<float> verts(n*3);read(verts.data(),n*3);std::vector<unsigned short> indices(ni);for(auto& j:indices){unsigned k;std::cin>>k;j=(unsigned short)k;}
            float closest[3],h=-17,t=-19,center[3];dtClosestPtPointTriangle(closest,p,a,b,c);bool height=dtClosestHeightPointTriangle(p,a,b,c,h);float d=dtDistancePtSegSqr2D(p,a,b,t);
            std::vector<float> ed(n+2,31),et(n+2,-37);bool inside=dtDistancePtPolyEdgesSqr(p,verts.data(),n,ed.data(),et.data());dtCalcPolyCenter(center,indices.data(),ni,verts.data());
            std::cout<<",[";floats(closest,3);std::cout<<",["<<height<<',';floats(&h,1);std::cout<<"],[";float dt[]={d,t};floats(dt,2);std::cout<<"],"<<dtPointInPolygon(p,verts.data(),n)<<','<<inside<<',';floats(ed.data(),n+2);std::cout<<',';floats(et.data(),n+2);std::cout<<',';floats(center,3);std::cout<<']';
        }
    } else if (dtStatusSucceed(status) && mode=="filter") {
        dtQueryFilter filter;
        std::cout << ',' << filter.getIncludeFlags() << ',' << filter.getExcludeFlags();
        for (int i=0; i<DT_MAX_AREAS; ++i) {
            float cost=filter.getAreaCost(i); unsigned bits;std::memcpy(&bits,&cost,4);
            std::cout << ',' << bits;
        }
        unsigned flags,include,exclude;
        while (std::cin >> flags >> include >> exclude) {
            dtPoly poly={};poly.flags=(unsigned short)flags;
            filter.setIncludeFlags((unsigned short)include);filter.setExcludeFlags((unsigned short)exclude);
            std::cout << ',' << filter.passFilter(0,0,&poly);
        }
    } else if (dtStatusSucceed(status) && mode=="pool") {
        int capacity,hash;std::cin>>capacity>>hash;dtNodePool pool(capacity,hash);std::string op;
        while(std::cin>>op) {
            std::cout<<",[";
            if(op=="G") {
                unsigned long long id;unsigned state,setPos,bits[5],parent,flags;
                std::cin>>id>>state>>setPos;for(unsigned& b:bits)std::cin>>b;std::cin>>parent>>flags;
                dtNode* n=pool.getNode((dtPolyRef)id,(unsigned char)state);
                std::cout<<pool.getNodeIdx(n);
                if(n) {
                    unsigned cost,total;std::memcpy(&cost,&n->cost,4);std::memcpy(&total,&n->total,4);
                    std::cout<<','<<n->pidx<<','<<cost<<','<<total<<','<<n->state<<','<<n->flags;
                    if(setPos)std::memcpy(n->pos,bits,12);
                    std::memcpy(&n->cost,bits+3,4);std::memcpy(&n->total,bits+4,4);n->pidx=parent;n->flags=flags;
                }
            } else if(op=="F") {
                unsigned long long id;unsigned state;std::cin>>id>>state;std::cout<<pool.getNodeIdx(pool.findNode((dtPolyRef)id,(unsigned char)state));
            } else if(op=="M") {
                unsigned long long id;int max;std::cin>>id>>max;std::vector<dtNode*> nodes(std::max(max,1));
                int count=pool.findNodes((dtPolyRef)id,nodes.data(),max);
                for(int i=0;i<count;++i) {if(i)std::cout<<',';std::cout<<pool.getNodeIdx(nodes[i]);}
            } else if(op=="A") {
                unsigned index;std::cin>>index;const dtNode* n=pool.getNodeAtIdx(index);std::cout<<pool.getNodeIdx(n);
                if(n){std::cout<<','<<n->id<<','<<n->pidx<<','<<n->state<<','<<n->flags;for(float value:{n->pos[0],n->pos[1],n->pos[2],n->cost,n->total}){unsigned bits;std::memcpy(&bits,&value,4);std::cout<<','<<bits;}}
            } else if(op=="W") {
                unsigned index,bits;std::cin>>index>>bits;dtNode* n=pool.getNodeAtIdx(index);std::memcpy(n->pos,&bits,4);
            } else if(op=="C") {pool.clear();}
            else if(op=="S") {
                nodePoolContents(pool);
            }
            std::cout<<']';
        }
    } else if (dtStatusSucceed(status) && mode=="queue_ops") {
        int count,capacity;std::cin>>count>>capacity;dtNodePool pool(count,1);dtNodeQueue queue(capacity);
        for(int i=0;i<count;++i)pool.getNode((dtPolyRef)(i+1));
        std::string op;bool initialized=false;
        while(std::cin>>op) {
            std::cout<<",[";
            if(op=="P") {unsigned i;std::cin>>i;queue.push(pool.getNodeAtIdx(i));initialized=true;}
            else if(op=="U") {unsigned i,bits;std::cin>>i>>bits;dtNode* n=pool.getNodeAtIdx(i);std::memcpy(&n->total,&bits,4);queue.modify(n);}
            else if(op=="X")std::cout<<(queue.empty()?0:pool.getNodeIdx(queue.pop()));
            else if(op=="T"){if(!initialized)std::exit(4);std::cout<<pool.getNodeIdx(queue.top());}
            else if(op=="M"){unsigned i;std::cin>>i;queue.modify(pool.getNodeAtIdx(i));}
            else if(op=="C")queue.clear();
            else if(op=="B")pool.clear();
            else if(op=="S"){if(!initialized)std::exit(4);std::cout<<queue.getCapacity()<<','<<queue.empty()<<','<<pool.getNodeIdx(queue.top());}
            std::cout<<']';
        }
    } else if (dtStatusSucceed(status) && mode=="queue") {
        int count;std::cin>>count;std::vector<dtNode> nodes(count);dtNodeQueue queue(count);
        for(int i=0;i<count;++i) {
            unsigned bits;std::cin>>bits;std::memcpy(&nodes[i].total,&bits,4);nodes[i].id=(dtPolyRef)(i+1);
            queue.push(&nodes[i]);
        }
        while(!queue.empty())std::cout<<','<<queue.pop()->id;
    } else if (dtStatusSucceed(status) && mode=="cycles") {
        int count; std::cin >> count;
        const unsigned short vertices[]={0,0,0,0,0,2,2,0,0};
        const unsigned short polys[]={0,1,2,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
        const unsigned short flags[]={1}; const unsigned char areas[]={63};
        dtNavMeshCreateParams p={};p.verts=vertices;p.vertCount=3;p.polys=polys;
        p.polyFlags=flags;p.polyAreas=areas;p.polyCount=1;p.nvp=6;p.cs=1;p.ch=1;
        p.bmax[0]=p.bmax[2]=2;p.bmax[1]=1;
        unsigned char* data=0;int size=0;
        if (!dtCreateNavMeshData(&p,&data,&size)) return 3;
        for (int i=0; i<count; ++i) {
            dtTileRef ref=0;const dtStatus added=mesh.addTile(data,size,0,0,&ref);
            if (dtStatusFailed(added)) return 4;
            const bool valid=mesh.isValidPolyRef(ref);
            const dtStatus removed=mesh.removeTile(ref,0,0);
            std::cout << ',' << ref << ',' << valid << ',' << removed << ',' << mesh.isValidPolyRef(ref);
        }
        dtFree(data);
    } else if (dtStatusSucceed(status)) {
        unsigned salt,tile,poly;
        while (std::cin >> salt >> tile >> poly) {
            const dtPolyRef reference=mesh.encodePolyId(salt,tile,poly);
            unsigned ds,dt,dp; mesh.decodePolyId(reference,ds,dt,dp);
            std::cout << ',' << reference << ',' << ds << ',' << dt << ',' << dp;
        }
    }
    std::cout << "]\n";
}
