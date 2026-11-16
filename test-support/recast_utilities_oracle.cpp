// Direct calls to unmodified, pinned Recast 1.6.0. Test-only driver.
#include "Recast.h"
#include "RecastAlloc.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

static uint32_t bits(float value) {
    uint32_t out; std::memcpy(&out, &value, sizeof(out)); return out;
}
static void point(float* p) { for (int i=0; i<3; ++i) std::cin >> p[i]; }
static void snapshot(const rcCompactHeightfield& hf) {
    std::cout << '[';
    bool first=true;
    auto emit = [&](int64_t value) { if (!first) std::cout << ','; first=false; std::cout << value; };
    emit(hf.borderSize); emit(hf.maxRegions);
    for (int i=0; i<hf.width*hf.height; ++i) { emit(hf.cells[i].index); emit(hf.cells[i].count); }
    for (int i=0; i<hf.spanCount; ++i) {
        const rcCompactSpan& span=hf.spans[i]; emit(span.y); emit(span.h); emit(span.reg);
        for (int d=0; d<4; ++d) emit(rcGetCon(span,d));
        emit(hf.areas[i]);
    }
    std::cout << ']';
}

using Values=std::vector<int64_t>;
static void output(const Values& values) {
    std::cout << '['; for (size_t i=0;i<values.size();++i) {if(i)std::cout<<',';std::cout<<values[i];}std::cout<<']';
}
struct RecordingContext : rcContext {
    mutable std::vector<Values> events;
    explicit RecordingContext(bool enabled=true):rcContext(enabled) {}
    void doResetLog() override {events.push_back({0});}
    void doLog(rcLogCategory category,const char* message,int length) override {
        Values v={1,category};for(int i=0;i<length;++i)v.push_back((unsigned char)message[i]);events.push_back(v);
    }
    void doResetTimers() override {events.push_back({2});}
    void doStartTimer(rcTimerLabel label) override {events.push_back({3,label});}
    void doStopTimer(rcTimerLabel label) override {events.push_back({4,label});}
    int doGetAccumulatedTime(rcTimerLabel label) const override {events.push_back({5,label});return (int)label*17+9;}
    void output() const {std::cout<<'[';for(size_t i=0;i<events.size();++i){if(i)std::cout<<',';::output(events[i]);}std::cout<<']';}
};
template<class T> static T* integers(int n) {
    T* data=(T*)rcAlloc(n*sizeof(T),RC_ALLOC_PERM);
    for(int i=0;i<n;++i){unsigned v;std::cin>>v;data[i]=(T)v;}return data;
}
static Values poly_snapshot(const rcPolyMesh& m) {
    Values v={m.nvp,m.borderSize,bits(m.maxEdgeError),m.nverts,m.npolys};
    for(int i=0;i<3;++i)v.push_back(bits(m.bmin[i]));for(int i=0;i<3;++i)v.push_back(bits(m.bmax[i]));
    v.push_back(bits(m.cs));v.push_back(bits(m.ch));
    for(int i=0;i<m.nverts*3;++i)v.push_back(m.verts[i]);
    for(int i=0;i<m.npolys*m.nvp*2;++i)v.push_back(m.polys[i]);
    for(int i=0;i<m.npolys;++i){v.push_back(m.regs[i]);v.push_back(m.areas[i]);v.push_back(m.flags[i]);}
    v.push_back(m.maxpolys);
    for(int i=m.npolys;i<m.maxpolys;++i)for(int j=0;j<m.nvp*2;++j)v.push_back(m.polys[i*m.nvp*2+j]);
    for(int i=m.npolys;i<m.maxpolys;++i){v.push_back(m.regs[i]);v.push_back(m.areas[i]);}
    return v;
}
static void merge_polys(bool observing=false,bool rawBits=false) {
    int count;std::cin>>count;std::vector<rcPolyMesh*> meshes;
    RecordingContext recorded;rcContext basic;rcContext& context=observing?(rcContext&)recorded:basic;if(observing)std::cout<<'[';std::cout<<'[';
    for(int i=0;i<count;++i) {
        rcPolyMesh* m=new rcPolyMesh;meshes.push_back(m);
        auto value=[&](float& v){if(rawBits){uint32_t raw;std::cin>>raw;std::memcpy(&v,&raw,4);}else std::cin>>v;};
        value(m->cs);value(m->ch);std::cin>>m->nvp>>m->borderSize;value(m->maxEdgeError);
        for(float& v:m->bmin)value(v);for(float& v:m->bmax)value(v);
        std::cin>>m->nverts>>m->npolys;m->maxpolys=m->npolys;
        m->verts=integers<unsigned short>(m->nverts*3);m->polys=integers<unsigned short>(m->npolys*m->nvp*2);
        m->regs=integers<unsigned short>(m->npolys);m->areas=integers<unsigned char>(m->npolys);m->flags=integers<unsigned short>(m->npolys);
        rcPolyMesh copy;if(!rcCopyPolyMesh(&context,*m,copy))std::exit(2);
        if(i)std::cout<<',';output(poly_snapshot(copy));
    }
    rcPolyMesh merged;if(!rcMergePolyMeshes(&context,meshes.data(),count,merged))std::exit(2);
    if(count)std::cout<<',';output(poly_snapshot(merged));std::cout<<']';if(observing){std::cout<<',';recorded.output();std::cout<<']';}std::cout<<'\n';
    for(auto m:meshes)delete m;
}
static Values detail_snapshot(const rcPolyMeshDetail& m) {
    Values v={m.nmeshes,m.nverts,m.ntris};
    for(int i=0;i<m.nmeshes*4;++i)v.push_back(m.meshes[i]);
    for(int i=0;i<m.nverts*3;++i)v.push_back(bits(m.verts[i]));
    for(int i=0;i<m.ntris*4;++i)v.push_back(m.tris[i]);return v;
}
static void merge_details() {
    int count;std::cin>>count;std::vector<rcPolyMeshDetail*> meshes;
    for(int i=0;i<count;++i) {
        int present;std::cin>>present;if(!present){meshes.push_back(nullptr);continue;}
        rcPolyMeshDetail* m=new rcPolyMeshDetail;meshes.push_back(m);
        std::cin>>m->nmeshes>>m->nverts>>m->ntris;
        m->meshes=integers<unsigned>(m->nmeshes*4);
        m->verts=(float*)rcAlloc(m->nverts*3*sizeof(float),RC_ALLOC_PERM);
        for(int j=0;j<m->nverts*3;++j)std::cin>>m->verts[j];
        m->tris=integers<unsigned char>(m->ntris*4);
    }
    rcContext context;rcPolyMeshDetail merged;
    if(!rcMergePolyMeshDetails(&context,meshes.data(),count,merged))std::exit(2);
    Values v={merged.nmeshes,merged.nverts,merged.ntris};
    for(int i=0;i<merged.nmeshes*4;++i)v.push_back(merged.meshes[i]);
    for(int i=0;i<merged.nverts*3;++i)v.push_back(bits(merged.verts[i]));
    for(int i=0;i<merged.ntris*4;++i)v.push_back(merged.tris[i]);output(v);std::cout<<'\n';
    for(auto m:meshes)delete m;
}

static void triangle_areas() {
    int nv,nt;float angle;std::cin>>nv>>nt>>angle;
    std::vector<float> vertices(nv*3);std::vector<int> triangles(nt*3);
    std::vector<unsigned char> initial(nt);for(auto& v:vertices)std::cin>>v;for(auto& i:triangles)std::cin>>i;
    for(auto& a:initial){int v;std::cin>>v;a=(unsigned char)v;}
    auto marked=initial,cleared=initial;rcContext context;
    rcMarkWalkableTriangles(&context,angle,vertices.data(),nv,triangles.data(),nt,marked.data());
    rcClearUnwalkableTriangles(&context,angle,vertices.data(),nv,triangles.data(),nt,cleared.data());
    std::cout<<'[';output(Values(marked.begin(),marked.end()));std::cout<<',';output(Values(cleared.begin(),cleared.end()));std::cout<<"]\n";
}
static void input_bounds() {
    int count;float cs;std::cin>>count>>cs;std::vector<float> vertices(count*3);for(auto& v:vertices)std::cin>>v;
    float lo[3],hi[3];rcCalcBounds(vertices.data(),count,lo,hi);int w,h;rcCalcGridSize(lo,hi,cs,&w,&h);
    Values result;for(float v:lo)result.push_back(bits(v));for(float v:hi)result.push_back(bits(v));result.push_back(w);result.push_back(h);output(result);std::cout<<'\n';
}

static Values solid_snapshot(const rcHeightfield& hf,rcContext& context) {
    Values v={rcGetHeightFieldSpanCount(&context,hf),hf.width,hf.height};
    for(int i=0;i<3;++i)v.push_back(bits(hf.bmin[i]));for(int i=0;i<3;++i)v.push_back(bits(hf.bmax[i]));
    v.push_back(bits(hf.cs));v.push_back(bits(hf.ch));
    for(int i=0;i<hf.width*hf.height;++i){int count=0;for(rcSpan* s=hf.spans[i];s;s=s->next)++count;v.push_back(count);for(rcSpan* s=hf.spans[i];s;s=s->next){v.push_back(s->smin);v.push_back(s->smax);v.push_back(s->area);}}
    return v;
}
static void solid_operations(bool observing=false) {
    RecordingContext recorded;rcContext basic;rcContext& context=observing?(rcContext&)recorded:basic;
    rcHeightfield hf;int w,h;float lo[3],hi[3],cs,ch;
    std::cin>>w>>h;point(lo);point(hi);std::cin>>cs>>ch;
    if(!rcCreateHeightfield(&context,hf,w,h,lo,hi,cs,ch))std::exit(2);
    auto emit=[&](){if(observing)std::cout<<'[';output(solid_snapshot(hf,context));if(observing){std::cout<<',';recorded.output();std::cout<<']';}};
    int count;std::cin>>count;std::cout<<'[';emit();
    for(int op=0;op<count;++op) {
        std::string mode;std::cin>>mode;
        if(mode=="timer") {int enabled;std::cin>>enabled;context.enableTimer(enabled!=0);}
        else if(mode=="add") {
            int x,z,min,max,area,threshold;std::cin>>x>>z>>min>>max>>area>>threshold;
            if(!rcAddSpan(&context,hf,x,z,(unsigned short)min,(unsigned short)max,(unsigned char)area,threshold))std::exit(2);
        } else if(mode=="low_hanging" || mode=="ledge" || mode=="low_height") {
            int height,climb;std::cin>>height>>climb;
            if(mode=="low_hanging")rcFilterLowHangingWalkableObstacles(&context,climb,hf);
            else if(mode=="ledge")rcFilterLedgeSpans(&context,height,climb,hf);
            else rcFilterWalkableLowHeightSpans(&context,height,hf);
        } else if(mode=="compact" || mode=="compact_context") {
            int height,climb;std::cin>>height>>climb;rcCompactHeightfield compact;
            RecordingContext recorded;rcContext& selected=mode=="compact_context"?(rcContext&)recorded:context;
            const bool ok=rcBuildCompactHeightfield(&selected,height,climb,hf,compact);if(!ok)std::exit(2);
            std::cout<<",[";if(mode=="compact_context")std::cout<<ok<<',';
            std::cout<<'['<<compact.width<<','<<compact.height<<','<<compact.walkableHeight<<','<<compact.walkableClimb;
            for(float v:compact.bmin)std::cout<<','<<bits(v);for(float v:compact.bmax)std::cout<<','<<bits(v);std::cout<<','<<bits(compact.cs)<<','<<bits(compact.ch)<<"],";snapshot(compact);
            if(mode=="compact_context"){std::cout<<',';recorded.output();}std::cout<<']';
        } else {
            int nv,nt,threshold;std::cin>>nv>>nt>>threshold;
            std::vector<float> vertices(nv*3);std::vector<int> triangles(nt*3);std::vector<unsigned char> areas(nt);
            for(auto& v:vertices)std::cin>>v;for(auto& i:triangles)std::cin>>i;for(auto& a:areas){int v;std::cin>>v;a=(unsigned char)v;}
            bool ok=false;
            if(mode=="u32")ok=rcRasterizeTriangles(&context,vertices.data(),nv,triangles.data(),areas.data(),nt,hf,threshold);
            else if(mode=="u16") {std::vector<unsigned short> t(triangles.begin(),triangles.end());ok=rcRasterizeTriangles(&context,vertices.data(),nv,t.data(),areas.data(),nt,hf,threshold);}
            else if(mode=="flat") {std::vector<float> v;for(int i:triangles)for(int a=0;a<3;++a)v.push_back(vertices[i*3+a]);ok=rcRasterizeTriangles(&context,v.data(),areas.data(),nt,hf,threshold);}
            else if(mode=="single") {ok=true;for(int i=0;i<nt;++i)ok=ok && rcRasterizeTriangle(&context,&vertices[triangles[i*3]*3],&vertices[triangles[i*3+1]*3],&vertices[triangles[i*3+2]*3],areas[i],hf,threshold);}
            if(!ok)std::exit(2);
        }
        std::cout<<',';emit();
    }
    std::cout<<"]\n";
}

static void polygon_capacity(bool observing=false,bool retained=false) {
    int total;std::cin>>total;const int first=total%3==1?4:total%3==2?5:3;
    rcContourSet contours;contours.nconts=(total-first)/3+1;contours.cs=contours.ch=1;
    contours.width=1024;contours.height=512;contours.bmax[0]=1024;contours.bmax[1]=8;contours.bmax[2]=512;
    contours.conts=(rcContour*)rcAlloc(contours.nconts*sizeof(rcContour),RC_ALLOC_PERM);
    std::memset(contours.conts,0,contours.nconts*sizeof(rcContour));
    const int points[5][2]={{0,0},{0,2},{1,3},{2,2},{2,0}};
    for(int i=0;i<contours.nconts;++i) {
        rcContour& c=contours.conts[i];c.nverts=i==0?first:3;c.reg=(unsigned short)(i+1);c.area=63;
        c.verts=(int*)rcAlloc(c.nverts*4*sizeof(int),RC_ALLOC_PERM);
        for(int j=0;j<c.nverts;++j){c.verts[j*4]=i%256*4+points[j][0];c.verts[j*4+1]=0;c.verts[j*4+2]=i/256*4+points[j][1];c.verts[j*4+3]=0;}
    }
    RecordingContext recorded;rcContext basic;rcContext& context=observing?(rcContext&)recorded:basic;rcPolyMesh mesh;
    if(retained){
        mesh.nverts=3;mesh.npolys=1;mesh.maxpolys=2;mesh.nvp=3;
        const unsigned short vertices[]={11,13,17,19,23,29,31,37,41},polygons[]={0,1,2,65535,65535,65535,2,1,0,3,5,7},regions[]={31,51},flags[]={0xcafe};const unsigned char areas[]={7,19};
        mesh.verts=(unsigned short*)rcAlloc(sizeof(vertices),RC_ALLOC_PERM);std::memcpy(mesh.verts,vertices,sizeof(vertices));
        mesh.polys=(unsigned short*)rcAlloc(sizeof(polygons),RC_ALLOC_PERM);std::memcpy(mesh.polys,polygons,sizeof(polygons));
        mesh.regs=(unsigned short*)rcAlloc(sizeof(regions),RC_ALLOC_PERM);std::memcpy(mesh.regs,regions,sizeof(regions));
        mesh.flags=(unsigned short*)rcAlloc(sizeof(flags),RC_ALLOC_PERM);std::memcpy(mesh.flags,flags,sizeof(flags));
        mesh.areas=(unsigned char*)rcAlloc(sizeof(areas),RC_ALLOC_PERM);std::memcpy(mesh.areas,areas,sizeof(areas));
    }
    const bool ok=rcBuildPolyMesh(&context,contours,3,mesh);
    Values result={ok,mesh.nverts,mesh.npolys};const auto data=poly_snapshot(mesh);result.insert(result.end(),data.begin(),data.end());if(ok){rcPolyMesh copy;if(!rcCopyPolyMesh(&context,mesh,copy))std::exit(2);const auto copied=poly_snapshot(copy);result.insert(result.end(),copied.begin(),copied.end());}
    if(observing)std::cout<<'[';output(result);if(observing){std::cout<<',';recorded.output();std::cout<<']';}std::cout<<'\n';
}

static Values contour_snapshot(const rcContourSet& c) {
    Values v={c.width,c.height,c.borderSize,bits(c.maxError),c.nconts};
    for(int i=0;i<3;++i)v.push_back(bits(c.bmin[i]));for(int i=0;i<3;++i)v.push_back(bits(c.bmax[i]));
    v.push_back(bits(c.cs));v.push_back(bits(c.ch));
    for(int i=0;i<c.nconts;++i){const auto& t=c.conts[i];v.push_back(t.reg);v.push_back(t.area);v.push_back(t.nverts);v.push_back(t.nrverts);for(int j=0;j<t.nverts*4;++j)v.push_back(t.verts[j]);for(int j=0;j<t.nrverts*4;++j)v.push_back(t.rverts[j]);}
    return v;
}
static void poly_contours(bool observing=false) {
    rcContourSet c;int nvp;std::cin>>c.cs>>c.ch>>nvp>>c.width>>c.height>>c.borderSize>>c.maxError;
    point(c.bmin);point(c.bmax);std::cin>>c.nconts;
    c.conts=(rcContour*)rcAlloc(sizeof(rcContour)*c.nconts,RC_ALLOC_PERM);std::memset(c.conts,0,sizeof(rcContour)*c.nconts);
    for(int i=0;i<c.nconts;++i){auto& v=c.conts[i];unsigned reg,area;std::cin>>v.nverts>>reg>>area;v.reg=(unsigned short)reg;v.area=(unsigned char)area;v.verts=integers<int>(v.nverts*4);}
    RecordingContext recorded;rcContext basic;rcContext& context=observing?(rcContext&)recorded:basic;rcPolyMesh mesh;bool ok=rcBuildPolyMesh(&context,c,nvp,mesh);
    Values result={ok,mesh.nverts,mesh.npolys};if(ok){auto data=poly_snapshot(mesh);result.insert(result.end(),data.begin(),data.end());rcPolyMesh copy;if(!rcCopyPolyMesh(&context,mesh,copy))std::exit(2);data=poly_snapshot(copy);result.insert(result.end(),data.begin(),data.end());}
    if(observing)std::cout<<'[';output(result);if(observing){std::cout<<',';recorded.output();std::cout<<']';}
}
static void packed_fields() {
    int count;std::cin>>count;std::cout<<'[';
    for(int i=0;i<count;++i) {
        unsigned index,n,con,height;int direction,neighbor,x,z;
        std::cin>>index>>n>>con>>height>>direction>>neighbor>>x>>z;
        rcCompactCell cell={};cell.index=index;cell.count=n;
        Values v={cell.index,cell.count};cell.count++;v.push_back(cell.count);
        rcCompactSpan span={};span.con=con;span.h=height;
        v.push_back(span.con);v.push_back(span.h);v.push_back(rcGetCon(span,direction));
        rcSetCon(span,direction,neighbor);
        v.push_back(span.con);v.push_back(span.h);v.push_back(rcGetCon(span,direction));
        v.push_back(rcGetDirOffsetX(direction));v.push_back(rcGetDirOffsetY(direction));
        v.push_back(rcGetDirForOffset(x,z));if(i)std::cout<<',';output(v);
    }
    std::cout<<"]\n";
}

static void common_helpers() {
    int count;std::cin>>count;std::cout<<'[';
    for(int i=0;i<count;++i) {
        float a[3],b[3],s;
        auto read=[](float& value){uint32_t u;std::cin>>u;std::memcpy(&value,&u,4);};
        for(float& v:a)read(v);for(float& v:b)read(v);read(s);
        Values v;float out[3];auto emit=[&](){for(float x:out)v.push_back(bits(x));};
        rcVcross(out,a,b);emit();rcVmad(out,a,b,s);emit();rcVadd(out,a,b);emit();rcVsub(out,a,b);emit();
        rcVcopy(out,a);rcVmin(out,b);emit();rcVcopy(out,a);rcVmax(out,b);emit();
        rcVcopy(out,a);rcVnormalize(out);emit();
        for(float x:{rcVdot(a,b),rcVdist(a,b),rcVdistSqr(a,b),rcMin(a[0],b[0]),rcMax(a[0],b[0]),rcAbs(a[0]),rcSqr(s),rcClamp(s,a[0],b[0]),rcSqrt(s)})v.push_back(bits(x));
        if(i)std::cout<<',';output(v);
    }
    std::cout<<"]\n";
}

static void context_operations() {
    int enabled,count;std::cin>>enabled>>count;RecordingContext ctx(enabled!=0);rcContext base(enabled!=0);Values values={RC_MAX_TIMERS,RC_TIMER_TOTAL,RC_TIMER_TEMP,RC_TIMER_RASTERIZE_TRIANGLES,RC_TIMER_BUILD_COMPACTHEIGHTFIELD,RC_TIMER_BUILD_CONTOURS,RC_TIMER_BUILD_CONTOURS_TRACE,RC_TIMER_BUILD_CONTOURS_SIMPLIFY,RC_TIMER_FILTER_BORDER,RC_TIMER_FILTER_WALKABLE,RC_TIMER_MEDIAN_AREA,RC_TIMER_FILTER_LOW_OBSTACLES,RC_TIMER_BUILD_POLYMESH,RC_TIMER_MERGE_POLYMESH,RC_TIMER_ERODE_AREA,RC_TIMER_MARK_BOX_AREA,RC_TIMER_MARK_CYLINDER_AREA,RC_TIMER_MARK_CONVEXPOLY_AREA,RC_TIMER_BUILD_DISTANCEFIELD,RC_TIMER_BUILD_DISTANCEFIELD_DIST,RC_TIMER_BUILD_DISTANCEFIELD_BLUR,RC_TIMER_BUILD_REGIONS,RC_TIMER_BUILD_REGIONS_WATERSHED,RC_TIMER_BUILD_REGIONS_EXPAND,RC_TIMER_BUILD_REGIONS_FLOOD,RC_TIMER_BUILD_REGIONS_FILTER,RC_TIMER_BUILD_LAYERS,RC_TIMER_BUILD_POLYMESHDETAIL,RC_TIMER_MERGE_POLYMESHDETAIL};
    for(int i=0;i<count;++i) {
        std::string op;std::cin>>op;
        if(op=="E" || op=="e") {int state;std::cin>>state;if(op=="E"){ctx.enableLog(state!=0);base.enableLog(state!=0);}else{ctx.enableTimer(state!=0);base.enableTimer(state!=0);}}
        else if(op=="R"){ctx.resetLog();base.resetLog();}
        else if(op=="T"){ctx.resetTimers();base.resetTimers();}
        else if(op=="L" || op=="F") {
            int category,n,value=0;std::cin>>category;if(op=="F")std::cin>>value;std::cin>>n;std::vector<char> text(n+1,0);for(int j=0;j<n;++j){unsigned b;std::cin>>b;text[j]=(char)b;}
            if(op=="F")ctx.log((rcLogCategory)category,"value=%d; text=%s",value,text.data());else ctx.log((rcLogCategory)category,"%.*s",n,text.data());
        } else if(op=="Z"){int category;std::cin>>category;ctx.log((rcLogCategory)category,"%c",0);}
        else if(op=="S" || op=="P" || op=="G") {
            int label;std::cin>>label;
            if(op=="S"){ctx.startTimer((rcTimerLabel)label);base.startTimer((rcTimerLabel)label);}
            else if(op=="P"){ctx.stopTimer((rcTimerLabel)label);base.stopTimer((rcTimerLabel)label);}
            else{values.push_back(ctx.getAccumulatedTime((rcTimerLabel)label));values.push_back(base.getAccumulatedTime((rcTimerLabel)label));}
        } else if(op=="C") {
            int label,start,end;std::cin>>label>>start>>end;ctx.enableTimer(start!=0);base.enableTimer(end!=0);
            {rcScopedTimer timer(&ctx,(rcTimerLabel)label);ctx.enableTimer(end!=0);}
        } else if(op=="M") {
            rcPolyMesh mesh;rcPolyMeshDetail detail;
            if(!rcMergePolyMeshes(&ctx,nullptr,0,mesh) || !rcMergePolyMeshDetails(&ctx,nullptr,0,detail))std::exit(2);
        } else std::exit(3);
    }
    std::cout<<'[';ctx.output();std::cout<<',';output(values);std::cout<<"]\n";
}

int main() {
    std::string mode; std::cin >> mode;
    if (mode=="sort_keys") {
        struct Record {int key, ordinal;};
        int cases;std::cin>>cases;std::cout<<'[';
        for(int c=0;c<cases;++c) {
            int n;std::cin>>n;std::vector<Record> records(n+1);
            for(int i=0;i<n;++i){std::cin>>records[i].key;records[i].ordinal=i;}
            std::qsort(records.data(),n,sizeof(Record),[](const void* a,const void* b){
                int x=((const Record*)a)->key,y=((const Record*)b)->key;return x<y?-1:x>y?1:0;
            });
            if(c)std::cout<<',';std::cout<<'[';
            for(int i=0;i<n;++i){if(i)std::cout<<',';output({records[i].key,records[i].ordinal});}
            std::cout<<']';
        }
        std::cout<<"]\n";
    }
    else if (mode=="empty_mesh_metadata") {
        int nvp,vertices;float distance;std::cin>>nvp>>vertices>>distance;
        RecordingContext context;rcContourSet contours;rcPolyMesh mesh;rcCompactHeightfield hf;rcPolyMeshDetail detail;
        bool ok=rcBuildPolyMesh(&context,contours,nvp,mesh);
        if(vertices){rcFree(mesh.verts);mesh.verts=(unsigned short*)rcAlloc(vertices*3*sizeof(unsigned short),RC_ALLOC_PERM);std::memset(mesh.verts,0,vertices*3*sizeof(unsigned short));mesh.nverts=vertices;}
        ok=ok && rcBuildPolyMeshDetail(&context,mesh,hf,distance,0,detail);
        std::cout<<'['<<ok<<',';output(poly_snapshot(mesh));std::cout<<',';output(detail_snapshot(detail));std::cout<<',';context.output();std::cout<<"]\n";
    }
    else if (mode=="context") {context_operations();}
    else if (mode=="common") {common_helpers();}
    else if (mode=="packed") {packed_fields();}
    else if (mode=="poly_contours" || mode=="poly_contours_context") {poly_contours(mode=="poly_contours_context");}
    else if (mode=="poly_capacity" || mode=="poly_capacity_context") {polygon_capacity(mode=="poly_capacity_context");}
    else if(mode=="poly_capacity_retained_context"){polygon_capacity(true,true);}
    else if (mode=="solid" || mode=="solid_context") {solid_operations(mode=="solid_context");}
    else if (mode=="triangle_areas") {triangle_areas();}
    else if (mode=="bounds") {input_bounds();}
    else if (mode=="merge_poly" || mode=="merge_poly_context" || mode=="merge_poly_bits") {merge_polys(mode=="merge_poly_context",mode=="merge_poly_bits");}
    else if (mode=="merge_poly_bits_batch") {
        int count;std::cin>>count;std::cout<<'[';
        for(int i=0;i<count;++i){if(i)std::cout<<',';merge_polys(false,true);}std::cout<<']';
    }
    else if (mode=="merge_detail") {merge_details();}
    else if (mode=="offset") {
        int count, capacity; float offset;
        std::cin >> count >> offset >> capacity;
        std::vector<float> input(count*3), output(capacity*3, -123.25f);
        for (auto& v:input) std::cin >> v;
        const int result=rcOffsetPoly(input.data(),count,offset,output.data(),capacity);
        std::cout << '[' << result;
        for (auto v:output) std::cout << ',' << bits(v);
        std::cout << "]\n";
    } else if (mode=="offset_bits") {
        int cases;std::cin>>cases;std::cout<<'[';
        for(int i=0;i<cases;++i){
            int count,capacity;unsigned raw;std::cin>>count>>capacity>>raw;
            float offset;std::memcpy(&offset,&raw,4);
            std::vector<float> input(count*3),out(capacity*3,-123.25f);
            for(float& value:input){std::cin>>raw;std::memcpy(&value,&raw,4);}
            const int result=rcOffsetPoly(input.data(),count,offset,out.data(),capacity);
            if(i)std::cout<<',';std::cout<<'['<<result;
            for(float value:out)std::cout<<','<<bits(value);std::cout<<']';
        }
        std::cout<<']';
    } else if (mode=="areas" || mode=="areas_context") {
        rcCompactHeightfield hf;
        std::cin >> hf.width >> hf.height >> hf.cs >> hf.ch;
        point(hf.bmin); point(hf.bmax);
        std::cin >> hf.spanCount;
        const int nc=hf.width*hf.height;
        hf.cells=(rcCompactCell*)rcAlloc(nc*sizeof(rcCompactCell),RC_ALLOC_PERM);
        hf.spans=(rcCompactSpan*)rcAlloc(hf.spanCount*sizeof(rcCompactSpan),RC_ALLOC_PERM);
        hf.areas=(unsigned char*)rcAlloc(hf.spanCount,RC_ALLOC_PERM);
        if (!hf.cells || !hf.spans || !hf.areas) return 2;
        for (int i=0; i<nc; ++i) {
            unsigned index,count; std::cin >> index >> count;
            hf.cells[i].index=index; hf.cells[i].count=count;
        }
        for (int i=0; i<hf.spanCount; ++i) {
            unsigned y,h,reg,area; std::cin >> y >> h >> reg;
            hf.spans[i].y=y; hf.spans[i].h=h; hf.spans[i].reg=reg; hf.spans[i].con=0;
            for (int d=0; d<4; ++d) { int c; std::cin >> c; rcSetCon(hf.spans[i],d,c); }
            std::cin >> area; hf.areas[i]=area;
        }
        RecordingContext recorded;rcContext basic;const bool observing=mode=="areas_context";rcContext& context=observing?(rcContext&)recorded:basic;
        auto emit=[&](){if(observing)std::cout<<'[';snapshot(hf);if(observing){std::cout<<',';recorded.output();std::cout<<']';}};
        int operations; std::cin >> operations;
        std::cout << '['; emit();
        for (int i=0; i<operations; ++i) {
            std::string op; std::cin >> op;
            if (op=="timer") {int enabled;std::cin>>enabled;context.enableTimer(enabled!=0);}
            else if (op=="box") {
                float lo[3],hi[3]; int area; point(lo); point(hi); std::cin >> area;
                rcMarkBoxArea(&context,lo,hi,(unsigned char)area,hf);
            } else if (op=="cylinder") {
                float p[3],r,h; int area; point(p); std::cin >> r >> h >> area;
                rcMarkCylinderArea(&context,p,r,h,(unsigned char)area,hf);
            } else if (op=="convex") {
                int count,area; float lo,hi; std::cin >> count >> lo >> hi >> area;
                std::vector<float> vertices(count*3); for (auto& v:vertices) std::cin >> v;
                rcMarkConvexPolyArea(&context,vertices.data(),count,lo,hi,(unsigned char)area,hf);
            } else if (op=="contours") {
                float error;int edge,flags;std::cin>>error>>edge>>flags;rcContourSet contours;
                if(!rcBuildContours(&context,hf,error,edge,contours,flags))return 2;
                std::cout<<',';output(contour_snapshot(contours));
            } else if(op=="meshes" || op=="meshes_seed") {
                float error,distance,detailError;int edge,flags,nvp;
                std::cin>>error>>edge>>flags>>nvp>>distance>>detailError;
                rcContourSet contours;rcPolyMesh mesh;rcPolyMeshDetail detail;
                if(!rcBuildContours(&context,hf,error,edge,contours,flags))return 2;
                if(!rcBuildPolyMesh(&context,contours,nvp,mesh))return 2;
                if(op=="meshes_seed") {
                    int padding,region,bounded;std::cin>>padding>>region>>bounded;
                    unsigned short* vertices=(unsigned short*)rcAlloc((mesh.nverts+padding)*3*sizeof(unsigned short),RC_ALLOC_PERM);
                    if(!vertices)return 2;
                    std::memset(vertices,0,padding*3*sizeof(unsigned short));
                    std::memcpy(vertices+padding*3,mesh.verts,mesh.nverts*3*sizeof(unsigned short));
                    rcFree(mesh.verts);mesh.verts=vertices;mesh.nverts+=padding;
                    if(bounded) {
                        for(int v=0;v<mesh.nverts;++v)mesh.verts[v*3+1]=hf.spans[0].y;
                        for(int c=0;c<hf.width*hf.height;++c)if(hf.cells[c].count)hf.cells[c].count=255;
                    }
                    for(int p=0;p<mesh.npolys;++p) {
                        mesh.regs[p]=(unsigned short)region;
                        for(int v=0;v<mesh.nvp;++v) {
                            unsigned short& index=mesh.polys[p*mesh.nvp*2+v];
                            if(index!=RC_MESH_NULL_IDX)index=(unsigned short)(index+padding);
                        }
                    }
                }
                if(!rcBuildPolyMeshDetail(&context,mesh,hf,distance,detailError,detail))return 2;
                std::cout<<",[";output(contour_snapshot(contours));std::cout<<',';output(poly_snapshot(mesh));std::cout<<',';output(detail_snapshot(detail));std::cout<<']';
            } else if(op=="detail_polygon") {
                int count;float distance,error;std::cin>>count>>distance>>error;
                if(count<3 || count>6)return 3;
                rcPolyMesh mesh;rcPolyMeshDetail detail;
                mesh.nverts=mesh.nvp=count;mesh.npolys=mesh.maxpolys=1;
                mesh.cs=hf.cs;mesh.ch=hf.ch;mesh.maxEdgeError=1;
                rcVcopy(mesh.bmin,hf.bmin);rcVcopy(mesh.bmax,hf.bmax);
                mesh.verts=integers<unsigned short>(count*3);
                mesh.polys=(unsigned short*)rcAlloc(count*2*sizeof(unsigned short),RC_ALLOC_PERM);
                mesh.regs=(unsigned short*)rcAlloc(sizeof(unsigned short),RC_ALLOC_PERM);
                mesh.areas=(unsigned char*)rcAlloc(sizeof(unsigned char),RC_ALLOC_PERM);
                mesh.flags=(unsigned short*)rcAlloc(sizeof(unsigned short),RC_ALLOC_PERM);
                if(!mesh.polys || !mesh.regs || !mesh.areas || !mesh.flags)return 2;
                for(int i=0;i<count;++i){mesh.polys[i]=(unsigned short)i;mesh.polys[count+i]=RC_MESH_NULL_IDX;}
                mesh.regs[0]=1;mesh.areas[0]=RC_WALKABLE_AREA;mesh.flags[0]=0;
                if(!rcBuildPolyMeshDetail(&context,mesh,hf,distance,error,detail))return 2;
                std::cout<<',';output(detail_snapshot(detail));
            } else if(op=="set_region_meta") {
                int maximum,border;std::cin>>maximum>>border;hf.maxRegions=(unsigned short)maximum;hf.borderSize=border;
            } else if(op=="set_distance") {
                unsigned maximum;std::cin>>maximum;hf.maxDistance=(unsigned short)maximum;rcFree(hf.dist);hf.dist=integers<unsigned short>(hf.spanCount);
            } else if (op=="monotone" || op=="layer" || op=="watershed" || op=="watershed_try") {
                int border,minArea,mergeArea;std::cin>>border>>minArea>>mergeArea;
                if(op=="watershed" && !rcBuildDistanceField(&context,hf))return 2;
                const bool ok=(op=="watershed" || op=="watershed_try") ? rcBuildRegions(&context,hf,border,minArea,mergeArea) : op=="monotone" ? rcBuildRegionsMonotone(&context,hf,border,minArea,mergeArea)
                    : rcBuildLayerRegions(&context,hf,border,minArea);
                if(op=="watershed_try")std::cout<<','<<ok;
                else if(!ok)return 2;
            } else if (op=="distance") {
                if(!rcBuildDistanceField(&context,hf))return 2;
                Values out={hf.maxDistance};for(int j=0;j<hf.spanCount;++j)out.push_back(hf.dist[j]);std::cout<<',';output(out);
            } else if (op=="layers" || op=="layers_border" || op=="layers_try") {
                int height,border=0;if(op!="layers")std::cin>>border;std::cin>>height;rcHeightfieldLayerSet layers;bool ok=rcBuildHeightfieldLayers(&context,hf,border,height,layers);if(!ok && op!="layers_try")return 2;
                Values out={layers.nlayers};for(int j=0;j<layers.nlayers;++j){const auto& l=layers.layers[j];out.insert(out.end(),{l.width,l.height,l.minx,l.maxx,l.miny,l.maxy,l.hmin,l.hmax});for(float v:l.bmin)out.push_back(bits(v));for(float v:l.bmax)out.push_back(bits(v));out.push_back(bits(l.cs));out.push_back(bits(l.ch));for(const auto* grid:{l.heights,l.areas,l.cons})for(int k=0;k<l.width*l.height;++k)out.push_back(grid[k]);}std::cout<<',';if(op=="layers_try")std::cout<<'['<<ok<<',';output(out);if(op=="layers_try")std::cout<<']';
            } else if (op=="erode") {
                int radius;std::cin>>radius;if(!rcErodeWalkableArea(&context,radius,hf))return 2;
            } else if (op=="median") {
                if (!rcMedianFilterWalkableArea(&context,hf)) return 2;
            } else return 3;
            std::cout << ','; emit();
        }
        std::cout << "]\n";
    } else return 3;
    return std::cin.fail() ? 4 : 0;
}
