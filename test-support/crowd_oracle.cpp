// Test-only calls into pinned, unmodified DetourCrowd.
#include "DetourProximityGrid.h"
#include "DetourLocalBoundary.h"
#include "DetourPathQueue.h"
#include "DetourPathCorridor.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCrowd.h"
#include <cstdint>
#include "detour_node_snapshot.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourAlloc.h"
#include <algorithm>
#include <iostream>
#include <cstring>
#include <string>
#include <vector>
static float number(){unsigned bits;std::cin>>bits;float value;std::memcpy(&value,&bits,4);return value;}
static void grid(){
    int size;std::cin>>size;float cs=number();dtProximityGrid grid;if(!grid.init(size,cs)){std::cout<<"null";return;}
    auto snapshot=[&](){const int* b=grid.getBounds();unsigned bits;float cell=grid.getCellSize();std::memcpy(&bits,&cell,4);std::cout<<"[["<<b[0]<<','<<b[1]<<','<<b[2]<<','<<b[3]<<"],"<<bits<<']';};
    unsigned short backing[32];std::fill(backing,backing+32,0xface);
    std::cout<<"[[null,";snapshot();std::cout<<']';int count;std::cin>>count;
    for(int i=0;i<count;++i){char op;std::cin>>op;std::cout<<",[";
        if(op=='A'){unsigned id;int repeat;std::cin>>id>>repeat;float a=number(),b=number(),c=number(),d=number();for(int j=0;j<repeat;++j)grid.addItem((unsigned short)id,a,b,c,d);std::cout<<"null";}
        else if(op=='Q'){float a=number(),b=number(),c=number(),d=number();int cap;std::cin>>cap;std::vector<unsigned short> ids(cap?cap:1);int n=grid.queryItems(a,b,c,d,ids.data(),cap);std::cout<<'[';for(int j=0;j<n;++j){if(j)std::cout<<',';std::cout<<ids[j];}std::cout<<']';}
        else if(op=='B'){float a=number(),b=number(),c=number(),d=number();int cap;std::cin>>cap;if(cap<0||cap>32)std::exit(2);int n=grid.queryItems(a,b,c,d,backing,cap);std::cout<<'['<<n<<",[";for(int j=0;j<32;++j){if(j)std::cout<<',';std::cout<<backing[j];}std::cout<<"]]";}
        else if(op=='N'){int x,y;std::cin>>x>>y;std::cout<<grid.getItemCountAt(x,y);}
        else if(op=='C'){grid.clear();std::cout<<"null";}
        else if(op=='I'){int n;std::cin>>n;float cell=number();std::cout<<grid.init(n,cell);}
        std::cout<<',';snapshot();std::cout<<']';
    }
    std::cout<<']';
}

static void point(float* p){for(int i=0;i<3;++i)p[i]=number();}
static void bits(const float* p,int count){std::cout<<'[';for(int i=0;i<count;++i){unsigned v;std::memcpy(&v,p+i,4);if(i)std::cout<<',';std::cout<<v;}std::cout<<']';}
static bool meshInput(dtNavMesh& nav,int& nodes){
    dtNavMeshParams p={};point(p.orig);p.tileWidth=number();p.tileHeight=number();int ntiles;std::cin>>p.maxTiles>>p.maxPolys>>nodes>>ntiles;
    if(dtStatusFailed(nav.init(&p)))return false;
    for(int i=0;i<ntiles;++i){int n;std::cin>>n;auto* bytes=(unsigned char*)dtAlloc(n,DT_ALLOC_PERM);for(int j=0;j<n;++j){unsigned b;std::cin>>b;bytes[j]=(unsigned char)b;}if(dtStatusFailed(nav.addTile(bytes,n,DT_TILE_FREE_DATA,0,0))){return false;}}
    return true;
}
static void boundary(){
    dtNavMesh nav;int nodes;if(!meshInput(nav,nodes)){std::cout<<"null";return;}
    dtNavMeshQuery query;query.init(&nav,nodes);dtQueryFilter filter;dtLocalBoundary boundary;int initialized=0;
    auto snapshot=[&](){initialized=std::max(initialized,boundary.getSegmentCount());std::cout<<'[';bits(boundary.getCenter(),3);std::cout<<','<<boundary.getSegmentCount()<<",[";
        for(int i=0;i<8;++i){if(i)std::cout<<',';if(i<initialized)bits(boundary.getSegment(i),6);else std::cout<<"null";}
        std::cout<<"],"<<boundary.isValid(&query,&filter)<<']';};
    std::cout<<"[[null,";snapshot();std::cout<<']';int count;std::cin>>count;
    for(int i=0;i<count;++i){char op;std::cin>>op;std::cout<<",[";
        if(op=='U'){dtPolyRef ref;std::cin>>ref;float pos[3];point(pos);float range=number();boundary.update(ref,pos,range,&query,&filter);std::cout<<"null";}
        else if(op=='R'){boundary.reset();std::cout<<"null";}
        else if(op=='F'){unsigned inc,exc;std::cin>>inc>>exc;filter.setIncludeFlags((unsigned short)inc);filter.setExcludeFlags((unsigned short)exc);std::cout<<"null";}
        else if(op=='D'){dtTileRef ref;std::cin>>ref;std::cout<<nav.removeTile(ref,0,0);}
        else if(op=='G'){dtPolyRef ref;unsigned flags;std::cin>>ref>>flags;std::cout<<nav.setPolyFlags(ref,(unsigned short)flags);}
        std::cout<<',';snapshot();std::cout<<']';
    }
    std::cout<<']';
}

static void pathQueue(){
    dtNavMesh nav;int nodes;if(!meshInput(nav,nodes)){std::cout<<"null";return;}
    int pathSize;std::cin>>pathSize;dtPathQueue queue;if(!queue.init(pathSize,nodes,&nav)){std::cout<<"null";return;}
    dtQueryFilter filters[3];std::vector<dtPathQueueRef> issued;
    auto snapshot=[&](){std::cout<<"[[";for(size_t i=0;i<issued.size();++i){if(i)std::cout<<',';std::cout<<queue.getRequestStatus(issued[i]);}std::cout<<"],[";nodePoolContents(*queue.getNavQuery()->getNodePool());std::cout<<"]]";};
    std::cout<<"[[null,";snapshot();std::cout<<']';int count;std::cin>>count;
    for(int i=0;i<count;++i){char op;std::cin>>op;std::cout<<",[";
        if(op=='Q'){dtPolyRef start,end;int filter;std::cin>>start>>end;float a[3],b[3];point(a);point(b);std::cin>>filter;auto r=queue.request(start,end,a,b,filter<0?0:&filters[filter]);if(r)issued.push_back(r);std::cout<<r;}
        else if(op=='U'){int n;std::cin>>n;queue.update(n);std::cout<<"null";}
        else if(op=='S'){unsigned ref;std::cin>>ref;std::cout<<queue.getRequestStatus(ref);}
        else if(op=='P'){unsigned ref;int cap;std::cin>>ref>>cap;std::vector<dtPolyRef> path(cap+2,(dtPolyRef)0xface);int n=cap+2;auto status=queue.getPathResult(ref,path.data(),&n,cap);std::cout<<'['<<status<<','<<n<<",[";for(int j=0;j<n;++j){if(j)std::cout<<',';std::cout<<path[j];}std::cout<<"]]";}
        else if(op=='F'){int index,area;unsigned inc,exc;std::cin>>index>>inc>>exc>>area;float cost=number();filters[index].setIncludeFlags((unsigned short)inc);filters[index].setExcludeFlags((unsigned short)exc);filters[index].setAreaCost(area,cost);std::cout<<"null";}
        else if(op=='D'){dtTileRef ref;std::cin>>ref;std::cout<<nav.removeTile(ref,0,0);}
        else if(op=='I'){int n;std::cin>>pathSize>>n;std::cout<<queue.init(pathSize,n,&nav);}
        std::cout<<',';snapshot();std::cout<<']';
    }
    std::cout<<']';
}

static void merges(){
    int count;std::cin>>count;std::cout<<'[';
    for(int i=0;i<count;++i){int mode,storage,npath,maximum,nvisited;std::cin>>mode>>storage>>npath>>maximum>>nvisited;std::vector<dtPolyRef> path(storage),visited(nvisited);for(auto& p:path)std::cin>>p;for(auto& v:visited)std::cin>>v;
        int n=mode==0?dtMergeCorridorStartMoved(path.data(),npath,maximum,visited.data(),nvisited):mode==1?dtMergeCorridorEndMoved(path.data(),npath,maximum,visited.data(),nvisited):dtMergeCorridorStartShortcut(path.data(),npath,maximum,visited.data(),nvisited);
        if(i)std::cout<<',';std::cout<<'['<<n<<",[";for(int j=0;j<storage;++j){if(j)std::cout<<',';std::cout<<path[j];}std::cout<<"]]";
    }
    std::cout<<']';
}
static void corridor(){
    dtNavMesh nav;int nodes;if(!meshInput(nav,nodes)){std::cout<<"null";return;}
    int capacity;dtPolyRef initial;std::cin>>capacity>>initial;float pos[3];point(pos);dtPathCorridor corridor;corridor.init(capacity);corridor.reset(initial,pos);
    dtNavMeshQuery query;query.init(&nav,nodes);dtQueryFilter filter;
    auto snapshot=[&](){std::cout<<'[';bits(corridor.getPos(),3);std::cout<<',';bits(corridor.getTarget(),3);std::cout<<",[";for(int i=0;i<corridor.getPathCount();++i){if(i)std::cout<<',';std::cout<<corridor.getPath()[i];}std::cout<<"],"<<corridor.getFirstPoly()<<','<<corridor.getLastPoly()<<",[";nodePoolContents(*query.getNodePool());std::cout<<"]]";};
    std::cout<<"[[null,";snapshot();std::cout<<']';int count;std::cin>>count;
    for(int i=0;i<count;++i){char op;std::cin>>op;std::cout<<",[";
        if(op=='R'){dtPolyRef ref;std::cin>>ref;float p[3];point(p);corridor.reset(ref,p);std::cout<<"null";}
        else if(op=='S'){float target[3];point(target);int n;std::cin>>n;std::vector<dtPolyRef> path(n);for(auto& p:path)std::cin>>p;corridor.setCorridor(target,path.data(),n);std::cout<<"null";}
        else if(op=='C'){int cap;std::cin>>cap;std::vector<float> positions(std::max(cap,1)*3);std::vector<unsigned char> flags(std::max(cap,1));std::vector<dtPolyRef> refs(std::max(cap,1));int n=corridor.findCorners(positions.data(),flags.data(),refs.data(),cap,&query,&filter);std::cout<<'['<<n<<",[";for(int j=0;j<cap;++j){if(j)std::cout<<',';std::cout<<'[';bits(positions.data()+j*3,3);std::cout<<','<<(unsigned)flags[j]<<','<<refs[j]<<']';}std::cout<<"]]";}
        else if(op=='V'){float next[3];point(next);float range=number();corridor.optimizePathVisibility(next,range,&query,&filter);std::cout<<"null";}
        else if(op=='T'){std::cout<<corridor.optimizePathTopology(&query,&filter);}
        else if(op=='M'){float next[3];point(next);std::cout<<corridor.movePosition(next,&query,&filter);}
        else if(op=='E'){float next[3];point(next);std::cout<<corridor.moveTargetPosition(next,&query,&filter);}
        else if(op=='F'){unsigned inc,exc;std::cin>>inc>>exc;filter.setIncludeFlags((unsigned short)inc);filter.setExcludeFlags((unsigned short)exc);std::cout<<"null";}
        else if(op=='G'){dtPolyRef ref;unsigned flags;std::cin>>ref>>flags;std::cout<<nav.setPolyFlags(ref,(unsigned short)flags);}
        else if(op=='D'){dtTileRef ref;std::cin>>ref;std::cout<<nav.removeTile(ref,0,0);}
        else if(op=='H'){int n;std::cin>>n;std::cout<<corridor.isValid(n,&query,&filter);}
        else if(op=='X'){dtPolyRef ref;std::cin>>ref;float pos[3];point(pos);std::cout<<corridor.fixPathStart(ref,pos);}
        else if(op=='I'){dtPolyRef ref;std::cin>>ref;float pos[3];point(pos);std::cout<<corridor.trimInvalidPath(ref,pos,&query,&filter);}
        else if(op=='O'){dtPolyRef ref;std::cin>>ref;dtPolyRef refs[2]={0xface,0xfade};float start[3]={-17,-19,-23},end[3]={29,31,37};bool ok=corridor.moveOverOffmeshConnection(ref,refs,start,end,&query);std::cout<<'['<<ok<<",["<<refs[0]<<','<<refs[1]<<"],";bits(start,3);std::cout<<',';bits(end,3);std::cout<<']';}
        std::cout<<',';snapshot();std::cout<<']';
    }
    std::cout<<']';
}

static void avoidance(){
    int nc,ns,nd;std::cin>>nc>>ns>>nd;dtObstacleAvoidanceQuery query;query.init(nc,ns);dtObstacleAvoidanceDebugData debug;if(nd)debug.init(nd);std::vector<bool> initialized(nd,false);
    auto snapshot=[&](){
        std::cout<<'['<<query.getObstacleCircleCount()<<','<<query.getObstacleSegmentCount()<<",[";
        for(int i=0;i<nc;++i){if(i)std::cout<<',';auto* c=query.getObstacleCircle(i);std::cout<<'[';bits(c->p,3);std::cout<<',';bits(c->vel,3);std::cout<<',';bits(c->dvel,3);std::cout<<',';bits(&c->rad,1);std::cout<<',';bits(c->dp,3);std::cout<<',';bits(c->np,3);std::cout<<']';}
        std::cout<<"],[";for(int i=0;i<ns;++i){if(i)std::cout<<',';auto* s=query.getObstacleSegment(i);std::cout<<'[';bits(s->p,3);std::cout<<',';bits(s->q,3);std::cout<<','<<s->touch<<']';}
        std::cout<<"],"<<debug.getSampleCount()<<",[";
        for(int i=0;i<nd;++i){if(i)std::cout<<',';if(!initialized[i]){std::cout<<"null";continue;}
            std::cout<<'[';bits(debug.getSampleVelocity(i),3);std::cout<<',';float size=debug.getSampleSize(i);bits(&size,1);std::cout<<',';
            float penalties[]={debug.getSamplePenalty(i),debug.getSampleDesiredVelocityPenalty(i),debug.getSampleCurrentVelocityPenalty(i),debug.getSamplePreferredSidePenalty(i),debug.getSampleCollisionTimePenalty(i)};bits(penalties,5);std::cout<<']';}
        std::cout<<"]]";
    };
    std::cout<<"[[null,";snapshot();std::cout<<']';int count;std::cin>>count;
    for(int i=0;i<count;++i){char op;std::cin>>op;std::cout<<",[";
        if(op=='C'){float p[3],v[3],dv[3];point(p);float radius=number();point(v);point(dv);query.addCircle(p,radius,v,dv);std::cout<<"null";}
        else if(op=='S'){float p[3],q[3];point(p);point(q);query.addSegment(p,q);std::cout<<"null";}
        else if(op=='R'){query.reset();std::cout<<"null";}
        else if(op=='I'){std::cin>>nc>>ns;std::cout<<query.init(nc,ns);}
        else if(op=='D'){debug.reset();std::cout<<"null";}
        else if(op=='J'){std::cin>>nd;bool ok=debug.init(nd);initialized.assign(nd,false);std::cout<<ok;}
        else if(op=='B'){float v[3];point(v);float size=number(),p=number(),vp=number(),vcp=number(),sp=number(),tp=number();int old=debug.getSampleCount();debug.addSample(v,size,p,vp,vcp,sp,tp);if(debug.getSampleCount()>old)initialized[old]=true;std::cout<<"null";}
        else if(op=='N'){debug.normalizeSamples();std::cout<<DT_SUCCESS;}
        else if(op=='G'||op=='A'){
            float pos[3],vel[3],desired[3],out[3];point(pos);float radius=number(),maximum=number();point(vel);point(desired);
            dtObstacleAvoidanceParams p;p.velBias=number();p.weightDesVel=number();p.weightCurVel=number();p.weightSide=number();p.weightToi=number();p.horizTime=number();
            unsigned grid,divs,rings,depth;bool record;std::cin>>grid>>divs>>rings>>depth>>record;p.gridSize=(unsigned char)grid;p.adaptiveDivs=(unsigned char)divs;p.adaptiveRings=(unsigned char)rings;p.adaptiveDepth=(unsigned char)depth;
            int n=op=='G'?query.sampleVelocityGrid(pos,radius,maximum,vel,desired,out,&p,record?&debug:0):query.sampleVelocityAdaptive(pos,radius,maximum,vel,desired,out,&p,record?&debug:0);
            if(record)for(int j=0;j<debug.getSampleCount();++j)initialized[j]=true;
            std::cout<<'['<<n<<',';bits(out,3);std::cout<<']';
        }
        std::cout<<',';snapshot();std::cout<<']';
    }
    std::cout<<']';
}

static dtCrowdAgentParams agentParams(){
    dtCrowdAgentParams p={};p.radius=number();p.height=number();p.maxAcceleration=number();p.maxSpeed=number();p.collisionQueryRange=number();p.pathOptimizationRange=number();p.separationWeight=number();
    unsigned flags,avoid,filter;std::uintptr_t data;std::cin>>flags>>avoid>>filter>>data;p.updateFlags=(unsigned char)flags;p.obstacleAvoidanceType=(unsigned char)avoid;p.queryFilterType=(unsigned char)filter;p.userData=(void*)data;return p;
}
static void agentParamBits(const dtCrowdAgentParams& p){float values[]={p.radius,p.height,p.maxAcceleration,p.maxSpeed,p.collisionQueryRange,p.pathOptimizationRange,p.separationWeight};std::cout<<'[';bits(values,7);std::cout<<','<<(unsigned)p.updateFlags<<','<<(unsigned)p.obstacleAvoidanceType<<','<<(unsigned)p.queryFilterType<<','<<(std::uintptr_t)p.userData<<']';}
static dtObstacleAvoidanceParams avoidParams(){dtObstacleAvoidanceParams p={};p.velBias=number();p.weightDesVel=number();p.weightCurVel=number();p.weightSide=number();p.weightToi=number();p.horizTime=number();unsigned g,d,r,n;std::cin>>g>>d>>r>>n;p.gridSize=g;p.adaptiveDivs=d;p.adaptiveRings=r;p.adaptiveDepth=n;return p;}
static void avoidParamBits(const dtObstacleAvoidanceParams& p){float values[]={p.velBias,p.weightDesVel,p.weightCurVel,p.weightSide,p.weightToi,p.horizTime};std::cout<<'[';bits(values,6);std::cout<<','<<(unsigned)p.gridSize<<','<<(unsigned)p.adaptiveDivs<<','<<(unsigned)p.adaptiveRings<<','<<(unsigned)p.adaptiveDepth<<']';}
static void debugSamples(const dtObstacleAvoidanceDebugData& data){std::cout<<'[';for(int i=0;i<data.getSampleCount();++i){if(i)std::cout<<',';std::cout<<'[';bits(data.getSampleVelocity(i),3);std::cout<<',';float size=data.getSampleSize(i);bits(&size,1);std::cout<<',';float p[]={data.getSamplePenalty(i),data.getSampleDesiredVelocityPenalty(i),data.getSampleCurrentVelocityPenalty(i),data.getSamplePreferredSidePenalty(i),data.getSampleCollisionTimePenalty(i)};bits(p,5);std::cout<<']';}std::cout<<']';}
static void simulation(){
    dtNavMesh nav;int ignored;if(!meshInput(nav,ignored)){std::cout<<"null";return;}int maximum;std::cin>>maximum;float radius=number();dtCrowd crowd;if(!crowd.init(maximum,radius,&nav)){std::cout<<"null";return;}
    dtObstacleAvoidanceDebugData data;data.init(128);dtCrowdAgentDebugInfo debug={};debug.idx=-1;float start[]={-17,-19,-23},end[]={29,31,37};std::memcpy(debug.optStart,start,12);std::memcpy(debug.optEnd,end,12);debug.vod=&data;
    auto activeIds=[&](int cap){std::vector<dtCrowdAgent*> agents(std::max(cap,1));int n=crowd.getActiveAgents(agents.data(),cap);std::cout<<'[';for(int i=0;i<n;++i){if(i)std::cout<<',';std::cout<<(agents[i]-crowd.getAgent(0));}std::cout<<']';};
    auto snapshot=[&](){
        std::cout<<'['<<crowd.getAgentCount()<<',';bits(crowd.getQueryHalfExtents(),3);std::cout<<','<<crowd.getVelocitySampleCount()<<",[";const auto* grid=crowd.getGrid();const int* b=grid->getBounds();std::cout<<'['<<b[0]<<','<<b[1]<<','<<b[2]<<','<<b[3]<<"],";float cell=grid->getCellSize();bits(&cell,1);std::cout<<",[";
        for(int z=-2;z<=4;++z)for(int x=-2;x<=4;++x){if(z!=-2||x!=-2)std::cout<<',';std::cout<<grid->getItemCountAt(x,z);}std::cout<<"],[";unsigned short ids[64];int ni=grid->queryItems(-2,-2,4,4,ids,64);for(int i=0;i<ni;++i){if(i)std::cout<<',';std::cout<<ids[i];}std::cout<<"]],";activeIds(crowd.getAgentCount());std::cout<<",[";
        for(int i=0;i<crowd.getAgentCount();++i){if(i)std::cout<<',';auto* a=crowd.getAgent(i);std::cout<<'['<<a->active<<','<<(unsigned)a->state<<','<<a->partial<<',';float times[]={a->topologyOptTime,a->desiredSpeed,a->targetReplanTime};bits(times,3);std::cout<<",[";
            for(int j=0;j<5;++j){if(j)std::cout<<',';bits(j==0?a->npos:j==1?a->disp:j==2?a->dvel:j==3?a->nvel:a->vel,3);}std::cout<<"],";agentParamBits(a->params);std::cout<<",[";
            for(int j=0;j<a->nneis;++j){if(j)std::cout<<',';std::cout<<'['<<a->neis[j].idx<<',';bits(&a->neis[j].dist,1);std::cout<<']';}std::cout<<"],[";
            for(int j=0;j<a->ncorners;++j){if(j)std::cout<<',';std::cout<<'[';bits(a->cornerVerts+j*3,3);std::cout<<','<<(unsigned)a->cornerFlags[j]<<','<<a->cornerPolys[j]<<']';}std::cout<<"],["<<(unsigned)a->targetState<<','<<a->targetRef<<',';bits(a->targetPos,3);std::cout<<','<<a->targetPathqRef<<','<<a->targetReplan<<"],[";
            bits(a->corridor.getPos(),3);std::cout<<',';bits(a->corridor.getTarget(),3);std::cout<<",[";for(int j=0;j<a->corridor.getPathCount();++j){if(j)std::cout<<',';std::cout<<a->corridor.getPath()[j];}std::cout<<"]],[";
            bits(a->boundary.getCenter(),3);std::cout<<','<<a->boundary.getSegmentCount()<<",[";for(int j=0;j<8;++j){if(j)std::cout<<',';bits(a->boundary.getSegment(j),6);}std::cout<<"]],";
            if(a->targetPathqRef)std::cout<<crowd.getPathQueue()->getRequestStatus(a->targetPathqRef);else std::cout<<"null";
            std::cout<<",["<<a->nneis<<",[";for(int j=0;j<DT_CROWDAGENT_MAX_NEIGHBOURS;++j){if(j)std::cout<<',';std::cout<<'['<<a->neis[j].idx<<',';bits(&a->neis[j].dist,1);std::cout<<']';}
            std::cout<<"]],["<<a->ncorners<<",[";for(int j=0;j<DT_CROWDAGENT_MAX_CORNERS;++j){if(j)std::cout<<',';std::cout<<'[';bits(a->cornerVerts+j*3,3);std::cout<<','<<(unsigned)a->cornerFlags[j]<<','<<a->cornerPolys[j]<<']';}std::cout<<"]]]";}
        std::cout<<"],[";for(int i=0;i<16;++i){if(i)std::cout<<',';auto* f=crowd.getFilter(i);std::cout<<'['<<f->getIncludeFlags()<<','<<f->getExcludeFlags()<<',';float costs[64];for(int j=0;j<64;++j)costs[j]=f->getAreaCost(j);bits(costs,64);std::cout<<']';}
        std::cout<<"],[";for(int i=0;i<8;++i){if(i)std::cout<<',';avoidParamBits(*crowd.getObstacleAvoidanceParams(i));}std::cout<<"],[";nodePoolContents(*crowd.getNavMeshQuery()->getNodePool());std::cout<<"],[";nodePoolContents(*crowd.getPathQueue()->getNavQuery()->getNodePool());std::cout<<"],["<<debug.idx<<',';bits(debug.optStart,3);std::cout<<',';bits(debug.optEnd,3);std::cout<<',';debugSamples(data);std::cout<<"]]";
    };
    std::cout<<"[[null,";snapshot();std::cout<<']';int count;std::cin>>count;
    for(int i=0;i<count;++i){char op;std::cin>>op;std::cout<<",[";
        if(op=='A'){float pos[3];point(pos);auto params=agentParams();std::cout<<crowd.addAgent(pos,&params);}
        else if(op=='P'){int index;std::cin>>index;auto params=agentParams();crowd.updateAgentParameters(index,&params);std::cout<<"null";}
        else if(op=='R'){int index;std::cin>>index;crowd.removeAgent(index);std::cout<<"null";}
        else if(op=='T'){int index;dtPolyRef ref;std::cin>>index>>ref;float pos[3];point(pos);std::cout<<crowd.requestMoveTarget(index,ref,pos);}
        else if(op=='V'){int index;std::cin>>index;float vel[3];point(vel);std::cout<<crowd.requestMoveVelocity(index,vel);}
        else if(op=='X'){int index;std::cin>>index;std::cout<<crowd.resetMoveTarget(index);}
        else if(op=='U'){float dt=number();bool enabled,record;std::cin>>enabled>>debug.idx>>record;debug.vod=record?&data:0;crowd.update(dt,enabled?&debug:0);std::cout<<"null";}
        else if(op=='F'){int index,area;unsigned inc,exc;std::cin>>index>>inc>>exc>>area;float cost=number();auto* f=crowd.getEditableFilter(index);if(f){f->setIncludeFlags(inc);f->setExcludeFlags(exc);f->setAreaCost(area,cost);}std::cout<<(f!=0);}
        else if(op=='O'){int index;std::cin>>index;auto params=avoidParams();crowd.setObstacleAvoidanceParams(index,&params);std::cout<<"null";}
        else if(op=='E'){int index;std::cin>>index;auto* a=crowd.getEditableAgent(index);if(a){const uint32_t distance=0xffc01871,position[]={0x80000000,0xffc01275,0x7f800000};a->neis[5].idx=-17;std::memcpy(&a->neis[5].dist,&distance,4);std::memcpy(a->cornerVerts+9,position,12);a->cornerFlags[3]=0xa5;a->cornerPolys[3]=~dtPolyRef(0);}std::cout<<(a!=nullptr);}
        else if(op=='H'){int a,f,p;std::cin>>a>>f>>p;std::cout<<'['<<(crowd.getAgent(a)!=0)<<','<<(crowd.getEditableAgent(a)!=0)<<','<<(crowd.getFilter(f)!=0)<<','<<(crowd.getEditableFilter(f)!=0)<<','<<(crowd.getObstacleAvoidanceParams(p)!=0)<<']';}
        else if(op=='L'){int cap;std::cin>>cap;activeIds(cap);}
        else if(op=='I'){std::cin>>maximum;radius=number();std::cout<<crowd.init(maximum,radius,&nav);}
        else if(op=='D'){dtTileRef ref;std::cin>>ref;std::cout<<nav.removeTile(ref,0,0);}
        else if(op=='B'){int n;dtTileRef last;std::cin>>last>>n;auto* bytes=(unsigned char*)dtAlloc(n,DT_ALLOC_PERM);for(int j=0;j<n;++j){unsigned b;std::cin>>b;bytes[j]=b;}dtTileRef ref=0;auto status=nav.addTile(bytes,n,DT_TILE_FREE_DATA,last,&ref);if(dtStatusFailed(status))dtFree(bytes);std::cout<<'['<<status<<','<<ref<<']';}
        std::cout<<',';snapshot();std::cout<<']';
    }
    std::cout<<']';
}
int main(){std::string mode;std::cin>>mode;if(mode=="grid")grid();else if(mode=="boundary")boundary();else if(mode=="queue")pathQueue();else if(mode=="merges")merges();else if(mode=="corridor")corridor();else if(mode=="avoidance")avoidance();else if(mode=="simulation")simulation();else return 2;}
