// Test-only executable against the unmodified, pinned C++ Recast sources.
#include "Recast.h"
#include "DetourTileCacheBuilder.h"
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using Values = std::vector<int64_t>;
static std::map<std::string, Values> snapshots;
struct RecordingContext : rcContext {
    Values events;
    void doLog(rcLogCategory category,const char* message,int length) override {
        events.insert(events.end(),{1,category,length});for(int i=0;i<length;++i)events.push_back((unsigned char)message[i]);
    }
    void doStartTimer(rcTimerLabel label) override {events.insert(events.end(),{3,label});}
    void doStopTimer(rcTimerLabel label) override {events.insert(events.end(),{4,label});}
};
static int64_t bits(float f) { uint32_t u; std::memcpy(&u, &f, sizeof(u)); return u; }
static void require(bool ok, const char* stage) { if (!ok) throw std::runtime_error(stage); }
static void bounds(Values& out, const float* lo, const float* hi, float cs, float ch) {
    for (int i = 0; i < 3; ++i) out.push_back(bits(lo[i]));
    for (int i = 0; i < 3; ++i) out.push_back(bits(hi[i]));
    out.push_back(bits(cs)); out.push_back(bits(ch));
}
static void solid_snapshot(const char* name, const rcHeightfield& hf) {
    Values& out = snapshots[name];
    out.push_back(hf.width); out.push_back(hf.height);
    bounds(out, hf.bmin, hf.bmax, hf.cs, hf.ch);
    for (int i = 0; i < hf.width * hf.height; ++i) {
        int n = 0;
        for (rcSpan* s = hf.spans[i]; s; s = s->next) ++n;
        out.push_back(n);
        for (rcSpan* s = hf.spans[i]; s; s = s->next) {
            out.push_back(s->smin); out.push_back(s->smax); out.push_back(s->area);
        }
    }
}
static void compact_snapshot(const char* name, const rcCompactHeightfield& hf) {
    Values& out = snapshots[name];
    out = {hf.width, hf.height, hf.walkableHeight, hf.walkableClimb, hf.borderSize,
           hf.maxDistance, hf.maxRegions, hf.spanCount};
    bounds(out, hf.bmin, hf.bmax, hf.cs, hf.ch);
    for (int i = 0; i < hf.width * hf.height; ++i) {
        out.push_back(hf.cells[i].index); out.push_back(hf.cells[i].count);
    }
    for (int i = 0; i < hf.spanCount; ++i) {
        const rcCompactSpan& s = hf.spans[i];
        out.push_back(s.y); out.push_back(s.h); out.push_back(s.reg);
        for (int dir = 0; dir < 4; ++dir) out.push_back(rcGetCon(s, dir));
        out.push_back(hf.areas[i]);
    }
    out.push_back(hf.dist ? hf.spanCount : 0);
    if (hf.dist) for (int i = 0; i < hf.spanCount; ++i) out.push_back(hf.dist[i]);
}

static void cache_snapshot(const std::string& prefix, const rcHeightfieldLayer& hl, const rcConfig& c, bool blocked) {
    dtTileCacheLayerHeader header = {};
    header.width=hl.width; header.height=hl.height;
    std::memcpy(header.bmin,hl.bmin,12); std::memcpy(header.bmax,hl.bmax,12);
    std::vector<unsigned char> heights(hl.heights,hl.heights+hl.width*hl.height);
    std::vector<unsigned char> areas(hl.areas,hl.areas+hl.width*hl.height);
    std::vector<unsigned char> regs(hl.width*hl.height);
    dtTileCacheLayer layer = {};
    layer.header=&header; layer.heights=heights.data(); layer.areas=areas.data(); layer.cons=hl.cons; layer.regs=regs.data();
    if (blocked) {float lo[]={6,-1,6},hi[]={10,8,10};require(!dtStatusFailed(dtMarkBoxArea(layer,hl.bmin,c.cs,c.ch,lo,hi,0)),"mark box");}
    dtTileCacheAlloc alloc;
    require(!dtStatusFailed(dtBuildTileCacheRegions(&alloc,layer,c.walkableClimb)),"cache regions");
    Values& rs=snapshots[prefix+"/regions"];rs.push_back(layer.regCount);
    for (auto a:areas) rs.push_back(a);for (auto r:regs) rs.push_back(r);
    dtTileCacheContourSet* contours=dtAllocTileCacheContourSet(&alloc);
    require(!dtStatusFailed(dtBuildTileCacheContours(&alloc,layer,c.walkableClimb,0,*contours)),"cache contours");
    Values& ct=snapshots[prefix+"/contours"];ct.push_back(contours->nconts);
    for (int i=0;i<contours->nconts;++i) {const auto& t=contours->conts[i];ct.insert(ct.end(),{t.reg,t.area,t.nverts});for (int j=0;j<t.nverts*4;++j) ct.push_back(t.verts[j]);}
    dtTileCachePolyMesh* mesh=dtAllocTileCachePolyMesh(&alloc);
    require(!dtStatusFailed(dtBuildTileCachePolyMesh(&alloc,*contours,*mesh)),"cache polygons");
    Values& pm=snapshots[prefix+"/polygons"];pm.insert(pm.end(),{mesh->nverts,mesh->npolys,mesh->nvp});
    for (int i=0;i<mesh->nverts*3;++i) pm.push_back(mesh->verts[i]);
    for (int i=0;i<mesh->npolys*12;++i) pm.push_back(mesh->polys[i]);
    for (int i=0;i<mesh->npolys;++i) pm.push_back(mesh->areas[i]);
    dtFreeTileCachePolyMesh(&alloc,mesh);dtFreeTileCacheContourSet(&alloc,contours);
}

int main() {
    try {
        rcConfig c = {};
        int partitioning=0,contourFlags=RC_CONTOUR_TESS_WALL_EDGES;
        for (int i = 0; i < 3; ++i) std::cin >> c.bmin[i];
        for (int i = 0; i < 3; ++i) std::cin >> c.bmax[i];
        std::cin >> c.cs >> c.ch >> c.walkableSlopeAngle >> c.walkableHeight
            >> c.walkableClimb >> c.walkableRadius >> c.borderSize >> c.minRegionArea
            >> c.mergeRegionArea >> c.maxEdgeLen >> c.maxSimplificationError
            >> c.maxVertsPerPoly >> c.detailSampleDist >> c.detailSampleMaxError >> partitioning >> contourFlags;
        int nv, nt;
        std::cin >> nv >> nt;
        require(std::cin.good() && nv > 0 && nt > 0, "input header");
        std::vector<float> vertices(nv * 3);
        std::vector<int> triangles(nt * 3);
        for (float& v : vertices) std::cin >> v;
        for (int& i : triangles) std::cin >> i;
        require(!std::cin.fail(), "input geometry");
        RecordingContext ctx;
        rcCalcGridSize(c.bmin, c.bmax, c.cs, &c.width, &c.height);
        std::vector<unsigned char> areas(nt, RC_NULL_AREA);
        rcMarkWalkableTriangles(&ctx, c.walkableSlopeAngle, vertices.data(), nv,
                               triangles.data(), nt, areas.data());
        for (auto a : areas) snapshots["triangle_areas"].push_back(a);
        rcHeightfield solid;
        require(rcCreateHeightfield(&ctx, solid, c.width, c.height, c.bmin, c.bmax, c.cs, c.ch), "heightfield");
        require(rcRasterizeTriangles(&ctx, vertices.data(), nv, triangles.data(), areas.data(), nt, solid, c.walkableClimb), "rasterize");
        solid_snapshot("rasterized", solid);
        rcFilterLowHangingWalkableObstacles(&ctx, c.walkableClimb, solid);
        solid_snapshot("low_hanging", solid);
        rcFilterLedgeSpans(&ctx, c.walkableHeight, c.walkableClimb, solid);
        solid_snapshot("ledges", solid);
        rcFilterWalkableLowHeightSpans(&ctx, c.walkableHeight, solid);
        solid_snapshot("low_height", solid);
        rcCompactHeightfield compact;
        require(rcBuildCompactHeightfield(&ctx, c.walkableHeight, c.walkableClimb, solid, compact), "compact");
        compact_snapshot("compact", compact);
        require(rcErodeWalkableArea(&ctx, c.walkableRadius, compact), "erosion");
        compact_snapshot("eroded", compact);
        rcHeightfieldLayerSet layers;
        require(rcBuildHeightfieldLayers(&ctx, compact, c.borderSize, c.walkableHeight, layers), "layers");
        Values& ls = snapshots["layers"];
        ls.push_back(layers.nlayers);
        for (int i = 0; i < layers.nlayers; ++i) {
            const rcHeightfieldLayer& l = layers.layers[i];
            ls.insert(ls.end(), {l.width,l.height,l.minx,l.maxx,l.miny,l.maxy,l.hmin,l.hmax});
            bounds(ls,l.bmin,l.bmax,l.cs,l.ch);
            for (int j=0; j<l.width*l.height; ++j) ls.push_back(l.heights[j]);
            for (int j=0; j<l.width*l.height; ++j) ls.push_back(l.areas[j]);
            for (int j=0; j<l.width*l.height; ++j) ls.push_back(l.cons[j]);
            if (l.width<=255 && l.height<=255) {
                const std::string prefix="cache/"+std::to_string(i);
                cache_snapshot(prefix+"/base",l,c,false);
                cache_snapshot(prefix+"/box",l,c,true);
            }
        }
        require(rcBuildDistanceField(&ctx, compact), "distance");
        compact_snapshot("distance", compact);
        const bool regions=partitioning==0 ? rcBuildRegions(&ctx,compact,c.borderSize,c.minRegionArea,c.mergeRegionArea)
            : partitioning==1 ? rcBuildRegionsMonotone(&ctx,compact,c.borderSize,c.minRegionArea,c.mergeRegionArea)
            : rcBuildLayerRegions(&ctx,compact,c.borderSize,c.minRegionArea);
        require(regions,"regions");
        compact_snapshot("regions", compact);
        rcContourSet contours;
        require(rcBuildContours(&ctx, compact, c.maxSimplificationError, c.maxEdgeLen, contours, contourFlags), "contours");
        Values& ct = snapshots["contours"];
        ct = {contours.width, contours.height, contours.borderSize, bits(contours.maxError), contours.nconts};
        bounds(ct, contours.bmin, contours.bmax, contours.cs, contours.ch);
        for (int i = 0; i < contours.nconts; ++i) {
            const rcContour& t = contours.conts[i];
            ct.push_back(t.reg); ct.push_back(t.area); ct.push_back(t.nverts); ct.push_back(t.nrverts);
            for (int j = 0; j < t.nverts * 4; ++j) ct.push_back(t.verts[j]);
            for (int j = 0; j < t.nrverts * 4; ++j) ct.push_back(t.rverts[j]);
        }
        rcPolyMesh mesh;
        require(rcBuildPolyMesh(&ctx, contours, c.maxVertsPerPoly, mesh), "polygons");
        Values& pm = snapshots["polygons"];
        pm = {mesh.nverts, mesh.npolys, mesh.nvp, mesh.borderSize, bits(mesh.maxEdgeError)};
        bounds(pm, mesh.bmin, mesh.bmax, mesh.cs, mesh.ch);
        for (int i = 0; i < mesh.nverts * 3; ++i) pm.push_back(mesh.verts[i]);
        for (int i = 0; i < mesh.npolys; ++i) for (int j = 0; j < mesh.nvp; ++j) pm.push_back(mesh.polys[i * mesh.nvp * 2 + j]);
        for (int i = 0; i < mesh.npolys; ++i) for (int j = 0; j < mesh.nvp; ++j) pm.push_back(mesh.polys[i * mesh.nvp * 2 + mesh.nvp + j]);
        for (int i = 0; i < mesh.npolys; ++i) { pm.push_back(mesh.regs[i]); pm.push_back(mesh.areas[i]); pm.push_back(mesh.flags[i]); }
        pm.push_back(mesh.maxpolys);
        for(int i=mesh.npolys;i<mesh.maxpolys;++i)for(int j=0;j<mesh.nvp*2;++j)pm.push_back(mesh.polys[i*mesh.nvp*2+j]);
        for(int i=mesh.npolys;i<mesh.maxpolys;++i){pm.push_back(mesh.regs[i]);pm.push_back(mesh.areas[i]);}
        rcPolyMeshDetail detail;
        require(rcBuildPolyMeshDetail(&ctx, mesh, compact, c.detailSampleDist, c.detailSampleMaxError, detail), "detail");
        Values& dm = snapshots["detail"];
        dm = {detail.nmeshes, detail.nverts, detail.ntris};
        for (int i = 0; i < detail.nmeshes * 4; ++i) dm.push_back(detail.meshes[i]);
        for (int i = 0; i < detail.nverts * 3; ++i) dm.push_back(bits(detail.verts[i]));
        for (int i = 0; i < detail.ntris * 4; ++i) dm.push_back(detail.tris[i]);
        snapshots["context"]=ctx.events;
        std::cout << '{';
        bool first_stage = true;
        for (const auto& stage : snapshots) {
            if (!first_stage) std::cout << ',';
            first_stage = false;
            std::cout << '"' << stage.first << "\":[";
            for (size_t i = 0; i < stage.second.size(); ++i) {
                if (i) std::cout << ',';
                std::cout << stage.second[i];
            }
            std::cout << ']';
        }
        std::cout << "}\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
