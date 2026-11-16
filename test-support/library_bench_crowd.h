// Matched scaling and off-mesh episodes. The pinned library is unchanged.
static void crowdFloat(Hash& h, float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    h.word(bits);
}

static void crowdStart(int i, bool dense, bool offmesh, float* p) {
    p[1] = 0;
    if (offmesh) {
        p[0] = 1.25f + float(i % 2) * .5f;
        p[2] = 1.25f + float(i / 2) * .5f;
    } else if (dense) {
        p[0] = 13 + float(i % 16) * .375f;
        p[2] = 14.5f + float(i / 16) * .375f;
    } else {
        p[0] = .5f + float(i % 4) * 8;
        p[2] = .5f + float(i / 4) * 24;
    }
}

static void crowdTarget(dtCrowd& c, const dtNavMesh& nav, int i, bool dense, bool offmesh, bool reverse) {
    float p[3], end[3];
    crowdStart(i, dense, offmesh, p);
    end[0] = reverse ? p[0] : (offmesh ? 12 : 32) - p[0];
    end[1] = 0;
    end[2] = reverse || offmesh ? p[2] : 32 - p[2];
    const int poly = offmesh ? int(!reverse) : int(end[2]) * 32 + int(end[0]);
    require(c.requestMoveTarget(i, nav.getPolyRefBase(nav.getTile(0)) | poly, end));
}

static void crowdSnapshot(dtCrowd& c, Hash& h) {
    for (int i = 0; i < c.getAgentCount(); ++i) {
        const dtCrowdAgent& a = *c.getAgent(i);
        h.word(a.active);
        if (!a.active) continue;
        h.word(a.state);
        h.word(a.partial);
        crowdFloat(h, a.topologyOptTime);
        h.word(a.nneis);
        for (const auto& n : a.neis) { h.word(n.idx); crowdFloat(h, n.dist); }
        crowdFloat(h, a.desiredSpeed);
        h.point(a.npos); h.point(a.disp); h.point(a.dvel); h.point(a.nvel); h.point(a.vel);
        const auto& p = a.params;
        for (float value : {p.radius, p.height, p.maxAcceleration, p.maxSpeed, p.collisionQueryRange, p.pathOptimizationRange, p.separationWeight}) crowdFloat(h, value);
        h.word(p.updateFlags);
        h.word(p.obstacleAvoidanceType);
        h.word(p.queryFilterType);
        h.word(reinterpret_cast<uintptr_t>(p.userData));
        h.word(a.ncorners);
        for (int j = 0; j < DT_CROWDAGENT_MAX_CORNERS; ++j) {
            h.point(a.cornerVerts + j * 3); h.word(a.cornerFlags[j]); h.word(a.cornerPolys[j]);
        }
        h.word(a.targetState);
        h.word(a.targetRef);
        h.point(a.targetPos);
        h.word(a.targetPathqRef);
        h.word(a.targetReplan);
        crowdFloat(h, a.targetReplanTime);
        h.point(a.corridor.getPos());
        h.point(a.corridor.getTarget());
        h.word(a.corridor.getPathCount());
        for (int j = 0; j < a.corridor.getPathCount(); ++j) h.word(a.corridor.getPath()[j]);
        h.point(a.boundary.getCenter());
        h.word(a.boundary.getSegmentCount());
        // Aggregate crowd agents initialize all eight backing entries.
        for (int j = 0; j < 8; ++j) { h.point(a.boundary.getSegment(j)); h.point(a.boundary.getSegment(j) + 3); }
    }
}

static uint64_t crowdCases(dtNavMesh& nav, bool dense, bool offmesh) {
    const int count = dense ? 128 : 8;
    dtCrowd c;
    require(c.init(count, .25f, &nav));
    dtCrowdAgentParams params = {};
    params.radius = .25f; params.height = 2; params.maxAcceleration = 8; params.maxSpeed = 3.5f;
    params.collisionQueryRange = 3; params.pathOptimizationRange = 7.5f;
    params.separationWeight = 2; params.updateFlags = 31;
    for (int i = 0; i < count; ++i) {
        auto p = params;
        p.userData = reinterpret_cast<void*>(uintptr_t(i + 1));
        float pos[3]; crowdStart(i, dense, offmesh, pos);
        require(c.addAgent(pos, &p) == i);
        crowdTarget(c, nav, i, dense, offmesh, false);
    }
    Hash h;
    int pause = 0;
    bool sawLink = false, finishedLink = false, sawReturnLink = false, finishedReturnLink = false;
    const int frames = offmesh ? 240 : 96;
    for (int frame = 0; frame < frames; ++frame) {
        if (!offmesh) {
            if (frame == 16) { float velocity[] = {.5f, 0, -.25f}; require(c.requestMoveVelocity(0, velocity)); }
            if (frame == 24) require(c.resetMoveTarget(0));
            if (frame == 32) c.removeAgent(1);
            if (frame == 36) { float pos[3]; crowdStart(1, dense, false, pos); require(c.addAgent(pos, &params) == 1); }
            if (frame == 48) for (int i = 0; i < count; ++i) crowdTarget(c, nav, i, dense, false, false);
        } else if (frame == 120) {
            for (int i = 0; i < count; ++i) crowdTarget(c, nav, i, false, true, true);
        }
        float dt = .05f;
        if (pause > 0) { --pause; dt = 0; }
        c.update(dt, nullptr);
        h.word(c.getVelocitySampleCount());
        if (offmesh) {
            for (int i = 0; i < count; ++i) {
                const auto& a = *c.getAgent(i);
                h.word(a.state);
                if (a.state == DT_CROWDAGENT_STATE_OFFMESH && !sawLink) { sawLink = true; pause = 3; }
                if (sawLink && a.state == DT_CROWDAGENT_STATE_WALKING && a.npos[0] > 8) finishedLink = true;
                if (frame >= 120 && a.state == DT_CROWDAGENT_STATE_OFFMESH) sawReturnLink = true;
                if (sawReturnLink && a.state == DT_CROWDAGENT_STATE_WALKING && a.npos[0] < 4) finishedReturnLink = true;
            }
        }
        if (frame % 12 == 0 || frame == frames - 1) crowdSnapshot(c, h);
    }
    if (offmesh) require(sawLink && finishedLink && sawReturnLink && finishedReturnLink);
    return h.value;
}

struct CrowdComponentBuffers {
    dtPolyRef path[128] = {}, cornerRefs[4] = {}, result[128];
    float corners[12] = {};
    unsigned char flags[4] = {};
    int resultCount = 128;
    CrowdComponentBuffers() { for (auto& r : result) r = 0xface; }
};

static uint64_t crowdCorridor(dtNavMesh& nav, CrowdComponentBuffers& output) {
    dtNavMeshQuery q;
    require(dtStatusSucceed(q.init(&nav, 512)));
    dtPathCorridor c;
    require(c.init(128));
    dtLocalBoundary boundary;
    int initializedSegments = 0;
    dtQueryFilter filter;
    const dtPolyRef base = nav.getPolyRefBase(static_cast<const dtNavMesh&>(nav).getTile(0));
    Hash h;
    for (int round = 0; round < 32; ++round) {
        const int a = round * 37 % 1024, b = (round * 113 + 17) % 1024;
        float start[3], end[3]; point(a, start); point(b, end);
        int count = 0;
        h.word(q.findPath(base | a, base | b, start, end, &filter, output.path, &count, 128));
        require(count > 0 && count < 128);
        c.reset(base | a, start);
        c.setCorridor(end, output.path, count);
        for (int step = 0; step < 4; ++step) {
            const int corners = c.findCorners(output.corners, output.flags, output.cornerRefs, 4, &q, &filter);
            h.word(corners);
            for (int j = 0; j < 4; ++j) { h.point(output.corners + j * 3); h.word(output.flags[j]); h.word(output.cornerRefs[j]); }
            c.optimizePathVisibility(corners > 0 ? output.corners : end, 7.5f, &q, &filter);
            h.word(c.optimizePathTopology(&q, &filter));
            float next[] = {start[0] + float(step + 1) * .125f, 0, start[2] + float(step + 1) * .0625f};
            h.word(c.movePosition(next, &q, &filter));
            float target[] = {end[0] - float(step + 1) * .0625f, 0, end[2] - float(step + 1) * .125f};
            h.word(c.moveTargetPosition(target, &q, &filter));
            boundary.update(c.getFirstPoly(), c.getPos(), 3, &q, &filter);
            h.word(boundary.isValid(&q, &filter));
            h.point(c.getPos()); h.point(c.getTarget());
            h.word(c.getPathCount());
            for (int j = 0; j < c.getPathCount(); ++j) h.word(c.getPath()[j]);
            h.point(boundary.getCenter()); h.word(boundary.getSegmentCount());
            if (boundary.getSegmentCount() > initializedSegments) initializedSegments = boundary.getSegmentCount();
            for (int j = 0; j < initializedSegments; ++j) { h.point(boundary.getSegment(j)); h.point(boundary.getSegment(j) + 3); }
        }
    }
    return h.value;
}

static void crowdQueueResult(Hash& h, dtPathQueue& q, dtPathQueueRef handle, int capacity, CrowdComponentBuffers& output) {
    h.word(q.getPathResult(handle, output.result, &output.resultCount, capacity));
    h.word(output.resultCount);
    for (int j = 0; j < output.resultCount; ++j) h.word(output.result[j]);
}

static uint64_t crowdQueue(dtNavMesh& nav, CrowdComponentBuffers& output) {
    dtPathQueue q;
    require(q.init(128, 512, &nav));
    dtQueryFilter filters[2];
    const dtPolyRef base = nav.getPolyRefBase(static_cast<const dtNavMesh&>(nav).getTile(0));
    Hash h;
    for (int round = 0; round < 32; ++round) {
        filters[1].setAreaCost(0, round % 2 == 0 ? 1.25f : 1.5f);
        dtPathQueueRef handles[8];
        for (int i = 0; i < 8; ++i) {
            const int a = (round * 37 + i * 31) % 1024, b = (round * 113 + i * 73 + 17) % 1024;
            float start[3], end[3]; point(a, start); point(b, end);
            handles[i] = q.request(base | a, base | b, start, end, &filters[i % 2]);
            require(handles[i] != 0);
            h.word(handles[i]);
        }
        float start[3], end[3]; point(0, start); point(1, end);
        h.word(q.request(base, base | 1, start, end, &filters[0]));
        bool done[8] = {}, complete = false;
        const int budgets[] = {0, 1, 7, 23, 100}, capacities[] = {0, 1, 8, 128};
        for (int step = 0; step < 4096; ++step) {
            q.update(budgets[step % 5]);
            for (int i = 0; i < 8; ++i) {
                if (done[i]) continue;
                const dtStatus status = q.getRequestStatus(handles[i]);
                h.word(status);
                if ((i == 7 && status == DT_FAILURE) || (i != 7 && (status & (DT_SUCCESS | DT_FAILURE)) != 0)) {
                    crowdQueueResult(h, q, handles[i], capacities[i % 4], output);
                    h.word(q.getRequestStatus(handles[i]));
                    crowdQueueResult(h, q, handles[i], capacities[i % 4], output);
                    done[i] = true;
                }
            }
            complete = true;
            for (bool value : done) if (!value) complete = false;
            if (complete) break;
        }
        require(complete);
    }
    return h.value;
}
