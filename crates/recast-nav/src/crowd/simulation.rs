// Altered Rust translation of DetourCrowd.cpp, RecastNavigation v1.6.0.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
use super::*;
use crate::common::{
    minimum as min, vector_add as add, vector_multiply_add as mad, vector_normalize as normalize,
    vector_scale as scale,
};
use crate::{
    nav::*,
    query::{NavMeshQuery, SurfaceMove},
};
pub const STATE_INVALID: u8 = 0;
pub const STATE_WALKING: u8 = 1;
pub const STATE_OFFMESH: u8 = 2;
pub const TARGET_NONE: u8 = 0;
pub const TARGET_FAILED: u8 = 1;
pub const TARGET_VALID: u8 = 2;
pub const TARGET_REQUESTING: u8 = 3;
pub const TARGET_WAITING_FOR_QUEUE: u8 = 4;
pub const TARGET_WAITING_FOR_PATH: u8 = 5;
pub const TARGET_VELOCITY: u8 = 6;
pub const ANTICIPATE_TURNS: u8 = 1;
pub const OBSTACLE_AVOIDANCE: u8 = 2;
pub const SEPARATION: u8 = 4;
pub const OPTIMIZE_VISIBILITY: u8 = 8;
pub const OPTIMIZE_TOPOLOGY: u8 = 16;
#[derive(Clone, Copy, Debug, Default)]
pub struct AgentParams {
    pub radius: f32,
    pub height: f32,
    pub max_acceleration: f32,
    pub max_speed: f32,
    pub collision_query_range: f32,
    pub path_optimization_range: f32,
    pub separation_weight: f32,
    pub update_flags: u8,
    pub obstacle_avoidance_type: u8,
    pub query_filter_type: u8,
    pub user_data: usize,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Neighbor {
    pub index: i32,
    pub distance_squared: f32,
}
#[derive(Clone, Debug)]
pub struct Agent {
    pub active: bool,
    pub state: u8,
    pub partial: bool,
    pub corridor: PathCorridor,
    pub boundary: LocalBoundary,
    pub topology_optimization_time: f32,
    /// Native fixed backing; initialized slots remain after the live count shrinks.
    pub neighbors: [Neighbor; MAX_NEIGHBORS],
    pub neighbor_count: usize,
    pub desired_speed: f32,
    pub position: Point,
    pub displacement: Point,
    pub desired_velocity: Point,
    pub new_velocity: Point,
    pub velocity: Point,
    pub params: AgentParams,
    /// Native fixed backing, including slots retained by corner pruning.
    pub corners: [Corner; MAX_CORNERS],
    pub corner_count: usize,
    pub target_state: u8,
    pub target_reference: PolyRef,
    pub target_position: Point,
    pub target_path_queue_reference: PathQueueRef,
    pub target_replan: bool,
    pub target_replan_time: f32,
}
impl Agent {
    /// The live prefix of the native neighbor backing.
    pub fn neighbors(&self) -> &[Neighbor] {
        &self.neighbors[..self.neighbor_count]
    }
    /// The live prefix of the native corner backing.
    pub fn corners(&self) -> &[Corner] {
        &self.corners[..self.corner_count]
    }
    fn new() -> Result<Self, u32> {
        Ok(Self {
            active: false,
            state: STATE_INVALID,
            partial: false,
            corridor: PathCorridor::new(256)?,
            boundary: LocalBoundary::zero_initialized(),
            topology_optimization_time: 0.,
            neighbors: [Neighbor::default(); MAX_NEIGHBORS],
            neighbor_count: 0,
            desired_speed: 0.,
            position: [0.; 3],
            displacement: [0.; 3],
            desired_velocity: [0.; 3],
            new_velocity: [0.; 3],
            velocity: [0.; 3],
            params: AgentParams::default(),
            corners: [Corner::default(); MAX_CORNERS],
            corner_count: 0,
            target_state: TARGET_NONE,
            target_reference: 0,
            target_position: [0.; 3],
            target_path_queue_reference: 0,
            target_replan: false,
            target_replan_time: 0.,
        })
    }
    fn request_target(&mut self, reference: PolyRef, position: Point, replan: bool) {
        self.target_reference = reference;
        self.target_position = position;
        self.target_path_queue_reference = 0;
        self.target_replan = replan;
        self.target_state = if reference != 0 {
            TARGET_REQUESTING
        } else {
            TARGET_FAILED
        };
    }
}
/// Native standalone `dtCrowdAgentAnimation` data. Crowd owns its attached
/// animation storage; this value exposes the same fields without a raw pointer.
#[derive(Clone, Copy, Debug, Default)]
pub struct AgentAnimation {
    pub active: bool,
    pub initial_position: Point,
    pub start_position: Point,
    pub end_position: Point,
    pub reference: PolyRef,
    pub time: f32,
    pub max_time: f32,
}
#[derive(Debug, Default)]
pub struct CrowdDebugInfo<'a> {
    pub index: i32,
    pub optimization_start: Point,
    pub optimization_end: Point,
    pub avoidance: Option<&'a mut AvoidanceDebugData>,
}
/// Supply the same mesh for operations, permitting safe tile mutation between
/// updates instead of retaining an attached native pointer.
pub struct Crowd {
    agents: Vec<Agent>,
    animations: Vec<AgentAnimation>,
    filters: [Filter; MAX_QUERY_FILTERS],
    avoidance_parameters: [AvoidanceParams; MAX_AVOIDANCE_PARAMS],
    avoidance: ObstacleAvoidanceQuery,
    grid: ProximityGrid,
    path_queue: PathQueue<usize>,
    query: NavMeshQuery,
    path_result: Vec<PolyRef>,
    half_extents: Point,
    active_scratch: Vec<usize>,
    corner_scratch: StraightPath,
    raycast_scratch: RaycastHit,
    surface_scratch: SurfaceMove,
    velocity_sample_count: usize,
}
fn allocated<T>(count: usize, mut value: impl FnMut() -> Result<T, u32>) -> Result<Vec<T>, u32> {
    let mut out = Vec::new();
    out.try_reserve_exact(count)
        .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
    for _ in 0..count {
        out.push(value()?);
    }
    Ok(out)
}
fn preset() -> AvoidanceParams {
    AvoidanceParams {
        velocity_bias: 0.4,
        weight_desired_velocity: 2.,
        weight_current_velocity: 0.75,
        weight_side: 0.75,
        weight_time_of_impact: 2.5,
        horizon_time: 2.5,
        grid_size: 33,
        adaptive_divisions: 7,
        adaptive_rings: 2,
        adaptive_depth: 5,
    }
}
fn squared_2d(a: Point, b: Point) -> f32 {
    let d = sub(b, a);
    madd(d[0], d[0], d[2] * d[2])
}
fn prioritized(
    queue: &mut [usize],
    count: usize,
    index: usize,
    time: impl Fn(usize) -> f32,
) -> Result<usize, u32> {
    let limit = queue.len();
    let value = time(index);
    let position = if count == 0 {
        0
    } else if value <= time(queue[count - 1]) {
        if count >= limit {
            return Ok(count);
        }
        count
    } else {
        queue[..count]
            .iter()
            .position(|&i| value >= time(i))
            .unwrap_or(count)
    };
    if position >= limit {
        return Err(FAILURE | INVALID_PARAM);
    }
    let moved = count.min(limit - 1) - position;
    queue.copy_within(position..position + moved, position + 1);
    queue[position] = index;
    Ok((count + 1).min(limit))
}

impl Crowd {
    pub fn new(max_agents: usize, max_agent_radius: f32) -> Result<Self, u32> {
        let pool = max_agents
            .checked_mul(4)
            .filter(|&n| n <= i32::MAX as usize)
            .ok_or(FAILURE | INVALID_PARAM)?;
        let grid = ProximityGrid::new(pool, max_agent_radius * 3.)?;
        let agents = allocated(max_agents, Agent::new)?;
        let animations = allocated(max_agents, || Ok(AgentAnimation::default()))?;
        let mut path_result = Vec::new();
        path_result
            .try_reserve_exact(256)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        let mut active_scratch = Vec::new();
        active_scratch
            .try_reserve_exact(max_agents)
            .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
        Ok(Self {
            agents,
            animations,
            filters: std::array::from_fn(|_| Filter::default()),
            avoidance_parameters: [preset(); MAX_AVOIDANCE_PARAMS],
            avoidance: ObstacleAvoidanceQuery::new(6, 8)?,
            grid,
            path_queue: PathQueue::new(256, 4096)?,
            query: NavMeshQuery::new(512)?,
            path_result,
            active_scratch,
            corner_scratch: StraightPath::default(),
            raycast_scratch: RaycastHit::default(),
            surface_scratch: SurfaceMove::default(),
            half_extents: [
                max_agent_radius * 2.,
                max_agent_radius * 1.5,
                max_agent_radius * 2.,
            ],
            velocity_sample_count: 0,
        })
    }
    pub fn reinitialize(&mut self, max_agents: usize, max_agent_radius: f32) -> Result<(), u32> {
        let fresh = Self::new(max_agents, max_agent_radius)?;
        self.agents = fresh.agents;
        self.animations = fresh.animations;
        self.avoidance_parameters = fresh.avoidance_parameters;
        self.avoidance = fresh.avoidance;
        self.grid = fresh.grid;
        self.path_queue.replace_storage(fresh.path_queue);
        self.query = fresh.query;
        self.path_result = fresh.path_result;
        self.active_scratch = fresh.active_scratch;
        self.corner_scratch = fresh.corner_scratch;
        self.raycast_scratch = fresh.raycast_scratch;
        self.surface_scratch = fresh.surface_scratch;
        self.half_extents = fresh.half_extents;
        Ok(())
    }
    pub fn agent_count(&self) -> usize {
        self.agents.len()
    }
    pub fn agent(&self, index: i32) -> Option<&Agent> {
        self.agents.get(index as usize)
    }
    pub fn editable_agent(&mut self, index: i32) -> Option<&mut Agent> {
        self.agents.get_mut(index as usize)
    }
    pub fn active_agents(&self, capacity: usize) -> Vec<usize> {
        self.agents
            .iter()
            .enumerate()
            .filter_map(|(i, a)| a.active.then_some(i))
            .take(capacity)
            .collect()
    }
    pub fn filter(&self, index: i32) -> Option<&Filter> {
        self.filters.get(index as usize)
    }
    pub fn editable_filter(&mut self, index: i32) -> Option<&mut Filter> {
        self.filters.get_mut(index as usize)
    }
    pub fn avoidance_params(&self, index: i32) -> Option<&AvoidanceParams> {
        self.avoidance_parameters.get(index as usize)
    }
    pub fn set_avoidance_params(&mut self, index: i32, params: &AvoidanceParams) {
        if let Some(p) = self.avoidance_parameters.get_mut(index as usize) {
            *p = *params;
        }
    }
    pub fn query_half_extents(&self) -> Point {
        self.half_extents
    }
    pub fn velocity_sample_count(&self) -> usize {
        self.velocity_sample_count
    }
    pub fn grid(&self) -> &ProximityGrid {
        &self.grid
    }
    pub fn path_queue(&self) -> &PathQueue<usize> {
        &self.path_queue
    }
    pub fn query(&self) -> &NavMeshQuery {
        &self.query
    }
    pub fn update_agent_parameters(&mut self, index: i32, params: &AgentParams) {
        if let Some(a) = self.editable_agent(index) {
            a.params = *params;
        }
    }
    pub fn add_agent(
        &mut self,
        mesh: &NavMesh,
        position: Point,
        params: &AgentParams,
    ) -> Result<i32, u32> {
        let Some(index) = self.agents.iter().position(|a| !a.active) else {
            return Ok(-1);
        };
        let agent = &mut self.agents[index];
        agent.params = *params;
        let filter = self
            .filters
            .get(params.query_filter_type as usize)
            .ok_or(FAILURE | INVALID_PARAM)?;
        let (reference, position) = mesh
            .nearest(position, self.half_extents, filter)
            .1
            .unwrap_or((0, position));
        agent.corridor.reset(reference, position);
        agent.boundary.reset();
        agent.partial = false;
        agent.topology_optimization_time = 0.;
        agent.target_replan_time = 0.;
        agent.neighbor_count = 0;
        agent.desired_velocity = [0.; 3];
        agent.new_velocity = [0.; 3];
        agent.velocity = [0.; 3];
        agent.position = position;
        agent.desired_speed = 0.;
        agent.state = if reference != 0 {
            STATE_WALKING
        } else {
            STATE_INVALID
        };
        agent.target_state = TARGET_NONE;
        agent.active = true;
        Ok(index as i32)
    }
    pub fn remove_agent(&mut self, index: i32) {
        if let Some(a) = self.editable_agent(index) {
            a.active = false;
        }
    }
    pub fn request_move_target(&mut self, index: i32, reference: PolyRef, position: Point) -> bool {
        if reference == 0 {
            return false;
        }
        let Some(a) = self.editable_agent(index) else {
            return false;
        };
        a.request_target(reference, position, false);
        true
    }
    pub fn request_move_velocity(&mut self, index: i32, velocity: Point) -> bool {
        let Some(a) = self.editable_agent(index) else {
            return false;
        };
        a.target_reference = 0;
        a.target_position = velocity;
        a.target_path_queue_reference = 0;
        a.target_replan = false;
        a.target_state = TARGET_VELOCITY;
        true
    }
    pub fn reset_move_target(&mut self, index: i32) -> bool {
        let Some(a) = self.editable_agent(index) else {
            return false;
        };
        a.target_reference = 0;
        a.target_position = [0.; 3];
        a.desired_velocity = [0.; 3];
        a.target_path_queue_reference = 0;
        a.target_replan = false;
        a.target_state = TARGET_NONE;
        true
    }
    fn check_path_validity(
        &mut self,
        mesh: &NavMesh,
        active: &[usize],
        dt: f32,
    ) -> Result<(), u32> {
        for &index in active {
            let agent = &mut self.agents[index];
            if agent.state != STATE_WALKING {
                continue;
            }
            agent.target_replan_time += dt;
            let filter = self
                .filters
                .get(agent.params.query_filter_type as usize)
                .ok_or(FAILURE | INVALID_PARAM)?;
            let mut replan = false;
            let mut position = agent.position;
            let mut reference = agent.corridor.first_polygon();
            if !self.query.is_valid_poly_ref(mesh, reference, filter) {
                let (_, nearest) = mesh.nearest(agent.position, self.half_extents, filter);
                reference = nearest.map_or(0, |p| p.0);
                if let Some(p) = nearest {
                    position = p.1;
                }
                if reference == 0 {
                    agent.corridor.reset(0, position);
                    agent.partial = false;
                    agent.boundary.reset();
                    agent.state = STATE_INVALID;
                    continue;
                }
                agent.corridor.fix_path_start(reference, position);
                agent.boundary.reset();
                agent.position = position;
                replan = true;
            }
            if agent.target_state == TARGET_NONE || agent.target_state == TARGET_VELOCITY {
                continue;
            }
            if agent.target_state != TARGET_NONE && agent.target_state != TARGET_FAILED {
                if !self
                    .query
                    .is_valid_poly_ref(mesh, agent.target_reference, filter)
                {
                    let (_, nearest) =
                        mesh.nearest(agent.target_position, self.half_extents, filter);
                    agent.target_reference = nearest.map_or(0, |p| p.0);
                    if let Some(p) = nearest {
                        agent.target_position = p.1;
                    }
                    replan = true;
                }
                if agent.target_reference == 0 {
                    agent.corridor.reset(reference, position);
                    agent.partial = false;
                    agent.target_state = TARGET_NONE;
                }
            }
            if !agent.corridor.is_valid(10, mesh, &self.query, filter) {
                replan = true;
            }
            if agent.target_state == TARGET_VALID
                && agent.target_replan_time > 1.
                && agent.corridor.path().len() < 10
                && agent.corridor.last_polygon() != agent.target_reference
            {
                replan = true;
            }
            if replan && agent.target_state != TARGET_NONE {
                agent.request_target(agent.target_reference, agent.target_position, true);
            }
        }
        Ok(())
    }
    fn update_move_requests(&mut self, mesh: &NavMesh) -> Result<(), u32> {
        let mut waiting = [0; 8];
        let mut waiting_count = 0;
        for index in 0..self.agents.len() {
            let a = &mut self.agents[index];
            if !a.active
                || a.state == STATE_INVALID
                || a.target_state == TARGET_NONE
                || a.target_state == TARGET_VELOCITY
            {
                continue;
            }
            if a.target_state == TARGET_REQUESTING {
                let first = a.corridor.first_polygon();
                if a.corridor.path().is_empty() {
                    return Err(FAILURE | INVALID_PARAM);
                }
                let filter = self
                    .filters
                    .get(a.params.query_filter_type as usize)
                    .ok_or(FAILURE | INVALID_PARAM)?;
                self.query.init_sliced_find_path(
                    mesh,
                    first,
                    a.target_reference,
                    a.position,
                    a.target_position,
                    filter,
                    0,
                );
                self.query.update_sliced_find_path(mesh, filter, 20, None);
                let mut result = Corridor {
                    polygons: std::mem::take(&mut self.path_result),
                    ..Default::default()
                };
                if a.target_replan {
                    self.query.finalize_sliced_find_path_partial_into(
                        mesh,
                        a.corridor.path(),
                        filter,
                        32,
                        &mut result,
                    );
                } else {
                    self.query
                        .finalize_sliced_find_path_into(mesh, filter, 32, &mut result);
                }
                let mut target = a.target_position;
                if result.status & FAILURE == 0 && !result.polygons.is_empty() {
                    if result.polygons.last() != Some(&a.target_reference) {
                        if let Some((p, _)) =
                            mesh.closest_point(*result.polygons.last().unwrap(), a.target_position)
                        {
                            target = p;
                        } else {
                            result.polygons.clear();
                        }
                    }
                } else {
                    result.polygons.clear();
                }
                if result.polygons.is_empty() {
                    target = a.position;
                    result.polygons.push(first);
                }
                a.corridor.set_corridor(target, &result.polygons);
                a.boundary.reset();
                a.partial = false;
                if result.polygons.last() == Some(&a.target_reference) {
                    a.target_state = TARGET_VALID;
                    a.target_replan_time = 0.;
                } else {
                    a.target_state = TARGET_WAITING_FOR_QUEUE;
                }
                self.path_result = result.polygons;
            }
            if a.target_state == TARGET_WAITING_FOR_QUEUE {
                waiting_count = prioritized(&mut waiting, waiting_count, index, |i| {
                    self.agents[i].target_replan_time
                })?;
            }
        }
        for &index in &waiting[..waiting_count] {
            let a = &mut self.agents[index];
            let key = a.params.query_filter_type as usize;
            if key >= 16 {
                return Err(FAILURE | INVALID_PARAM);
            }
            a.target_path_queue_reference = self.path_queue.request(
                a.corridor.last_polygon(),
                a.target_reference,
                a.corridor.target(),
                a.target_position,
                Some(key),
            );
            if a.target_path_queue_reference != 0 {
                a.target_state = TARGET_WAITING_FOR_PATH;
            }
        }
        self.path_queue.update(mesh, 100, |&i| &self.filters[i]);
        for a in &mut self.agents {
            if !a.active || a.target_state != TARGET_WAITING_FOR_PATH {
                continue;
            }
            let status = self
                .path_queue
                .request_status(a.target_path_queue_reference);
            if status & FAILURE != 0 {
                a.target_path_queue_reference = 0;
                a.target_state = if a.target_reference != 0 {
                    TARGET_REQUESTING
                } else {
                    TARGET_FAILED
                };
                a.target_replan_time = 0.;
            } else if status & SUCCESS != 0 {
                let path = a.corridor.path();
                if path.is_empty() {
                    return Err(FAILURE | INVALID_PARAM);
                }
                let mut target = a.target_position;
                let status = self.path_queue.path_result_into(
                    a.target_path_queue_reference,
                    256,
                    &mut self.path_result,
                );
                let mut valid = status & FAILURE == 0 && !self.path_result.is_empty();
                a.partial = status & PARTIAL_RESULT != 0;
                if valid && path.last() != self.path_result.first() {
                    valid = false;
                }
                if valid {
                    if path.len() > 1 {
                        let prefix = path.len() - 1;
                        if prefix > 256 {
                            return Err(FAILURE | INVALID_PARAM);
                        }
                        self.path_result.truncate(256 - prefix);
                        self.path_result
                            .splice(0..0, path[..prefix].iter().copied());
                        let mut j = 0isize;
                        while j < self.path_result.len() as isize {
                            if j > 0
                                && j + 1 < self.path_result.len() as isize
                                && self.path_result[j as usize - 1]
                                    == self.path_result[j as usize + 1]
                            {
                                self.path_result.drain(j as usize - 1..j as usize + 1);
                                j -= 2;
                            }
                            j += 1;
                        }
                    }
                    if let Some(&last) = self.path_result.last() {
                        if last != a.target_reference {
                            if let Some((p, _)) = mesh.closest_point(last, target) {
                                target = p;
                            } else {
                                valid = false;
                            }
                        }
                    } else {
                        valid = false;
                    }
                }
                if valid {
                    if !a.corridor.set_corridor(target, &self.path_result) {
                        return Err(FAILURE | INVALID_PARAM);
                    }
                    a.boundary.reset();
                    a.target_state = TARGET_VALID;
                } else {
                    a.target_state = TARGET_FAILED;
                }
                a.target_replan_time = 0.;
            }
        }
        Ok(())
    }
    fn update_topology(&mut self, mesh: &NavMesh, active: &[usize], dt: f32) -> Result<(), u32> {
        let mut queue = [0; 1];
        let mut queue_count = 0;
        for &index in active {
            let a = &mut self.agents[index];
            if a.state != STATE_WALKING
                || a.target_state == TARGET_NONE
                || a.target_state == TARGET_VELOCITY
                || a.params.update_flags & OPTIMIZE_TOPOLOGY == 0
            {
                continue;
            }
            a.topology_optimization_time += dt;
            if a.topology_optimization_time >= 0.5 {
                queue_count = prioritized(&mut queue, queue_count, index, |i| {
                    self.agents[i].topology_optimization_time
                })?;
            }
        }
        for &index in &queue[..queue_count] {
            let a = &mut self.agents[index];
            let filter = self
                .filters
                .get(a.params.query_filter_type as usize)
                .ok_or(FAILURE | INVALID_PARAM)?;
            let mut result = Corridor {
                polygons: std::mem::take(&mut self.path_result),
                ..Default::default()
            };
            a.corridor
                .optimize_topology_with_storage(mesh, &mut self.query, filter, &mut result);
            self.path_result = result.polygons;
            a.topology_optimization_time = 0.;
        }
        Ok(())
    }
    fn update_neighbors(&mut self, active: &[usize], index: usize) -> Result<(), u32> {
        let a = &self.agents[index];
        let range = a.params.collision_query_range;
        let mut ids = [0; 32];
        let id_count = self.grid.query_items_slice(
            [
                [a.position[0] - range, a.position[2] - range],
                [a.position[0] + range, a.position[2] + range],
            ],
            &mut ids,
        )?;
        let mut neighbors = a.neighbors;
        let mut count = 0;
        for &id in &ids[..id_count] {
            let other = *active.get(id as usize).ok_or(FAILURE | INVALID_PARAM)?;
            if other == index {
                continue;
            }
            let b = &self.agents[other];
            let mut difference = sub(a.position, b.position);
            if difference[1].abs() >= (a.params.height + b.params.height) / 2. {
                continue;
            }
            difference[1] = 0.;
            let distance_squared = length_squared(difference);
            if distance_squared > range * range {
                continue;
            }
            let position = if count == 0 {
                0
            } else if distance_squared >= neighbors[count - 1].distance_squared {
                if count == MAX_NEIGHBORS {
                    continue;
                }
                count
            } else {
                neighbors[..count]
                    .iter()
                    .position(|n| distance_squared <= n.distance_squared)
                    .unwrap_or(count)
            };
            if position >= MAX_NEIGHBORS {
                return Err(FAILURE | INVALID_PARAM);
            }
            let moved = count.min(MAX_NEIGHBORS - 1) - position;
            neighbors.copy_within(position..position + moved, position + 1);
            neighbors[position] = Neighbor {
                index: other as i32,
                distance_squared,
            };
            count = (count + 1).min(MAX_NEIGHBORS);
        }
        self.agents[index].neighbors = neighbors;
        self.agents[index].neighbor_count = count;
        Ok(())
    }
    /// Runs native phases in order. Failures check native invalid memory or
    /// conversion domains; defined numeric edge cases retain native behavior.
    pub fn update(
        &mut self,
        mesh: &NavMesh,
        dt: f32,
        debug: Option<&mut CrowdDebugInfo<'_>>,
    ) -> Result<(), u32> {
        self.velocity_sample_count = 0;
        // Move the vector header so phase methods can borrow the whole Crowd.
        // Always return its capacity, including on checked errors.
        let mut active = std::mem::take(&mut self.active_scratch);
        active.clear();
        active.extend(
            self.agents
                .iter()
                .enumerate()
                .filter_map(|(i, a)| a.active.then_some(i)),
        );
        let result = self.update_active(mesh, dt, debug, &active);
        self.active_scratch = active;
        result
    }
    fn update_active(
        &mut self,
        mesh: &NavMesh,
        dt: f32,
        mut debug: Option<&mut CrowdDebugInfo<'_>>,
        active: &[usize],
    ) -> Result<(), u32> {
        let debug_index = debug.as_ref().map_or(-1, |d| d.index);
        self.check_path_validity(mesh, active, dt)?;
        self.update_move_requests(mesh)?;
        self.update_topology(mesh, active, dt)?;
        self.grid.clear();
        for (ordinal, &index) in active.iter().enumerate() {
            let a = &self.agents[index];
            let p = a.position;
            let r = a.params.radius;
            let status = self
                .grid
                .add_item(ordinal as u16, [[p[0] - r, p[2] - r], [p[0] + r, p[2] + r]]);
            if status & FAILURE != 0 {
                return Err(status);
            }
        }
        for &index in active {
            let a = &mut self.agents[index];
            if a.state != STATE_WALKING {
                continue;
            }
            let filter = self
                .filters
                .get(a.params.query_filter_type as usize)
                .ok_or(FAILURE | INVALID_PARAM)?;
            let threshold = a.params.collision_query_range * 0.25;
            if squared_2d(a.position, a.boundary.center()) > threshold * threshold
                || !a.boundary.is_valid(mesh, &self.query, filter)
            {
                a.boundary.update(
                    a.corridor.first_polygon(),
                    a.position,
                    a.params.collision_query_range,
                    mesh,
                    &mut self.query,
                    filter,
                );
            }
            self.update_neighbors(active, index)?;
        }
        for (ordinal, &index) in active.iter().enumerate() {
            let a = &mut self.agents[index];
            if a.state != STATE_WALKING
                || a.target_state == TARGET_NONE
                || a.target_state == TARGET_VELOCITY
            {
                continue;
            }
            a.corner_count = a.corridor.find_corners_into(
                mesh,
                &self.query,
                &mut a.corners,
                &mut self.corner_scratch,
            );
            if a.params.update_flags & OPTIMIZE_VISIBILITY != 0 && !a.corners().is_empty() {
                let target = a.corners[1.min(a.corners().len() - 1)].position;
                let filter = self
                    .filters
                    .get(a.params.query_filter_type as usize)
                    .ok_or(FAILURE | INVALID_PARAM)?;
                a.corridor.optimize_visibility_with_storage(
                    target,
                    a.params.path_optimization_range,
                    mesh,
                    &self.query,
                    filter,
                    &mut self.raycast_scratch,
                );
                if debug_index == ordinal as i32 {
                    let d = debug.as_deref_mut().unwrap();
                    d.optimization_start = a.corridor.position();
                    d.optimization_end = target;
                }
            } else if debug_index == ordinal as i32 {
                let d = debug.as_deref_mut().unwrap();
                d.optimization_start = [0.; 3];
                d.optimization_end = [0.; 3];
            }
        }
        for &index in active {
            let a = &mut self.agents[index];
            if a.state != STATE_WALKING
                || a.target_state == TARGET_NONE
                || a.target_state == TARGET_VELOCITY
            {
                continue;
            }
            let radius = a.params.radius * 2.25;
            let Some(last) = a.corners().last() else {
                continue;
            };
            if last.flags & 4 != 0 && squared_2d(a.position, last.position) < radius * radius {
                let animation = &mut self.animations[index];
                let mut output = OffMeshMove {
                    references: [0; 2],
                    positions: [animation.start_position, animation.end_position],
                };
                let success = a.corridor.move_over_off_mesh_connection_into(
                    last.reference,
                    mesh,
                    &mut output,
                );
                animation.start_position = output.positions[0];
                animation.end_position = output.positions[1];
                if success {
                    animation.initial_position = a.position;
                    animation.reference = output.references[1];
                    animation.active = true;
                    animation.time = 0.;
                    animation.max_time =
                        (squared_2d(animation.start_position, animation.end_position).sqrt()
                            / a.params.max_speed)
                            * 0.5;
                    a.state = STATE_OFFMESH;
                    a.corner_count = 0;
                    a.neighbor_count = 0;
                }
            }
        }
        for &index in active {
            let a = &self.agents[index];
            if a.state != STATE_WALKING || a.target_state == TARGET_NONE {
                continue;
            }
            let mut desired = [0.; 3];
            let desired_speed;
            if a.target_state == TARGET_VELOCITY {
                desired = a.target_position;
                desired_speed = length_squared(a.target_position).sqrt();
            } else {
                if !a.corners().is_empty() {
                    let mut direction = sub(a.corners[0].position, a.position);
                    direction[1] = 0.;
                    if a.params.update_flags & ANTICIPATE_TURNS != 0 {
                        let mut second =
                            sub(a.corners[1.min(a.corners().len() - 1)].position, a.position);
                        second[1] = 0.;
                        let first_length = length_squared(direction).sqrt();
                        let second_length = length_squared(second).sqrt();
                        if second_length > 0.001 {
                            second = scale(second, 1. / second_length);
                        }
                        direction = [
                            madd(-(second[0] * first_length), 0.5, direction[0]),
                            0.,
                            madd(-(second[2] * first_length), 0.5, direction[2]),
                        ];
                    }
                    desired = normalize(direction);
                }
                let radius = a.params.radius * 2.;
                let goal_distance =
                    if let Some(last) = a.corners().last().filter(|c| c.flags & 2 != 0) {
                        min(squared_2d(a.position, last.position).sqrt(), radius)
                    } else {
                        radius
                    };
                desired_speed = a.params.max_speed;
                desired = scale(desired, desired_speed * (goal_distance / radius));
            }
            if a.params.update_flags & SEPARATION != 0 {
                let range = a.params.collision_query_range;
                let inverse = 1. / range;
                let mut weight = 0.;
                let mut displacement = [0.; 3];
                for n in a.neighbors() {
                    let b = self
                        .agents
                        .get(n.index as usize)
                        .ok_or(FAILURE | INVALID_PARAM)?;
                    let mut difference = sub(a.position, b.position);
                    difference[1] = 0.;
                    let squared = length_squared(difference);
                    if squared < 0.00001 || squared > range * range {
                        continue;
                    }
                    let distance = squared.sqrt();
                    let relative = distance * inverse;
                    let contribution = a.params.separation_weight * (1. - relative * relative);
                    displacement = mad(displacement, difference, contribution / distance);
                    weight += 1.;
                }
                if weight > 0.0001 {
                    desired = mad(desired, displacement, 1. / weight);
                    let squared = length_squared(desired);
                    let limit = desired_speed * desired_speed;
                    if squared > limit {
                        desired = scale(desired, limit / squared);
                    }
                }
            }
            self.agents[index].desired_speed = desired_speed;
            self.agents[index].desired_velocity = desired;
        }
        for (ordinal, &index) in active.iter().enumerate() {
            let a = &self.agents[index];
            if a.state != STATE_WALKING {
                continue;
            }
            let velocity = if a.params.update_flags & OBSTACLE_AVOIDANCE != 0 {
                self.avoidance.reset();
                for neighbor in a.neighbors() {
                    let b = self
                        .agents
                        .get(neighbor.index as usize)
                        .ok_or(FAILURE | INVALID_PARAM)?;
                    self.avoidance.add_circle(
                        b.position,
                        b.params.radius,
                        b.velocity,
                        b.desired_velocity,
                    );
                }
                for s in a.boundary.segments() {
                    if area(a.position, s.positions[0], s.positions[1]) < 0. {
                        continue;
                    }
                    self.avoidance.add_segment(s.positions);
                }
                let data = if debug_index == ordinal as i32 {
                    debug.as_deref_mut().unwrap().avoidance.as_deref_mut()
                } else {
                    None
                };
                let params = self
                    .avoidance_parameters
                    .get(a.params.obstacle_avoidance_type as usize)
                    .ok_or(FAILURE | INVALID_PARAM)?;
                let result = self.avoidance.sample_velocity_adaptive(
                    VelocityInput {
                        position: a.position,
                        radius: a.params.radius,
                        maximum_speed: a.desired_speed,
                        velocity: a.velocity,
                        desired_velocity: a.desired_velocity,
                    },
                    params,
                    data,
                )?;
                self.velocity_sample_count += result.sample_count;
                result.velocity
            } else {
                a.desired_velocity
            };
            self.agents[index].new_velocity = velocity;
        }
        for &index in active {
            let a = &mut self.agents[index];
            if a.state != STATE_WALKING {
                continue;
            }
            let maximum = a.params.max_acceleration * dt;
            let mut difference = sub(a.new_velocity, a.velocity);
            let length = length_squared(difference).sqrt();
            if length > maximum {
                difference = scale(difference, maximum / length);
            }
            a.velocity = add(a.velocity, difference);
            if length_squared(a.velocity).sqrt() > 0.0001 {
                a.position = mad(a.position, a.velocity, dt);
            } else {
                a.velocity = [0.; 3];
            }
        }
        for _ in 0..4 {
            for &index in active {
                let a = &self.agents[index];
                if a.state != STATE_WALKING {
                    continue;
                }
                let mut displacement = [0.; 3];
                let mut weight = 0.;
                for neighbor in a.neighbors() {
                    let b = self
                        .agents
                        .get(neighbor.index as usize)
                        .ok_or(FAILURE | INVALID_PARAM)?;
                    let mut difference = sub(a.position, b.position);
                    difference[1] = 0.;
                    let mut distance = length_squared(difference);
                    let radius = a.params.radius + b.params.radius;
                    if distance > radius * radius {
                        continue;
                    }
                    distance = distance.sqrt();
                    let mut penetration = radius - distance;
                    if distance < 0.0001 {
                        difference = if index as i32 > neighbor.index {
                            [-a.desired_velocity[2], 0., a.desired_velocity[0]]
                        } else {
                            [a.desired_velocity[2], 0., -a.desired_velocity[0]]
                        };
                        penetration = 0.01;
                    } else {
                        penetration = (1. / distance) * (penetration * 0.5) * 0.7;
                    }
                    displacement = mad(displacement, difference, penetration);
                    weight += 1.;
                }
                if weight > 0.0001 {
                    displacement = scale(displacement, 1. / weight);
                }
                self.agents[index].displacement = displacement;
            }
            for &index in active {
                let a = &mut self.agents[index];
                if a.state == STATE_WALKING {
                    a.position = add(a.position, a.displacement);
                }
            }
        }
        for &index in active {
            let a = &mut self.agents[index];
            if a.state != STATE_WALKING {
                continue;
            }
            let filter = self
                .filters
                .get(a.params.query_filter_type as usize)
                .ok_or(FAILURE | INVALID_PARAM)?;
            a.corridor.move_position_with_storage(
                a.position,
                mesh,
                &mut self.query,
                filter,
                &mut self.surface_scratch,
            );
            a.position = a.corridor.position();
            if a.target_state == TARGET_NONE || a.target_state == TARGET_VELOCITY {
                a.corridor.reset(a.corridor.first_polygon(), a.position);
                a.partial = false;
            }
        }
        for &index in active {
            let animation = &mut self.animations[index];
            if !animation.active {
                continue;
            }
            let a = &mut self.agents[index];
            animation.time += dt;
            if animation.time > animation.max_time {
                animation.active = false;
                a.state = STATE_WALKING;
                continue;
            }
            let threshold = animation.max_time * 0.15;
            let (from, to, start, end) = if animation.time < threshold {
                (
                    animation.initial_position,
                    animation.start_position,
                    0.,
                    threshold,
                )
            } else {
                (
                    animation.start_position,
                    animation.end_position,
                    threshold,
                    animation.max_time,
                )
            };
            let value = ((animation.time - start) / (end - start)).clamp(0., 1.);
            a.position = lerp(from, to, value);
            a.velocity = [0.; 3];
            a.desired_velocity = [0.; 3];
        }
        Ok(())
    }
}
