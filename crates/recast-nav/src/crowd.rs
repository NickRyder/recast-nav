//! DetourCrowd simulation and independently usable components.
//! Algorithms preserve native ordering and capacities;
//! Rust ownership replaces native allocation and pointer lifetimes.
pub const MAX_NEIGHBORS: usize = 6;
pub const MAX_CORNERS: usize = 4;
pub const MAX_AVOIDANCE_PARAMS: usize = 8;
pub const MAX_QUERY_FILTERS: usize = 16;
pub const MAX_PATTERN_DIVISIONS: usize = 32;
pub const MAX_PATTERN_RINGS: usize = 4;
pub const PATHQ_INVALID: PathQueueRef = 0;

mod proximity_grid;
pub use proximity_grid::ProximityGrid;
mod local_boundary;
pub use local_boundary::{BoundarySegment, LocalBoundary};
mod path_queue;
pub use path_queue::{PathQueue, PathQueueRef};
mod path_corridor;
pub use path_corridor::{
    Corner, OffMeshMove, PathCorridor, merge_corridor_end_moved, merge_corridor_start_moved,
    merge_corridor_start_shortcut,
};
mod avoidance;
pub use avoidance::{
    AvoidanceDebugData, AvoidanceDebugSample, AvoidanceParams, ObstacleAvoidanceQuery,
    ObstacleCircle, ObstacleSegment, VelocityInput, VelocitySample,
};
mod simulation;
pub use simulation::{
    ANTICIPATE_TURNS, Agent, AgentAnimation, AgentParams, Crowd, CrowdDebugInfo, Neighbor,
    OBSTACLE_AVOIDANCE, OPTIMIZE_TOPOLOGY, OPTIMIZE_VISIBILITY, SEPARATION, STATE_INVALID,
    STATE_OFFMESH, STATE_WALKING, TARGET_FAILED, TARGET_NONE, TARGET_REQUESTING, TARGET_VALID,
    TARGET_VELOCITY, TARGET_WAITING_FOR_PATH, TARGET_WAITING_FOR_QUEUE,
};
