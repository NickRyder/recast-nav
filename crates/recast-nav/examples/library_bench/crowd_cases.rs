//! Matched Crowd and component episodes; internal setup and teardown are timed.
use super::{
    AgentParams, Corridor, Crowd, Filter, Hash, NavMesh, NavMeshQuery, Point, StraightPath, nav,
    nav_data, point,
};
use nav_data::{CreatePolygon, OwnedCreateParams};
use recast_nav::crowd::{
    Corner, LocalBoundary, PathCorridor, PathQueue, STATE_OFFMESH, STATE_WALKING,
};
use std::{fs, path::Path};

pub fn prepare(dir: &Path) {
    let mut p = OwnedCreateParams {
        vertices_per_polygon: 4,
        vertices: vec![
            [0, 0, 0],
            [0, 0, 4],
            [4, 0, 4],
            [4, 0, 0],
            [8, 0, 0],
            [8, 0, 4],
            [12, 0, 4],
            [12, 0, 0],
        ],
        bounds: [[0.; 3], [12., 2., 4.]],
        walkable_height: 2.,
        walkable_radius: 0.25,
        walkable_climb: 0.5,
        cell_size: 1.,
        cell_height: 0.125,
        build_bv_tree: true,
        ..Default::default()
    };
    for a in [0, 4] {
        p.polygons.push(CreatePolygon {
            vertices: [a, a + 1, a + 2, a + 3, u16::MAX, u16::MAX],
            neighbors: [u16::MAX; 6],
            flags: 1,
            area: 0,
        });
    }
    p.off_mesh_connections
        .push(nav_data::CreateOffMeshConnection {
            positions: [[3.5, 0., 2.], [8.5, 0., 2.]],
            radius: 0.6,
            flags: 1,
            area: 0,
            direction: 1,
        });
    p.off_mesh_user_ids = Some(vec![0x7351]);
    fs::write(
        dir.join("nav-crowd-link.bin"),
        nav_data::create(&p.as_params()).unwrap(),
    )
    .unwrap();
}

fn start(i: i32, dense: bool, offmesh: bool) -> Point {
    if offmesh {
        [1.25 + (i % 2) as f32 * 0.5, 0., 1.25 + (i / 2) as f32 * 0.5]
    } else if dense {
        [
            13. + (i % 16) as f32 * 0.375,
            0.,
            14.5 + (i / 16) as f32 * 0.375,
        ]
    } else {
        [0.5 + (i % 4) as f32 * 8., 0., 0.5 + (i / 4) as f32 * 24.]
    }
}

fn target(c: &mut Crowd, mesh: &NavMesh, i: i32, dense: bool, offmesh: bool, reverse: bool) {
    let p = start(i, dense, offmesh);
    let end = if reverse {
        p
    } else if offmesh {
        [12. - p[0], 0., p[2]]
    } else {
        [32. - p[0], 0., 32. - p[2]]
    };
    let poly = if offmesh {
        usize::from(!reverse)
    } else {
        end[2] as usize * 32 + end[0] as usize
    };
    assert!(c.request_move_target(i, mesh.reference(0, poly), end));
}

fn snapshot(c: &Crowd, h: &mut Hash) {
    for i in 0..c.agent_count() {
        let a = c.agent(i as i32).unwrap();
        h.word(u8::from(a.active));
        if !a.active {
            continue;
        }
        h.word(a.state);
        h.word(u8::from(a.partial));
        h.word(a.topology_optimization_time.to_bits());
        h.word(a.neighbor_count as u64);
        // Aggregate native agents initialize the entire fixed backing to zero.
        for n in a.neighbors {
            h.word(n.index as u64);
            h.word(n.distance_squared.to_bits());
        }
        h.word(a.desired_speed.to_bits());
        for p in [
            a.position,
            a.displacement,
            a.desired_velocity,
            a.new_velocity,
            a.velocity,
        ] {
            h.point(p);
        }
        let p = &a.params;
        for v in [
            p.radius,
            p.height,
            p.max_acceleration,
            p.max_speed,
            p.collision_query_range,
            p.path_optimization_range,
            p.separation_weight,
        ] {
            h.word(v.to_bits());
        }
        h.word(p.update_flags);
        h.word(p.obstacle_avoidance_type);
        h.word(p.query_filter_type);
        h.word(p.user_data as u64);
        h.word(a.corner_count as u64);
        for corner in a.corners {
            h.point(corner.position);
            h.word(corner.flags);
            h.word(corner.reference as u64);
        }
        h.word(a.target_state);
        h.word(a.target_reference as u64);
        h.point(a.target_position);
        h.word(a.target_path_queue_reference);
        h.word(u8::from(a.target_replan));
        h.word(a.target_replan_time.to_bits());
        h.point(a.corridor.position());
        h.point(a.corridor.target());
        h.word(a.corridor.path().len() as u64);
        for &r in a.corridor.path() {
            h.word(r as u64);
        }
        h.point(a.boundary.center());
        h.word(a.boundary.segment_count() as u64);
        for j in 0..8 {
            for p in a.boundary.segment(j).unwrap().positions {
                h.point(p);
            }
        }
    }
}

pub fn run(mesh: &NavMesh, dense: bool, offmesh: bool) -> u64 {
    let count = if dense { 128 } else { 8 };
    let mut c = Crowd::new(count as usize, 0.25).unwrap();
    let params = AgentParams {
        radius: 0.25,
        height: 2.,
        max_acceleration: 8.,
        max_speed: 3.5,
        collision_query_range: 3.,
        path_optimization_range: 7.5,
        separation_weight: 2.,
        update_flags: 31,
        ..Default::default()
    };
    for i in 0..count {
        let mut p = params;
        p.user_data = i as usize + 1;
        assert_eq!(c.add_agent(mesh, start(i, dense, offmesh), &p).unwrap(), i);
        target(&mut c, mesh, i, dense, offmesh, false);
    }
    let mut h = Hash::new();
    let mut pause = 0;
    let mut saw_link = false;
    let mut finished_link = false;
    let mut saw_return_link = false;
    let mut finished_return_link = false;
    let frames = if offmesh { 240 } else { 96 };
    for frame in 0..frames {
        if !offmesh {
            if frame == 16 {
                assert!(c.request_move_velocity(0, [0.5, 0., -0.25]));
            }
            if frame == 24 {
                assert!(c.reset_move_target(0));
            }
            if frame == 32 {
                c.remove_agent(1);
            }
            if frame == 36 {
                assert_eq!(
                    c.add_agent(mesh, start(1, dense, false), &params).unwrap(),
                    1
                );
            }
            if frame == 48 {
                for i in 0..count {
                    target(&mut c, mesh, i, dense, false, false);
                }
            }
        } else if frame == 120 {
            for i in 0..count {
                target(&mut c, mesh, i, false, true, true);
            }
        }
        let dt = if pause > 0 {
            pause -= 1;
            0.
        } else {
            0.05
        };
        c.update(mesh, dt, None).unwrap();
        h.word(c.velocity_sample_count() as u64);
        if offmesh {
            for i in 0..count {
                let a = c.agent(i).unwrap();
                h.word(a.state);
                if a.state == STATE_OFFMESH && !saw_link {
                    saw_link = true;
                    pause = 3;
                }
                if saw_link && a.state == STATE_WALKING && a.position[0] > 8. {
                    finished_link = true;
                }
                if frame >= 120 && a.state == STATE_OFFMESH {
                    saw_return_link = true;
                }
                if saw_return_link && a.state == STATE_WALKING && a.position[0] < 4. {
                    finished_return_link = true;
                }
            }
        }
        if frame % 12 == 0 || frame == frames - 1 {
            snapshot(&c, &mut h);
        }
    }
    if offmesh {
        assert!(
            saw_link && finished_link && saw_return_link && finished_return_link,
            "off-mesh episode must complete traversal in both directions"
        );
    }
    h.0
}

pub struct ComponentBuffers {
    path: Corridor,
    straight: StraightPath,
    corners: [Corner; 4],
    result: Vec<nav::PolyRef>,
}
impl ComponentBuffers {
    pub fn new() -> Self {
        Self {
            path: Corridor {
                polygons: Vec::with_capacity(128),
                ..Default::default()
            },
            straight: StraightPath {
                points: Vec::with_capacity(4),
                flags: Vec::with_capacity(4),
                references: Vec::with_capacity(4),
                ..Default::default()
            },
            corners: [Corner::default(); 4],
            result: vec![0xface; 128],
        }
    }
}

pub fn corridor(mesh: &NavMesh, output: &mut ComponentBuffers) -> u64 {
    let mut q = NavMeshQuery::new(512).unwrap();
    let mut c = PathCorridor::new(128).unwrap();
    let mut boundary = LocalBoundary::default();
    let mut initialized_segments = 0;
    let filter = Filter::default();
    let mut h = Hash::new();
    for round in 0..32 {
        let a = round * 37 % 1024;
        let b = (round * 113 + 17) % 1024;
        let start = point(a);
        let end = point(b);
        let reference = mesh.reference(0, a as usize);
        h.word(q.find_path_into(
            mesh,
            reference,
            mesh.reference(0, b as usize),
            start,
            end,
            &filter,
            128,
            &mut output.path,
        ));
        assert!(c.reset(reference, start));
        assert!(c.set_corridor(end, &output.path.polygons));
        for step in 0..4 {
            let count = c.find_corners_into(mesh, &q, &mut output.corners, &mut output.straight);
            h.word(count as u64);
            for corner in output.corners {
                h.point(corner.position);
                h.word(corner.flags);
                h.word(corner.reference as u64);
            }
            let next = if count > 0 {
                output.corners[0].position
            } else {
                end
            };
            c.optimize_visibility(next, 7.5, mesh, &q, &filter);
            h.word(u8::from(c.optimize_topology(mesh, &mut q, &filter)));
            let next = [
                start[0] + (step + 1) as f32 * 0.125,
                0.,
                start[2] + (step + 1) as f32 * 0.0625,
            ];
            h.word(u8::from(c.move_position(next, mesh, &mut q, &filter)));
            let next = [
                end[0] - (step + 1) as f32 * 0.0625,
                0.,
                end[2] - (step + 1) as f32 * 0.125,
            ];
            h.word(u8::from(
                c.move_target_position(next, mesh, &mut q, &filter),
            ));
            boundary.update(c.first_polygon(), c.position(), 3., mesh, &mut q, &filter);
            h.word(u8::from(boundary.is_valid(mesh, &q, &filter)));
            h.point(c.position());
            h.point(c.target());
            h.word(c.path().len() as u64);
            for &r in c.path() {
                h.word(r as u64);
            }
            h.point(boundary.center());
            h.word(boundary.segment_count() as u64);
            initialized_segments = initialized_segments.max(boundary.segment_count());
            // Standalone boundaries do not initialize unused native segments.
            for j in 0..initialized_segments {
                for p in boundary.segment(j).unwrap().positions {
                    h.point(p);
                }
            }
        }
    }
    h.0
}

fn queue_result(
    h: &mut Hash,
    q: &mut PathQueue<usize>,
    handle: u32,
    capacity: usize,
    output: &mut Vec<nav::PolyRef>,
) {
    h.word(q.path_result_into(handle, capacity, output));
    h.word(output.len() as u64);
    for &r in output.iter() {
        h.word(r as u64);
    }
}

pub fn queue(mesh: &NavMesh, output: &mut ComponentBuffers) -> u64 {
    let mut q = PathQueue::new(128, 512).unwrap();
    let mut filters = [Filter::default(), Filter::default()];
    let mut h = Hash::new();
    for round in 0..32 {
        filters[1].area_costs[0] = if round % 2 == 0 { 1.25 } else { 1.5 };
        let mut handles = [0; 8];
        for (i, handle) in handles.iter_mut().enumerate() {
            let a = (round * 37 + i * 31) % 1024;
            let b = (round * 113 + i * 73 + 17) % 1024;
            *handle = q.request(
                mesh.reference(0, a),
                mesh.reference(0, b),
                point(a as u32),
                point(b as u32),
                Some(i % 2),
            );
            assert_ne!(*handle, 0);
            h.word(*handle);
        }
        h.word(q.request(
            mesh.reference(0, 0),
            mesh.reference(0, 1),
            point(0),
            point(1),
            Some(0),
        ));
        let mut done = [false; 8];
        let mut complete = false;
        for step in 0..4096 {
            let budget = [0, 1, 7, 23, 100][step % 5];
            q.update(mesh, budget, |&key| &filters[key]);
            for i in 0..8 {
                if done[i] {
                    continue;
                }
                let status = q.request_status(handles[i]);
                h.word(status);
                // One result deliberately ages out; others are consumed immediately.
                if (i == 7 && status == nav::FAILURE)
                    || (i != 7 && status & (nav::SUCCESS | nav::FAILURE) != 0)
                {
                    let capacity = [0, 1, 8, 128][i % 4];
                    queue_result(&mut h, &mut q, handles[i], capacity, &mut output.result);
                    h.word(q.request_status(handles[i]));
                    queue_result(&mut h, &mut q, handles[i], capacity, &mut output.result);
                    done[i] = true;
                }
            }
            if done.iter().all(|&done| done) {
                complete = true;
                break;
            }
        }
        assert!(complete, "path queue must complete or expire every request");
    }
    h.0
}
