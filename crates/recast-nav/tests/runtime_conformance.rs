use recast_nav::{
    TriangleMesh,
    nav::{self, Filter},
    runtime::{BoxObstacle, Navigation, NavigationSettings},
};
use std::{
    fmt::Write as _,
    io::Write,
    process::{Command, Stdio},
    str::{FromStr, SplitWhitespace},
};
fn bits(p: [f32; 3], out: &mut Vec<i64>) {
    out.extend(p.map(|f| i64::from(f.to_bits())));
}
fn take<T: FromStr>(input: &mut SplitWhitespace<'_>) -> T
where
    T::Err: std::fmt::Debug,
{
    input.next().unwrap().parse().unwrap()
}
fn point(input: &mut SplitWhitespace<'_>) -> [f32; 3] {
    [take(input), take(input), take(input)]
}
fn execute(nav: &mut Navigation, command: &str) -> Vec<i64> {
    let mut words = command.split_whitespace();
    let op = words.next().unwrap();
    let mut out = Vec::new();
    match op {
        "S" => {
            out.push(nav.nav_mesh().tiles.len() as i64);
            for (i, tile) in nav.nav_mesh().tiles.iter().enumerate() {
                out.push(i64::from(tile.is_some()));
                let Some(t) = tile else {
                    continue;
                };
                out.extend([
                    nav.nav_mesh().reference(i, 0) as i64,
                    t.coordinate[0].into(),
                    t.coordinate[1].into(),
                    t.coordinate[2].into(),
                    t.vertex_count() as i64,
                    t.polygon_count() as i64,
                ]);
                bits(
                    [
                        t.header.walkable_height,
                        t.header.walkable_radius,
                        t.header.walkable_climb,
                    ],
                    &mut out,
                );
                for v in t.vertices() {
                    bits(v, &mut out);
                }
                for p in t.polygons() {
                    out.extend([p.vertices().len() as i64, p.flags().into(), p.area().into()]);
                    out.extend(p.vertices().map(|v| v as i64));
                    out.extend(p.neighbors().map(i64::from));
                    out.push(t.links(p).count() as i64);
                    for l in t.links(p) {
                        out.extend([
                            l.reference() as i64,
                            l.edge() as i64,
                            l.side().into(),
                            l.min().into(),
                            l.max().into(),
                        ]);
                    }
                }
            }
            out.push(nav.obstacles().len() as i64);
            let tilebits = nav.settings().max_tiles.next_power_of_two().ilog2();
            for ob in nav.obstacles() {
                out.extend([
                    i64::from(ob.salt),
                    ob.state.into(),
                    ob.touched.len() as i64,
                    ob.pending.len() as i64,
                ]);
                out.extend(
                    ob.touched
                        .iter()
                        .chain(&ob.pending)
                        .map(|&i| ((1u32 << tilebits) | i as u32) as i64),
                );
            }
            out.push(nav.obstacle_count() as i64);
            out.push(nav.requested_revision() as i64);
        }
        "A" => {
            let key = take(&mut words);
            let bounds = BoxObstacle {
                min: point(&mut words),
                max: point(&mut words),
            };
            match nav.add_or_replace_box(key, bounds) {
                Ok(changed) => out.extend([
                    i64::from(nav::SUCCESS),
                    nav.key_reference(key).unwrap().into(),
                    changed.into(),
                ]),
                Err(e) => panic!("{e}"),
            }
        }
        "B" => {
            let bounds = BoxObstacle {
                min: point(&mut words),
                max: point(&mut words),
            };
            match nav.add_box(bounds) {
                Ok(reference) => out.extend([i64::from(nav::SUCCESS), reference.into(), 1]),
                Err(status) => out.extend([i64::from(status), 0, 1]),
            }
        }
        "D" => {
            let key = take(&mut words);
            out.extend([i64::from(nav::SUCCESS), nav.remove_key(key).unwrap().into()]);
        }
        "R" => out.push(nav.remove_obstacle(take(&mut words)).into()),
        "U" | "L" => {
            let limit = if op == "U" { 100000 } else { take(&mut words) };
            let mut calls = 0;
            let mut done = false;
            while calls < limit && !done {
                done = nav.update().unwrap().up_to_date;
                calls += 1;
            }
            out.extend([i64::from(nav::SUCCESS), calls, done.into()]);
        }
        "P" => {
            let p = point(&mut words);
            let ext = point(&mut words);
            let (status, nearest) = nav.nav_mesh().nearest(p, ext, &Filter::default());
            out.extend([
                i64::from(status),
                nearest.map(|(r, _)| r as i64).unwrap_or(0),
            ]);
            bits(nearest.map(|(_, p)| p).unwrap_or([0.; 3]), &mut out);
        }
        "V" => out.push(nav.nav_mesh().resolve(take(&mut words)).is_some().into()),
        "Q" => {
            let start = point(&mut words);
            let end = point(&mut words);
            let nodes = take(&mut words);
            let max_path = take(&mut words);
            let max_straight = take(&mut words);
            let filter = Filter::default();
            let (ss, s) = nav.nav_mesh().nearest(start, [1., 100., 1.], &filter);
            let (es, e) = nav.nav_mesh().nearest(end, [1., 100., 1.], &filter);
            out.extend([
                i64::from(ss),
                es.into(),
                s.map(|(r, _)| r as i64).unwrap_or(0),
                e.map(|(r, _)| r as i64).unwrap_or(0),
            ]);
            bits(s.map(|(_, p)| p).unwrap_or([0.; 3]), &mut out);
            bits(e.map(|(_, p)| p).unwrap_or([0.; 3]), &mut out);
            if let (Some((sr, sp)), Some((er, ep))) = (s, e) {
                let path = nav
                    .nav_mesh()
                    .find_path(sr, er, sp, ep, &filter, nodes, max_path);
                let straight = nav
                    .nav_mesh()
                    .straight_path(sp, ep, &path.polygons, max_straight);
                out.extend([i64::from(path.status), path.polygons.len() as i64]);
                out.extend(path.polygons.iter().map(|&r| r as i64));
                out.extend([i64::from(straight.status), straight.points.len() as i64]);
                for i in 0..straight.points.len() {
                    bits(straight.points[i], &mut out);
                    out.extend([i64::from(straight.flags[i]), straight.references[i] as i64]);
                }
            }
        }
        "H" | "G" => {
            let key = if op == "G" {
                Some(take(&mut words))
            } else {
                None
            };
            let partial: i32 = take(&mut words);
            let start = point(&mut words);
            let end = point(&mut words);
            let path = if let Some(key) = key {
                nav.path_for_agent(key, start, end, partial != 0)
            } else {
                nav.path(start, end, partial != 0)
            }
            .unwrap();
            if let Some(path) = path {
                out.extend([
                    1,
                    path.reaches_goal.into(),
                    path.corridor_status.into(),
                    path.straight_status.into(),
                    path.points.len() as i64,
                ]);
                for p in path.points {
                    bits(p, &mut out);
                }
            } else {
                out.push(0);
            }
        }
        _ => panic!("unknown {op}"),
    }
    out
}
fn compare(name: &str, mesh: TriangleMesh, settings: NavigationSettings, commands: Vec<String>) {
    let mut input = String::new();
    for v in settings.origin {
        write!(input, "{v} ").unwrap();
    }
    writeln!(
        input,
        "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        settings.maximum_y,
        settings.tiles[0],
        settings.tiles[1],
        settings.tile_voxels,
        settings.cell_size,
        settings.cell_height,
        settings.agent_height,
        settings.agent_radius,
        settings.agent_climb,
        settings.slope_degrees,
        settings.max_tiles,
        settings.max_polygons_per_tile,
        settings.max_obstacles,
        settings.workers,
        mesh.vertices.len(),
        mesh.triangles.len()
    )
    .unwrap();
    for v in mesh.vertices.iter().flatten() {
        write!(input, "{v} ").unwrap();
    }
    for i in mesh.triangles.iter().flatten() {
        write!(input, "{i} ").unwrap();
    }
    input.push('\n');
    for command in &commands {
        writeln!(input, "{command}").unwrap();
    }
    let mut child = Command::new(env!("RECAST_RUNTIME_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child
        .stdin
        .take()
        .unwrap()
        .write_all(input.as_bytes())
        .unwrap();
    let result = child.wait_with_output().unwrap();
    assert!(
        result.status.success(),
        "{name}: status {}: {}",
        result.status,
        String::from_utf8_lossy(&result.stderr)
    );
    let expected: Vec<Vec<i64>> = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(expected.len(), commands.len());
    let mut nav = Navigation::build(&mesh, settings).unwrap();
    for (command, expected) in commands.iter().zip(expected) {
        let actual = execute(&mut nav, command);
        let first = actual.iter().zip(&expected).position(|(a, b)| a != b);
        assert!(
            actual == expected,
            "{name}: command '{command}': difference {first:?} Rust {:?} C++ {:?}, lengths {} vs {}",
            first.and_then(|i| actual.get(i)),
            first.and_then(|i| expected.get(i)),
            actual.len(),
            expected.len()
        );
    }
}
fn quad(mesh: &mut TriangleMesh, v: [[f32; 3]; 4]) {
    let n = mesh.vertices.len() as u32;
    mesh.vertices.extend(v);
    mesh.triangles
        .extend([[n, n + 1, n + 2], [n, n + 2, n + 3]]);
}
fn floor(mesh: &mut TriangleMesh, x: [f32; 2], z: [f32; 2], y: f32) {
    quad(
        mesh,
        [
            [x[0], y, z[0]],
            [x[0], y, z[1]],
            [x[1], y, z[1]],
            [x[1], y, z[0]],
        ],
    );
}
fn empty() -> TriangleMesh {
    TriangleMesh {
        vertices: vec![],
        triangles: vec![],
    }
}
fn commands(strings: &[&str]) -> Vec<String> {
    strings.iter().map(|s| s.to_string()).collect()
}
#[test]
fn tiled_paths_and_obstacle_lifecycle_match_cpp() {
    let mut mesh = empty();
    floor(&mut mesh, [0., 16.], [0., 16.], 0.);
    compare(
        "tiled floor",
        mesh,
        NavigationSettings {
            max_obstacles: 16,
            ..Default::default()
        },
        commands(&[
            "S",
            "V 524288",
            "P 8 0 8 1 100 1",
            "Q 2 0 2 14 0 14 4096 256 256",
            "Q 14 0 2 2 0 14 4096 256 256",
            "Q 2 0 2 14 0 14 4 256 256",
            "Q 2 0 2 14 0 14 4096 1 256",
            "Q 2 0 2 14 0 14 4096 256 1",
            "H 0 2 0 2 14 0 14",
            "A 10 6 -1 0 10 3 16",
            "S",
            "L 1",
            "V 524288",
            "S",
            "U",
            "S",
            "Q 2 0 2 14 0 14 4096 256 256",
            "H 0 2 0 2 14 0 14",
            "H 1 2 0 2 14 0 14",
            "A 10 6 -1 0 10 3 16",
            "U",
            "S",
            "A 10 6 -1 -0 10 3 16",
            "U",
            "S",
            "A 10 6 -1 6 10 3 10",
            "U",
            "S",
            "Q 2 0 2 14 0 14 4096 256 256",
            "H 1 2 0 2 8 0 8",
            "H 0 2 0 2 8 0 8",
            "G 10 1 8 0 8 14 0 14",
            "D 10",
            "U",
            "S",
            "Q 2 0 2 14 0 14 4096 256 256",
            "R 65536",
            "U",
            "S",
            "D 10",
        ]),
    );
}
#[test]
fn ramps_layers_and_disconnected_goals_match_cpp() {
    let mut ramp = empty();
    floor(&mut ramp, [0., 16.], [0., 4.], 0.);
    quad(
        &mut ramp,
        [[0., 0., 4.], [0., 3., 12.], [16., 3., 12.], [16., 0., 4.]],
    );
    floor(&mut ramp, [0., 16.], [12., 16.], 3.);
    compare(
        "ramp across seams",
        ramp,
        NavigationSettings {
            max_obstacles: 16,
            ..Default::default()
        },
        commands(&[
            "S",
            "P 8 1 8 1 100 1",
            "Q 2 0 2 14 3 14 4096 256 256",
            "Q 14 3 14 2 0 2 4096 256 256",
            "A 1 7 -1 4 9 6 12",
            "U",
            "S",
            "Q 2 0 2 14 3 14 4096 256 256",
            "D 1",
            "U",
            "S",
        ]),
    );
    let mut layers = empty();
    floor(&mut layers, [0., 16.], [0., 16.], 0.);
    floor(&mut layers, [2., 14.], [2., 14.], 4.);
    compare(
        "stacked floors",
        layers,
        NavigationSettings {
            max_obstacles: 16,
            ..Default::default()
        },
        commands(&[
            "S",
            "P 8 0 8 1 100 1",
            "P 8 4 8 1 100 1",
            "Q 4 4 4 12 4 12 4096 256 256",
            "H 1 4 0 4 12 4 12",
            "A 1 6 3 6 10 6 10",
            "U",
            "S",
            "H 1 4 4 4 12 4 12",
            "D 1",
            "U",
            "S",
        ]),
    );
    let mut islands = empty();
    floor(&mut islands, [0., 6.], [0., 16.], 0.);
    floor(&mut islands, [10., 16.], [0., 16.], 0.);
    compare(
        "islands",
        islands,
        NavigationSettings {
            max_obstacles: 16,
            ..Default::default()
        },
        commands(&[
            "S",
            "H 0 2 0 8 14 0 8",
            "H 1 2 0 8 14 0 8",
            "Q 2 0 8 14 0 8 4096 256 256",
        ]),
    );
}

#[test]
fn queued_updates_stale_handles_and_capacity_statuses_match_cpp() {
    let mut mesh = empty();
    floor(&mut mesh, [0., 16.], [0., 16.], 0.);
    let mut steps = commands(&["S"]);
    for _ in 0..65 {
        steps.push("B 6 -1 6 10 3 10".into());
    }
    steps.extend(commands(&[
        "R 65536",
        "S",
        "L 1",
        "S",
        "B 4 -1 4 5 3 5",
        "R 65536",
        "S",
        "L 1",
        "S",
        "U",
        "S",
        "R 65536",
        "U",
        "S",
        "B 2 -1 2 3 3 3",
        "U",
        "S",
        "Q 2 0 2 14 0 14 4 256 256",
        "Q 2 0 2 14 0 14 4096 2 2",
    ]));
    compare(
        "queued obstacle updates",
        mesh.clone(),
        NavigationSettings {
            max_obstacles: 128,
            ..Default::default()
        },
        steps,
    );
    compare(
        "fully carved and restored",
        mesh.clone(),
        NavigationSettings {
            max_obstacles: 16,
            ..Default::default()
        },
        commands(&[
            "A 1 -5 -1 -5 25 6 25",
            "U",
            "S",
            "P 8 0 8 1 100 1",
            "H 1 2 0 2 14 0 14",
            "D 1",
            "U",
            "S",
            "Q 2 0 2 14 0 14 4096 256 256",
        ]),
    );
    compare(
        "obstacle slot exhaustion",
        mesh,
        NavigationSettings {
            max_obstacles: 2,
            ..Default::default()
        },
        commands(&[
            "B 2 -1 2 3 3 3",
            "B 4 -1 4 5 3 5",
            "B 6 -1 6 7 3 7",
            "U",
            "S",
            "R 65536",
            "U",
            "S",
            "B 6 -1 6 7 3 7",
            "U",
            "R 65536",
            "U",
            "S",
        ]),
    );
}

#[test]
fn seed_queries_and_parallel_builds_match_cpp() {
    for seed in 1..=32u32 {
        let mut rng = seed;
        let mut heights = [[0.; 9]; 9];
        for row in &mut heights {
            for h in row {
                rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
                *h = ((rng >> 24) % 5) as f32 * 0.25;
            }
        }
        let mut mesh = empty();
        for z in 0..8 {
            for x in 0..8 {
                let xx = x as f32 * 2.;
                let zz = z as f32 * 2.;
                quad(
                    &mut mesh,
                    [
                        [xx, heights[z][x], zz],
                        [xx, heights[z + 1][x], zz + 2.],
                        [xx + 2., heights[z + 1][x + 1], zz + 2.],
                        [xx + 2., heights[z][x + 1], zz],
                    ],
                );
            }
        }
        let mut steps = commands(&["S", "A 1 6 -1 6 10 3 10", "U", "S"]);
        for _ in 0..32 {
            let mut coords = [0.; 4];
            for c in &mut coords {
                rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
                *c = 1.25 + ((rng >> 24) % 54) as f32 * 0.25;
            }
            steps.push(format!(
                "Q {} 0 {} {} 0 {} 4096 256 256",
                coords[0], coords[1], coords[2], coords[3]
            ));
        }
        steps.extend(commands(&["D 1", "U", "S"]));
        compare(
            &format!("terrain seed {seed}"),
            mesh.clone(),
            NavigationSettings {
                max_obstacles: 16,
                ..Default::default()
            },
            steps,
        );
        if seed == 1 {
            let mut parallel = Navigation::build(
                &mesh,
                NavigationSettings {
                    max_obstacles: 16,
                    ..Default::default()
                },
            )
            .unwrap();
            let mut sequential = Navigation::build(
                &mesh,
                NavigationSettings {
                    max_obstacles: 16,
                    workers: 1,
                    ..Default::default()
                },
            )
            .unwrap();
            assert_eq!(execute(&mut parallel, "S"), execute(&mut sequential, "S"));
        }
    }
}

#[test]
fn large_tiles_complex_boundaries_and_vertical_separation_match_cpp() {
    let mut mesh = empty();
    let mut random = 723u32;
    for z in 0..16 {
        for x in 0..16 {
            random = random.wrapping_mul(1664525).wrapping_add(1013904223);
            if (random >> 24).is_multiple_of(7) {
                continue;
            }
            let xx = x as f32 * 5.;
            let zz = z as f32 * 5.;
            floor(&mut mesh, [xx, xx + 5.], [zz, zz + 5.], 0.);
        }
    }
    let mut steps = commands(&[
        "S",
        "P -1 0 -1 4 100 4",
        "A 1 38 -1 0 42 4 80",
        "A 2 20 -1 38 60 4 42",
        "L 1",
        "S",
        "U",
        "S",
    ]);
    for z in [5, 15, 35, 45, 65, 75] {
        for x in [5, 15, 35, 45, 65, 75] {
            steps.push(format!("H 1 5 0 5 {x} 0 {z}"));
        }
    }
    steps.extend(commands(&["D 1", "D 2", "U", "S"]));
    compare(
        "large fragmented tiles",
        mesh,
        NavigationSettings {
            tile_voxels: 40,
            tiles: [4, 4],
            max_obstacles: 16,
            ..Default::default()
        },
        steps,
    );
    let mut cliffs = empty();
    floor(&mut cliffs, [0., 8.], [0., 16.], 0.);
    floor(&mut cliffs, [8., 16.], [0., 16.], 3.);
    compare(
        "raised platform edge",
        cliffs,
        NavigationSettings {
            max_obstacles: 16,
            ..Default::default()
        },
        commands(&[
            "S",
            "H 0 4 0 8 12 3 8",
            "H 1 4 0 8 12 3 8",
            "Q 4 0 8 12 3 8 4096 256 256",
        ]),
    );
}

#[test]
fn fractional_ramp_matches_cpp() {
    let mut mesh = empty();
    quad(
        &mut mesh,
        [
            [-10., 0., -10.],
            [-10., 0., 10.],
            [10., 4., 10.],
            [10., 4., -10.],
        ],
    );
    compare(
        "fractional ramp",
        mesh,
        NavigationSettings {
            origin: [-11.25, -0.25, -11.25],
            maximum_y: 6.95,
            tile_voxels: 40,
            max_tiles: 4,
            max_obstacles: 16,
            ..Default::default()
        },
        commands(&["S", "P -8 0 -8 1 100 1", "Q -8 0 -8 8 0 8 4096 256 256"]),
    );
}

#[test]
fn non_binary_voxel_scales_and_rotated_boundaries_match_cpp() {
    let mut mesh = empty();
    quad(
        &mut mesh,
        [
            [-4., 0., -4.],
            [-4., 0., 20.],
            [20., 3., 20.],
            [20., 3., -4.],
        ],
    );
    let theta = 0.7_f32;
    let (s, c) = theta.sin_cos();
    let corners = [[-2., -2.], [-2., 2.], [2., 2.], [2., -2.]]
        .map(|[x, z]| [8. + x * c - z * s, 5., 8. + x * s + z * c]);
    quad(&mut mesh, corners);
    for (cs, ch) in [(0.3, 0.2), (0.4, 0.15), (0.5, 0.25)] {
        let mut steps = commands(&[
            "S",
            "P 7.33 1 7.61 1 100 1",
            "Q 2.1 0 2.3 15.1 0 15.3 4096 256 256",
            "A 1 4.2 -1 5.1 10.6 2.7 6.9",
            "U",
            "S",
            "H 1 2.1 0 2.3 15.1 0 15.3",
            "D 1",
            "U",
            "S",
        ]);
        // Short queries sample interpolation on the slope without depending on tile coverage.
        steps.push("Q 1.1 0 1.7 9.3 0 11.1 4096 256 256".into());
        compare(
            &format!("voxel scale {cs}/{ch}"),
            mesh.clone(),
            NavigationSettings {
                origin: [-1.3, -0.2, -1.3],
                maximum_y: 8.,
                tiles: [2, 2],
                tile_voxels: 40,
                cell_size: cs,
                cell_height: ch,
                max_obstacles: 16,
                ..Default::default()
            },
            steps,
        );
    }
    // Detail triangles below Detour's area threshold must use its edge-height
    // fallback while preserving the query's XZ coordinates for interior points.
    let mut tiny = empty();
    floor(&mut tiny, [0., 0.00016], [0., 0.00016], 0.);
    compare(
        "detail edge-height fallback",
        tiny,
        NavigationSettings {
            tiles: [1, 1],
            cell_size: 0.00001,
            agent_radius: 0.,
            max_obstacles: 1,
            ..Default::default()
        },
        commands(&["S", "P 0.00007 0 0.00009 0.00002 100 0.00002"]),
    );
}

#[test]
fn configurable_polygon_budgets_match_cpp() {
    for budget in [1, 3, 256, 131072] {
        let mut mesh = TriangleMesh {
            vertices: vec![],
            triangles: vec![],
        };
        floor(&mut mesh, [0., 8.], [0., 8.], 0.);
        let settings = NavigationSettings {
            tiles: [1, 1],
            max_tiles: 8,
            max_obstacles: 8,
            max_polygons_per_tile: budget,
            ..Default::default()
        };
        compare(
            "polygon budgets",
            mesh,
            settings,
            commands(&["S", "P 2 0 2 1 100 1", "Q 2 0 2 6 0 6 32 8 8"]),
        );
    }
}

#[test]
fn padded_tile_selection_preserves_boundary_triangles_and_empty_tiles() {
    for cell_size in [0.5, 0.3] {
        let settings = NavigationSettings {
            tiles: [5, 3],
            cell_size,
            max_tiles: 32,
            ..Default::default()
        };
        let config = settings.build_config(1, 1).unwrap();
        let mut mesh = empty();
        floor(&mut mesh, [2., 18.], [2., 18.], 0.);
        // Vertical faces on each closed padded boundary must still participate
        // in rasterization. Remote geometry leaves empty tiles on the grid.
        for x in [config.bounds[0][0], config.bounds[1][0]] {
            quad(
                &mut mesh,
                [[x, 0., 2.], [x, 3., 2.], [x, 3., 18.], [x, 0., 18.]],
            );
        }
        for z in [config.bounds[0][2], config.bounds[1][2]] {
            quad(
                &mut mesh,
                [[2., 0., z], [18., 0., z], [18., 3., z], [2., 3., z]],
            );
        }
        floor(&mut mesh, [1000., 1008.], [1000., 1008.], 0.);
        floor(&mut mesh, [-1008., -1000.], [-1008., -1000.], 0.);
        compare(
            "padded selection",
            mesh,
            settings,
            commands(&["S", "A 1 7 -1 7 9 3 9", "U", "S", "D 1", "U", "S"]),
        );
    }
}
