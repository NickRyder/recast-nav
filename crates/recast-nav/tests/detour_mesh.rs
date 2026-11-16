#![cfg(feature = "cpp-oracle")]
use recast_nav::nav::{self, NavMesh};
use std::{
    fmt::Write as _,
    io::Write,
    process::{Command, Stdio},
};

fn oracle(input: &str) -> Vec<u64> {
    let mut child = Command::new(env!("DETOUR_MESH_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let output = std::thread::scope(|scope| {
        let mut stdin = child.stdin.take().unwrap();
        let writer = scope.spawn(move || stdin.write_all(input.as_bytes()).unwrap());
        let output = child.wait_with_output().unwrap();
        writer.join().unwrap();
        output
    });
    assert!(output.status.success());
    serde_json::from_slice(&output.stdout).unwrap()
}

#[test]
fn native_filter_defaults_and_include_exclude_behavior() {
    let (layer, data) = triangle();
    let mut mesh = NavMesh::new([0.; 3], 2., 3., 1, 1).unwrap();
    mesh.add_tile([0; 3], &layer, &data, 0., 0., 0.).unwrap();
    let default = nav::Filter::default();
    let mut actual = vec![
        nav::SUCCESS as u64,
        default.include as u64,
        default.exclude as u64,
    ];
    actual.extend(default.area_costs.map(|v| v.to_bits() as u64));
    let mut input = "1 1 filter\n".to_owned();
    for flags in [0, 1, 2, 0x8000, 0xffff] {
        mesh.tiles[0]
            .as_mut()
            .unwrap()
            .polygon_mut(0)
            .unwrap()
            .set_flags(flags);
        for (include, exclude) in [
            (default.include, default.exclude),
            (0, 0),
            (2, 0),
            (0xffff, 2),
            (0x8001, 0x8000),
        ] {
            writeln!(input, "{flags} {include} {exclude}").unwrap();
            let filter = nav::Filter {
                include,
                exclude,
                ..default.clone()
            };
            let (status, found) = mesh.nearest([0.25, 0., 0.25], [1., 1., 1.], &filter);
            assert_eq!(status, nav::SUCCESS);
            actual.push(u64::from(found.is_some()));
        }
    }
    assert_eq!(actual, oracle(&input));
}

#[test]
fn upstream_capacity_and_raw_reference_layout() {
    // Includes capacities above the old 1024-slot and 65536-poly guards,
    // non-powers of two, and the exact ten-bit salt boundary.
    let capacities = [
        (1, 1),
        (7, 9),
        (2048, 1024),
        (3, 131072),
        (1, 1048576),
        (1, 4194304),
        (1, 4194305),
        (1025, 4096),
    ];
    for (tiles, polys) in capacities {
        let mesh = NavMesh::new([0.; 3], 2., 3., tiles, polys);
        let mut input = format!("{tiles} {polys} refs\n");
        let mut actual = vec![if mesh.is_ok() {
            nav::SUCCESS as u64
        } else {
            (nav::FAILURE | nav::INVALID_PARAM) as u64
        }];
        if let Ok(mesh) = mesh {
            let layout = mesh.reference_layout();
            for (salt, tile, poly) in [
                (1, 0, 0),
                (3, tiles as u32 - 1, polys as u32 - 1),
                (u32::MAX, 0x0fff_ffff, 0x000f_ffff),
            ] {
                writeln!(input, "{salt} {tile} {poly}").unwrap();
                let reference = layout.encode(salt, tile, poly);
                let (s, t, p) = layout.decode(reference);
                #[cfg(not(feature = "polyref64"))]
                actual.push(u64::from(reference));
                #[cfg(feature = "polyref64")]
                actual.push(reference);
                actual.extend([u64::from(s), u64::from(t), u64::from(p)]);
            }
        }
        assert_eq!(actual, oracle(&input), "{tiles} tiles, {polys} polys");
    }
}

#[test]
fn tile_salts_wrap_and_removed_references_expire_like_detour() {
    #[cfg(feature = "polyref64")]
    let (budget, cycles) = (1, 65537);
    #[cfg(not(feature = "polyref64"))]
    let (budget, cycles) = (4194304, 1025);
    let expected = oracle(&format!("1 {budget} cycles {cycles}\n"));
    let mut mesh = NavMesh::new([0.; 3], 2., 3., 1, budget).unwrap();
    let (layer, data) = triangle();
    let mut actual = vec![nav::SUCCESS as u64];
    for _ in 0..cycles {
        mesh.add_tile([0; 3], &layer, &data, 0., 0., 0.).unwrap();
        let reference = mesh.reference(0, 0);
        #[cfg(feature = "polyref64")]
        actual.push(reference);
        #[cfg(not(feature = "polyref64"))]
        actual.push(u64::from(reference));
        actual.push(u64::from(mesh.resolve(reference).is_some()));
        assert!(mesh.remove_tile([0; 3]));
        actual.push(nav::SUCCESS as u64);
        actual.push(u64::from(mesh.resolve(reference).is_some()));
    }
    assert_eq!(actual, expected);
}

fn triangle() -> (
    recast_nav::layers::HeightfieldLayer,
    recast_nav::cache::CacheMesh,
) {
    use recast_nav::{cache::CacheMesh, layers::HeightfieldLayer};
    let layer = HeightfieldLayer {
        bounds: [[0.; 3], [2., 1., 2.]],
        cell_size: 1.,
        cell_height: 1.,
        width: 2,
        height: 2,
        min_x: 0,
        max_x: 1,
        min_z: 0,
        max_z: 1,
        hmin: 0,
        hmax: 0,
        heights: vec![0; 4],
        areas: vec![63; 4],
        connections: vec![0; 4],
    };
    let mut poly = [u16::MAX; 12];
    poly[..3].copy_from_slice(&[0, 1, 2]);
    let data = CacheMesh {
        vertices: vec![[0, 0, 0], [0, 0, 2], [2, 0, 0]],
        polygons: vec![poly],
        areas: vec![63],
        flags: vec![0],
    };
    (layer, data)
}

#[test]
fn native_polygon_packed_area_type_assignment_and_field_preservation() {
    let (layer, data) = triangle();
    let mut mesh = NavMesh::new([0.; 3], 2., 3., 1, 1).unwrap();
    mesh.add_tile([0; 3], &layer, &data, 0., 0., 0.).unwrap();
    let data = mesh.tile_data(mesh.reference(0, 0)).unwrap();
    let mut raw = recast_nav::nav_data::TileData::parse(&data)
        .unwrap()
        .polygons()
        .next()
        .unwrap();
    let mut poly = mesh.tiles[0].as_mut().unwrap().polygon_mut(0).unwrap();
    let mut input = String::from("1 1 poly_properties\n");
    let mut actual = vec![u64::from(nav::SUCCESS)];
    for initial in 0..=255u8 {
        for (area, kind) in [(0, 0), (1, 1), (63, 2), (64, 3), (255, 4), (137, 255)] {
            writeln!(input, "{initial} {area} {kind}").unwrap();
            poly.set_area_and_type(initial);
            raw.area_and_type = initial;
            for stage in 0..3 {
                actual.extend([
                    u64::from(poly.view().area_and_type()),
                    u64::from(poly.view().area()),
                    u64::from(poly.view().kind()),
                ]);
                assert_eq!(
                    (
                        poly.view().area_and_type(),
                        poly.view().area(),
                        poly.view().kind()
                    ),
                    (raw.area_and_type, raw.area(), raw.kind())
                );
                if stage == 0 {
                    poly.set_area(area);
                    raw.set_area(area);
                } else if stage == 1 {
                    poly.set_kind(kind);
                    raw.set_kind(kind);
                }
            }
        }
    }
    assert_eq!(actual, oracle(&input));
}
