#![cfg(feature = "cpp-oracle")]
#[path = "support/float_comparison.rs"]
mod float_comparison;
#[path = "support/float_operands.rs"]
mod float_operands;
#[path = "support/parity_trace.rs"]
mod parity_trace;
use recast_nav::{common::*, nav::*};
use std::{
    fmt::Write as _,
    io::Write as _,
    process::{Command, Stdio},
};

#[test]
fn segment_polygon_clip_results_and_partial_outputs_match_native() {
    let shapes = [
        vec![],
        vec![[1., 2., 3.]],
        vec![[0., 0., 0.], [3., 1., 4.]],
        vec![[0., 0., 0.], [0., 1., 4.], [4., 2., 4.], [4., 1., 0.]],
        vec![
            [-1.1, 0., 0.13],
            [0.32, 1., 5.43],
            [7.12, 2., 4.25],
            [4.13, 3., -2.34],
        ],
    ];
    let mut seed = 0x75a6_83c1u32;
    for vertices in shapes {
        for i in 0..64 {
            let mut point = || {
                std::array::from_fn(|_| {
                    seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                    ((seed >> 16) as i32 - 32768) as f32 / 2048.
                })
            };
            let start: Point = point();
            let mut end = point();
            if i == 0 {
                end = start;
            }
            if i == 1 {
                end[2] = start[2] + 0.0000001;
            }
            let hit = intersect_segment_polygon_2d(start, end, &vertices);
            let mut input = format!("1 1 intersection {} ", vertices.len());
            for v in start
                .into_iter()
                .chain(end)
                .chain(vertices.iter().flatten().copied())
            {
                write!(input, "{} ", v.to_bits()).unwrap();
            }
            let mut child = Command::new(env!("DETOUR_MESH_ORACLE"))
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .spawn()
                .unwrap();
            child
                .stdin
                .take()
                .unwrap()
                .write_all(input.as_bytes())
                .unwrap();
            let out = child.wait_with_output().unwrap();
            assert!(out.status.success());
            let expected: Vec<i64> = serde_json::from_slice(&out.stdout).unwrap();
            assert_eq!(
                vec![
                    SUCCESS as i64,
                    i64::from(hit.intersects),
                    hit.t_min.to_bits().into(),
                    hit.t_max.to_bits().into(),
                    hit.entering_edge.into(),
                    hit.leaving_edge.into()
                ],
                expected,
                "start={start:?} end={end:?} verts={vertices:?}"
            );
        }
    }
}

#[test]
fn segment_intersection_parameters_match_native_without_clamping() {
    let mut seed = 0x5213_8ab1u32;
    let mut input = format!("1 1 segments {} ", 256 + float_operands::CASE_COUNT);
    let mut expected = vec![serde_json::json!(SUCCESS)];
    for i in 0..256 + float_operands::CASE_COUNT {
        let mut points: [Point; 4] = std::array::from_fn(|_| {
            std::array::from_fn(|_| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                ((seed >> 16) as i32 - 32768) as f32 / 2048.
            })
        });
        if i >= 256 {
            let [x, y, z] = float_operands::operands(i - 256);
            points = [[1., 2., 3.], [x, y, z], [y, z, x], [z, x, y]];
        }
        match i {
            0 => points[1] = points[0],
            1 => points[3] = points[2],
            2 => points = [[0.; 3], [1., 0., 0.], [0., 0., 1.], [1., 0., 1.]],
            3 => points = [[0.; 3], [1., 0., 0.], [0., 0., 1.], [1., 0., 1.0000005]],
            4 => points = [[0.; 3], [1., 0., 0.], [4., 0., 1.], [4., 0., 2.]],
            _ => {}
        }
        let result = intersect_segments_2d(points[0], points[1], points[2], points[3]);
        let (s, t) = result.unwrap_or((-17., 19.));
        expected.push(serde_json::json!([
            u8::from(result.is_some()),
            s.to_bits(),
            t.to_bits()
        ]));
        for v in points.into_iter().flatten() {
            write!(input, "{} ", v.to_bits()).unwrap();
        }
    }
    let mut child = Command::new(env!("DETOUR_MESH_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let output = std::thread::scope(|scope| {
        let bytes = input.as_bytes();
        let writer = scope.spawn(move || stdin.write_all(bytes).unwrap());
        let output = child.wait_with_output().unwrap();
        writer.join().unwrap();
        output
    });
    assert!(output.status.success());
    let actual: Vec<serde_json::Value> = serde_json::from_slice(&output.stdout).unwrap();
    parity_trace::write("segments", &input, &expected, &actual);
    assert_eq!(actual.len(), expected.len());
    for (i, (a, b)) in actual.iter().zip(&expected).enumerate() {
        float_comparison::assert_eq(
            b,
            a,
            |path| matches!(path, [1 | 2]),
            &format!("segment sample {i}"),
        );
    }
}

#[test]
fn polygon_overlap_axes_tolerances_and_degenerate_inputs_match_native() {
    let mut cases = Vec::new();
    let mut seed = 0x3496_a325u32;
    for i in 0..128 {
        let a = if i == 0 {
            vec![]
        } else if i == 1 {
            vec![[0.; 3]]
        } else {
            vec![[0.; 3], [0., 1., 4.], [4., 2., 4.], [4., 1., 0.]]
        };
        let mut b = a.clone();
        let x = match i {
            0..=2 => 0.,
            3 => 4.,
            4 => 3.999999,
            5 => 3.9999,
            6 => 4.00001,
            _ => {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                (seed >> 16) as f32 / 4096. - 8.
            }
        };
        for vertex in &mut b {
            vertex[0] += x;
            vertex[1] += 10.;
        }
        cases.push((a, b));
    }
    // Exercise repeated and distinct directions beyond four edges, including
    // unequal vertex counts and exceptional coordinates in both planar axes.
    let octagon = vec![
        [-2., 0., 0.],
        [-1., 0., 2.],
        [0., 0., 3.],
        [2., 0., 2.],
        [3., 0., 0.],
        [2., 0., -2.],
        [0., 0., -3.],
        [-1., 0., -2.],
    ];
    cases.push((octagon.clone(), octagon.clone()));
    cases.push((octagon[..3].to_vec(), octagon));
    for bits in [
        0x7fc1_2345,
        0xffc6_7890,
        0x7fa1_2345,
        0x7f80_0000,
        0xff80_0000,
        0x8000_0000,
        1,
        0x7f7f_ffff,
    ] {
        for axis in [0, 2] {
            let mut a = vec![[0.; 3], [0., 0., 4.], [4., 0., 4.], [4., 0., 0.]];
            a[1][axis] = f32::from_bits(bits);
            cases.push((a.clone(), a));
        }
    }
    let mut input = format!("1 1 overlap {} ", cases.len());
    let mut expected = vec![SUCCESS];
    for (a, b) in cases {
        write!(input, "{} {} ", a.len(), b.len()).unwrap();
        for vertex in a.iter().chain(&b) {
            for v in vertex {
                write!(input, "{} ", v.to_bits()).unwrap();
            }
        }
        expected.push(u32::from(overlap_polygons_2d(&a, &b).unwrap()));
    }
    let mut child = Command::new(env!("DETOUR_MESH_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let output = std::thread::scope(|scope| {
        let writer = scope.spawn(move || stdin.write_all(input.as_bytes()).unwrap());
        let output = child.wait_with_output().unwrap();
        writer.join().unwrap();
        output
    });
    assert!(output.status.success());
    let actual: Vec<u32> = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(actual, expected);
    assert_eq!(overlap_polygons_2d(&[], &[[0.; 3]]), None);
}

#[test]
fn convex_polygon_random_weights_scratch_and_point_bits_match_native() {
    let shapes = vec![
        vec![[0.; 3], [2., 3., 4.]],
        vec![[0.; 3], [0.12, 1.13, 2.53], [3.13, 4.2, 0.32]],
        vec![[0.; 3], [0., 1., 4.], [4., 2., 4.], [4., 1., 0.]],
        vec![[0.; 3]; 4],
        vec![
            [f32::from_bits(0x7fc1_2345); 3],
            [f32::from_bits(0xffc5_4321); 3],
        ],
        vec![
            [f32::from_bits(0x7f81_2345), -0., f32::INFINITY],
            [f32::from_bits(0xff85_4321), 1., f32::NEG_INFINITY],
            [1., f32::from_bits(0x7fc1_2345), f32::from_bits(0xffc5_4321)],
        ],
        vec![
            [0.; 3],
            [0., 0., 0.00001],
            [0.00001, 0., 0.00001],
            [0.00001, 0., 0.],
        ],
        (0..25)
            .map(|i| {
                let angle = -(i as f32) * std::f32::consts::TAU / 25.;
                [angle.cos() * 2., i as f32 * 0.13, angle.sin() * 3.]
            })
            .collect(),
    ];
    let samples = [
        [0., 0.],
        [1., 1.],
        [0.5, 0.25],
        [0.13579, 0.97531],
        [-1., 0.4],
        [0.33, -1.],
        [f32::NAN, 0.5],
    ];
    let mut input = format!("1 1 convex_random {} ", shapes.len() * samples.len());
    let mut expected = vec![serde_json::json!(SUCCESS)];
    for vertices in shapes {
        for [s, t] in samples {
            write!(input, "{} {} {} ", vertices.len(), s.to_bits(), t.to_bits()).unwrap();
            let mut areas: Vec<_> = (0..vertices.len() + 2).map(|i| i as f32 + 0.123).collect();
            for v in vertices.iter().flatten().chain(&areas) {
                write!(input, "{} ", v.to_bits()).unwrap();
            }
            let point = random_point_in_convex_polygon(&vertices, &mut areas, s, t).unwrap();
            expected.push(serde_json::json!(
                point
                    .into_iter()
                    .chain(areas)
                    .map(f32::to_bits)
                    .collect::<Vec<_>>()
            ));
        }
    }
    let mut child = Command::new(env!("DETOUR_MESH_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let output = std::thread::scope(|scope| {
        let bytes = input.as_bytes();
        let writer = scope.spawn(move || stdin.write_all(bytes).unwrap());
        let output = child.wait_with_output().unwrap();
        writer.join().unwrap();
        output
    });
    assert!(output.status.success());
    let actual: Vec<serde_json::Value> = serde_json::from_slice(&output.stdout).unwrap();
    parity_trace::write("convex-random", &input, &expected, &actual);
    assert_eq!(actual.len(), expected.len());
    for (i, (a, b)) in actual.iter().zip(&expected).enumerate() {
        float_comparison::assert_eq(
            b,
            a,
            |path| matches!(path, [j] if *j < 3 || (*j >= 5 && *j < b.as_array().unwrap().len()-2)),
            &format!("convex sample {i}"),
        );
    }
}

fn common_oracle(input: &str) -> Vec<serde_json::Value> {
    let mut child = Command::new(env!("DETOUR_MESH_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let output = std::thread::scope(|scope| {
        let writer = scope.spawn(move || stdin.write_all(input.as_bytes()).unwrap());
        let out = child.wait_with_output().unwrap();
        writer.join().unwrap();
        out
    });
    assert!(output.status.success());
    serde_json::from_slice(&output.stdout).unwrap()
}
fn common_point(seed: &mut u32) -> Point {
    std::array::from_fn(|_| {
        *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        ((*seed >> 16) as i32 - 32768) as f32 / 2048.
    })
}
fn common_input(input: &mut String, points: impl IntoIterator<Item = f32>) {
    for v in points {
        write!(input, "{} ", v.to_bits()).unwrap();
    }
}
#[test]
fn native_common_vector_scalar_bounds_and_integer_utilities() {
    use serde_json::json;
    let mut seed = 0x1298_4465;
    let ordered_cases = 256 + float_operands::CASE_COUNT;
    let cases = ordered_cases + 1024;
    let mut input = format!("1 1 math {cases} ");
    let mut actual = vec![json!(SUCCESS)];
    for i in 0..cases {
        let mut a = common_point(&mut seed);
        let mut b = common_point(&mut seed);
        let mut c = common_point(&mut seed);
        let mut s = common_point(&mut seed)[0];
        if i >= ordered_cases {
            // Exercise overflow, underflow, cancellation and payloads across
            // independent operand bit patterns, beyond the ordered triples.
            let mut bits = || {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                f32::from_bits(seed)
            };
            a = std::array::from_fn(|_| bits());
            b = std::array::from_fn(|_| bits());
            c = std::array::from_fn(|_| bits());
            s = bits();
        } else if i >= 256 {
            let [x, y, z] = float_operands::operands(i - 256);
            a = [x, y, z];
            b = [y, z, x];
            c = [z, x, y];
            s = z;
        }
        match i {
            0 => {
                a = [0.; 3];
                b = a;
            }
            1 => {
                a = [-0., 0., -0.];
                b = [0., -0., 0.];
            }
            2 => a = [f32::INFINITY, 1., -2.],
            3 => a = [1., f32::NAN, 2.],
            4 => a = [f32::NAN, -1., 3.],
            5 => {
                a = [1., 2., 3.];
                b = [1. + 1. / 16384., 2., 3.];
            }
            6 => b = a,
            _ => {}
        }
        let word = match i {
            0 => 0,
            1 => 1,
            2 => 0x8000_0000,
            3 => 0x8000_0001,
            4 => u32::MAX,
            _ => seed,
        };
        let side = if i == 0 { i32::MIN } else { (seed as i32) >> 3 };
        common_input(&mut input, a.into_iter().chain(b).chain(c).chain([s]));
        write!(input, "{word} {side} ").unwrap();
        let scalars = [
            vector_dot(a, b),
            vector_length(a),
            vector_length_squared(a),
            vector_distance(a, b),
            vector_distance_squared(a, b),
            vector_distance_2d(a, b),
            vector_distance_squared_2d(a, b),
            vector_dot_2d(a, b),
            vector_perpendicular_2d(a, b),
            triangle_area_2d(a, b, c),
            minimum(a[0], b[0]),
            maximum(a[0], b[0]),
            absolute(a[0]),
            square(s),
            clamp(s, a[0], b[0]),
        ]
        .map(f32::to_bits);
        let quant = |p: Point| p.map(|v| v.to_bits() as u16);
        actual.push(json!([
            vector_cross(a, b).map(f32::to_bits),
            vector_multiply_add(a, b, s).map(f32::to_bits),
            vector_lerp(a, b, s).map(f32::to_bits),
            vector_add(a, b).map(f32::to_bits),
            vector_subtract(a, b).map(f32::to_bits),
            vector_scale(a, s).map(f32::to_bits),
            vector_minimum(a, b).map(f32::to_bits),
            vector_maximum(a, b).map(f32::to_bits),
            vector_normalize(a).map(f32::to_bits),
            scalars,
            [
                u8::from(vector_equal(a, b)),
                u8::from(vector_is_finite(a)),
                u8::from(vector_is_finite_2d(a)),
                u8::from(overlap_bounds([a, b], [b, c])),
                u8::from(overlap_quantized_bounds(
                    [quant(a), quant(b)],
                    [quant(b), quant(c)]
                ))
            ],
            [
                json!(next_power_of_two(word)),
                json!(integer_log2(word)),
                json!(align4(side).unwrap()),
                json!(opposite_tile(side).unwrap()),
                json!(swap_endian_u16(word as u16)),
                json!(swap_endian_i16(word as i16)),
                json!(swap_endian_u32(word)),
                json!(swap_endian_i32(word as i32)),
                json!(swap_endian_f32(f32::from_bits(word)).to_bits())
            ]
        ]));
    }
    let expected = common_oracle(&input);
    parity_trace::write("common-math", &input, &actual, &expected);
    for (i, (a, b)) in actual.iter().zip(&expected).enumerate() {
        float_comparison::assert_eq(a, b, |path| match path {
            [field, _] => matches!(*field, 0..=5 | 8),
            _ => false,
        } || matches!(path, [9, scalar] if *scalar <= 9 || *scalar == 13), &format!("math case {i}"));
    }
    assert_eq!(actual.len(), expected.len());
    assert_eq!(align4(i32::MAX), None);
    assert_eq!(opposite_tile(i32::MAX), None);
}
#[test]
fn scalar_templates_preserve_integer_promotion_and_double_bits() {
    use serde_json::json;
    let mut records = String::new();
    let mut actual = vec![json!(SUCCESS)];
    macro_rules! integer_cases {
        ($type:ty, $cases:expr) => {
            let cases: [[$type; 3]; 3] = $cases;
            for [a, b, s] in cases {
                write!(records, "{} {a:?} {b:?} {s:?} ", stringify!($type)).unwrap();
                let values = [
                    minimum(a, b),
                    maximum(a, b),
                    absolute(a),
                    square(s),
                    clamp(s, a, b),
                ];
                actual.push(json!(values.into_iter().chain(values).collect::<Vec<_>>()));
            }
        };
    }
    // Small native integers promote to int before multiplication, then narrow
    // back to T. Wider signed products stay inside the native defined domain.
    integer_cases!(
        bool,
        [
            [false, true, false],
            [true, false, true],
            [true, true, false]
        ]
    );
    integer_cases!(i8, [[-128, 127, -128], [-1, 1, 127], [17, -19, -3]]);
    integer_cases!(u8, [[0, 255, 255], [255, 0, 129], [1, 17, 0]]);
    integer_cases!(
        i16,
        [[-32768, 32767, -32768], [-1, 1, 32767], [17, -19, -3]]
    );
    integer_cases!(u16, [[0, 65535, 46340], [65535, 0, 256], [1, 17, 0]]);
    integer_cases!(
        i32,
        [
            [i32::MIN + 1, i32::MAX, 46340],
            [-1, 1, -46340],
            [17, -19, -3]
        ]
    );
    integer_cases!(
        u32,
        [[0, u32::MAX, u32::MAX], [u32::MAX, 0, 65536], [1, 17, 0]]
    );
    integer_cases!(
        i64,
        [
            [i64::MIN + 1, i64::MAX, 3037000499],
            [-1, 1, -3037000499],
            [17, -19, -3]
        ]
    );
    integer_cases!(
        u64,
        [[0, u64::MAX, u64::MAX], [u64::MAX, 0, 1 << 32], [1, 17, 0]]
    );
    integer_cases!(
        isize,
        [
            [isize::MIN + 1, isize::MAX, 46340],
            [-1, 1, -46340],
            [17, -19, -3]
        ]
    );
    integer_cases!(
        usize,
        [
            [0, usize::MAX, usize::MAX],
            [usize::MAX, 0, 65536],
            [1, 17, 0]
        ]
    );
    for [a, b, s] in [
        [-0., 0., -0.],
        [
            f64::from_bits(0xfff8_1234_5678_9abc),
            1.,
            f64::from_bits(0x7ff8_0012_3456_789a),
        ],
        [1., f64::NAN, f64::INFINITY],
        [-f64::MAX, f64::MAX, f64::from_bits(1)],
        [-17.125, 31.5, -0.00123456789012345],
    ] {
        write!(
            records,
            "f64 {} {} {} ",
            a.to_bits(),
            b.to_bits(),
            s.to_bits()
        )
        .unwrap();
        let values = [
            minimum(a, b),
            maximum(a, b),
            absolute(a),
            square(s),
            clamp(s, a, b),
        ]
        .map(f64::to_bits);
        actual.push(json!(values.into_iter().chain(values).collect::<Vec<_>>()));
    }
    let input = format!("1 1 scalar_templates {} {records}", actual.len() - 1);
    assert_eq!(actual, common_oracle(&input));
}

#[test]
fn native_common_triangle_polygon_distances_and_centers() {
    use serde_json::json;
    let mut seed = 0x1938_9988;
    let mut input = format!("1 1 geometry {} ", 256 + float_operands::CASE_COUNT);
    let mut actual = vec![json!(SUCCESS)];
    for i in 0..256 + float_operands::CASE_COUNT {
        let mut p = common_point(&mut seed);
        let mut a = common_point(&mut seed);
        let mut b = common_point(&mut seed);
        let mut c = common_point(&mut seed);
        if i >= 256 {
            let [x, y, z] = float_operands::operands(i - 256);
            p = [z, x, y];
            a = [1., 2., 3.];
            b = [x, y, z];
            c = [y, z, x];
        }
        match i {
            0 => {
                b = a;
                c = a;
            }
            1 => c = b,
            2 => p = a,
            3 => p = b,
            4 => p = c,
            5 => p = [f32::NAN, 1., 2.],
            6 => p[1] = f32::INFINITY,
            _ => {}
        }
        let vertices = match i % 5 {
            0 => vec![],
            1 => vec![a],
            2 => vec![a, b],
            3 => vec![a, b, c],
            _ => vec![a, b, c, common_point(&mut seed)],
        };
        let indices = (0..vertices.len())
            .rev()
            .map(|j| j as u16)
            .collect::<Vec<_>>();
        common_input(&mut input, p.into_iter().chain(a).chain(b).chain(c));
        write!(input, "{} {} ", vertices.len(), indices.len()).unwrap();
        common_input(&mut input, vertices.iter().flatten().copied());
        for j in &indices {
            write!(input, "{j} ").unwrap();
        }
        let mut h = -17.;
        let height = closest_height_point_triangle_into(p, a, b, c, &mut h);
        let (d, t) = distance_point_segment_squared_2d(p, a, b);
        let mut ed = vec![31.; vertices.len() + 2];
        let mut et = vec![-37.; vertices.len() + 2];
        let inside = distance_point_polygon_edges_squared(p, &vertices, &mut ed, &mut et).unwrap();
        actual.push(json!([
            closest_point_on_triangle(p, a, b, c).map(f32::to_bits),
            [json!(u8::from(height)), json!([h.to_bits()])],
            [[d.to_bits(), t.to_bits()]],
            u8::from(point_in_polygon(p, &vertices)),
            u8::from(inside),
            ed.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
            et.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
            polygon_center(&indices, &vertices)
                .unwrap()
                .map(f32::to_bits)
        ]));
    }
    let expected = common_oracle(&input);
    parity_trace::write("common-geometry", &input, &actual, &expected);
    for (i, (a, b)) in actual.iter().zip(&expected).enumerate() {
        float_comparison::assert_eq(
            a,
            b,
            |path| {
                matches!(path, [0 | 5 | 6 | 7, _] | [2, 0, _])
                    || matches!(path, [1, 1, 0] if b[1][0] == 1)
            },
            &format!("geometry case {i}"),
        );
    }
    assert_eq!(actual.len(), expected.len());
    assert_eq!(polygon_center(&[1], &[[0.; 3]]), None);
    assert_eq!(
        distance_point_polygon_edges_squared([0.; 3], &[[0.; 3]], &mut [], &mut []),
        None
    );
}

#[test]
fn native_math_backend_matches_rust_float_primitives() {
    use serde_json::json;
    let special = [
        0_u32,
        0x8000_0000,
        1,
        0x8000_0001,
        0x0080_0000,
        0x7f7f_ffff,
        0x7f80_0000,
        0xff80_0000,
        0x7fc0_1275,
        0xffc0_1871,
        0x7f80_0001,
        0xff80_0001,
        1_f32.to_bits(),
        (-1_f32).to_bits(),
        std::f32::consts::PI.to_bits(),
        (-std::f32::consts::PI).to_bits(),
        std::f32::consts::FRAC_PI_2.to_bits(),
        ((1_f32 / 31.) * std::f32::consts::PI * 2.).to_bits(),
        ((1_f32 / 31.) * std::f32::consts::PI).to_bits(),
    ];
    let mut cases: Vec<_> = special
        .into_iter()
        .flat_map(|a| special.into_iter().map(move |b| (a, b)))
        .collect();
    let mut seed = 0x1875_31b1_u32;
    for _ in 0..256 {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let a = seed;
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        cases.push((a, seed));
    }
    let mut input = format!("1 1 backend_math {}\n", cases.len());
    for &(a, b) in &cases {
        writeln!(input, "{a} {b}").unwrap();
    }
    let expected = common_oracle(&input);
    assert_eq!(expected[0], json!(SUCCESS));
    let mut actual = vec![json!(SUCCESS)];
    for &(a, b) in &cases {
        let (x, y) = (f32::from_bits(a), f32::from_bits(b));
        let mut values: Vec<_> = [
            x.abs(),
            x.sqrt(),
            floor(x),
            ceil(x),
            cosine(x),
            sine(x),
            x.atan2(y),
        ]
        .map(f32::to_bits)
        .into();
        values.push(u32::from(x.is_finite()));
        actual.push(json!(values));
    }
    parity_trace::write("backend-math", &input, &actual, &expected);
    for (i, (a, b)) in cases.into_iter().enumerate() {
        assert_eq!(
            actual[i + 1],
            expected[i + 1],
            "math backend case {i}: a={a:08x}, b={b:08x}"
        );
    }
}
