#![cfg(feature = "cpp-oracle")]
use recast_nav::{
    cache_data::{CompressionResult, Compressor, Header},
    nav::*,
    nav_data::*,
    tile_cache::*,
};
use serde_json::{Value, json};
use std::{
    cell::RefCell,
    fmt::Write as _,
    io::Write as _,
    process::{Command, Stdio},
    rc::Rc,
};

struct CopyCodec {
    status: u32,
    log: Vec<i64>,
}
impl Compressor for CopyCodec {
    fn max_compressed_size(&mut self, n: usize) -> usize {
        n
    }
    fn compress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult {
        output[..input.len()].copy_from_slice(input);
        CompressionResult {
            status: SUCCESS,
            size: input.len(),
        }
    }
    fn decompress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult {
        self.log
            .extend([2, input.len() as i64, output.len() as i64]);
        self.log
            .extend(input.iter().chain(output.iter()).map(|&b| i64::from(b)));
        let n = input.len().min(output.len());
        output[..n].copy_from_slice(&input[..n]);
        CompressionResult {
            status: self.status,
            size: n,
        }
    }
}
#[derive(Default)]
struct ProcessState {
    mode: i32,
    log: Vec<Value>,
}
struct Process(Rc<RefCell<ProcessState>>);
impl MeshProcess for Process {
    fn process(&mut self, p: &mut OwnedCreateParams) {
        let mut state = self.0.borrow_mut();
        let mode = state.mode;
        state.log.push(json!([
            mode,
            p.vertices.len(),
            p.polygons.len(),
            p.vertices_per_polygon,
            p.coordinate,
            p.bounds[0].map(f32::to_bits),
            p.bounds[1].map(f32::to_bits),
            p.polygons
                .iter()
                .map(|p| [u32::from(p.area), u32::from(p.flags)])
                .collect::<Vec<_>>()
        ]));
        if mode == 0 {
            return;
        }
        for (i, p) in p.polygons.iter_mut().enumerate() {
            p.area = ((i32::from(p.area) + mode) % 64) as u8;
            p.flags = ((i + 1) * 3) as u16;
        }
        match mode {
            2 => p.polygons.clear(),
            3 => p.coordinate[0] += 1,
            4 => {
                p.user_id = 0x87654321;
                p.build_bv_tree = true;
            }
            5 => {
                p.off_mesh_connections = vec![CreateOffMeshConnection {
                    positions: [[0.5, 0., 0.5], [3.5, 0., 3.5]],
                    radius: 0.75,
                    flags: 7,
                    area: 5,
                    direction: 1,
                }];
                p.off_mesh_user_ids = Some(vec![0xfedcba98]);
            }
            _ => {}
        }
    }
}
fn params(tiles: i32, obs: i32) -> Params {
    Params {
        origin: [0.; 3],
        cell_size: 1.,
        cell_height: 0.5,
        width: 4,
        height: 4,
        walkable_height: 2.,
        walkable_radius: 0.5,
        walkable_climb: 0.5,
        max_simplification_error: 1.3,
        max_tiles: tiles,
        max_obstacles: obs,
    }
}
fn floats(s: &mut String, a: impl IntoIterator<Item = f32>) {
    for v in a {
        write!(s, "{} ", v.to_bits()).unwrap();
    }
}
fn param_input(s: &mut String, p: &Params) {
    floats(s, p.origin);
    floats(s, [p.cell_size, p.cell_height]);
    write!(s, "{} {} ", p.width, p.height).unwrap();
    floats(
        s,
        [
            p.walkable_height,
            p.walkable_radius,
            p.walkable_climb,
            p.max_simplification_error,
        ],
    );
    write!(s, "{} {} ", p.max_tiles, p.max_obstacles).unwrap();
}
fn tile(p: &Params, coord: [i32; 3], empty: bool) -> Vec<u8> {
    let cs = p.cell_size;
    let x = coord[0] as f32 * 4. * cs;
    let z = coord[1] as f32 * 4. * cs;
    let y = coord[2] as f32 * 4.;
    let h = Header {
        coordinate: coord,
        bounds: [[x, y, z], [x + 4. * cs, y + 2., z + 4. * cs]],
        width: 4,
        height: 4,
        usable_bounds: [[0, 0], [3, 3]],
        ..Header::default()
    };
    let mut data = h.to_bytes().to_vec();
    data.extend([0; 16]);
    data.extend([if empty { 0 } else { 63 }; 16]);
    for z in 0..4 {
        for x in 0..4 {
            let mut con = 0;
            for (d, (dx, dz)) in [(-1, 0), (0, 1), (1, 0), (0, -1)].into_iter().enumerate() {
                if (0..4).contains(&(x + dx)) && (0..4).contains(&(z + dz)) {
                    con |= 1 << d;
                } else {
                    con |= 1 << (d + 4);
                }
            }
            data.push(con);
        }
    }
    data
}
#[derive(Clone)]
enum Op {
    Add(Vec<u8>, u8),
    Remove(u32),
    Cylinder(Point, f32, f32),
    Box([Point; 2]),
    Oriented(Point, Point, f32),
    Delete(u32),
    Update(f32),
    Build(u32),
    BuildAt(i32, i32),
    Query([Point; 2], usize),
    At(i32, i32, usize),
    Handles(u32, u32),
    Encode(u32, u32),
    Tight(usize),
    Codec(u32),
    Process(i32),
    Init(Params),
}
struct Harness {
    cache: TileCache<CopyCodec>,
    mesh: NavMesh,
    state: Rc<RefCell<ProcessState>>,
    input: String,
    ops: Vec<String>,
    actual: Vec<Value>,
}
impl Harness {
    fn new(p: Params, nav_tiles: usize, nav_polys: usize, mode: i32) -> Self {
        let mut input = "object ".to_owned();
        param_input(&mut input, &p);
        write!(
            input,
            "{nav_tiles} {nav_polys} {mode} 0 0 -1 -1 {SUCCESS} {SUCCESS} "
        )
        .unwrap();
        let state = Rc::new(RefCell::new(ProcessState {
            mode,
            ..ProcessState::default()
        }));
        let process = (mode >= 0).then(|| Box::new(Process(state.clone())) as Box<dyn MeshProcess>);
        let mesh = NavMesh::new(
            p.origin,
            p.width as f32 * p.cell_size,
            p.height as f32 * p.cell_size,
            nav_tiles,
            nav_polys,
        )
        .unwrap();
        let cache = TileCache::new(
            p,
            CopyCodec {
                status: SUCCESS,
                log: vec![],
            },
            process,
        )
        .ok()
        .unwrap();
        let mut h = Self {
            cache,
            mesh,
            state,
            input,
            ops: vec![],
            actual: vec![],
        };
        let snap = h.snapshot();
        h.actual.push(json!([SUCCESS, snap]));
        h
    }
    fn snapshot(&mut self) -> Value {
        let tiles = self
            .cache
            .tiles()
            .iter()
            .map(|t| {
                json!([
                    t.salt,
                    t.header.as_ref().map(|h| h.to_bytes().to_vec()),
                    t.data,
                    t.flags,
                    t.next
                ])
            })
            .collect::<Vec<_>>();
        let obs = self
            .cache
            .obstacles()
            .iter()
            .map(|o| {
                let b = o.bounds();
                json!([
                    o.shape.map(f32::to_bits),
                    o.touched,
                    o.pending,
                    o.salt,
                    o.kind,
                    o.state,
                    o.touched_count,
                    o.pending_count,
                    o.next,
                    b[0].map(f32::to_bits),
                    b[1].map(f32::to_bits)
                ])
            })
            .collect::<Vec<_>>();
        let mesh = self
            .mesh
            .tiles
            .iter()
            .enumerate()
            .map(|(i, t)| {
                json!([
                    self.mesh.reference(i, 0),
                    t.as_ref().map(|t| t.data().unwrap()),
                    self.mesh.tile(i as i32).unwrap().flags,
                    self.mesh.tile(i as i32).unwrap().data_size,
                    self.mesh.tile(i as i32).unwrap().free_link
                ])
            })
            .collect::<Vec<_>>();
        json!([
            tiles,
            obs,
            mesh,
            std::mem::take(&mut self.cache.compressor().log),
            std::mem::take(&mut self.state.borrow_mut().log)
        ])
    }
    fn op(&mut self, op: Op) -> Value {
        let mut s = String::new();
        let result = match op {
            Op::Add(data, flags) => {
                write!(s, "A {flags} {} ", data.len()).unwrap();
                for b in &data {
                    write!(s, "{b} ").unwrap();
                }
                reference(self.cache.add_tile(data, flags))
            }
            Op::Remove(r) => {
                write!(s, "R {r} ").unwrap();
                match self.cache.remove_tile(r) {
                    Ok(d) => json!([SUCCESS, d]),
                    Err(e) => json!([e, null]),
                }
            }
            Op::Cylinder(p, r, h) => {
                s.push_str("C ");
                floats(&mut s, p);
                floats(&mut s, [r, h]);
                reference(self.cache.add_cylinder(p, r, h))
            }
            Op::Box(b) => {
                s.push_str("B ");
                floats(&mut s, b.into_iter().flatten());
                reference(self.cache.add_box(b))
            }
            Op::Oriented(p, h, a) => {
                s.push_str("O ");
                floats(&mut s, p);
                floats(&mut s, h);
                floats(&mut s, [a]);
                reference(self.cache.add_oriented_box(p, h, a))
            }
            Op::Delete(r) => {
                write!(s, "X {r} ").unwrap();
                json!(self.cache.remove_obstacle(r))
            }
            Op::Update(dt) => {
                s.push_str("U ");
                floats(&mut s, [dt]);
                let (status, done) = self.cache.update(dt, &mut self.mesh);
                json!([status, u8::from(done)])
            }
            Op::Build(r) => {
                write!(s, "T {r} ").unwrap();
                json!(self.cache.build_tile(r, &mut self.mesh))
            }
            Op::BuildAt(x, y) => {
                write!(s, "V {x} {y} ").unwrap();
                json!(self.cache.build_tiles_at(x, y, &mut self.mesh))
            }
            Op::Query(b, c) => {
                s.push_str("Q ");
                floats(&mut s, b.into_iter().flatten());
                write!(s, "{c} ").unwrap();
                match self.cache.query_tiles(b, c) {
                    Ok(r) => json!([SUCCESS, r]),
                    Err(e) => json!([e, []]),
                }
            }
            Op::At(x, y, c) => {
                write!(s, "G {x} {y} {c} ").unwrap();
                json!(self.cache.tiles_at(x, y, c))
            }
            Op::Handles(t, o) => {
                write!(s, "H {t} {o} ").unwrap();
                let ti = self
                    .cache
                    .tile_by_ref(t)
                    .map(|_| self.cache.decode_tile_id(t).1);
                let oi = self
                    .cache
                    .obstacle_by_ref(o)
                    .map(|_| TileCache::<CopyCodec>::decode_obstacle_id(o).1);
                json!([ti, oi])
            }
            Op::Tight(index) => {
                write!(s, "L {index} ").unwrap();
                let b = self
                    .cache
                    .tight_tile_bounds(self.cache.tiles()[index].header.as_ref().unwrap());
                json!([b[0].map(f32::to_bits), b[1].map(f32::to_bits)])
            }
            Op::Encode(a, b) => {
                write!(s, "E {a} {b} ").unwrap();
                let t = self.cache.encode_tile_id(a, b);
                let (ts, ti) = self.cache.decode_tile_id(t);
                let o = TileCache::<CopyCodec>::encode_obstacle_id(a, b);
                let (os, oi) = TileCache::<CopyCodec>::decode_obstacle_id(o);
                json!([t, ts, ti, o, os, oi])
            }
            Op::Codec(status) => {
                write!(s, "D {status} ").unwrap();
                self.cache.compressor().status = status;
                json!(SUCCESS)
            }
            Op::Process(mode) => {
                write!(s, "P {mode} ").unwrap();
                self.state.borrow_mut().mode = mode;
                json!(SUCCESS)
            }
            Op::Init(p) => {
                s.push_str("I ");
                param_input(&mut s, &p);
                json!(match self.cache.reinitialize(p) {
                    Ok(()) => SUCCESS,
                    Err(e) => e,
                })
            }
        };
        let snap = self.snapshot();
        self.actual.push(json!([result, snap]));
        self.ops.push(s);
        result
    }
    fn check(mut self) {
        write!(self.input, "{} ", self.ops.len()).unwrap();
        for op in &self.ops {
            self.input.push_str(op);
        }
        let mut child = Command::new(env!("TILE_CACHE_ORACLE"))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .unwrap();
        let mut stdin = child.stdin.take().unwrap();
        let input = self.input.as_bytes();
        let (out, input_result) = std::thread::scope(|scope| {
            let writer = scope.spawn(move || stdin.write_all(input));
            let out = child.wait_with_output().unwrap();
            (out, writer.join().unwrap())
        });
        if !out.status.success() || input_result.is_err() {
            self.save_failure_input();
            panic!(
                "native cache oracle exited with {}; input result: {input_result:?}",
                out.status
            );
        }
        let expected: Value = serde_json::from_slice(&out.stdout).unwrap();
        assert_eq!(self.actual.len(), expected.as_array().unwrap().len());
        for (i, a) in self.actual.iter().enumerate() {
            if let Some((path, a, b)) = difference(a, &expected[i], String::new()) {
                self.save_failure_input();
                panic!(
                    "operation {i} {} at {path}: Rust {a}, native {b}",
                    i.checked_sub(1)
                        .map_or("init", |j| self.ops[j].split_whitespace().next().unwrap())
                );
            }
        }
    }
    fn save_failure_input(&self) {
        let directory =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/parity-failures");
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(directory.join("cache-object.txt"), &self.input).unwrap();
    }
}
fn difference<'a>(
    a: &'a Value,
    b: &'a Value,
    path: String,
) -> Option<(String, &'a Value, &'a Value)> {
    if a == b {
        return None;
    }
    if let (Some(a), Some(b)) = (a.as_array(), b.as_array())
        && a.len() == b.len()
    {
        for (i, (a, b)) in a.iter().zip(b).enumerate() {
            if let Some(d) = difference(a, b, format!("{path}/{i}")) {
                return Some(d);
            }
        }
    }
    Some((path, a, b))
}
fn reference(r: Result<u32, u32>) -> Value {
    match r {
        Ok(r) => json!([SUCCESS, r]),
        Err(e) => json!([e, 0xabcdefu32]),
    }
}
fn returned(v: Value) -> u32 {
    assert_eq!(v[0], SUCCESS);
    v[1].as_u64().unwrap() as u32
}

#[test]
fn native_cache_streaming_shapes_callbacks_and_failed_rebuild_lifecycle() {
    for mode in [-1, 0, 1, 4, 5] {
        let mut p = params(8, 6);
        if mode == 1 {
            p.cell_size = 0.3;
            p.cell_height = 0.2;
        }
        let mut h = Harness::new(p.clone(), 8, 64, mode);
        h.op(Op::Handles(h.cache.tile_ref(0), h.cache.obstacle_ref(0)));
        for (a, b) in [(0, 0), (1, 7), (0xffffffff, 0xffffffff), (0x12345, 0x10003)] {
            h.op(Op::Encode(a, b));
        }
        let mut refs = vec![];
        for coord in [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]] {
            refs.push(returned(h.op(Op::Add(tile(&p, coord, false), 0))));
        }
        h.op(Op::Add(tile(&p, [0, 0, 0], false), 1));
        for i in 0..4 {
            h.op(Op::Tight(i));
        }
        let mut special = tile(&p, [-1, 1, 0], false);
        special[50..54].copy_from_slice(&[1, 255, 2, 254]);
        let special = returned(h.op(Op::Add(special, 1)));
        h.op(Op::Tight(h.cache.decode_tile_id(special).1 as usize));
        h.op(Op::Query([[-1., f32::NAN, -1.], [9., f32::NAN, 9.]], 20));
        h.op(Op::BuildAt(0, 0));
        h.op(Op::BuildAt(1, 0));
        for cap in [0, 1, 8] {
            h.op(Op::At(0, 0, cap));
            h.op(Op::Query([[-1., -1., -1.], [9., 9., 9.]], cap));
        }
        let c = returned(h.op(Op::Cylinder([3.8, 0., 2.], 0.6, 1.2)));
        let b = returned(h.op(Op::Box([[1., 0., 1.], [2., 1., 2.]])));
        let o = returned(h.op(Op::Oriented([2., 0., 3.7], [0.8, 0.5, 0.4], 0.731)));
        h.op(Op::Update(f32::NAN));
        h.op(Op::Delete(c));
        h.op(Op::Update(0.1));
        h.op(Op::Update(0.));
        h.op(Op::Update(1.));
        h.op(Op::Update(-1.));
        h.op(Op::Delete(b));
        h.op(Op::Delete(o));
        for _ in 0..5 {
            h.op(Op::Update(0.));
        }
        h.op(Op::Handles(refs[0], c));
        h.op(Op::Delete(c));
        h.op(Op::Delete(0));
        let far = returned(h.op(Op::Cylinder([100., 0., 100.], 0.1, 1.)));
        h.op(Op::Update(0.));
        assert_eq!(h.cache.obstacle_by_ref(far).unwrap().state, PROCESSING);
        h.op(Op::Delete(far));
        h.op(Op::Update(0.));
        assert_eq!(h.cache.obstacle_by_ref(far).unwrap().state, REMOVING);
        let c = returned(h.op(Op::Cylinder([0.6, 0., 0.6], 0.2, 1.)));
        h.op(Op::Codec(FAILURE | WRONG_VERSION));
        h.op(Op::Update(0.));
        assert_eq!(h.cache.obstacle_by_ref(c).unwrap().state, PROCESSED);
        h.op(Op::Codec(SUCCESS));
        h.op(Op::Handles(refs[0], far));
        h.op(Op::Remove(refs[0]));
        h.op(Op::Remove(refs[0]));
        h.op(Op::Handles(refs[0], c));
        let r = returned(h.op(Op::Add(tile(&p, [0, 0, 0], true), FREE_DATA)));
        h.op(Op::Build(r));
        h.op(Op::Remove(r));
        if mode >= 0 {
            h.op(Op::Process(2));
            h.op(Op::Build(refs[1]));
            h.op(Op::Process(3));
            h.op(Op::Build(refs[2]));
            h.op(Op::Build(refs[2]));
        }
        h.check();
    }
}
#[test]
fn native_cache_limits_layer_order_and_queued_streaming() {
    let p = params(72, 80);
    let mut h = Harness::new(p.clone(), 72, 64, 1);
    let mut refs = vec![];
    for layer in 0..34 {
        refs.push(returned(h.op(Op::Add(
            tile(&p, [0, 0, layer], false),
            if layer % 2 == 0 { 1 } else { 0 },
        ))));
    }
    for cap in [0, 1, 31, 32, 33, 100] {
        h.op(Op::At(0, 0, cap));
        h.op(Op::Query([[-1., -1., -1.], [4., 200., 4.]], cap));
    }
    h.op(Op::BuildAt(0, 0));
    for i in 0..64 {
        h.op(Op::Delete(0xf0000000 + i));
    }
    h.op(Op::Cylinder([1., 0., 1.], 0.1, 1.));
    h.op(Op::Delete(0));
    h.op(Op::Delete(0xffffffff));
    h.op(Op::Update(0.));
    let c = returned(h.op(Op::Cylinder([1., 0., 1.], 0.2, 200.)));
    h.op(Op::Update(0.));
    h.op(Op::Remove(refs[32]));
    h.op(Op::Delete(c));
    for _ in 0..16 {
        h.op(Op::Update(0.));
    }
    h.op(Op::Init(p));
    h.op(Op::Handles(refs[0], c));
    h.op(Op::Update(0.));
    h.check();
}

#[test]
fn native_cache_pool_exhaustion_update_saturation_and_reinitialization() {
    let p = params(2, 1);
    let mut h = Harness::new(p.clone(), 1, 64, 1);
    let r = returned(h.op(Op::Add(tile(&p, [0, 0, 0], false), 1)));
    let other = returned(h.op(Op::Add(tile(&p, [1, 0, 0], false), 1)));
    assert_eq!(
        h.op(Op::Add(tile(&p, [2, 0, 0], false), 0))[0],
        FAILURE | OUT_OF_MEMORY
    );
    let mut invalid = tile(&p, [2, 0, 0], false);
    invalid[0] ^= 1;
    assert_eq!(h.op(Op::Add(invalid, 0))[0], FAILURE | WRONG_MAGIC);
    let mut invalid = tile(&p, [2, 0, 0], false);
    invalid[4] ^= 1;
    assert_eq!(h.op(Op::Add(invalid, 0))[0], FAILURE | WRONG_VERSION);
    h.op(Op::Build(r));
    assert_eq!(h.op(Op::Build(other)), FAILURE | OUT_OF_MEMORY);
    let ob = returned(h.op(Op::Oriented([1., 0., 1.], [0.1, 0.5, 0.2], -2.713)));
    assert_eq!(
        h.op(Op::Box([[0.; 3], [1.; 3]]))[0],
        FAILURE | OUT_OF_MEMORY
    );
    h.op(Op::Codec(IN_PROGRESS | PARTIAL_RESULT));
    h.op(Op::Update(0.));
    h.op(Op::Delete(ob));
    h.op(Op::Update(0.));
    let next = returned(h.op(Op::Cylinder([1., 0., 1.], 0.1, 0.5)));
    assert_ne!(next, ob);
    h.op(Op::Handles(r, ob));
    h.check();

    let p = params(72, 12);
    let mut h = Harness::new(p.clone(), 72, 64, 1);
    for y in 0..9 {
        for x in 0..8 {
            h.op(Op::Add(tile(&p, [x, y, 0], false), 1));
        }
    }
    for y in 0..9 {
        h.op(Op::Box([
            [0.1, 0., y as f32 * 4. + 0.1],
            [31.9, 1., y as f32 * 4. + 3.9],
        ]));
    }
    // This duplicate also receives no pending entries because the 64-slot
    // update queue is already full when its request is processed.
    let duplicate = returned(h.op(Op::Cylinder([1., 0., 1.], 0.1, 1.)));
    h.op(Op::Update(0.));
    assert_eq!(h.cache.obstacle_by_ref(duplicate).unwrap().pending_count, 0);
    assert_eq!(h.cache.obstacle_by_ref(duplicate).unwrap().state, PROCESSED);
    // Reinitialization retains 63 queued old references. Free slots are valid
    // ref lookups, but decompression fails; each failure still consumes a slot.
    h.op(Op::Init(p));
    for _ in 0..64 {
        h.op(Op::Update(0.));
    }
    h.check();
}

#[test]
fn native_cache_seeded_streaming_and_obstacle_operation_sequences() {
    // Failed native adds retain the caller's sentinel handle. Keep those in
    // the stream too, alongside valid and stale handles.
    fn handle(value: Value) -> u32 {
        value[1].as_u64().unwrap() as u32
    }
    fn word(seed: &mut u32) -> u32 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 17;
        *seed ^= *seed << 5;
        *seed
    }
    fn point(seed: &mut u32) -> Point {
        [
            (word(seed) % 21) as f32 * 0.23 - 0.4,
            (word(seed) % 5) as f32 * 0.2,
            (word(seed) % 21) as f32 * 0.19 - 0.3,
        ]
    }
    for initial in [0x317a_9b45, 0xa19d_3c67, 0x1397_8b29, 0x743b_91d3]
        .into_iter()
        .chain((1..=28u32).map(|seed| seed.wrapping_mul(0x9e37_79b9)))
    {
        let mut seed = initial;
        let mut p = params(6, 8);
        p.cell_size = 0.3;
        p.cell_height = 0.2;
        let mut h = Harness::new(p.clone(), 6, 64, 0);
        let mut tiles = Vec::new();
        let mut obstacles = vec![0];
        for coordinate in [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]] {
            let r = returned(h.op(Op::Add(tile(&p, coordinate, false), 0)));
            tiles.push(r);
            h.op(Op::Build(r));
        }
        for step in 0..192 {
            if step == 96 {
                h.op(Op::Init(p.clone()));
                for coordinate in [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]] {
                    let r = returned(h.op(Op::Add(tile(&p, coordinate, false), FREE_DATA)));
                    tiles.push(r);
                    h.op(Op::Build(r));
                }
            }
            let choice = word(&mut seed) % 16;
            match choice {
                0 => {
                    let coordinate = [
                        (word(&mut seed) % 3) as i32,
                        (word(&mut seed) % 3) as i32,
                        (word(&mut seed) % 2) as i32,
                    ];
                    let empty = word(&mut seed).is_multiple_of(5);
                    let flags = (word(&mut seed) & 1) as u8;
                    let r = handle(h.op(Op::Add(tile(&p, coordinate, empty), flags)));
                    tiles.push(r);
                }
                1 => {
                    let r = tiles[(word(&mut seed) as usize) % tiles.len()];
                    if h.cache
                        .tile_by_ref(r)
                        .is_some_and(|tile| tile.header.is_none())
                    {
                        // Reinit can revive an old salt on a vacant slot.
                        // Native lookup permits it, but removal dereferences
                        // the null header. Compare only the defined lookup.
                        assert_eq!(h.cache.remove_tile(r), Err(FAILURE | INVALID_PARAM));
                        h.op(Op::Handles(r, 0));
                    } else {
                        h.op(Op::Remove(r));
                    }
                }
                2 => {
                    let position = point(&mut seed);
                    let radius = (word(&mut seed) % 6) as f32 * 0.17;
                    let height = 0.1 + (word(&mut seed) % 9) as f32 * 0.13;
                    obstacles.push(handle(h.op(Op::Cylinder(position, radius, height))));
                }
                3 => {
                    let min = point(&mut seed);
                    let max =
                        std::array::from_fn(|i| min[i] + 0.1 + (word(&mut seed) % 7) as f32 * 0.13);
                    obstacles.push(handle(h.op(Op::Box([min, max]))));
                }
                4 => {
                    let position = point(&mut seed);
                    let extents =
                        std::array::from_fn(|_| 0.1 + (word(&mut seed) % 6) as f32 * 0.11);
                    let angle = (word(&mut seed) % 13) as f32 * 0.19 - 0.7;
                    obstacles.push(handle(h.op(Op::Oriented(position, extents, angle))));
                }
                5 => {
                    let r = obstacles[(word(&mut seed) as usize) % obstacles.len()];
                    h.op(Op::Delete(r));
                }
                6 => {
                    let r = tiles[(word(&mut seed) as usize) % tiles.len()];
                    h.op(Op::Build(r));
                }
                7 => {
                    h.op(Op::BuildAt(
                        (word(&mut seed) % 3) as i32,
                        (word(&mut seed) % 3) as i32,
                    ));
                }
                8 => {
                    h.op(Op::Process((word(&mut seed) % 6) as i32));
                }
                9 => {
                    let status = if word(&mut seed).is_multiple_of(4) {
                        FAILURE | WRONG_VERSION
                    } else {
                        SUCCESS
                    };
                    h.op(Op::Codec(status));
                }
                10 => {
                    let a = point(&mut seed);
                    let b = std::array::from_fn(|i| a[i] + 1.7);
                    h.op(Op::Query([a, b], (word(&mut seed) % 9) as usize));
                }
                11 => {
                    h.op(Op::At(
                        (word(&mut seed) % 3) as i32,
                        (word(&mut seed) % 3) as i32,
                        (word(&mut seed) % 9) as usize,
                    ));
                }
                _ => {
                    h.op(Op::Update(0.));
                }
            }
        }
        h.op(Op::Codec(SUCCESS));
        for _ in 0..16 {
            h.op(Op::Update(0.));
        }
        eprintln!("TileCache lifecycle seed {initial:#x}");
        h.check();
    }
}
