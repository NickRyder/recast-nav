#![cfg(feature = "cpp-oracle")]
use recast_nav::{nav::*, nodes::*};
use serde_json::{Value, json};
use std::{
    fmt::Write as _,
    io::Write as _,
    process::{Command, Stdio},
};
#[path = "support/node_snapshot.rs"]
mod node_snapshot;

fn oracle(input: String) -> Value {
    let mut child = Command::new(env!("DETOUR_MESH_ORACLE"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let out = std::thread::scope(|s| {
        let writer = s.spawn(move || stdin.write_all(input.as_bytes()).unwrap());
        let out = child.wait_with_output().unwrap();
        writer.join().unwrap();
        out
    });
    assert!(out.status.success());
    serde_json::from_slice(&out.stdout).unwrap()
}
struct PoolRun {
    pool: NodePool,
    input: String,
    actual: Vec<Value>,
    seed: u32,
}
impl PoolRun {
    fn new(capacity: usize, hash: usize) -> Self {
        Self {
            pool: NodePool::new(capacity, hash).unwrap(),
            input: format!("1 1 pool {capacity} {hash}\n"),
            actual: vec![json!(SUCCESS)],
            seed: 0x34a1_794e,
        }
    }
    fn random(&mut self) -> u32 {
        self.seed = self.seed.wrapping_mul(1664525).wrapping_add(1013904223);
        self.seed
    }
    fn get(&mut self, r: PolyRef, state: u8, set_position: bool) {
        let bits: [u32; 5] = std::array::from_fn(|_| self.random());
        let parent = self.random();
        let flags = self.random() as u8;
        write!(self.input, "G {r} {state} {} ", u8::from(set_position)).unwrap();
        for b in bits {
            write!(self.input, "{b} ").unwrap();
        }
        writeln!(self.input, "{parent} {flags}").unwrap();
        if let Some(index) = self.pool.get_node(r, state) {
            let n = self.pool.node_mut(index).unwrap();
            self.actual.push(json!([
                index,
                n.parent_index(),
                n.cost.to_bits(),
                n.total.to_bits(),
                n.state(),
                n.flags()
            ]));
            if set_position {
                n.position = std::array::from_fn(|i| f32::from_bits(bits[i]));
            }
            n.cost = f32::from_bits(bits[3]);
            n.total = f32::from_bits(bits[4]);
            n.set_parent_index(parent);
            n.set_flags(flags);
        } else {
            self.actual.push(json!([0]));
        }
    }
    fn find(&mut self, r: PolyRef, state: u8) {
        writeln!(self.input, "F {r} {state}").unwrap();
        self.actual
            .push(json!([self.pool.find_node(r, state).unwrap_or(0)]));
    }
    fn find_many(&mut self, r: PolyRef, capacity: usize) {
        writeln!(self.input, "M {r} {capacity}").unwrap();
        self.actual.push(json!(self.pool.find_nodes(r, capacity)));
    }
    fn clear(&mut self) {
        self.input.push_str("C\n");
        self.pool.clear();
        self.actual.push(json!([]));
    }
    fn backing(&mut self, index: u32) {
        writeln!(self.input, "A {index}").unwrap();
        let mut values = vec![i64::from(index)];
        if let Some(n) = self.pool.node_at_index(index) {
            values.extend([
                n.reference as i64,
                n.parent_index().into(),
                n.state().into(),
                n.flags().into(),
            ]);
            values.extend(
                n.position
                    .into_iter()
                    .chain([n.cost, n.total])
                    .map(|v| i64::from(v.to_bits())),
            );
        } else {
            values[0] = 0;
        }
        self.actual.push(json!(values));
    }
    fn write_backing(&mut self, index: u32, bits: u32) {
        writeln!(self.input, "W {index} {bits}").unwrap();
        self.pool.node_at_index_mut(index).unwrap().position[0] = f32::from_bits(bits);
        self.actual.push(json!([]));
    }
    fn snapshot(&mut self) {
        self.input.push_str("S\n");
        self.actual.push(node_snapshot::snapshot(&self.pool));
    }
    fn finish(self) {
        assert_eq!(Value::Array(self.actual), oracle(self.input));
    }
}
#[test]
fn node_pool_hash_chains_states_capacity_and_clear_match_detour() {
    for (capacity, hash) in [(1, 1), (7, 1), (17, 8), (65, 32)] {
        let mut r = PoolRun::new(capacity, hash);
        r.snapshot();
        for i in 0..capacity {
            r.get((10000 + i) as PolyRef, (i % 4) as u8, true);
        }
        r.snapshot();
        r.get(10000, 0, true);
        r.get(10000, 7, true);
        r.get(1, 0, true);
        r.snapshot();
        r.clear();
        r.snapshot();
        r.backing(0);
        for index in 1..=capacity as u32 {
            r.backing(index);
        }
        r.write_backing(1, 0x7fc0_2197);
        r.backing(1);
        let ids = [
            0,
            1,
            0x8000_0001,
            0x1234_5678,
            0x0001_0000_0000_0001,
            u64::MAX,
        ];
        for i in 0..capacity + 9 {
            let id = ids[i % ids.len()] as PolyRef;
            let state = [0, 1, 2, 3, 7, 255][i % 6];
            r.get(id, state, false);
            for find_state in [0, 3, 7, 255] {
                r.find(id, find_state);
            }
            for max in [0, 1, 2, 7] {
                r.find_many(id, max);
            }
        }
        r.snapshot();
        r.finish();
    }
    for (capacity, hash) in [(0, 1), (65536, 1), (4, 3)] {
        assert!(matches!(NodePool::new(capacity, hash), Err(s) if s == FAILURE | INVALID_PARAM));
    }
}

#[test]
fn node_queue_order_matches_native_for_ties_and_unordered_costs() {
    for bits in [
        vec![1f32.to_bits(), 1f32.to_bits(), 0, 2f32.to_bits()],
        vec![0, 0x7fc0_1234, 1f32.to_bits(), 2f32.to_bits()],
        vec![
            f32::INFINITY.to_bits(),
            0,
            f32::NEG_INFINITY.to_bits(),
            0x8000_0000,
        ],
        vec![0x7fc0_0001, 0x7fc0_0002, 0x7fc0_0003, 0],
        // Draining crosses complete and lone-child levels with ties and NaNs.
        (0..32)
            .map(|i| {
                [
                    0,
                    0x8000_0000,
                    0x3f80_0000,
                    0x3f80_0000,
                    0x7fc0_1234,
                    0x7f80_0000,
                    0xff80_0000,
                    0xbf80_0000,
                ][i % 8]
            })
            .collect(),
    ] {
        let mut input = format!("1 1 queue {} ", bits.len());
        let mut pool = NodePool::new(bits.len(), 1).unwrap();
        let mut queue = NodeQueue::new(bits.len()).unwrap();
        for (i, bits) in bits.into_iter().enumerate() {
            write!(input, "{bits} ").unwrap();
            let index = pool.get_node((i + 1) as PolyRef, 0).unwrap();
            pool.node_mut(index).unwrap().total = f32::from_bits(bits);
            queue.push(index, &pool).unwrap();
        }
        let mut actual = vec![json!(SUCCESS)];
        while let Some(index) = queue.pop(&pool) {
            actual.push(json!(pool.node(index).unwrap().reference));
        }
        assert_eq!(Value::Array(actual), oracle(input));
    }
}

#[test]
fn node_queue_modify_clear_duplicates_and_retained_pool_storage() {
    let mut pool = NodePool::new(5, 1).unwrap();
    let mut queue = NodeQueue::new(8).unwrap();
    for i in 1..=5 {
        pool.get_node(i as PolyRef, 0).unwrap();
    }
    let commands = [
        "P 1",
        "P 2",
        "P 3",
        "S",
        "U 3 3204448256",
        "T",
        "P 4",
        "U 4 2143289345",
        "P 2",
        "X",
        "U 2 4286578688",
        "P 5",
        "U 5 2139095040",
        "X",
        "X",
        "X",
        "X",
        "X",
        "S",
        "T",
        "P 1",
        "P 2",
        "C",
        "U 1 3212836864",
        "P 1",
        "P 2",
        "S",
        "B",
        "S",
        "X",
        "X",
        "P 3",
        "S",
        "X",
        "C",
    ];
    let mut input = "1 1 queue_ops 5 8\n".to_owned();
    let mut actual = vec![json!(SUCCESS)];
    for command in commands {
        writeln!(input, "{command}").unwrap();
        let mut parts = command.split_whitespace();
        let value = match parts.next().unwrap() {
            "P" => {
                queue
                    .push(parts.next().unwrap().parse().unwrap(), &pool)
                    .unwrap();
                json!([])
            }
            "U" => {
                let i = parts.next().unwrap().parse().unwrap();
                let bits = parts.next().unwrap().parse().unwrap();
                pool.node_mut(i).unwrap().total = f32::from_bits(bits);
                queue.modify(i, &pool);
                json!([])
            }
            "X" => json!([queue.pop(&pool).unwrap_or(0)]),
            "T" => json!([queue.top().unwrap_or(0)]),
            "S" => json!([
                queue.capacity(),
                u8::from(queue.is_empty()),
                queue.top().unwrap_or(0)
            ]),
            "C" => {
                queue.clear();
                json!([])
            }
            "B" => {
                pool.clear();
                json!([])
            }
            _ => unreachable!(),
        };
        actual.push(value);
    }
    assert_eq!(Value::Array(actual), oracle(input));
    assert_eq!(queue.len(), 0);
    assert_eq!(queue.push(6, &pool), Err(FAILURE | INVALID_PARAM));
    assert!(matches!(NodeQueue::new(0), Err(s) if s == FAILURE | INVALID_PARAM));
}

#[test]
fn node_queue_retained_head_spare_slot_and_initialized_null_match_native() {
    let mut pool = NodePool::new(3, 1).unwrap();
    for reference in 1..=3 {
        pool.get_node(reference, 0).unwrap();
    }
    for commands in [
        vec![
            "P 1", "P 2", "P 3", "S", "T", "X", "X", "X", "S", "T", "C", "S", "X",
        ],
        vec!["P 2", "C", "S", "T", "P 1", "X", "S", "T"],
        vec!["P 0", "S", "T", "M 0", "X", "S", "T", "C", "P 1", "X", "S"],
    ] {
        let mut queue = NodeQueue::new(2).unwrap();
        assert_eq!(queue.top(), None);
        let mut input = String::from("1 1 queue_ops 3 2\n");
        let mut actual = vec![json!(SUCCESS)];
        for command in commands {
            writeln!(input, "{command}").unwrap();
            let mut tokens = command.split_whitespace();
            let value = match tokens.next().unwrap() {
                "P" => {
                    queue
                        .push(tokens.next().unwrap().parse().unwrap(), &pool)
                        .unwrap();
                    json!([])
                }
                "M" => {
                    queue.modify(tokens.next().unwrap().parse().unwrap(), &pool);
                    json!([])
                }
                "X" => json!([queue.pop(&pool).unwrap_or(0)]),
                "T" => json!([queue.top().unwrap()]),
                "S" => json!([
                    queue.capacity(),
                    u8::from(queue.is_empty()),
                    queue.top().unwrap()
                ]),
                "C" => {
                    queue.clear();
                    json!([])
                }
                _ => unreachable!(),
            };
            actual.push(value);
        }
        assert_eq!(json!(actual), oracle(input));
    }
    let mut queue = NodeQueue::new(1).unwrap();
    queue.push(1, &pool).unwrap();
    queue.push(2, &pool).unwrap();
    assert!(queue.push(3, &pool).is_err()); // Beyond the allocated native spare slot.
    assert!(pool.node_at_index(0).is_none());
    assert!(pool.node_at_index(4).is_none());
    assert!(pool.node_at_index_mut(4).is_none());
}
