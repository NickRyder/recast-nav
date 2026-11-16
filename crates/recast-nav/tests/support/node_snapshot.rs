use recast_nav::nodes::NodePool;
use serde_json::{Value, json};

pub fn snapshot(pool: &NodePool) -> Value {
    let first: Vec<_> = (0..pool.hash_size())
        .map(|i| pool.first(i).unwrap())
        .collect();
    let next: Vec<_> = (0..pool.max_nodes())
        .map(|i| pool.next(i).unwrap())
        .collect();
    let nodes: Vec<_> = pool
        .nodes()
        .iter()
        .map(|n| {
            json!([
                n.reference,
                n.parent_index(),
                n.state(),
                n.flags(),
                n.position[0].to_bits(),
                n.position[1].to_bits(),
                n.position[2].to_bits(),
                n.cost.to_bits(),
                n.total.to_bits()
            ])
        })
        .collect();
    json!([
        pool.node_count(),
        pool.max_nodes(),
        pool.hash_size(),
        first,
        next,
        nodes
    ])
}
