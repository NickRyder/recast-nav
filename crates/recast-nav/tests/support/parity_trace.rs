//! Optional full snapshots for investigating native differential failures.
use serde_json::{Value, json};
use std::sync::atomic::{AtomicUsize, Ordering};

pub fn write(label: &str, input: &str, actual: &[Value], expected: &[Value]) {
    let Some(directory) = std::env::var_os("RECAST_PARITY_DIAGNOSTICS_DIR") else {
        return;
    };
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let thread = std::thread::current();
    let name: String = thread
        .name()
        .unwrap_or("test")
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect();
    let path = std::path::PathBuf::from(directory);
    std::fs::create_dir_all(&path).unwrap();
    let path = path.join(format!(
        "{label}-{name}-{}.json",
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let output = std::fs::File::create(path).unwrap();
    serde_json::to_writer(
        output,
        &json!({"label":label,"input":input,"actual":actual,"expected":expected}),
    )
    .unwrap();
}
