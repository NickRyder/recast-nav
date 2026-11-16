//! Exact snapshots with an explicit, field-local exception for arithmetic QNaNs.
use serde_json::Value;

pub fn difference(a: &Value, b: &Value, arithmetic: impl Fn(&[usize]) -> bool) -> Option<String> {
    fn quiet_nan(value: &Value) -> bool {
        value
            .as_u64()
            .and_then(|v| u32::try_from(v).ok())
            .is_some_and(|bits| bits & 0x7fc0_0000 == 0x7fc0_0000)
    }
    fn visit(
        a: &Value,
        b: &Value,
        path: &mut Vec<usize>,
        arithmetic: &impl Fn(&[usize]) -> bool,
    ) -> Option<String> {
        if a == b {
            return None;
        }
        if let (Value::Array(a), Value::Array(b)) = (a, b) {
            if a.len() != b.len() {
                return Some(format!("{path:?}: lengths {} / {}", a.len(), b.len()));
            }
            for (i, (a, b)) in a.iter().zip(b).enumerate() {
                path.push(i);
                let result = visit(a, b, path, arithmetic);
                path.pop();
                if result.is_some() {
                    return result;
                }
            }
            return None;
        }
        if arithmetic(path) && quiet_nan(a) && quiet_nan(b) {
            return None;
        }
        Some(format!("{path:?}: Rust {a}, native {b}"))
    }
    visit(a, b, &mut Vec::new(), &arithmetic)
}

// Separate integration-test binaries use different subsets of this support API.
#[allow(dead_code)]
#[track_caller]
pub fn assert_eq(a: &Value, b: &Value, arithmetic: impl Fn(&[usize]) -> bool, context: &str) {
    if let Some(message) = difference(a, b, arithmetic) {
        panic!("{context}: {message}");
    }
}
