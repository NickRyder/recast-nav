#[path = "support/float_comparison.rs"]
mod float_comparison;
use serde_json::json;

#[test]
fn arithmetic_nan_exception_preserves_exact_snapshot_contracts() {
    let arithmetic = |path: &[usize]| path == [0];
    let a = json!([0x7fc1_2345u32, 0x7fc1_2345u32]);
    let b = json!([0xffc5_4321u32, 0x7fc1_2345u32]);
    float_comparison::assert_eq(&a, &b, arithmetic, "arithmetic QNaN");
    assert!(
        float_comparison::difference(&a, &json!([0xffc5_4321u32, 0xffc5_4321u32]), arithmetic)
            .is_some()
    );
    for (a, b) in [
        (0x3f80_0000u32, 0x3f80_0001u32),
        (0, 0x8000_0000),
        (0x7fc0_0000, 0x7f80_0000),
        (0x7f80_0001, 0x7fc0_0001),
    ] {
        assert!(float_comparison::difference(&json!([a]), &json!([b]), arithmetic).is_some());
    }
    assert!(float_comparison::difference(&json!([0]), &json!([0, 0]), arithmetic).is_some());
}
