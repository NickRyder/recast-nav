//! Mixed operands for differential arithmetic cases, including distinct quiet
//! and signaling NaN payloads. Every ordered triple is visited once.
const VALUES: [f32; 10] = [
    0.,
    -0.,
    1.,
    -1.,
    f32::from_bits(0x7fc1_2345),
    f32::from_bits(0xffc5_4321),
    f32::from_bits(0x7f81_2345),
    f32::from_bits(0xff85_4321),
    f32::INFINITY,
    f32::NEG_INFINITY,
];
pub const CASE_COUNT: usize = VALUES.len().pow(3);

pub fn operands(case: usize) -> [f32; 3] {
    let base = VALUES.len();
    [
        VALUES[case % base],
        VALUES[case / base % base],
        VALUES[case / (base * base)],
    ]
}
