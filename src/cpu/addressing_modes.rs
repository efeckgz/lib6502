#[derive(Clone)]
pub enum AddressingMode {
    Accumulator,
    Immediate,
    Implied,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Relative,
    IndirectX,
    IndirectY,
}
