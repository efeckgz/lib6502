// Instruction infromation found in this website: http://www.6502.org/tutorials/6502opcodes.html

use super::{addressing_modes::AddressingMode, CPU};
use crate::memory::Memory;

pub struct Instruction<M: Memory> {
    opcode: u8,                         // The opcode of the instruction
    pub mode: AddressingMode,           // The addressing mode used
    bytes: u8,                          // The length of the instruction in memory
    cycles: u8, // The cycles this instruction takes to complete. In some instructions this could increment.
    pub executer: fn(&mut CPU<M>, u16), // The function that executes this instruction
}

impl<M: Memory + 'static> Instruction<M> {
    pub fn new(
        opcode: u8,
        mode: AddressingMode,
        bytes: u8,
        cycles: u8,
        executer: fn(&mut CPU<M>, u16),
    ) -> Self {
        Self {
            opcode,
            mode,
            bytes,
            cycles,
            executer,
        }
    }
}
