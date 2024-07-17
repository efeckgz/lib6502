use crate::memory::Memory;

use super::CPU;

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

impl<M: Memory + 'static> CPU<M> {
    pub fn decode_operand(&mut self, mode: AddressingMode) -> Option<u16> {
        // println!("PC before decode: {0:#04x}", self.pc);
        match mode {
            AddressingMode::Accumulator => None, // The accumulator is the operand.
            AddressingMode::Immediate => {
                let addr = self.pc;
                self.pc += 1;
                Some(addr)
            }
            AddressingMode::Absolute => {
                let lo = self.read_and_inc_pc() as u16;
                let hi = self.read_and_inc_pc() as u16;

                Some((hi << 8) | lo)
            }
            AddressingMode::ZeroPage => {
                // The next byte in memory. Casting to u16 makes the hi part 0.
                let lo = self.read_and_inc_pc() as u16;
                Some(lo)
            }
            AddressingMode::ZeroPageX => {
                let lo = self.read_and_inc_pc();
                let addr = lo.wrapping_add(self.x);
                Some(addr as u16)
            }
            AddressingMode::ZeroPageY => {
                let lo = self.read_and_inc_pc();
                let addr = lo.wrapping_add(self.y);
                Some(addr as u16)
            }
            AddressingMode::AbsoluteX => {
                let lo = self.read_and_inc_pc() as u16;
                let hi = self.read_and_inc_pc() as u16;
                let absolute = (hi << 8) | lo;

                // Maybe use overflowing_add and if did_overflow pc++
                Some(absolute.wrapping_add(self.x as u16))
            }
            AddressingMode::AbsoluteY => {
                let lo = self.read_and_inc_pc() as u16;
                let hi = self.read_and_inc_pc() as u16;
                let absolute = (hi << 8) | lo;
                Some(absolute.wrapping_add(self.y as u16))
            }
            AddressingMode::Implied => None, // The address is implied by the instruction. No operand.
            AddressingMode::Relative => {
                // This addressing mode is used by branching instructions. The next byte after the operand is read
                // and is stored as an offset. The offest is a signed 8 bit integer (i8) to be added to the program
                // counter for branching. As such, nothing will return here.
                None
            }
            AddressingMode::IndirectX => {
                let second_byte = self.read_and_inc_pc();
                let plus_x = second_byte.wrapping_add(self.x);
                let lo = self.memory.read_byte(plus_x as u16) as u16;
                let hi = self.memory.read_byte((plus_x.wrapping_add(1)) as u16) as u16;
                let addr = (hi << 8) | lo;
                Some(addr)
            }
            AddressingMode::IndirectY => {
                // Construct the zero page address given in the next two bytes of the instruction.
                let zero_page_lo = self.read_and_inc_pc() as u16;
                let zero_page_hi = self.read_and_inc_pc() as u16;
                let zero_page_addr = (zero_page_hi << 8) | zero_page_lo;

                // add the value of register y to it.
                let plus_y = zero_page_addr + self.y as u16;
                Some(plus_y)
            }
        }
    }
}
