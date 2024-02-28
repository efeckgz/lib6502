use crate::memory::{self, Memory};

pub enum AddressingMode {
    Accumulator,
    Immediate,
    Implied,
    Relative,
    Absolute,
    ZeroPage,
    AbsoluteX,
    AbsoluteY,
    ZeroPageX,
    ZeroPageY,
    IndirectX,
    IndirectY,
}

const CARRY: u8 = 0;
const ZERO: u8 = 1;
const INTERRUPT_DISABLE: u8 = 2;
const DECIMAL: u8 = 3;
const OVERFLOW: u8 = 6;
const NEGATIVE: u8 = 7;

type InstructionExecuter<M: Memory> = fn(&mut CPU<M>, u16);

pub struct CPU<M: Memory> {
    a: u8,
    x: u8,
    y: u8,
    pub pc: u16,
    sp: u8,
    flag: u8,

    memory: M,
}

impl<M: Memory + 'static> CPU<M> {
    pub fn new(memory: M) -> Self {
        Self {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            sp: 0,
            flag: 0,
            memory,
        }
    }

    // flag_raised returns the raised status of the given flag.
    pub fn flag_raised(&self, flag: u8) -> bool {
        ((self.flag >> flag) & 1) == 1
    }

    // raise_flag raises the given flag if the given condition is satisfied.
    fn raise_flag(&mut self, flag: u8, condition: bool) {
        if condition {
            self.flag = self.flag | (1 << flag);
        }
    }

    fn read_and_inc_pc(&mut self) -> u8 {
        let val = self.memory.read(self.pc);
        self.pc += 1;
        val
    }

    pub fn fetch(&mut self) -> u8 {
        let opcode = self.memory.read(self.pc);
        self.pc += 1;
        opcode
    }

    pub fn decode(&self, opcode: u8) -> (InstructionExecuter<M>, AddressingMode) {
        match opcode {
            0x69 => (CPU::adc, AddressingMode::Immediate),
            0x65 => (CPU::adc, AddressingMode::ZeroPage),
            0x75 => (CPU::adc, AddressingMode::ZeroPageX),
            0x6D => (CPU::adc, AddressingMode::Absolute),
            0x7D => (CPU::adc, AddressingMode::AbsoluteX),
            0x79 => (CPU::adc, AddressingMode::AbsoluteY),
            0x61 => (CPU::adc, AddressingMode::IndirectX),
            0x71 => (CPU::adc, AddressingMode::IndirectY),

            _ => todo!("Unimplemented opcode: {opcode}"),
        }
    }

    pub fn execute(&mut self, operation: InstructionExecuter<M>, mode: AddressingMode) {
        let operand = self.decode_operand(mode);
        operation(self, operand.unwrap()); // Unwrap is safe to use here because in cases where operand is None there is no need for it anyway.
    }

    pub fn decode_operand(&mut self, mode: AddressingMode) -> Option<u16> {
        match mode {
            AddressingMode::Accumulator => Some(self.a as u16),
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
                let lo = self.read_and_inc_pc() as u16; // The next byte in memory. Casting to u16 makes the hi part 0.
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
                let lo = self.memory.read(plus_x as u16) as u16;
                let hi = self.memory.read((plus_x.wrapping_add(1)) as u16) as u16;
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

            _ => panic!("Unrecognized addressing mode!"),
        }
    }

    pub fn adc(&mut self, operand: u16) {
        let a = self.a;
        let to_add = self.memory.read(operand);
        let (result, did_overflow) = a.overflowing_add(to_add);
        self.a = result;

        self.raise_flag(CARRY, did_overflow);
        self.raise_flag(NEGATIVE, result < 0)
    }
}
