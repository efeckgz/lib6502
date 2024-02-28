use std::ops::Add;

use crate::memory::Memory;

pub enum AddressingMode {
    Accumulator,
    Immediate,
    Implied,
    Relative,
    Absolute,
    ZeroPage,
    Indirect,
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

    pub fn decode(&self, opcode: u8) -> (fn(&mut CPU<M>), AddressingMode) {
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

    pub fn execute(&mut self, operation: fn(&mut CPU<M>), mode: AddressingMode) {
        let operand = self.decode_operand(mode);
        operation(self);
    }

    pub fn decode_operand(&mut self, mode: AddressingMode) -> u16 {
        match mode {
            AddressingMode::Accumulator => self.a as u16,
            AddressingMode::Immediate => self.pc, // what
            AddressingMode::Absolute => {
                let lo = self.read_and_inc_pc() as u16;
                let hi = self.read_and_inc_pc() as u16;

                (hi << 8) | lo
            }
            AddressingMode::ZeroPage => {
                let lo = self.read_and_inc_pc() as u16; // The next byte in memory. Casting to u16 makes the hi part 0.
                lo
            }
            AddressingMode::ZeroPageX => {
                let lo = self.read_and_inc_pc();
                (lo + self.x) as u16 // Take another look
            }

            _ => panic!("Unrecognized addressing mode!"),
        }
    }

    pub fn adc(&mut self) {
        unimplemented!()
    }
}
