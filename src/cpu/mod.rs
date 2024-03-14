pub mod addressing_modes;

use crate::memory::Memory;
use addressing_modes::AddressingMode;

pub enum FlagBitPos {
    Carry = 0,
    Zero = 1,
    InterrputDisable = 2,
    Decimal = 3,
    Overflow = 6,
    Negative = 7,
}

type InstructionExecuter<M> = fn(&mut CPU<M>, u16);

pub struct CPU<M: Memory> {
    pub a: u8,
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
            pc: 0x600,
            sp: 0,
            flag: 0,
            memory,
        }
    }

    pub fn load_program(&mut self, program: &[u8]) {
        for (offset, &byte) in program.iter().enumerate() {
            let address = self.pc + offset as u16;
            self.memory.write(address, byte);
        }
    }

    pub fn reset(&mut self) {
        self.memory.reset();
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.pc = 0x600;
        self.sp = 0;
        self.flag = 0;
    }

    // flag_raised returns the raised status of the given flag.
    pub fn flag_raised(&self, pos: FlagBitPos) -> bool {
        (self.flag & (1 << pos as u8)) != 0
    }

    // set_flag sets or clears the given flag according to the condition.
    fn set_flag(&mut self, pos: FlagBitPos, condition: bool) {
        if condition {
            self.flag |= 1 << pos as u8; // set the flag
        } else {
            self.flag &= !(1 << pos as u8); // clear the flag
        }
    }

    fn read_and_inc_pc(&mut self) -> u8 {
        let val = self.memory.read(self.pc);
        self.pc += 1;
        val
    }

    pub fn fetch(&mut self) -> u8 {
        let opcode = self.read_and_inc_pc();
        opcode
    }

    pub fn decode(&self, opcode: u8) -> (InstructionExecuter<M>, AddressingMode) {
        match opcode {
            // ADC
            0x69 => (CPU::adc, AddressingMode::Immediate),
            0x65 => (CPU::adc, AddressingMode::ZeroPage),
            0x75 => (CPU::adc, AddressingMode::ZeroPageX),
            0x6D => (CPU::adc, AddressingMode::Absolute),
            0x7D => (CPU::adc, AddressingMode::AbsoluteX),
            0x79 => (CPU::adc, AddressingMode::AbsoluteY),
            0x61 => (CPU::adc, AddressingMode::IndirectX),
            0x71 => (CPU::adc, AddressingMode::IndirectY),

            // AND
            0x29 => (CPU::and, AddressingMode::Immediate),
            0x25 => (CPU::and, AddressingMode::ZeroPage),
            0x35 => (CPU::and, AddressingMode::ZeroPageX),
            0x2D => (CPU::and, AddressingMode::Absolute),
            0x3D => (CPU::and, AddressingMode::AbsoluteX),
            0x39 => (CPU::and, AddressingMode::AbsoluteY),
            0x21 => (CPU::and, AddressingMode::IndirectX),
            0x31 => (CPU::and, AddressingMode::IndirectY),

            _ => todo!("opcode {opcode:#04x}, pc {0:#04x}", self.pc),
        }
    }

    pub fn execute(&mut self, operation: InstructionExecuter<M>, mode: AddressingMode) {
        let operand = self.decode_operand(mode);
        println!("PC after decode: {0:#04x}", self.pc);
        operation(self, operand.unwrap()); // Unwrap is safe to use here because in cases where operand is None there is no need for it anyway.
    }

    pub fn run_for(&mut self, instructions: i32) {
        let mut executed = 0;
        while executed < instructions {
            let opcode = self.fetch();
            let (executer, mode) = self.decode(opcode);
            self.execute(executer, mode);
            executed += 1;
        }
    }

    pub fn adc(&mut self, operand: u16) {
        let a = self.a;
        let to_add = self.memory.read(operand);
        println!("operand: {operand:#04x}");
        println!("to add: {to_add:#04x}");

        let (mut result, mut did_overflow) = a.overflowing_add(to_add);
        if self.flag_raised(FlagBitPos::Carry) {
            (result, did_overflow) = result.overflowing_add(1); // if the carry flag is set add it.
        }

        self.a = result;

        self.set_flag(FlagBitPos::Carry, did_overflow);
        self.set_flag(FlagBitPos::Zero, self.a == 0);
        self.set_flag(
            FlagBitPos::Overflow,
            ((self.a ^ result) & (to_add ^ result) & 0x80) != 0,
        );
        self.set_flag(FlagBitPos::Negative, (self.a as i8) < 0);
    }

    fn and(&mut self, operand: u16) {
        let val = self.memory.read(operand);
        let a = self.a;
        let result = val & a;
        self.a = result;

        self.set_flag(FlagBitPos::Negative, (result as i8) < 0);
        self.set_flag(FlagBitPos::Zero, result == 0);
    }
}
