pub mod addressing_modes;

use core::mem::zeroed;

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

type InstructionExecuter<M> = fn(&mut CPU<M>, Option<u16>);

#[derive(Debug)]
pub struct CPU<M: Memory> {
    a: u8,
    x: u8,
    y: u8,
    pc: u16,
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

    pub fn from_pc(pc: u16, memory: M) -> Self {
        Self {
            a: 0,
            x: 0,
            y: 0,
            pc,
            sp: 0,
            flag: 0,
            memory,
        }
    }

    pub fn load_program(&mut self, program: &[u8]) {
        for (offset, &byte) in program.iter().enumerate() {
            let address = self.pc + offset as u16;
            self.memory.write_byte(address, byte);
        }
    }

    // Find a better way to handle pc
    pub fn reset(&mut self, pc: u16) {
        self.memory.reset();
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.pc = pc;
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
        let val = self.memory.read_byte(self.pc);
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

            // ASL
            0x0A => (CPU::asl, AddressingMode::Accumulator),
            0x06 => (CPU::asl, AddressingMode::ZeroPage),
            0x16 => (CPU::asl, AddressingMode::ZeroPageX),
            0x0E => (CPU::asl, AddressingMode::Absolute),
            0x1E => (CPU::asl, AddressingMode::AbsoluteX),

            // LDA
            0xA9 => (CPU::lda, AddressingMode::Immediate),
            0xA5 => (CPU::lda, AddressingMode::ZeroPage),
            0xB5 => (CPU::lda, AddressingMode::ZeroPageX),
            0xAD => (CPU::lda, AddressingMode::Absolute),
            0xBD => (CPU::lda, AddressingMode::AbsoluteX),
            0xB9 => (CPU::lda, AddressingMode::AbsoluteY),
            0xA1 => (CPU::lda, AddressingMode::IndirectX),
            0xB1 => (CPU::lda, AddressingMode::IndirectY),

            // LDX
            0xA2 => (CPU::ldx, AddressingMode::Immediate),
            0xA6 => (CPU::ldx, AddressingMode::ZeroPage),
            0xB6 => (CPU::ldx, AddressingMode::ZeroPageY),
            0xAE => (CPU::ldx, AddressingMode::Absolute),
            0xBE => (CPU::ldx, AddressingMode::AbsoluteY),

            // LDY
            0xA0 => (CPU::ldy, AddressingMode::Immediate),
            0xA4 => (CPU::ldy, AddressingMode::ZeroPage),
            0xB4 => (CPU::ldy, AddressingMode::ZeroPageX),
            0xAC => (CPU::ldy, AddressingMode::Absolute),
            0xBC => (CPU::ldy, AddressingMode::AbsoluteX),

            _ => todo!("opcode {opcode:#04x}, pc {0:#04x}", self.pc),
        }
    }

    pub fn execute(&mut self, operation: InstructionExecuter<M>, mode: AddressingMode) {
        let operand = self.decode_operand(mode);
        // println!("PC after decode: {0:#04x}", self.pc);
        operation(self, operand); // Unwrap is safe to use here because in cases where operand is None there is no need for it anyway.
    }

    pub fn run_for(&mut self, instructions: i32) {
        for _ in 0..instructions {
            let opcode = self.fetch();
            let (executer, mode) = self.decode(opcode);
            self.execute(executer, mode);
        }
    }

    fn adc(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let a = self.a;
            let to_add = self.memory.read_byte(actual_operand);
            // println!("operand: {operand:#04x}");
            // println!("to add: {to_add:#04x}");

            let (mut result, mut did_overflow) = a.overflowing_add(to_add);
            if self.flag_raised(FlagBitPos::Carry) {
                (result, did_overflow) = result.overflowing_add(1); // if the carry flag is set add it.
            }

            self.a = result;

            self.set_flag(FlagBitPos::Carry, did_overflow);
            self.set_flag(FlagBitPos::Zero, self.a == 0);
            // The sign of both inputs is different than the sign of the output.
            self.set_flag(
                FlagBitPos::Overflow,
                ((self.a ^ result) & (to_add ^ result) & 0x80) != 0,
            );
            self.set_flag(FlagBitPos::Negative, (self.a as i8) < 0);
        }
    }

    fn and(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            let a = self.a;
            let result = val & a;
            self.a = result;

            self.set_flag(FlagBitPos::Negative, (result as i8) < 0);
            self.set_flag(FlagBitPos::Zero, result == 0);
        }
    }

    fn asl(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let mut value = self.memory.read_byte(actual_operand);
            let shifted_out = value >> 7; // Bit 7 is shifted out
            value <<= 1;
            self.memory.write_byte(actual_operand, value);

            // Set the flags
            self.set_flag(FlagBitPos::Carry, shifted_out == 1);
            self.set_flag(FlagBitPos::Zero, value == 0);
            self.set_flag(FlagBitPos::Negative, (value as i8) < 0);
        } else {
            // Accumulator addressing
            let shifted_out = self.a >> 7; // Bit 7 is shifted out
            self.a <<= 1; // Left shift the accumulator

            // Set the flags
            self.set_flag(FlagBitPos::Carry, shifted_out == 1);
            self.set_flag(FlagBitPos::Zero, self.a == 0);
            self.set_flag(FlagBitPos::Negative, (self.a as i8) < 0);
        }
    }

    fn lda(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            self.a = val;

            // Set the status flags
            self.set_flag(FlagBitPos::Negative, (val as i8) < 0);
            self.set_flag(FlagBitPos::Zero, val == 0);
        }
    }

    fn ldx(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            self.x = val;

            // Set the status flags
            self.set_flag(FlagBitPos::Negative, (val as i8) < 0);
            self.set_flag(FlagBitPos::Zero, val == 0);
        }
    }

    fn ldy(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            self.y = val;

            // Set the status flags
            self.set_flag(FlagBitPos::Negative, (val as i8) < 0);
            self.set_flag(FlagBitPos::Zero, val == 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug)]
    struct Mem {
        pub bytes: [u8; 4096],
    }

    impl Memory for Mem {
        fn read_byte(&self, addr: u16) -> u8 {
            self.bytes[addr as usize]
        }

        fn write_byte(&mut self, addr: u16, data: u8) {
            self.bytes[addr as usize] = data;
        }

        fn reset(&mut self) {
            self.bytes = [0; 4096];
        }
    }

    impl Mem {
        fn new() -> Mem {
            Mem { bytes: [0; 4096] }
        }
    }

    #[test]
    fn adc_works() {
        let memory = Mem::new();
        let mut cpu = CPU::from_pc(0x000, memory);

        // Immidiate addressing
        let mut program: [u8; 2048] = [0; 2048];
        program[0] = 0x69_u8;
        program[1] = 0xFF_u8;

        cpu.load_program(&program);
        cpu.run_for(1);
        assert_eq!(cpu.a, 0xFF);

        cpu.reset(0x0000);

        // Zero Page addressing
        let zpaddr = 0xFF_u8;
        let mut program: [u8; 2048] = [0; 2048];
        program[0] = 0x65_u8;
        program[1] = zpaddr;
        program[0x00FF] = 0x42;

        cpu.load_program(&program);
        cpu.run_for(1);
        assert_eq!(cpu.a, 0x42);
        assert!(!cpu.flag_raised(FlagBitPos::Zero));
        assert!(!cpu.flag_raised(FlagBitPos::Negative));

        cpu.reset(0x0000);

        // Zero Page X addressing
        let zpaddr = 0xC0_u8; // Base address given.
        let mut program: [u8; 2048] = [0; 2048];

        // Load an initial value into X register
        program[0] = 0xA2_u8;
        program[1] = 0x60_u8; // Computed address should be (0x60 + 0xC0) % 0xFF = 0x20

        // Test the instruction
        program[2] = 0x75_u8;
        program[3] = zpaddr;
        program[0x20] = 0x42;

        cpu.load_program(&program);
        cpu.run_for(2);

        assert_eq!(cpu.a, 0x42);
        assert!(!cpu.flag_raised(FlagBitPos::Zero));
        assert!(!cpu.flag_raised(FlagBitPos::Negative));

        cpu.reset(0x0000);

        // Absolute addressing
        let lo = 0x10_u8;
        let hi = 0x06_u8;

        let mut program: [u8; 2048] = [0; 2048];
        program[0] = 0x6D_u8;
        program[1] = lo;
        program[2] = hi;
        program[0x610] = 0x19_u8;

        cpu.load_program(&program);
        cpu.run_for(1);
        assert_eq!(cpu.a, 0x19);

        cpu.reset(0x0000);

        // Absolute X addressing mode
        // Base address will be 0x0610
        let lo = 0x10_u8;
        let hi = 0x06_u8;
        let init_x = 0x15_u8; // Initial X register value to use as offset

        let mut program: [u8; 2048] = [0; 2048];
        // Load an initial value into X register
        program[0] = 0xA2_u8;
        program[1] = init_x; // Computed address will be 0x0625

        // Test the instruction
        program[2] = 0x7D_u8;
        program[3] = lo;
        program[4] = hi;
        program[0x0625] = 0x42_u8;

        cpu.load_program(&program);
        cpu.run_for(2);

        assert_eq!(cpu.a, 0x42_u8);

        cpu.reset(0x0000);

        // Absolute Y addressing
        // Base address will be 0x0610
        let lo = 0x10_u8;
        let hi = 0x06_u8;
        let init_y = 0x15_u8; // Initial Y register value to use as offset

        let mut program: [u8; 2048] = [0; 2048];
        // Load an initial value into X register
        program[0] = 0xA0_u8;
        program[1] = init_y; // Computed address will be 0x0625

        // Test the instruction
        program[2] = 0x79_u8;
        program[3] = lo;
        program[4] = hi;
        program[0x0625] = 0x42_u8;

        cpu.load_program(&program);
        cpu.run_for(2);

        assert_eq!(cpu.a, 0x42_u8);

        cpu.reset(0x0000);

        // (Indirect, X) addresing
    }

    #[test]
    fn and_works() {
        let memory = Mem::new();
        let mut cpu = CPU::new(memory);

        // A test value to load to the a register.
        // lda with test_a, then and a with test_val
        let test_a: u8 = 0x15;
        let test_val: u8 = 0x01; // we will and accumulator with this value.

        let program: [u8; 4] = [0xA9, test_a, 0x29, test_val];
        cpu.load_program(&program);

        cpu.run_for(2);
        assert_eq!(cpu.a, test_a & test_val);
    }

    #[test]
    fn lda_works() {
        let memory = Mem::new();
        let mut cpu = CPU::new(memory);

        let mut program = [0; 2048];
        program[0] = 0xA9_u8;
        program[1] = 0x00_u8;

        // program[0] = 0xA5_u8; // at 0x600 - zero page addressing
        // program[1] = 0xFA_u8; // at 0x601
        // program[0x00FA - 0x600] = 0x15;

        cpu.load_program(&program);
        cpu.run_for(1);

        assert_eq!(cpu.a, 0x00);
        assert!(!cpu.flag_raised(FlagBitPos::Negative));
        assert!(cpu.flag_raised(FlagBitPos::Zero));
    }

    #[test]
    fn ldx_works() {
        let memory = Mem::new();
        let mut cpu = CPU::new(memory);

        let mut program = [0; 2048];
        program[0] = 0xA2_u8;
        program[1] = 0x24_u8;

        cpu.load_program(&program);
        cpu.run_for(1);

        assert_eq!(cpu.x, 0x24);
        assert!(!cpu.flag_raised(FlagBitPos::Negative));
        assert!(!cpu.flag_raised(FlagBitPos::Zero));
    }

    #[test]
    fn ldy_works() {
        let memory = Mem::new();
        let mut cpu = CPU::new(memory);

        let mut program = [0; 2048];
        program[0] = 0xA0_u8;
        program[1] = 0x24_u8;

        cpu.load_program(&program);
        cpu.run_for(1);

        assert_eq!(cpu.y, 0x24);
        assert!(!cpu.flag_raised(FlagBitPos::Negative));
        assert!(!cpu.flag_raised(FlagBitPos::Zero));
    }
}
