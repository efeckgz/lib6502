pub mod addressing_modes;

use crate::memory::Memory;
use addressing_modes::AddressingMode;

pub enum Flags {
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
    pub fn flag_raised(&self, pos: Flags) -> bool {
        (self.flag & (1 << pos as u8)) != 0
    }

    // set_flag sets or clears the given flag according to the condition.
    fn set_flag(&mut self, pos: Flags, condition: bool) {
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

            // BCC
            0x90 => (CPU::bcc, AddressingMode::Relative),

            // BCS
            0xB0 => (CPU::bcs, AddressingMode::Relative),

            // BEQ
            0xF0 => (CPU::beq, AddressingMode::Relative),

            // BIT
            0x24 => (CPU::bit, AddressingMode::ZeroPage),
            0x2C => (CPU::bit, AddressingMode::Absolute),

            // BMI
            0x30 => (CPU::bmi, AddressingMode::Relative),

            // BNE
            0xD0 => (CPU::bne, AddressingMode::Relative),

            // BPL
            0x10 => (CPU::bpl, AddressingMode::Relative),

            // BRK
            0x00 => (CPU::brk, AddressingMode::Implied),

            // BVC
            0x050 => (CPU::bvc, AddressingMode::Relative),

            // BVS
            0x70 => (CPU::bvs, AddressingMode::Relative),

            // CLC
            0x18 => (CPU::clc, AddressingMode::Implied),

            // CLD
            0xD8 => (CPU::cld, AddressingMode::Implied),

            // CLI
            0x58 => (CPU::cli, AddressingMode::Implied),

            // CLV
            0xB8 => (CPU::clv, AddressingMode::Implied),

            // CMP
            0xC9 => (CPU::cmp, AddressingMode::Immediate),
            0xC5 => (CPU::cmp, AddressingMode::ZeroPage),
            0xD5 => (CPU::cmp, AddressingMode::ZeroPageX),
            0xCD => (CPU::cmp, AddressingMode::Absolute),
            0xDD => (CPU::cmp, AddressingMode::AbsoluteX),
            0xD9 => (CPU::cmp, AddressingMode::AbsoluteY),
            0xC1 => (CPU::cmp, AddressingMode::IndirectX),
            0xD1 => (CPU::cmp, AddressingMode::IndirectY),

            // CPX
            0xE0 => (CPU::cpx, AddressingMode::Immediate),
            0xE4 => (CPU::cpx, AddressingMode::ZeroPage),
            0xEC => (CPU::cpx, AddressingMode::Absolute),

            // CPY
            0xC0 => (CPU::cpy, AddressingMode::Immediate),
            0xC4 => (CPU::cpy, AddressingMode::ZeroPage),
            0xCC => (CPU::cpy, AddressingMode::Absolute),

            // DEC
            0xC6 => (CPU::dec, AddressingMode::ZeroPage),
            0xD6 => (CPU::dec, AddressingMode::ZeroPageX),
            0xCE => (CPU::dec, AddressingMode::Absolute),
            0xDE => (CPU::dec, AddressingMode::AbsoluteX),

            // DEX
            0xCA => (CPU::dex, AddressingMode::Implied),

            // DEY
            0x88 => (CPU::dey, AddressingMode::Implied),

            // EOR
            0x49 => (CPU::eor, AddressingMode::Immediate),
            0x45 => (CPU::eor, AddressingMode::ZeroPage),
            0x55 => (CPU::eor, AddressingMode::ZeroPageX),
            0x4D => (CPU::eor, AddressingMode::Absolute),
            0x5D => (CPU::eor, AddressingMode::AbsoluteX),
            0x59 => (CPU::eor, AddressingMode::AbsoluteY),
            0x41 => (CPU::eor, AddressingMode::IndirectX),
            0x51 => (CPU::eor, AddressingMode::IndirectY),

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

            let (mut result, mut did_overflow) = a.overflowing_add(to_add);
            if self.flag_raised(Flags::Carry) {
                (result, did_overflow) = result.overflowing_add(1); // if the carry flag is set add it.
            }

            self.a = result;

            self.set_flag(Flags::Carry, did_overflow);
            self.set_flag(Flags::Zero, self.a == 0);
            // The sign of both inputs is different than the sign of the output.
            self.set_flag(
                Flags::Overflow,
                ((self.a ^ result) & (to_add ^ result) & 0x80) != 0,
            );
            self.set_flag(Flags::Negative, (self.a as i8) < 0);
        }
    }

    fn and(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            let a = self.a;
            let result = val & a;
            self.a = result;

            self.set_flag(Flags::Negative, (result as i8) < 0);
            self.set_flag(Flags::Zero, result == 0);
        }
    }

    fn asl(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let mut value = self.memory.read_byte(actual_operand);
            let shifted_out = value >> 7; // Bit 7 is shifted out
            value <<= 1;
            self.memory.write_byte(actual_operand, value);

            // Set the flags
            self.set_flag(Flags::Carry, shifted_out == 1);
            self.set_flag(Flags::Zero, value == 0);
            self.set_flag(Flags::Negative, (value as i8) < 0);
        } else {
            // Accumulator addressing
            let shifted_out = self.a >> 7; // Bit 7 is shifted out
            self.a <<= 1; // Left shift the accumulator

            // Set the flags
            self.set_flag(Flags::Carry, shifted_out == 1);
            self.set_flag(Flags::Zero, self.a == 0);
            self.set_flag(Flags::Negative, (self.a as i8) < 0);
        }
    }

    // branch_general handles the general branching logic.
    // It reads the memory at program counter to get the signed branching offset and
    // branches to the address pc + offset. This is only called within conditional branching functions.
    fn branch_general(&mut self) {
        // Cast the offset and pc to signed type to handle backward branching
        let offset = self.read_and_inc_pc() as i8;
        let pc = self.pc as i16;

        let branch_to = pc.wrapping_add(offset as i16);
        self.pc = branch_to as u16;
    }

    // Second argument is passed in as per self.decode return type requirement and is unnecessary here.
    fn bcc(&mut self, _: Option<u16>) {
        if !self.flag_raised(Flags::Carry) {
            self.branch_general();
        }
    }

    // Second argument is passed in as per self.decode return type requirement and is unnecessary here.
    fn bcs(&mut self, _: Option<u16>) {
        if self.flag_raised(Flags::Carry) {
            self.branch_general();
        }
    }

    // Second argument is passed in as per self.decode return type requirement and is unnecessary here.
    fn beq(&mut self, _: Option<u16>) {
        if self.flag_raised(Flags::Zero) {
            self.branch_general();
        }
    }

    fn bit(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let mask = self.a;
            let value = self.memory.read_byte(actual_operand);
            let result = value & mask;

            let bit_6 = (value & (1 << 6)) >> 6;

            // set the flags
            self.set_flag(Flags::Zero, result == 0);
            self.set_flag(Flags::Overflow, bit_6 == 1);
            self.set_flag(Flags::Negative, (value as i8) < 0);
        }
    }

    fn bmi(&mut self, _: Option<u16>) {
        if self.flag_raised(Flags::Negative) {
            self.branch_general();
        }
    }

    fn bne(&mut self, _: Option<u16>) {
        if !self.flag_raised(Flags::Zero) {
            self.branch_general();
        }
    }

    fn bpl(&mut self, _: Option<u16>) {
        if !self.flag_raised(Flags::Negative) {
            self.branch_general();
        }
    }

    fn brk(&mut self, _: Option<u16>) {
        // brk instruction logic
        // This sets a Break Command flag in the cpu that is currently not implemented in this emulator.
        todo!("Implement the BRK instruction.")
    }

    fn bvc(&mut self, _: Option<u16>) {
        if !self.flag_raised(Flags::Overflow) {
            self.branch_general();
        }
    }

    fn bvs(&mut self, _: Option<u16>) {
        if self.flag_raised(Flags::Overflow) {
            self.branch_general();
        }
    }

    fn clc(&mut self, _: Option<u16>) {
        self.set_flag(Flags::Carry, false); // Explicitly clear the carry flag
    }

    fn cld(&mut self, _: Option<u16>) {
        self.set_flag(Flags::Decimal, false); // Explicitly clear the decimal flag
    }

    fn cli(&mut self, _: Option<u16>) {
        self.set_flag(Flags::InterrputDisable, false); // Explicitly clear the interrupt disable flag
    }

    fn clv(&mut self, _: Option<u16>) {
        self.set_flag(Flags::Overflow, false); // Explicitly clear the overflow flag
    }

    fn cmp(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            let (result, _) = self.a.overflowing_sub(val);

            self.set_flag(Flags::Carry, self.a >= val);
            self.set_flag(Flags::Zero, self.a == val);
            self.set_flag(Flags::Negative, (result as i8) < 0);
        }
    }

    fn cpx(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            let (result, _) = self.x.overflowing_sub(val);

            self.set_flag(Flags::Carry, self.x >= val);
            self.set_flag(Flags::Zero, self.x == val);
            self.set_flag(Flags::Negative, (result as i8) < 0);
        }
    }

    fn cpy(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            let (result, _) = self.y.overflowing_sub(val);

            self.set_flag(Flags::Carry, self.y >= val);
            self.set_flag(Flags::Zero, self.y == val);
            self.set_flag(Flags::Negative, (result as i8) < 0);
        }
    }

    fn dec(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            let result = val - 1;
            self.memory.write_byte(actual_operand, result); // Write the result back to the same mem address

            // Negative and Zero flags set accordingly
            self.set_flag(Flags::Zero, result == 0);
            self.set_flag(Flags::Negative, (result as i8) < 0);
        }
    }

    fn dex(&mut self, _: Option<u16>) {
        self.x -= 1;

        // Set flags
        self.set_flag(Flags::Zero, self.x == 0);
        self.set_flag(Flags::Negative, (self.x as i8) < 0);
    }

    fn dey(&mut self, _: Option<u16>) {
        self.y -= 1;

        // Set flags
        self.set_flag(Flags::Zero, self.y == 0);
        self.set_flag(Flags::Negative, (self.y as i8) < 0);
    }

    fn eor(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            let result = self.a ^ val;
            self.a = result;

            // Set the flags
            self.set_flag(Flags::Zero, self.a == 0);
            self.set_flag(Flags::Negative, (self.a as i8) < 0);
        }
    }

    fn lda(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            self.a = val;

            // Set the status flags
            self.set_flag(Flags::Negative, (val as i8) < 0);
            self.set_flag(Flags::Zero, val == 0);
        }
    }

    fn ldx(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            self.x = val;

            // Set the status flags
            self.set_flag(Flags::Negative, (val as i8) < 0);
            self.set_flag(Flags::Zero, val == 0);
        }
    }

    fn ldy(&mut self, operand: Option<u16>) {
        if let Some(actual_operand) = operand {
            let val = self.memory.read_byte(actual_operand);
            self.y = val;

            // Set the status flags
            self.set_flag(Flags::Negative, (val as i8) < 0);
            self.set_flag(Flags::Zero, val == 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MEMSIZE: usize = 8192;

    #[derive(Debug)]
    struct Mem {
        pub bytes: [u8; MEMSIZE],
    }

    impl Memory for Mem {
        fn read_byte(&self, addr: u16) -> u8 {
            self.bytes[addr as usize]
        }

        fn write_byte(&mut self, addr: u16, data: u8) {
            self.bytes[addr as usize] = data;
        }

        fn reset(&mut self) {
            self.bytes = [0; MEMSIZE];
        }
    }

    impl Mem {
        fn new() -> Mem {
            Mem { bytes: [0; 8192] }
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
        assert!(!cpu.flag_raised(Flags::Zero));
        assert!(!cpu.flag_raised(Flags::Negative));

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
        assert!(!cpu.flag_raised(Flags::Zero));
        assert!(!cpu.flag_raised(Flags::Negative));

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
        assert!(!cpu.flag_raised(Flags::Negative));
        assert!(cpu.flag_raised(Flags::Zero));
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
        assert!(!cpu.flag_raised(Flags::Negative));
        assert!(!cpu.flag_raised(Flags::Zero));
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
        assert!(!cpu.flag_raised(Flags::Negative));
        assert!(!cpu.flag_raised(Flags::Zero));
    }

    #[test]
    fn asl_works() {
        let memory = Mem::new();
        let mut cpu = CPU::new(memory);

        let mut program = [0; 2048];

        // Load the accumulator with some value to left shift
        // Load 0x15, when left shifted should be 0x2A.
        program[0] = 0xA9_u8;
        program[1] = 0x15_u8;

        // Left shift test
        program[2] = 0x0A_u8;

        cpu.load_program(&program);
        cpu.run_for(2);

        assert_eq!(cpu.a, 0x2A);
        assert!(!cpu.flag_raised(Flags::Carry)); // Carry should not be set
        assert!(!cpu.flag_raised(Flags::Zero)); // Zero should not be set
        assert!(!cpu.flag_raised(Flags::Negative)); // The result is not negative

        let memory = Mem::new();
        let mut cpu = CPU::from_pc(0x0000, memory);
        let mut program = [0; 2048];

        // Full address will be 0x0610
        let lo = 0x10;
        let hi = 0x06;

        program[0] = 0x0E_u8;
        program[1] = lo;
        program[2] = hi;
        program[0x0610] = 0x15_u8; // Left shifted should be 0x2A

        cpu.load_program(&program);
        cpu.run_for(1);

        assert_eq!(cpu.memory.read_byte(0x0610), 0x2A);
    }

    #[test]
    fn bcc_works() {
        let memory = Mem::new();
        let mut cpu = CPU::new(memory);
        cpu.reset(0x0000);
        let mut program = [0; 2048];

        // Do an operation that sets the carry flag
        // Load Accumulator 0xFF, and then add Accumulator 0x01
        program[0] = 0xA9_u8; // LDA
        program[1] = 0xFF_u8; // LDA 0xFF
        program[2] = 0x69_u8; // ADC
        program[3] = 0x01_u8; // ADC 0x01

        // Now do an operation that clears the carry flag
        // Load accumulator 0x01, and then  add Accumulator 0x01
        program[4] = 0xA9_u8; // LDA
        program[5] = 0x01_u8; // LDA 0x01
        program[6] = 0x69_u8; // ADC
        program[7] = 0x01_u8; // ADC 0x01

        // Carry flag is set and cleared, branch to 0x0700
        // Run a simple LDA to verify the branch was successful
        program[8] = 0x90_u8; // BCC
        program[9] = 0x01_u8; // offset 0x01, current pc 0x10, should branch to 0x11
        program[11] = 0xA9_u8; // LDA
        program[12] = 0x42_u8; // LDA 0x42

        // Now branch back to the start of the program.
        program[13] = 0x90_u8; // BCC
        program[14] = (!0xF + 1) as u8; // offset should be -15, current pc 15, should branch to 0x00

        cpu.load_program(&program);
        cpu.run_for(9);

        assert_eq!(cpu.a, 0x00);
    }

    #[test]
    fn bcs_works() {
        let memory = Mem::new();
        let mut cpu = CPU::new(memory);
        cpu.reset(0x0000);
        let mut program = [0; 2048];

        // Do an operation that sets the carry flag
        // Load Accumulator 0xFF, and then add Accumulator 0x01
        program[0] = 0xA9_u8; // LDA
        program[1] = 0xFF_u8; // LDA 0xFF
        program[2] = 0x69_u8; // ADC
        program[3] = 0x01_u8; // ADC 0x01

        // Carry flag is set, branch one step forward
        // Run a simple LDA to verify the branch was successful
        program[4] = 0xB0_u8; // BCS
        program[5] = 0x01_u8; // offset 0x1, current pc 0x06, should branch to 0x07
        program[7] = 0xA9_u8; // LDA
        program[8] = 0x42_u8; // LDA 0x42

        cpu.load_program(&program);
        cpu.run_for(4);

        assert_eq!(cpu.a, 0x42);
        assert!(cpu.flag_raised(Flags::Carry));
    }

    #[test]
    fn beq_works() {
        let memory = Mem::new();
        let mut cpu = CPU::from_pc(0x0000, memory);
        let mut program = [0; 2048];

        // Load Accumulator 0. This will set the 0 flag.
        program[0] = 0xA9_u8; // LDA
        program[1] = 0x00_u8; // LDA 0
        program[2] = 0xF0_u8; // BEQ
        program[3] = 0x01_u8; // offset 0x01, pc at 0x0604, should branch to 0x0605.
        program[5] = 0xA9_u8; // LDA
        program[6] = 0x42_u8; // LDA 0x42

        cpu.load_program(&program);
        cpu.run_for(3);

        assert_eq!(cpu.a, 0x42);
    }

    #[test]
    fn bit_works() {
        let memory = Mem::new();
        let mut cpu = CPU::from_pc(0x0000, memory);
        let mut program = [0; 2048];

        // Load Accumulator 0xAA, 1010 1010 in binary.
        // Load memory 0x55, 0101 0101 in binary.
        // The result of the AND will be 0x00, 0000 0000 in binary.
        // The zero flag will be set, overflow flag will be set, negative flag will not be set.
        program[0] = 0xA9_u8; // LDA
        program[1] = 0xAA_u8; // LDA 0xAA
        program[2] = 0x24_u8; // BIT Zero Page
        program[3] = 0xAA_u8; // The operand is in zero page address 0x00AA
        program[170] = 0x55_u8; // Load the memory with 0x55

        cpu.load_program(&program);
        cpu.run_for(2);

        assert!(cpu.flag_raised(Flags::Zero)); // zero flag must be set.
        assert!(cpu.flag_raised(Flags::Overflow)); // overflow flag must be set.
        assert!(!cpu.flag_raised(Flags::Negative)); // negative flag must be not set.

        // Now run the same exact test, with Absolute addressing this time.
        cpu.reset(0x0000);
        program = [0; 2048];

        program[0] = 0xA9_u8; // LDA
        program[1] = 0xAA_u8; // LDA 0xAA
        program[2] = 0x2C_u8; // BIT Absolute
        program[3] = 0xBB_u8; // Lo byte 0xBB
        program[4] = 0x06_u8; // Hi byte 0x06, absolute address 0x06BB
        program[1723] = 0x55_u8; // Store 0x55 in the memory address

        cpu.load_program(&program);
        cpu.run_for(2);

        assert!(cpu.flag_raised(Flags::Zero)); // zero flag must be set.
        assert!(cpu.flag_raised(Flags::Overflow)); // overflow flag must be set.
        assert!(!cpu.flag_raised(Flags::Negative)); // negative flag must be not set.
    }

    #[test]
    fn dec_works() {
        let memory = Mem::new();
        let mut cpu = CPU::from_pc(0x0, memory);
        let mut program = [0; 2048];

        // Hard code this into memory and test DEC on that address.
        let test_value: u8 = 42;
        let test_adress: u8 = 0xAA;

        program[0] = 0xC6; // Zero Page DEC
        program[1] = test_adress; // Operand in address 0xAA (170)
        program[test_adress as usize] = test_value;

        cpu.load_program(&program);
        cpu.run_for(1);

        assert!(cpu.memory.bytes[test_adress as usize] == test_value - 1);
        assert!(!cpu.flag_raised(Flags::Zero));
        assert!(!cpu.flag_raised(Flags::Negative));
    }

    #[test]
    fn eor_works() {
        // Test EOR instruction with Absolute X mode.
        let memory = Mem::new();
        let mut cpu = CPU::from_pc(0x0, memory);
        let mut program = [0; MEMSIZE];

        // Starting values for the accumulator and memory address.
        // In the end the accumulator should be 0xFF.
        let orig_a = 0xF0_u8;
        let orig_val = 0x0F_u8;

        // Load x with an initial index.
        program[0] = 0xA2_u8; // Immidiate ldx
        program[1] = 0x11_u8; // Load X 0x11.

        // Load Accumulator with initial value.
        program[2] = 0xA9_u8; // Immidiate lda
        program[3] = orig_a; // Load A 0xF0

        // Absolute address will be 0x1122, with X it will be 0x1133.
        program[4] = 0x5D_u8; // Absolute X addressing eor
        program[5] = 0x22_u8; // Lo byte
        program[6] = 0x11_u8; // Hi byte
        program[0x1133] = orig_val; // Load the original value at the designated memory address.

        cpu.load_program(&program);
        cpu.run_for(3);

        assert_eq!(cpu.a, 0xFF);
        assert!(!cpu.flag_raised(Flags::Zero)); // Set to false
        assert!(cpu.flag_raised(Flags::Negative)); // Set to true
    }
}
