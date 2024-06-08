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

    pub fn load_program(&mut self, program: &[u8]) {
        for (offset, &byte) in program.iter().enumerate() {
            let address = self.pc + offset as u16;
            self.memory.write_byte(address, byte);
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
        println!("PC after decode: {0:#04x}", self.pc);
        operation(self, operand.unwrap()); // Unwrap is safe to use here because in cases where operand is None there is no need for it anyway.
    }

    pub fn run_for(&mut self, instructions: i32) {
        for _ in 0..instructions {
            let opcode = self.fetch();
            let (executer, mode) = self.decode(opcode);
            self.execute(executer, mode);
        }
    }

    fn adc(&mut self, operand: u16) {
        let a = self.a;
        let to_add = self.memory.read_byte(operand);
        println!("operand: {operand:#04x}");
        println!("to add: {to_add:#04x}");

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

    fn and(&mut self, operand: u16) {
        let val = self.memory.read_byte(operand);
        let a = self.a;
        let result = val & a;
        self.a = result;

        self.set_flag(FlagBitPos::Negative, (result as i8) < 0);
        self.set_flag(FlagBitPos::Zero, result == 0);
    }

    fn lda(&mut self, operand: u16) {
        let val = self.memory.read_byte(operand);
        self.a = val;

        // Set the status flags
        self.set_flag(FlagBitPos::Negative, (val as i8) < 0);
        self.set_flag(FlagBitPos::Zero, val == 0);
    }

    fn ldx(&mut self, operand: u16) {
        let val = self.memory.read_byte(operand);
        self.x = val;

        // Set the status flags
        self.set_flag(FlagBitPos::Negative, (val as i8) < 0);
        self.set_flag(FlagBitPos::Zero, val == 0);
    }

    fn ldy(&mut self, operand: u16) {
        let val = self.memory.read_byte(operand);
        self.y = val;

        // Set the status flags
        self.set_flag(FlagBitPos::Negative, (val as i8) < 0);
        self.set_flag(FlagBitPos::Zero, val == 0);
    }
}

#[cfg(test)]
mod tests {
    // use self::cpu::CPU;
    use super::*;
    // use crate::cpu::FlagBitPos;
    // use memory::Memory;

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
        let mut cpu = CPU::new(memory);

        let tests = vec![
            (0x69_u8, 0xFF_u8, 0x18_u8),
            (0x69_u8, 0x1A_u8, 0xBD_u8),
            (0x69_u8, 0xFF_u8, 0x01_u8),
        ];

        let mut test_index = 1;
        for (opcode, add_1, add_2) in tests {
            cpu.reset();
            let (result, did_overflow) = add_1.overflowing_add(add_2);
            // let program = vec![opcode, add_1, opcode, add_2];
            let program: [u8; 4] = [opcode, add_1, opcode, add_2];
            cpu.load_program(&program);
            cpu.run_for(2);

            assert_eq!(cpu.a, result);
            assert!(cpu.flag_raised(FlagBitPos::Carry) == did_overflow);
            println!("Test {test_index} passed.");
            test_index += 1;
        }
    }

    #[test]
    fn abs_adc_works() {
        let memory = Mem::new();
        let mut cpu = CPU::new(memory);

        let lo = 0x10_u8;
        let hi = 0x06_u8;
        let addr: u16 = ((hi as u16) << 8) | (lo as u16); // should be 0x610
        println!("addr: {addr:#04x}");

        let mut program: [u8; 2048] = [0; 2048]; // 2k bytes of 0.
        program[0] = 0x6D_u8; // Absolute addressing adc instruction
        program[1] = lo;
        program[2] = hi;
        program[0x10] = 0x19_u8; // memory address 0x610 will have the value 0x19

        cpu.load_program(&program);

        cpu.run_for(1);
        assert_eq!(cpu.a, 0x19);
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
