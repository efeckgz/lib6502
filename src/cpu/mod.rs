use crate::memory::Memory;

#[derive(Clone)]
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

pub const CARRY: u8 = 0;
pub const ZERO: u8 = 1;
pub const INTERRUPT_DISABLE: u8 = 2;
pub const DECIMAL: u8 = 3;
pub const OVERFLOW: u8 = 6;
pub const NEGATIVE: u8 = 7;

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
        let mut start = self.pc;
        for byte in program {
            self.memory.write(start, *byte);
            start += 0x01;
        }
    }

    // flag_raised returns the raised status of the given flag.
    pub fn flag_raised(&self, flag: u8) -> bool {
        (self.flag & (1 << flag)) != 0
    }

    // set_flag sets or clears the given flag according to the condition.
    fn set_flag(&mut self, flag: u8, condition: bool) {
        if condition {
            self.flag |= 1 << flag; // set the flag
        } else {
            self.flag &= !(1 << flag); // clear the flag
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

            _ => todo!("opcode {opcode}, pc {0}", self.pc),
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
        }
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

        let (mut result, mut did_overflow) = a.overflowing_add(to_add);
        if self.flag_raised(CARRY) {
            (result, did_overflow) = result.overflowing_add(1); // if the carry flag is set add it.
        }

        self.a = result;

        self.set_flag(CARRY, did_overflow);
        self.set_flag(ZERO, self.a == 0);
        self.set_flag(
            OVERFLOW,
            ((self.a ^ result) & (to_add ^ result) & 0x80) != 0,
        );
        self.set_flag(NEGATIVE, (self.a as i8) < 0);
    }

    fn and(&mut self, operand: u16) {
        let val = self.memory.read(operand);
        let a = self.a;
        let result = val & a;
        self.a = result;

        self.set_flag(NEGATIVE, (result as i8) < 0);
        self.set_flag(ZERO, result == 0);
    }
}
