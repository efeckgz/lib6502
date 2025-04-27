mod lookup;

use crate::bus::BusDevice;
use lookup::{LOOKUP, Nmeonic};

pub struct Cpu<'a> {
    // Internal state
    pub a: u8,   // Accumulator
    pub x: u8,   // Index register x
    pub y: u8,   // Index register y
    pub pc: u16, // Program counter
    pub p: u8,   // Status register
    pub s: u8,   // Stack pointer

    // Internal control
    state: State,
    cur_op: u8, // Current opcode being worked on

    // Bus variables
    pub bus: &'a mut dyn BusDevice, // The bus itself
    pub addr: u16,                  // 16 bit address bus value
    pub data: u8,                   // 8 bit data bus value
    pub read: bool,                 // bus read/write mode control variable

    latch_u16: u16, // Latch to hold the 16 bit effective addr
}

#[derive(Copy, Clone)]
pub enum AddressingMode {
    Accumulator,
    Relative,
    Implied,
    Indirect,
    Immediate, // Immidiate
    ZeroPage,  // Zero Page
    ZeroPageX, // Zero Page, X
    ZeroPageY,
    Absolute,  // Absolute
    AbsoluteX, // Absolute, X
    AbsoluteY, // Absolute, Y
    IndirectX, // (Indirect, X)
    IndirectY, // (Indirect), Y
}

// Inner state of the processor, used in state machine.
enum State {
    FetchOpcode, // Fetch opcode state. Every instruction starts here.
    ExecImm,     // Immediate addressing mode execution state
    ExecAcc,     // Accumulator addressing mode execution state
    FetchAbsLo,
    FetchAbsHi,
    ExecAbs,

    // Read-Modify-Write states
    RmwRead,       // Read opcode from effective address
    RmwDummyWrite, // Dummy write the value read to the effective address
    RmwExec,       // Excecute the rmw instruction and write the result back
}

// Status flags. Used in the processor status register p.
pub enum Flags {
    Carry = 0,
    Zero = 1,
    InterrputDisable = 2,
    Decimal = 3,
    Overflow = 6,
    Negative = 7,
}

impl<'a> Cpu<'a> {
    pub fn new(bus: &'a mut dyn BusDevice) -> Self {
        Self {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            p: 0,
            s: 0,
            state: State::FetchOpcode,
            cur_op: 0,
            bus,
            addr: 0,
            data: 0,
            read: false,
            latch_u16: 0,
        }
    }

    // Emulate 1 cpu cycle.
    pub fn cycle(&mut self) {
        match self.state {
            State::FetchOpcode => {
                self.addr = self.pc;
                self.read = true;
                self.access_bus();

                self.pc = self.pc.wrapping_add(1);
                self.cur_op = self.data;
                if let Some(instruction) = LOOKUP[self.data as usize] {
                    match instruction.0 {
                        AddressingMode::Accumulator => self.state = State::ExecAcc,
                        AddressingMode::Immediate => self.state = State::ExecImm,
                        AddressingMode::Absolute => self.state = State::FetchAbsLo,
                        _ => todo!("Implement remaining states"),
                    }
                } else {
                    todo!("Implement illegal opcodes!");
                }
            }
            State::ExecImm => {
                self.addr = self.pc;
                self.read = true;
                self.access_bus();
                self.pc = self.pc.wrapping_add(1);

                if let Some(instruction) = LOOKUP[self.cur_op as usize] {
                    match instruction.1 {
                        Nmeonic::ADC => self.adc(),
                        Nmeonic::AND => self.and(),
                        Nmeonic::CMP => self.cmp(),
                        Nmeonic::CPX => self.cpx(),
                        Nmeonic::CPY => self.cpy(),
                        Nmeonic::EOR => self.eor(),
                        Nmeonic::LDA => self.lda(),
                        Nmeonic::LDX => self.ldx(),
                        Nmeonic::LDY => self.ldy(),
                        Nmeonic::ORA => self.ora(),

                        _ => panic!("Unrecognized opcode-addressing mode-nmeonic combination!"),
                    }
                }

                self.state = State::FetchOpcode;
            }
            State::ExecAcc => {
                // Perform a dummy bus access. No actual value is read from or written to the bus.
                // But the 6502 performs a bus access at each cycle - even if it is useless.
                self.access_bus();
                if let Some(ins) = LOOKUP[self.cur_op as usize] {
                    match ins.1 {
                        // Boolean parameter is_accumulator passed true for accumulator addressing mode
                        Nmeonic::ASL => self.asl(true),
                        Nmeonic::LSR => self.lsr(true),
                        Nmeonic::ROL => self.rol(true),
                        Nmeonic::ROR => self.ror(true),
                        _ => panic!("Unrecognized opcode-addressing mode-nmeonic combination!"),
                    }
                }
            }
            State::FetchAbsLo => {
                // Fetch the least significant byte of the effective address
                self.addr = self.pc;
                self.read = true;
                self.access_bus();
                self.pc = self.pc.wrapping_add(1);
                self.state = State::FetchAbsHi;
            }
            State::FetchAbsHi => {
                // Fetch the most significant byte of the effective address
                let lo = self.data as u16; // Data bus contains the least significant byte fetched in the previous cycle
                self.addr = self.pc;
                self.read = true;
                self.access_bus();
                self.pc = self.pc.wrapping_add(1);
                let hi = self.data as u16;
                self.latch_u16 = (hi << 8) | lo;

                if let Some(ins) = LOOKUP[self.cur_op as usize] {
                    match ins.1 {
                        Nmeonic::JMP => {
                            // Absolute JMP is only 3 cycles, there is no foruth cycle to fetch memroy from the effective address.
                            // pc is set to the effective address and state is back to fetch opcode.
                            self.jmp();
                            self.state = State::FetchOpcode;
                        }
                        // Read-Modify-Write instructions.
                        // These instructions take 6 cycles.
                        Nmeonic::ASL
                        | Nmeonic::DEC
                        | Nmeonic::INC
                        | Nmeonic::JSR
                        | Nmeonic::LSR
                        | Nmeonic::ROL
                        | Nmeonic::ROR => self.state = State::RmwRead,
                        _ => self.state = State::ExecAbs,
                    }
                }
            }
            State::ExecAbs => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();

                if let Some(ins) = LOOKUP[self.cur_op as usize] {
                    match ins.1 {
                        Nmeonic::ADC => self.adc(),
                        Nmeonic::AND => self.and(),
                        Nmeonic::BIT => self.bit(),
                        Nmeonic::CMP => self.cmp(),
                        Nmeonic::CPX => self.cpx(),
                        Nmeonic::CPY => self.cpy(),
                        Nmeonic::EOR => self.eor(),
                        Nmeonic::LDA => self.lda(),
                        Nmeonic::LDX => self.ldx(),
                        Nmeonic::LDY => self.ldy(),
                        Nmeonic::ORA => self.ora(),
                        Nmeonic::SBC => self.sbc(),
                        Nmeonic::STA => self.sta(),
                        Nmeonic::STX => self.stx(),
                        Nmeonic::STY => self.sty(),
                        _ => panic!("Unimplemented nmeonic for absolute addressing mode!"),
                    }
                }
            }
            State::RmwRead => {
                // Cycle 4 of r-m-w instructions.
                // Perform a read on the effective address.
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.state = State::RmwDummyWrite;
            }
            State::RmwDummyWrite => {
                // Cycle 5 of r-m-w instructions.
                // Perfrom a dummy write on the effective address of the value just read.
                self.addr = self.latch_u16;
                self.read = false;
                self.access_bus();
                self.state = State::RmwExec;
            }
            _ => todo!("Implement remaining states!"),
        }
    }

    fn access_bus(&mut self) {
        if self.read {
            self.data = self.bus.read(self.addr);
        } else {
            self.bus.write(self.addr, self.data);
        }
    }

    // Returns the set status of a flag in p register.
    pub fn flag_set(&self, flag: Flags) -> bool {
        (self.p & (1 << flag as u8)) != 0
    }

    // Set the given flag in p register based on a condition.
    fn set_flag(&mut self, flag: Flags, con: bool) {
        if con {
            self.p |= 1 << flag as u8;
        } else {
            self.p &= !(1 << flag as u8);
        }
    }

    fn adc(&mut self) {
        // Currently does not handle bcd mode addition.
        let val = self.data;
        let (mut result, mut overflow) = self.a.overflowing_add(val);
        if self.flag_set(Flags::Carry) {
            (result, overflow) = result.overflowing_add(1);
        }
        self.a = result;

        // Flags
        self.set_flag(Flags::Carry, overflow);
        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(
            Flags::Overflow,
            ((self.a ^ result) & (val ^ result) & 0x80) != 0,
        );
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn and(&mut self) {
        self.a ^= self.data;

        // Set flags
        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn asl(&mut self, is_accumulator: bool) {
        if is_accumulator {
            let shifted_out = (self.a & (1 << 7)) != 0; // True if bit 7 of a is set
            self.a <<= 1;

            self.set_flag(Flags::Carry, shifted_out);
            self.set_flag(Flags::Zero, self.a == 0);
            self.set_flag(Flags::Negative, (self.a as i8) < 0);
        }
    }

    fn bcc(&mut self) {
        unimplemented!();
    }

    fn bcs(&mut self) {
        unimplemented!();
    }

    fn beq(&mut self) {
        unimplemented!();
    }

    fn bit(&mut self) {
        unimplemented!();
    }

    fn bmi(&mut self) {
        unimplemented!();
    }

    fn bne(&mut self) {
        unimplemented!();
    }

    fn bpl(&mut self) {
        unimplemented!();
    }

    fn brk(&mut self) {
        unimplemented!();
    }

    fn bvc(&mut self) {
        unimplemented!();
    }

    fn bvs(&mut self) {
        unimplemented!();
    }

    fn clc(&mut self) {
        unimplemented!();
    }

    fn cld(&mut self) {
        unimplemented!();
    }

    fn cli(&mut self) {
        unimplemented!();
    }

    fn clv(&mut self) {
        unimplemented!();
    }

    fn cmp(&mut self) {
        let result = self.a.wrapping_sub(self.data);

        self.set_flag(Flags::Carry, self.a >= self.data);
        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
    }

    fn cpx(&mut self) {
        let result = self.x.wrapping_sub(self.data);

        self.set_flag(Flags::Carry, self.x >= self.data);
        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
    }

    fn cpy(&mut self) {
        let result = self.y.wrapping_sub(self.data);

        self.set_flag(Flags::Carry, self.y >= self.data);
        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
    }

    fn dec(&mut self) {
        unimplemented!();
    }

    fn dex(&mut self) {
        unimplemented!();
    }

    fn dey(&mut self) {
        unimplemented!();
    }

    fn eor(&mut self) {
        self.a ^= self.data;

        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn inc(&mut self) {
        unimplemented!();
    }

    fn inx(&mut self) {
        unimplemented!();
    }

    fn iny(&mut self) {
        unimplemented!();
    }

    fn jmp(&mut self) {
        self.pc = self.latch_u16;
    }

    fn jsr(&mut self) {
        unimplemented!();
    }

    fn lda(&mut self) {
        self.a = self.data;

        // Set flags
        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn ldx(&mut self) {
        self.x = self.data;

        self.set_flag(Flags::Zero, self.x == 0);
        self.set_flag(Flags::Negative, (self.x as i8) < 0);
    }

    fn ldy(&mut self) {
        self.y = self.data;

        self.set_flag(Flags::Zero, self.y == 0);
        self.set_flag(Flags::Negative, (self.y as i8) < 0);
    }

    fn lsr(&mut self, is_accumulator: bool) {
        if is_accumulator {
            let shifted_out = (self.a & 1) != 0;
            self.a >>= 1;

            self.set_flag(Flags::Carry, shifted_out);
            self.set_flag(Flags::Zero, self.a == 0);
            self.set_flag(Flags::Negative, (self.a as i8) < 0); // Should be impossible to set
        }
    }

    fn nop(&mut self) {
        unimplemented!();
    }

    fn ora(&mut self) {
        self.a |= self.data;

        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn pha(&mut self) {
        unimplemented!();
    }

    fn php(&mut self) {
        unimplemented!();
    }

    fn pla(&mut self) {
        unimplemented!();
    }

    fn plp(&mut self) {
        unimplemented!();
    }

    fn rol(&mut self, is_accumulator: bool) {
        if is_accumulator {
            let rotated_in = self.flag_set(Flags::Carry); // New bit 0 is the current value of carry flag
            let rotated_out = (self.a & (1 << 7)) != 0; // Carry flag set to old bit 7

            self.a <<= 1;
            if rotated_in {
                self.a |= 1;
            }

            self.set_flag(Flags::Carry, rotated_out);
            self.set_flag(Flags::Zero, self.a == 0);
            self.set_flag(Flags::Negative, (self.a as i8) < 0);
        }
    }

    fn ror(&mut self, is_accumulator: bool) {
        if is_accumulator {
            let rotated_in = self.flag_set(Flags::Carry); // New bit 7 is the current value of carry flag
            let rotated_out = (self.a & 1) != 0; // Carry flag set to old bit 0

            self.a >>= 1;
            if rotated_in {
                self.a |= (1 << 7);
            }

            self.set_flag(Flags::Carry, rotated_out);
            self.set_flag(Flags::Zero, self.a == 0);
            self.set_flag(Flags::Negative, (self.a as i8) < 0);
        }
    }

    fn rti(&mut self) {
        unimplemented!();
    }

    fn rts(&mut self) {
        unimplemented!();
    }

    fn sbc(&mut self) {
        unimplemented!();
    }

    fn sec(&mut self) {
        unimplemented!();
    }

    fn sed(&mut self) {
        unimplemented!();
    }

    fn sei(&mut self) {
        unimplemented!();
    }

    fn sta(&mut self) {
        unimplemented!();
    }

    fn stx(&mut self) {
        unimplemented!();
    }

    fn sty(&mut self) {
        unimplemented!();
    }

    fn tax(&mut self) {
        unimplemented!();
    }

    fn tay(&mut self) {
        unimplemented!();
    }

    fn tsx(&mut self) {
        unimplemented!();
    }

    fn txa(&mut self) {
        unimplemented!();
    }

    fn txs(&mut self) {
        unimplemented!();
    }

    fn tya(&mut self) {
        unimplemented!();
    }
}
