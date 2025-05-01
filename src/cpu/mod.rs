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
    pub state: State,
    cur_mode: AddressingMode,
    cur_nmeonic: Nmeonic,

    // Bus variables
    pub bus: &'a mut dyn BusDevice, // The bus itself
    pub addr: u16,                  // 16 bit address bus value
    pub data: u8,                   // 8 bit data bus value
    pub read: bool,                 // bus read/write mode control variable

    // Latches to hold temporary values
    latch_u8: u8,
    latch_u16: u16,
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
pub enum State {
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

    // JSR states - JSR is a 6 cycle absolute instruction but it works differently than others.
    JsrStorePcH,
    JsrStorePcL,
    JsrRead, // Read the new program counter here
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
            cur_mode: AddressingMode::Implied,
            cur_nmeonic: Nmeonic::BRK,
            bus,
            addr: 0,
            data: 0,
            read: false,
            latch_u8: 0,
            latch_u16: 0,
        }
    }

    // Emulate 1 cpu cycle.
    pub fn cycle(&mut self) {
        match self.state {
            State::FetchOpcode => self.fetch_opcode(),
            State::ExecImm => self.exec_imm(),
            State::ExecAcc => self.exec_acc(),
            State::FetchAbsLo => self.fetch_abs_lo(),
            State::FetchAbsHi => self.fetch_abs_hi(),
            State::ExecAbs => self.exec_abs(),
            State::RmwRead => self.rmw_read(),
            State::RmwDummyWrite => self.rmw_dummy_write(),
            State::RmwExec => self.rmw_exec(),
            State::JsrStorePcH => self.jsr_store_pch(),
            State::JsrStorePcL => self.jsr_store_pcl(),
            State::JsrRead => self.jsr_read(),
            _ => todo!("Implement remaining states!"),
        }
    }

    fn fetch_opcode(&mut self) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();

        self.pc = self.pc.wrapping_add(1);
        if let Some(instruction) = LOOKUP[self.data as usize] {
            let (mode, nm) = instruction;
            self.cur_mode = mode;
            self.cur_nmeonic = nm;
        } else {
            todo!("Implement illegal opcodes!");
        }
        match self.cur_mode {
            AddressingMode::Accumulator => self.state = State::ExecAcc,
            AddressingMode::Immediate => self.state = State::ExecImm,
            AddressingMode::Absolute => self.state = State::FetchAbsLo,
            _ => todo!("Implement remaining states"),
        }
    }

    fn exec_imm(&mut self) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.pc = self.pc.wrapping_add(1);

        match self.cur_nmeonic {
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

        self.state = State::FetchOpcode;
    }

    fn exec_acc(&mut self) {
        // Perform a dummy bus access. No actual value is read from or written to the bus.
        // But the 6502 performs a bus access at each cycle - even if it is useless.
        self.access_bus();
        match self.cur_nmeonic {
            // Boolean parameter is_accumulator passed true for accumulator addressing mode
            Nmeonic::ASL => self.asl(),
            Nmeonic::LSR => self.lsr(),
            Nmeonic::ROL => self.rol(),
            Nmeonic::ROR => self.ror(),
            _ => panic!("Unrecognized opcode-addressing mode-nmeonic combination!"),
        }
        self.state = State::FetchOpcode;
    }

    fn fetch_abs_lo(&mut self) {
        // Fetch the low byte of the effective address
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.latch_u8 = self.data;
        self.pc = self.pc.wrapping_add(1);

        match self.cur_nmeonic {
            Nmeonic::JSR => self.state = State::JsrStorePcH,
            _ => self.state = State::FetchAbsHi,
        }
    }

    fn fetch_abs_hi(&mut self) {
        // Fetch the high byte of the effective address
        let lo = self.latch_u8 as u16; // Data bus contains the least significant byte fetched in the previous cycle
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.pc = self.pc.wrapping_add(1);
        let hi = self.data as u16;
        self.latch_u16 = (hi << 8) | lo;

        match self.cur_nmeonic {
            Nmeonic::JMP => {
                // Absolute JMP is only 3 cycles, there is no foruth cycle to fetch memroy from the effective address.
                // pc is set to the effective address and state is back to fetch opcode.
                self.jmp();
                self.state = State::FetchOpcode;
            }
            Nmeonic::JSR => {
                self.pc = self.latch_u16;
                self.state = State::JsrRead;
            }
            // Read-Modify-Write instructions.
            // These instructions take 6 cycles.
            Nmeonic::ASL
            | Nmeonic::DEC
            | Nmeonic::INC
            | Nmeonic::LSR
            | Nmeonic::ROL
            | Nmeonic::ROR => self.state = State::RmwRead,
            _ => self.state = State::ExecAbs,
        }
    }

    fn exec_abs(&mut self) {
        match self.cur_nmeonic {
            Nmeonic::ADC => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.adc()
            }
            Nmeonic::AND => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.and()
            }
            Nmeonic::BIT => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.bit()
            }
            Nmeonic::CMP => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.cmp()
            }
            Nmeonic::CPX => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.cpx()
            }
            Nmeonic::CPY => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.cpy()
            }
            Nmeonic::EOR => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.eor()
            }
            Nmeonic::LDA => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.lda()
            }
            Nmeonic::LDX => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.ldx()
            }
            Nmeonic::LDY => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.ldy()
            }
            Nmeonic::ORA => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.ora()
            }
            Nmeonic::SBC => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.sbc()
            }
            // Store instruction functions perform their bus operations.
            // They perform bus write on latch_u16
            Nmeonic::STA => self.sta(),
            Nmeonic::STX => self.stx(),
            Nmeonic::STY => self.sty(),
            _ => panic!("Unimplemented nmeonic for absolute addressing mode!"),
        }
        self.state = State::FetchOpcode;
    }

    fn rmw_read(&mut self) {
        // Cycle 4 of r-m-w instructions.
        // Perform a read on the effective address.
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();
        self.state = State::RmwDummyWrite;
    }

    fn rmw_dummy_write(&mut self) {
        // Cycle 5 of r-m-w instructions.
        // Perfrom a dummy write on the effective address of the value just read.
        self.addr = self.latch_u16;
        self.read = false;
        self.access_bus();
        self.state = State::RmwExec;
    }

    fn rmw_exec(&mut self) {
        match self.cur_nmeonic {
            Nmeonic::ASL => self.asl(),
            Nmeonic::DEC => self.dec(),
            Nmeonic::INC => self.inc(),
            Nmeonic::JSR => self.jsr(), // jump to subroutine works differently!
            Nmeonic::LSR => self.lsr(),
            Nmeonic::ROL => self.rol(),
            Nmeonic::ROR => self.ror(),
            _ => unimplemented!(),
        }

        self.data = self.latch_u8;
        self.addr = self.latch_u16;
        self.read = false;
        self.access_bus();
        self.state = State::FetchOpcode;
    }

    fn jsr_store_pch(&mut self) {
        self.push_stack(((self.pc & 0xFF00) >> 7) as u8);
        self.state = State::JsrStorePcL;
    }

    fn jsr_store_pcl(&mut self) {
        self.push_stack((self.pc & 0xFF) as u8);
    }

    fn jsr_read(&mut self) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.state = State::FetchOpcode;
    }

    fn access_bus(&mut self) {
        if self.read {
            self.data = self.bus.read(self.addr);
        } else {
            self.bus.write(self.addr, self.data);
        }
    }

    fn push_stack(&mut self, val: u8) {
        self.addr = self.s as u16;
        self.data = val;
        self.read = false;
        self.access_bus();
        self.s = self.s.wrapping_sub(1);
    }

    fn pop_stack(&mut self) {
        self.addr = self.s as u16;
        self.read = true;
        self.access_bus(); // Stack top is at self.data now
        self.s = self.s.wrapping_add(1);
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

    fn asl(&mut self) {
        match self.cur_mode {
            AddressingMode::Accumulator => {
                let shifted_out = (self.a & (1 << 7)) != 0; // True if bit 7 of a is set
                self.a <<= 1;

                self.set_flag(Flags::Carry, shifted_out);
                self.set_flag(Flags::Zero, self.a == 0);
                self.set_flag(Flags::Negative, (self.a as i8) < 0);
            }
            AddressingMode::ZeroPage
            | AddressingMode::ZeroPageX
            | AddressingMode::Absolute
            | AddressingMode::AbsoluteX => {
                let mut val = self.data;
                let shifted_out = (val & (1 << 7)) != 0;
                val <<= 1;

                self.set_flag(Flags::Carry, shifted_out);
                self.set_flag(Flags::Zero, val == 0);
                self.set_flag(Flags::Negative, (val as i8) < 0);

                self.latch_u8 = val;
            }
            _ => panic!("Unrecognized addressing mode for asl instruction!"),
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
        let result = self.a & self.data;

        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Overflow, (self.data & (1 << 6)) != 0);
        self.set_flag(Flags::Negative, (self.data & (1 << 7)) != 0);
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
        let val = self.data;
        let result = val.wrapping_sub(1);

        self.latch_u8 = result;

        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
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
        let val = self.data;
        let result = val.wrapping_add(1);

        self.latch_u8 = result;

        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
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

    fn lsr(&mut self) {
        match self.cur_mode {
            AddressingMode::Accumulator => {
                let shifted_out = (self.a & 1) != 0;
                self.a >>= 1;

                self.set_flag(Flags::Carry, shifted_out);
                self.set_flag(Flags::Zero, self.a == 0);
                self.set_flag(Flags::Negative, (self.a as i8) < 0); // Should be impossible to set
            }
            AddressingMode::ZeroPage
            | AddressingMode::ZeroPageX
            | AddressingMode::Absolute
            | AddressingMode::AbsoluteX => {
                let val = self.data;
                let shifted_out = (val & 1) != 0;
                let result = val >> 1;

                self.set_flag(Flags::Carry, shifted_out);
                self.set_flag(Flags::Zero, result == 0);
                self.set_flag(Flags::Negative, (result as i8) < 0);

                self.latch_u8 = result;
            }
            _ => panic!("Unrecognized addressing mode for lsr instruction!"),
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

    fn rol(&mut self) {
        match self.cur_mode {
            AddressingMode::Accumulator => {
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
            AddressingMode::ZeroPage
            | AddressingMode::ZeroPageX
            | AddressingMode::Absolute
            | AddressingMode::AbsoluteX => {
                let val = self.data;
                let rotated_in = self.flag_set(Flags::Carry);
                let rotated_out = (val & (1 << 7)) != 0;

                let mut result = val << 1;
                if rotated_in {
                    result |= 1;
                }

                self.set_flag(Flags::Carry, rotated_out);
                self.set_flag(Flags::Zero, result == 0);
                self.set_flag(Flags::Negative, (result as i8) < 0);

                self.latch_u8 = result;
            }
            _ => panic!("Unrecognized addressing mode for rol instruction!"),
        }
    }

    fn ror(&mut self) {
        match self.cur_mode {
            AddressingMode::Accumulator => {
                let rotated_in = self.flag_set(Flags::Carry); // New bit 7 is the current value of carry flag
                let rotated_out = (self.a & 1) != 0; // Carry flag set to old bit 0

                self.a >>= 1;
                if rotated_in {
                    self.a |= 1 << 7;
                }

                self.set_flag(Flags::Carry, rotated_out);
                self.set_flag(Flags::Zero, self.a == 0);
                self.set_flag(Flags::Negative, (self.a as i8) < 0);
            }
            AddressingMode::ZeroPage
            | AddressingMode::ZeroPageX
            | AddressingMode::Absolute
            | AddressingMode::AbsoluteX => {
                let val = self.data;
                let rotated_in = self.flag_set(Flags::Carry);
                let rotated_out = (val & 1) != 0;

                let mut result = val >> 1;
                if rotated_in {
                    result |= 1 << 7;
                }

                self.set_flag(Flags::Carry, rotated_out);
                self.set_flag(Flags::Zero, result == 0);
                self.set_flag(Flags::Negative, (result as i8) < 0);

                self.latch_u8 = result;
            }
            _ => panic!("Unrecognized addressing mode for ror instruction!"),
        }
    }

    fn rti(&mut self) {
        unimplemented!();
    }

    fn rts(&mut self) {
        unimplemented!();
    }

    fn sbc(&mut self) {
        let val = self.data;
        let (mut result, mut overflow) = self.a.overflowing_sub(val);
        if !self.flag_set(Flags::Carry) {
            (result, overflow) = result.overflowing_sub(1);
        }

        self.a = result;

        self.set_flag(Flags::Carry, overflow);
        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(
            Flags::Overflow,
            ((self.a ^ result) & (val ^ result) & 0x80) != 0,
        );
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
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
        self.addr = self.latch_u16;
        self.data = self.a;
        self.read = false;
        self.access_bus();
    }

    fn stx(&mut self) {
        self.addr = self.latch_u16;
        self.data = self.x;
        self.read = false;
        self.access_bus();
    }

    fn sty(&mut self) {
        self.addr = self.latch_u16;
        self.data = self.y;
        self.read = false;
        self.access_bus();
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
