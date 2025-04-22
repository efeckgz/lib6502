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

enum State {
    FetchOpcode,
    ExecImm,
    // Many more states
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
        }
    }

    pub fn cycle(&mut self) {
        let data_prev = self.data;
        match self.state {
            State::FetchOpcode => {
                self.addr = self.pc;
                self.read = true;
                self.access_bus();

                self.pc = self.pc.wrapping_add(1);
                self.cur_op = self.data;
                if let Some(instruction) = LOOKUP[self.data as usize] {
                    match instruction.0 {
                        AddressingMode::Immediate => self.state = State::ExecImm,
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
                        Nmeonic::LDA => self.lda(),
                        _ => return,
                    }
                }

                self.state = State::FetchOpcode;
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

    fn adc(&mut self) {
        unimplemented!();
    }

    fn and(&mut self) {
        unimplemented!();
    }

    fn asl(&mut self) {
        unimplemented!();
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
        unimplemented!();
    }

    fn cpx(&mut self) {
        unimplemented!();
    }

    fn cpy(&mut self) {
        unimplemented!();
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
        unimplemented!();
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
        unimplemented!();
    }

    fn jsr(&mut self) {
        unimplemented!();
    }

    fn lda(&mut self) {
        self.a = self.data;
    }

    fn ldx(&mut self) {
        unimplemented!();
    }

    fn ldy(&mut self) {
        unimplemented!();
    }

    fn lsr(&mut self) {
        unimplemented!();
    }

    fn nop(&mut self) {
        unimplemented!();
    }

    fn ora(&mut self) {
        unimplemented!();
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
        unimplemented!();
    }

    fn ror(&mut self) {
        unimplemented!();
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
