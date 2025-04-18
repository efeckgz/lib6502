use crate::bus::BusDevice;
use lookup::LOOKUP;

mod lookup;

pub type Instruction<'a> = Option<(AddressingMode, fn(&'a mut Cpu<'a>))>;

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
    // cur_exec: Option<fn(&'a mut Cpu)>,
    cur_op: u8, // Current opcode being worked on

    // Bus variables
    pub bus: &'a mut dyn BusDevice, // The bus itself
    pub addr: u16,                  // 16 bit address bus value
    pub data: u8,                   // 8 bit data bus value
    pub read: bool,                 // bus read/write mode control variable
}

enum State {
    FetchOpcode,
    ExecImm,
    FetchOperand,
    Execute,
    // Many more states
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
            // cur_exec: None,
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
                    // self.cur_exec = Some(instruction.1);
                    match instruction.0 {
                        AddressingMode::Immediate => self.state = State::ExecImm,
                        _ => self.state = State::FetchOperand, // placeholder default
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
                    (instruction.1)(self);
                }
                // (self.cur_exec)(self);
                self.state = State::FetchOpcode;
            }
            State::FetchOperand => {
                self.addr = self.pc;
                self.read = true;
                self.access_bus();
                self.pc = self.pc.wrapping_add(1);
                self.state = State::Execute;
            }
            State::Execute => {
                self.addr = data_prev as u16;
                self.read = true;
                self.access_bus();
                self.a = self.data;
                self.state = State::FetchOpcode;
            }
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
        unimplemented!();
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

    pub fn execute(&mut self) {
        self.bus.write(0x00, 0x42);
    }
}
