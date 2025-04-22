use crate::bus::{Bus, BusDevice};
// use lookup::LOOKUP;

// mod lookup;

pub type Instruction<'a> = Option<(AddressingMode, fn(&'a mut Cpu<'a>))>;

pub struct Cpu<'a> {
    // Instruction lookup table, indexed by the opcode
    lookup: [Instruction<'a>; 256],

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
        let lookup: [Instruction<'a>; 256] = [
            Some((AddressingMode::Implied, Self::brk)),
            Some((AddressingMode::IndirectX, Cpu::ora)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPage, Cpu::ora)),
            Some((AddressingMode::ZeroPage, Cpu::asl)),
            None,
            Some((AddressingMode::Implied, Cpu::php)),
            Some((AddressingMode::Immediate, Cpu::ora)),
            Some((AddressingMode::Accumulator, Cpu::asl)),
            None,
            None,
            Some((AddressingMode::Absolute, Cpu::ora)),
            Some((AddressingMode::Absolute, Cpu::asl)),
            None,
            Some((AddressingMode::Relative, Cpu::bpl)),
            Some((AddressingMode::IndirectY, Cpu::ora)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Cpu::ora)),
            Some((AddressingMode::ZeroPageX, Cpu::asl)),
            None,
            Some((AddressingMode::Implied, Cpu::clc)),
            Some((AddressingMode::AbsoluteY, Cpu::ora)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Cpu::ora)),
            Some((AddressingMode::AbsoluteX, Cpu::asl)),
            None,
            Some((AddressingMode::Absolute, Cpu::jsr)),
            Some((AddressingMode::IndirectX, Cpu::and)),
            None,
            None,
            Some((AddressingMode::ZeroPage, Cpu::bit)),
            Some((AddressingMode::ZeroPage, Cpu::and)),
            Some((AddressingMode::ZeroPage, Cpu::rol)),
            None,
            Some((AddressingMode::Implied, Cpu::plp)),
            Some((AddressingMode::Immediate, Cpu::and)),
            Some((AddressingMode::Accumulator, Cpu::rol)),
            None,
            Some((AddressingMode::Absolute, Cpu::bit)),
            Some((AddressingMode::Absolute, Cpu::and)),
            Some((AddressingMode::Absolute, Cpu::rol)),
            None,
            Some((AddressingMode::Relative, Cpu::bmi)),
            Some((AddressingMode::IndirectY, Cpu::and)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Cpu::and)),
            Some((AddressingMode::ZeroPageX, Cpu::rol)),
            None,
            Some((AddressingMode::Implied, Cpu::sec)),
            Some((AddressingMode::AbsoluteY, Cpu::and)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Cpu::and)),
            Some((AddressingMode::AbsoluteX, Cpu::rol)),
            None,
            Some((AddressingMode::Implied, Cpu::rti)),
            Some((AddressingMode::IndirectX, Cpu::eor)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPage, Cpu::eor)),
            Some((AddressingMode::ZeroPage, Cpu::lsr)),
            None,
            Some((AddressingMode::Implied, Cpu::pha)),
            Some((AddressingMode::Immediate, Cpu::eor)),
            Some((AddressingMode::Accumulator, Cpu::lsr)),
            None,
            Some((AddressingMode::Absolute, Cpu::jmp)),
            Some((AddressingMode::Absolute, Cpu::eor)),
            Some((AddressingMode::Absolute, Cpu::lsr)),
            None,
            Some((AddressingMode::Relative, Cpu::bvc)),
            Some((AddressingMode::IndirectY, Cpu::eor)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Cpu::eor)),
            Some((AddressingMode::ZeroPageX, Cpu::lsr)),
            None,
            Some((AddressingMode::Implied, Cpu::cli)),
            Some((AddressingMode::AbsoluteY, Cpu::eor)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Cpu::eor)),
            Some((AddressingMode::AbsoluteX, Cpu::lsr)),
            None,
            Some((AddressingMode::Implied, Cpu::rts)),
            Some((AddressingMode::IndirectX, Cpu::adc)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPage, Cpu::adc)),
            Some((AddressingMode::ZeroPage, Cpu::ror)),
            None,
            Some((AddressingMode::Implied, Cpu::pla)),
            Some((AddressingMode::Immediate, Cpu::adc)),
            Some((AddressingMode::Accumulator, Cpu::ror)),
            None,
            Some((AddressingMode::Indirect, Cpu::jmp)),
            Some((AddressingMode::Absolute, Cpu::adc)),
            Some((AddressingMode::Absolute, Cpu::ror)),
            None,
            Some((AddressingMode::Relative, Cpu::bvs)),
            Some((AddressingMode::IndirectY, Cpu::adc)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Cpu::adc)),
            Some((AddressingMode::ZeroPageX, Cpu::ror)),
            None,
            Some((AddressingMode::Implied, Cpu::sei)),
            Some((AddressingMode::AbsoluteY, Cpu::adc)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Cpu::adc)),
            Some((AddressingMode::AbsoluteX, Cpu::ror)),
            None,
            None,
            Some((AddressingMode::IndirectX, Cpu::sta)),
            None,
            None,
            Some((AddressingMode::ZeroPage, Cpu::sty)),
            Some((AddressingMode::ZeroPage, Cpu::sta)),
            Some((AddressingMode::ZeroPage, Cpu::stx)),
            None,
            Some((AddressingMode::Implied, Cpu::dey)),
            None,
            Some((AddressingMode::Implied, Cpu::txa)),
            None,
            Some((AddressingMode::Absolute, Cpu::sty)),
            Some((AddressingMode::Absolute, Cpu::sta)),
            Some((AddressingMode::Absolute, Cpu::stx)),
            None,
            Some((AddressingMode::Relative, Cpu::bcc)),
            Some((AddressingMode::IndirectY, Cpu::sta)),
            None,
            None,
            Some((AddressingMode::ZeroPageX, Cpu::sty)),
            Some((AddressingMode::ZeroPageX, Cpu::sta)),
            Some((AddressingMode::ZeroPageY, Cpu::stx)),
            None,
            Some((AddressingMode::Implied, Cpu::tya)),
            Some((AddressingMode::AbsoluteY, Cpu::sta)),
            Some((AddressingMode::Implied, Cpu::txs)),
            None,
            None,
            Some((AddressingMode::AbsoluteX, Cpu::sta)),
            None,
            None,
            Some((AddressingMode::Immediate, Cpu::ldy)),
            Some((AddressingMode::IndirectX, Cpu::lda)),
            Some((AddressingMode::Immediate, Cpu::ldx)),
            None,
            Some((AddressingMode::ZeroPage, Cpu::ldy)),
            Some((AddressingMode::ZeroPage, Cpu::lda)),
            Some((AddressingMode::ZeroPage, Cpu::ldx)),
            None,
            Some((AddressingMode::Implied, Cpu::tay)),
            Some((AddressingMode::Immediate, Cpu::lda)),
            Some((AddressingMode::Implied, Cpu::tax)),
            None,
            Some((AddressingMode::Absolute, Cpu::ldy)),
            Some((AddressingMode::Absolute, Cpu::lda)),
            Some((AddressingMode::Absolute, Cpu::ldx)),
            None,
            Some((AddressingMode::Relative, Cpu::bcs)),
            Some((AddressingMode::IndirectY, Cpu::lda)),
            None,
            None,
            Some((AddressingMode::ZeroPageX, Cpu::ldy)),
            Some((AddressingMode::ZeroPageX, Cpu::lda)),
            Some((AddressingMode::ZeroPageY, Cpu::ldx)),
            None,
            Some((AddressingMode::Implied, Cpu::clv)),
            Some((AddressingMode::AbsoluteY, Cpu::lda)),
            Some((AddressingMode::Implied, Cpu::tsx)),
            None,
            Some((AddressingMode::AbsoluteX, Cpu::ldy)),
            Some((AddressingMode::AbsoluteX, Cpu::lda)),
            Some((AddressingMode::AbsoluteY, Cpu::ldx)),
            None,
            Some((AddressingMode::Immediate, Cpu::cpy)),
            Some((AddressingMode::IndirectX, Cpu::cmp)),
            None,
            None,
            Some((AddressingMode::ZeroPage, Cpu::cpy)),
            Some((AddressingMode::ZeroPage, Cpu::cmp)),
            Some((AddressingMode::ZeroPage, Cpu::dec)),
            None,
            Some((AddressingMode::Implied, Cpu::iny)),
            Some((AddressingMode::Immediate, Cpu::cmp)),
            Some((AddressingMode::Implied, Cpu::dex)),
            None,
            Some((AddressingMode::Absolute, Cpu::cpy)),
            Some((AddressingMode::Absolute, Cpu::cmp)),
            Some((AddressingMode::Absolute, Cpu::dec)),
            None,
            Some((AddressingMode::Relative, Cpu::bne)),
            Some((AddressingMode::IndirectY, Cpu::cmp)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Cpu::cmp)),
            Some((AddressingMode::ZeroPageX, Cpu::dec)),
            None,
            Some((AddressingMode::Implied, Cpu::cld)),
            Some((AddressingMode::AbsoluteY, Cpu::cmp)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Cpu::cmp)),
            Some((AddressingMode::AbsoluteX, Cpu::dec)),
            None,
            Some((AddressingMode::Immediate, Cpu::cpx)),
            Some((AddressingMode::IndirectX, Cpu::sbc)),
            None,
            None,
            Some((AddressingMode::ZeroPage, Cpu::cpx)),
            Some((AddressingMode::ZeroPage, Cpu::sbc)),
            Some((AddressingMode::ZeroPage, Cpu::inc)),
            None,
            Some((AddressingMode::Implied, Cpu::inx)),
            Some((AddressingMode::Immediate, Cpu::sbc)),
            Some((AddressingMode::Implied, Cpu::nop)),
            None,
            Some((AddressingMode::Absolute, Cpu::cpx)),
            Some((AddressingMode::Absolute, Cpu::sbc)),
            Some((AddressingMode::Absolute, Cpu::inc)),
            None,
            Some((AddressingMode::Relative, Cpu::beq)),
            Some((AddressingMode::IndirectY, Cpu::sbc)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Cpu::sbc)),
            Some((AddressingMode::ZeroPageX, Cpu::inc)),
            None,
            Some((AddressingMode::Implied, Cpu::sed)),
            Some((AddressingMode::AbsoluteY, Cpu::sbc)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Cpu::sbc)),
            Some((AddressingMode::AbsoluteX, Cpu::inc)),
            None,
        ];

        Self {
            lookup,
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
                if let Some(instruction) = self.lookup[self.data as usize] {
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
                match self.lookup[self.cur_op as usize] {
                    Some(i) => i.1(self),
                    None => return,
                }
                // if let Some(instruction) = self.lookup[self.cur_op as usize] {
                //     (instruction.clone().1)(self);
                // }
                // (self.cur_exec)(self)
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
        // self.bus.write(0x00, 0x42);
    }
}
