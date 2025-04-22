use crate::bus::BusDevice;

pub struct Cpu<'a> {
    // Instruction lookup table, indexed by the opcode
    lookup: [Instruction; 256],

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

type Instruction = Option<(AddressingMode, Nmeonic)>;

enum Nmeonic {
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,
}

enum State {
    FetchOpcode,
    ExecImm,
    FetchOperand,
    Execute,
    // Many more states
}

#[derive(Copy, Clone)]
enum AddressingMode {
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
        let lookup: [Instruction; 256] = [
            Some((AddressingMode::Implied, Nmeonic::BRK)),
            Some((AddressingMode::IndirectX, Nmeonic::ORA)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPage, Nmeonic::ORA)),
            Some((AddressingMode::ZeroPage, Nmeonic::ASL)),
            None,
            Some((AddressingMode::Implied, Nmeonic::PHP)),
            Some((AddressingMode::Immediate, Nmeonic::ORA)),
            Some((AddressingMode::Accumulator, Nmeonic::ASL)),
            None,
            None,
            Some((AddressingMode::Absolute, Nmeonic::ORA)),
            Some((AddressingMode::Absolute, Nmeonic::ASL)),
            None,
            Some((AddressingMode::Relative, Nmeonic::BPL)),
            Some((AddressingMode::IndirectY, Nmeonic::ORA)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Nmeonic::ORA)),
            Some((AddressingMode::ZeroPageX, Nmeonic::ASL)),
            None,
            Some((AddressingMode::Implied, Nmeonic::CLC)),
            Some((AddressingMode::AbsoluteY, Nmeonic::ORA)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Nmeonic::ORA)),
            Some((AddressingMode::AbsoluteX, Nmeonic::ASL)),
            None,
            Some((AddressingMode::Absolute, Nmeonic::JSR)),
            Some((AddressingMode::IndirectX, Nmeonic::AND)),
            None,
            None,
            Some((AddressingMode::ZeroPage, Nmeonic::BIT)),
            Some((AddressingMode::ZeroPage, Nmeonic::AND)),
            Some((AddressingMode::ZeroPage, Nmeonic::ROL)),
            None,
            Some((AddressingMode::Implied, Nmeonic::PLP)),
            Some((AddressingMode::Immediate, Nmeonic::AND)),
            Some((AddressingMode::Accumulator, Nmeonic::ROL)),
            None,
            Some((AddressingMode::Absolute, Nmeonic::BIT)),
            Some((AddressingMode::Absolute, Nmeonic::AND)),
            Some((AddressingMode::Absolute, Nmeonic::ROL)),
            None,
            Some((AddressingMode::Relative, Nmeonic::BMI)),
            Some((AddressingMode::IndirectY, Nmeonic::AND)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Nmeonic::AND)),
            Some((AddressingMode::ZeroPageX, Nmeonic::ROL)),
            None,
            Some((AddressingMode::Implied, Nmeonic::SEC)),
            Some((AddressingMode::AbsoluteY, Nmeonic::AND)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Nmeonic::AND)),
            Some((AddressingMode::AbsoluteX, Nmeonic::ROL)),
            None,
            Some((AddressingMode::Implied, Nmeonic::RTI)),
            Some((AddressingMode::IndirectX, Nmeonic::EOR)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPage, Nmeonic::EOR)),
            Some((AddressingMode::ZeroPage, Nmeonic::LSR)),
            None,
            Some((AddressingMode::Implied, Nmeonic::PHA)),
            Some((AddressingMode::Immediate, Nmeonic::EOR)),
            Some((AddressingMode::Accumulator, Nmeonic::LSR)),
            None,
            Some((AddressingMode::Absolute, Nmeonic::JMP)),
            Some((AddressingMode::Absolute, Nmeonic::EOR)),
            Some((AddressingMode::Absolute, Nmeonic::LSR)),
            None,
            Some((AddressingMode::Relative, Nmeonic::BVC)),
            Some((AddressingMode::IndirectY, Nmeonic::EOR)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Nmeonic::EOR)),
            Some((AddressingMode::ZeroPageX, Nmeonic::LSR)),
            None,
            Some((AddressingMode::Implied, Nmeonic::CLI)),
            Some((AddressingMode::AbsoluteY, Nmeonic::EOR)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Nmeonic::EOR)),
            Some((AddressingMode::AbsoluteX, Nmeonic::LSR)),
            None,
            Some((AddressingMode::Implied, Nmeonic::RTS)),
            Some((AddressingMode::IndirectX, Nmeonic::ADC)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPage, Nmeonic::ADC)),
            Some((AddressingMode::ZeroPage, Nmeonic::ROR)),
            None,
            Some((AddressingMode::Implied, Nmeonic::PLA)),
            Some((AddressingMode::Immediate, Nmeonic::ADC)),
            Some((AddressingMode::Accumulator, Nmeonic::ROR)),
            None,
            Some((AddressingMode::Indirect, Nmeonic::JMP)),
            Some((AddressingMode::Absolute, Nmeonic::ADC)),
            Some((AddressingMode::Absolute, Nmeonic::ROR)),
            None,
            Some((AddressingMode::Relative, Nmeonic::BVS)),
            Some((AddressingMode::IndirectY, Nmeonic::ADC)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Nmeonic::ADC)),
            Some((AddressingMode::ZeroPageX, Nmeonic::ROR)),
            None,
            Some((AddressingMode::Implied, Nmeonic::SEI)),
            Some((AddressingMode::AbsoluteY, Nmeonic::ADC)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Nmeonic::ADC)),
            Some((AddressingMode::AbsoluteX, Nmeonic::ROR)),
            None,
            None,
            Some((AddressingMode::IndirectX, Nmeonic::STA)),
            None,
            None,
            Some((AddressingMode::ZeroPage, Nmeonic::STY)),
            Some((AddressingMode::ZeroPage, Nmeonic::STA)),
            Some((AddressingMode::ZeroPage, Nmeonic::STX)),
            None,
            Some((AddressingMode::Implied, Nmeonic::DEY)),
            None,
            Some((AddressingMode::Implied, Nmeonic::TXA)),
            None,
            Some((AddressingMode::Absolute, Nmeonic::STY)),
            Some((AddressingMode::Absolute, Nmeonic::STA)),
            Some((AddressingMode::Absolute, Nmeonic::STX)),
            None,
            Some((AddressingMode::Relative, Nmeonic::BCC)),
            Some((AddressingMode::IndirectY, Nmeonic::STA)),
            None,
            None,
            Some((AddressingMode::ZeroPageX, Nmeonic::STY)),
            Some((AddressingMode::ZeroPageX, Nmeonic::STA)),
            Some((AddressingMode::ZeroPageY, Nmeonic::STX)),
            None,
            Some((AddressingMode::Implied, Nmeonic::TYA)),
            Some((AddressingMode::AbsoluteY, Nmeonic::STA)),
            Some((AddressingMode::Implied, Nmeonic::TXS)),
            None,
            None,
            Some((AddressingMode::AbsoluteX, Nmeonic::STA)),
            None,
            None,
            Some((AddressingMode::Immediate, Nmeonic::LDY)),
            Some((AddressingMode::IndirectX, Nmeonic::LDA)),
            Some((AddressingMode::Immediate, Nmeonic::LDX)),
            None,
            Some((AddressingMode::ZeroPage, Nmeonic::LDY)),
            Some((AddressingMode::ZeroPage, Nmeonic::LDA)),
            Some((AddressingMode::ZeroPage, Nmeonic::LDX)),
            None,
            Some((AddressingMode::Implied, Nmeonic::TAY)),
            Some((AddressingMode::Immediate, Nmeonic::LDA)),
            Some((AddressingMode::Implied, Nmeonic::TAX)),
            None,
            Some((AddressingMode::Absolute, Nmeonic::LDY)),
            Some((AddressingMode::Absolute, Nmeonic::LDA)),
            Some((AddressingMode::Absolute, Nmeonic::LDX)),
            None,
            Some((AddressingMode::Relative, Nmeonic::BCS)),
            Some((AddressingMode::IndirectY, Nmeonic::LDA)),
            None,
            None,
            Some((AddressingMode::ZeroPageX, Nmeonic::LDY)),
            Some((AddressingMode::ZeroPageX, Nmeonic::LDA)),
            Some((AddressingMode::ZeroPageY, Nmeonic::LDX)),
            None,
            Some((AddressingMode::Implied, Nmeonic::CLV)),
            Some((AddressingMode::AbsoluteY, Nmeonic::LDA)),
            Some((AddressingMode::Implied, Nmeonic::TSX)),
            None,
            Some((AddressingMode::AbsoluteX, Nmeonic::LDY)),
            Some((AddressingMode::AbsoluteX, Nmeonic::LDA)),
            Some((AddressingMode::AbsoluteY, Nmeonic::LDX)),
            None,
            Some((AddressingMode::Immediate, Nmeonic::CPY)),
            Some((AddressingMode::IndirectX, Nmeonic::CMP)),
            None,
            None,
            Some((AddressingMode::ZeroPage, Nmeonic::CPY)),
            Some((AddressingMode::ZeroPage, Nmeonic::CMP)),
            Some((AddressingMode::ZeroPage, Nmeonic::DEC)),
            None,
            Some((AddressingMode::Implied, Nmeonic::INY)),
            Some((AddressingMode::Immediate, Nmeonic::CMP)),
            Some((AddressingMode::Implied, Nmeonic::DEX)),
            None,
            Some((AddressingMode::Absolute, Nmeonic::CPY)),
            Some((AddressingMode::Absolute, Nmeonic::CMP)),
            Some((AddressingMode::Absolute, Nmeonic::DEC)),
            None,
            Some((AddressingMode::Relative, Nmeonic::BNE)),
            Some((AddressingMode::IndirectY, Nmeonic::CMP)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Nmeonic::CMP)),
            Some((AddressingMode::ZeroPageX, Nmeonic::DEC)),
            None,
            Some((AddressingMode::Implied, Nmeonic::CLD)),
            Some((AddressingMode::AbsoluteY, Nmeonic::CMP)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Nmeonic::CMP)),
            Some((AddressingMode::AbsoluteX, Nmeonic::DEC)),
            None,
            Some((AddressingMode::Immediate, Nmeonic::CPX)),
            Some((AddressingMode::IndirectX, Nmeonic::SBC)),
            None,
            None,
            Some((AddressingMode::ZeroPage, Nmeonic::CPX)),
            Some((AddressingMode::ZeroPage, Nmeonic::SBC)),
            Some((AddressingMode::ZeroPage, Nmeonic::INC)),
            None,
            Some((AddressingMode::Implied, Nmeonic::INX)),
            Some((AddressingMode::Immediate, Nmeonic::SBC)),
            Some((AddressingMode::Implied, Nmeonic::NOP)),
            None,
            Some((AddressingMode::Absolute, Nmeonic::CPX)),
            Some((AddressingMode::Absolute, Nmeonic::SBC)),
            Some((AddressingMode::Absolute, Nmeonic::INC)),
            None,
            Some((AddressingMode::Relative, Nmeonic::BEQ)),
            Some((AddressingMode::IndirectY, Nmeonic::SBC)),
            None,
            None,
            None,
            Some((AddressingMode::ZeroPageX, Nmeonic::SBC)),
            Some((AddressingMode::ZeroPageX, Nmeonic::INC)),
            None,
            Some((AddressingMode::Implied, Nmeonic::SED)),
            Some((AddressingMode::AbsoluteY, Nmeonic::SBC)),
            None,
            None,
            None,
            Some((AddressingMode::AbsoluteX, Nmeonic::SBC)),
            Some((AddressingMode::AbsoluteX, Nmeonic::INC)),
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

                if let Some(instruction) = self.lookup[self.cur_op as usize] {
                    match instruction.1 {
                        Nmeonic::LDA => self.lda(),
                        _ => return,
                    }
                }

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
