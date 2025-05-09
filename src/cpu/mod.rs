mod lookup;

use crate::bus::BusDevice;
use lookup::{LOOKUP, Nmeonic};

const STACK_BASE: u16 = 0x0100;

const FROM_BRANCH: bool = true;
const NOT_FROM_BRANCH: bool = false;

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
    None,
    Accumulator,
    Relative,
    Implied,
    Indirect,
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    IndirectX,
    IndirectY,
}

// pc, s, a, x, y, p
pub type RegisterState = (u16, u8, u8, u8, u8, u8);
// #[cfg_attr(test, derive(serde::Serialize, serde::Deserialize))]
// pub struct RegisterState {
//     pc: u16,
//     s: u8,
//     a: u8,
//     x: u8,
//     y: u8,
//     p: u8,
// }

// Inner state of the processor, used in state machine.
enum State {
    // Reset states
    ResetHold,
    FirstStart,
    SecondStart,
    ThirdStart,
    FourthStart,
    FetchFirstVec(Vectors), // The fields here hold the vector to fetch
    FetchSecondVec(Vectors),

    DummyReadPc, // Read on current pc val, dont increment pc and discard result

    FetchOpcode(bool), // Fetch opcode state. Every instruction starts here.

    // 2 cycle of 2 cycle ops
    ExecImpl, // Execute implied mode (2 cycle)
    ExecImm,  // Immediate addressing mode execution state
    ExecAcc,  // Accumulator addressing mode execution state

    // Stack operations
    ImplPush, // push stack implied mode
    ReadIncS, // Read the stack, discard the value, incremenet s
    ImplPull, // Pull from stack implied mode

    // Absolute mode states
    FetchAbsLo,
    FetchAbsHi,
    ExecAbs,

    // Zero page states
    FetchZP,
    ExecZP,

    // Read-Modify-Write states
    RmwRead,       // Read opcode from effective address
    RmwDummyWrite, // Dummy write the value read to the effective address
    RmwExec,       // Excecute the rmw instruction and write the result back

    // JSR states - JSR is a 6 cycle absolute instruction but it works differently than others.
    JsrDummyStack, // A dummy stack read is done before storing the pc
    PushPcH,
    PushPcL,
    PullPcH,
    PullPcL,

    PushP, // Push processor status register to the stack

    FetchOffset, // Fetch relative branch offset

    // Boolean flag indicates page crossed up.
    // Extra cycle due to page boundry cross.
    PageCrossed(bool),
}

// Status flags. Used in the processor status register p.
pub enum Flags {
    Carry = 0,
    Zero = 1,
    InterrputDisable = 2,
    Decimal = 3,
    Break = 4,
    Overflow = 6,
    Negative = 7,
}

#[derive(Clone, Copy)]
enum Vectors {
    NmiLo = 0xFFFA,
    NmiHi = 0xFFFB,
    ResLo = 0xFFFC,
    ResHi = 0xFFFD,
    BrkLo = 0xFFFE,
    BrkHi = 0xFFFF,
}

impl<'a> Cpu<'a> {
    pub fn new(bus: &'a mut dyn BusDevice) -> Self {
        Self {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            p: 0,
            s: 0xFF,                 // Start at stack top - not the specified behavior
            state: State::ResetHold, // Start the cpu at the reset state.
            cur_mode: AddressingMode::None,
            cur_nmeonic: Nmeonic::None,
            bus,
            addr: 0,
            data: 0,
            read: false,
            latch_u8: 0,
            latch_u16: 0,
        }
    }

    pub fn from_register_state(
        (pc, s, a, x, y, p): RegisterState,
        bus: &'a mut dyn BusDevice,
    ) -> Self {
        // let a = st.a;
        // let x = st.x;
        // let y = st.y;
        // let pc = st.pc;
        // let p = st.p;
        // let s = st.s;

        Self {
            a,
            x,
            y,
            pc,
            p,
            s,
            state: State::ResetHold,
            cur_mode: AddressingMode::None,
            cur_nmeonic: Nmeonic::None,
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
            State::ResetHold => self.reset_hold(),
            State::FirstStart => self.first_start(),
            State::SecondStart => self.second_start(),
            State::ThirdStart => self.third_start(),
            State::FourthStart => self.fourth_start(),
            State::FetchFirstVec(vector) => self.fetch_first_vec(vector),
            State::FetchSecondVec(vector) => self.fetch_second_vec(vector),
            State::FetchOpcode(from_branch) => self.fetch_opcode(from_branch),
            State::DummyReadPc => self.dummy_read_pc(),
            State::ExecImpl => self.exec_impl(),
            State::ExecImm => self.exec_imm(),
            State::ExecAcc => self.exec_acc(),
            State::ImplPush => self.impl_push(),
            State::ReadIncS => self.read_inc_s(),
            State::ImplPull => self.impl_pull(),
            State::FetchAbsLo => self.fetch_abs_lo(),
            State::FetchAbsHi => self.fetch_abs_hi(),
            State::ExecAbs => self.exec_abs(),
            State::FetchZP => self.fetch_zp(),
            State::ExecZP => self.exec_zp(),
            State::RmwRead => self.rmw_read(),
            State::RmwDummyWrite => self.rmw_dummy_write(),
            State::RmwExec => self.rmw_exec(),
            State::JsrDummyStack => self.jsr_dummy_stack(),
            State::PushPcH => self.push_pch(),
            State::PushPcL => self.push_pcl(),
            State::PullPcH => self.pull_pch(),
            State::PullPcL => self.pull_pcl(),
            State::PushP => self.push_p(),
            State::FetchOffset => self.fetch_offset(),
            State::PageCrossed(page_up) => self.page_crossed(page_up),
        }
    }

    // Runs the 7-cycle start sequence.
    pub fn start_sequence(&mut self) {
        if let State::ResetHold = self.state {
            for _ in 0..7 {
                self.cycle();
            }
        } else {
            panic!("Cannot run start sequence in this state!");
        }
    }

    fn reset_hold(&mut self) {
        // In read mode during reset, address and data bus are don't care
        self.read = true;
        self.access_bus();
        self.state = State::FirstStart;
    }

    fn first_start(&mut self) {
        self.read = true;
        self.addr = self.addr.wrapping_add(1); // Read from location next to previous cycle
        self.access_bus();
        self.state = State::SecondStart;
    }

    fn second_start(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus();
        self.state = State::ThirdStart;
    }

    fn third_start(&mut self) {
        self.addr = STACK_BASE + self.s.wrapping_sub(1) as u16;
        self.read = true;
        self.access_bus();
        self.state = State::FourthStart;
    }

    fn fourth_start(&mut self) {
        self.addr = STACK_BASE + self.s.wrapping_sub(2) as u16;
        self.read = true;
        self.access_bus();
        self.state = State::FetchFirstVec(Vectors::ResLo);
    }

    fn fetch_first_vec(&mut self, vector: Vectors) {
        self.addr = vector as u16;
        self.read = true;
        self.access_bus();
        self.latch_u8 = self.data; // Save the value read

        let next_vec = match vector {
            Vectors::NmiLo => Vectors::NmiHi,
            Vectors::ResLo => Vectors::ResHi,
            Vectors::BrkLo => Vectors::BrkHi,
            _ => panic!("Invalid vector given as first!"),
        };

        self.state = State::FetchSecondVec(next_vec);
    }

    fn fetch_second_vec(&mut self, vector: Vectors) {
        self.addr = vector as u16;
        self.read = true;
        self.access_bus();
        self.pc = ((self.data as u16) << 8) | self.latch_u8 as u16;
        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn dummy_read_pc(&mut self) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();

        // May adjust later to accomodate more states
        match self.cur_nmeonic {
            Nmeonic::PHP | Nmeonic::PHA => self.state = State::ImplPush,
            Nmeonic::PLP | Nmeonic::PLA | Nmeonic::RTI => self.state = State::ReadIncS,
            Nmeonic::BRK => self.state = State::PushPcH,
            _ => panic!("Unrecognized nmeonic!"),
        }
    }

    // Boolean parameter from_branch indicates the microprocessor should check status flags to see if there is a branch to be taken.
    // It is set to true if the state machine transitions from FetchOffset to FetchOpcode.
    fn fetch_opcode(&mut self, from_branch: bool) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();

        // Branch is taken if the flags are set accordingly and we are decoding a branch
        let branch_taken = match self.cur_nmeonic {
            Nmeonic::BCC => self.bcc() && from_branch,
            Nmeonic::BCS => self.bcs() && from_branch,
            Nmeonic::BEQ => self.beq() && from_branch,
            Nmeonic::BMI => self.bmi() && from_branch,
            Nmeonic::BNE => self.bne() && from_branch,
            Nmeonic::BPL => self.bpl() && from_branch,
            Nmeonic::BVC => self.bvc() && from_branch,
            Nmeonic::BVS => self.bvs() && from_branch,
            _ => false,
        };

        if branch_taken {
            // If we are here, it means the from_branch flag is set therefore self.latch_u8 contains the branch offset.
            let offset = self.latch_u8 as i8;
            let pch = (self.pc & 0xFF00) >> 8;
            let old_pcl = (self.pc & 0x00FF) as u8;
            let (new_pcl, boundry_crossed) = old_pcl.overflowing_add_signed(offset);

            println!(
                "old: {:#04X} new: {:#04X}, offset {}, offset_hex: {:#04X} latch_u8: {:#04X}",
                old_pcl, new_pcl, offset, offset, self.latch_u8
            );

            if boundry_crossed {
                // todo!("Take another cycle if branch results in page boundry cross");
                self.latch_u16 = pch + new_pcl as u16;
                self.state = State::PageCrossed(offset > 0);
            } else {
                self.pc = pch + new_pcl as u16;
                println!("New program counter no cross: {:#04X}.", self.pc);
                self.state = State::FetchOpcode(NOT_FROM_BRANCH);
            }

            return;
        }

        self.pc = self.pc.wrapping_add(1);
        if let Some(instruction) = LOOKUP[self.data as usize] {
            let (mode, nm) = instruction;
            self.cur_mode = mode;
            self.cur_nmeonic = nm;
        } else {
            todo!("Illegal opcode");
        }

        match self.cur_mode {
            AddressingMode::Implied => match self.cur_nmeonic {
                // These instructions require 2 cycles only.
                Nmeonic::CLC
                | Nmeonic::CLD
                | Nmeonic::CLV
                | Nmeonic::CLI
                | Nmeonic::DEX
                | Nmeonic::DEY
                | Nmeonic::INX
                | Nmeonic::INY
                | Nmeonic::NOP
                | Nmeonic::SEC
                | Nmeonic::SED
                | Nmeonic::SEI
                | Nmeonic::TAX
                | Nmeonic::TAY
                | Nmeonic::TSX
                | Nmeonic::TXA
                | Nmeonic::TXS
                | Nmeonic::TYA => self.state = State::ExecImpl,

                // These instructions are stack operations and require more than 2 cycles.
                Nmeonic::PHA | Nmeonic::PHP | Nmeonic::PLA | Nmeonic::PLP | Nmeonic::RTS => {
                    self.state = State::DummyReadPc
                }

                Nmeonic::BRK => {
                    self.brk();
                    self.state = State::DummyReadPc;
                }
                Nmeonic::RTI => self.state = State::DummyReadPc,
                _ => panic!("Unrecognized nmeonic for implied mode instruction"),
            },
            AddressingMode::Accumulator => self.state = State::ExecAcc,
            AddressingMode::Immediate => self.state = State::ExecImm,
            AddressingMode::Absolute => self.state = State::FetchAbsLo,
            AddressingMode::ZeroPage => self.state = State::FetchZP,
            AddressingMode::Relative => self.state = State::FetchOffset,
            _ => todo!("Implement remaining states"),
        }
    }

    fn exec_impl(&mut self) {
        // Read the new opcode and ignore it
        self.addr = self.pc;
        self.read = true;
        self.access_bus();

        match self.cur_nmeonic {
            Nmeonic::CLC => self.clc(),
            Nmeonic::CLD => self.cld(),
            Nmeonic::CLI => self.cli(),
            Nmeonic::CLV => self.clv(),
            Nmeonic::DEX => self.dex(),
            Nmeonic::DEY => self.dey(),
            Nmeonic::INX => self.inx(),
            Nmeonic::INY => self.iny(),
            Nmeonic::NOP => self.nop(),
            Nmeonic::SEC => self.sec(),
            Nmeonic::SED => self.sed(),
            Nmeonic::SEI => self.sei(),
            Nmeonic::TAX => self.tax(),
            Nmeonic::TAY => self.tay(),
            Nmeonic::TSX => self.tsx(),
            Nmeonic::TXA => self.txa(),
            Nmeonic::TXS => self.txs(),
            Nmeonic::TYA => self.tya(),
            _ => panic!("Unrecognized nmeonic for 2 cycle implied mode!"),
        }

        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
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

        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
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
        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn impl_push(&mut self) {
        match self.cur_nmeonic {
            Nmeonic::PHA => self.pha(),
            Nmeonic::PHP => self.php(),
            _ => panic!("Unrecognized push operation!"),
        }
        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn read_inc_s(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus();
        self.s = self.s.wrapping_add(1);
        self.state = State::ImplPull;
    }

    fn impl_pull(&mut self) {
        match self.cur_nmeonic {
            Nmeonic::PLA => self.pla(),
            Nmeonic::PLP => self.plp(),
            Nmeonic::RTI => self.rti(), // Only pull the p register from stack in this function.
            _ => panic!("Unrecognized pull operation!"),
        }

        if let Nmeonic::RTI = self.cur_nmeonic {
            self.state = State::PullPcL;
        } else {
            self.state = State::FetchOpcode(NOT_FROM_BRANCH);
        }
    }

    fn fetch_abs_lo(&mut self) {
        // Fetch the low byte of the effective address
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.latch_u8 = self.data;
        self.pc = self.pc.wrapping_add(1);

        match self.cur_nmeonic {
            Nmeonic::JSR => self.state = State::JsrDummyStack,
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
                self.state = State::FetchOpcode(NOT_FROM_BRANCH);
            }
            Nmeonic::JSR => {
                self.pc = self.latch_u16;
                self.state = State::FetchOpcode(NOT_FROM_BRANCH);
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
        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn fetch_zp(&mut self) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.latch_u8 = self.data;

        self.pc = self.pc.wrapping_add(1);
        // self.state = State::ExecZP;
        match self.cur_nmeonic {
            Nmeonic::ASL
            | Nmeonic::DEC
            | Nmeonic::INC
            | Nmeonic::LSR
            | Nmeonic::ROL
            | Nmeonic::ROR => {
                self.latch_u16 = self.latch_u8 as u16;
                self.state = State::RmwRead;
            }
            _ => self.state = State::ExecZP,
        }
    }

    fn exec_zp(&mut self) {
        match self.cur_nmeonic {
            Nmeonic::STA => {
                self.latch_u16 = self.latch_u8 as u16;
                self.sta();
            }
            Nmeonic::STX => {
                self.latch_u16 = self.latch_u8 as u16;
                self.stx();
            }
            Nmeonic::STY => {
                self.latch_u16 = self.latch_u8 as u16;
                self.sty();
            }
            Nmeonic::ADC => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.adc()
            }
            Nmeonic::AND => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.and()
            }
            Nmeonic::BIT => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.bit()
            }
            Nmeonic::CMP => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.cmp()
            }
            Nmeonic::CPX => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.cpx()
            }
            Nmeonic::CPY => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.cpy()
            }
            Nmeonic::EOR => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.eor()
            }
            Nmeonic::LDA => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.lda()
            }
            Nmeonic::LDX => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.ldx()
            }
            Nmeonic::LDY => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.ldy()
            }
            Nmeonic::ORA => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.ora()
            }
            Nmeonic::SBC => {
                self.addr = self.latch_u8 as u16;
                self.read = true;
                self.access_bus();
                self.sbc()
            }
            _ => todo!("Remaining zero page mode instructions"),
        }

        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
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
            Nmeonic::LSR => self.lsr(),
            Nmeonic::ROL => self.rol(),
            Nmeonic::ROR => self.ror(),
            _ => unimplemented!(),
        }

        self.data = self.latch_u8;
        self.addr = self.latch_u16;
        self.read = false;
        self.access_bus();
        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn jsr_dummy_stack(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus(); // The value read is discarded
        self.state = State::PushPcH;
    }

    fn push_pch(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        // self.data = ((self.pc & 0xFF00) >> 7) as u8;
        self.data = (self.pc >> 8) as u8;
        self.read = false;
        self.access_bus();
        self.s = self.s.wrapping_sub(1);
        // self.push_stack(((self.pc & 0xFF00) >> 7) as u8);
        self.state = State::PushPcL;
    }

    fn push_pcl(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.data = (self.pc & 0xFF) as u8;
        self.read = false;
        self.access_bus();
        self.s = self.s.wrapping_sub(1);
        // self.push_stack((self.pc & 0xFF) as u8);

        if let Nmeonic::BRK = self.cur_nmeonic {
            self.state = State::PushP;
        } else {
            self.state = State::FetchAbsHi;
        }
    }

    fn pull_pch(&mut self) {
        let lo = self.latch_u8 as u16;
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus(); // Program counter hi in data bus
        self.s = self.s.wrapping_add(1);
        let hi = self.data as u16;

        self.pc = (hi << 8) | lo;

        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn pull_pcl(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus(); // Program counter lo in data bus
        self.s = self.s.wrapping_add(1);
        self.latch_u8 = self.data;
        self.state = State::PullPcH;
    }

    fn push_p(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.data = self.p;
        self.read = false;
        self.access_bus();
        self.s = self.s.wrapping_sub(1);
        self.state = State::FetchFirstVec(Vectors::BrkLo);
    }

    fn fetch_offset(&mut self) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.pc = self.pc.wrapping_add(1);
        self.latch_u8 = self.data;
        self.state = State::FetchOpcode(FROM_BRANCH);
    }

    fn page_crossed(&mut self, page_up: bool) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();
        self.pc = if page_up {
            self.latch_u16 + 0x0100
        } else {
            self.latch_u16 - 0x0100
        };
        println!("New program counter page cross: {:#04X}.", self.pc);
        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
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
        let a_prev = self.a;
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
            ((a_prev ^ result) & (val ^ result) & 0x80) != 0,
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

    fn bcc(&mut self) -> bool {
        !self.flag_set(Flags::Carry)
    }

    fn bcs(&mut self) -> bool {
        self.flag_set(Flags::Carry)
    }

    fn beq(&mut self) -> bool {
        self.flag_set(Flags::Zero)
    }

    fn bit(&mut self) {
        let result = self.a & self.data;

        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Overflow, (self.data & (1 << 6)) != 0);
        self.set_flag(Flags::Negative, (self.data & (1 << 7)) != 0);
    }

    fn bmi(&mut self) -> bool {
        self.flag_set(Flags::Negative)
    }

    fn bne(&mut self) -> bool {
        !self.flag_set(Flags::Zero)
    }

    fn bpl(&mut self) -> bool {
        !self.flag_set(Flags::Negative)
    }

    fn brk(&mut self) {
        // Set the break flag in the status register before pushing it to stack
        self.set_flag(Flags::Break, true);
    }

    fn bvc(&mut self) -> bool {
        !self.flag_set(Flags::Overflow)
    }

    fn bvs(&mut self) -> bool {
        self.flag_set(Flags::Overflow)
    }

    fn clc(&mut self) {
        // Boolean argument set to false to clear the flag
        self.set_flag(Flags::Carry, false);
    }

    fn cld(&mut self) {
        self.set_flag(Flags::Decimal, false);
    }

    fn cli(&mut self) {
        self.set_flag(Flags::InterrputDisable, false);
    }

    fn clv(&mut self) {
        self.set_flag(Flags::Overflow, false);
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
        let result = self.x.wrapping_sub(1);
        self.x = result;

        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
    }

    fn dey(&mut self) {
        let result = self.y.wrapping_sub(1);
        self.y = result;

        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
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
        let result = self.x.wrapping_add(1);
        self.x = result;

        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
    }

    fn iny(&mut self) {
        let result = self.y.wrapping_add(1);
        self.y = result;

        self.set_flag(Flags::Zero, result == 0);
        self.set_flag(Flags::Negative, (result as i8) < 0);
    }

    fn jmp(&mut self) {
        self.pc = self.latch_u16;
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
        // No operation
        return;
    }

    fn ora(&mut self) {
        self.a |= self.data;

        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn pha(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.data = self.a;
        self.read = false;
        self.access_bus();
        self.s = self.s.wrapping_sub(1);
    }

    fn php(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.data = self.p;
        self.read = false;
        self.access_bus();
        self.s = self.s.wrapping_sub(1);
    }

    fn pla(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus();
        self.a = self.data;

        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn plp(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus();
        self.p = self.data;
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
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus(); // P register in data bus
        self.s = self.s.wrapping_add(1);
        self.p = self.data;
    }

    fn rts(&mut self) {
        unimplemented!();
    }

    fn sbc(&mut self) {
        let a_prev = self.a;
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
            ((a_prev ^ result) & (val ^ result) & 0x80) != 0,
        );
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn sec(&mut self) {
        // Boolean parameter set to true to set the flag
        self.set_flag(Flags::Carry, true);
    }

    fn sed(&mut self) {
        self.set_flag(Flags::Decimal, true);
    }

    fn sei(&mut self) {
        self.set_flag(Flags::InterrputDisable, true);
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
        self.x = self.a;

        self.set_flag(Flags::Zero, self.x == 0);
        self.set_flag(Flags::Negative, (self.x as i8) < 0);
    }

    fn tay(&mut self) {
        self.y = self.a;

        self.set_flag(Flags::Zero, self.y == 0);
        self.set_flag(Flags::Negative, (self.y as i8) < 0);
    }

    fn tsx(&mut self) {
        self.x = self.s;

        self.set_flag(Flags::Zero, self.x == 0);
        self.set_flag(Flags::Negative, (self.x as i8) < 0);
    }

    fn txa(&mut self) {
        self.a = self.x;

        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }

    fn txs(&mut self) {
        self.s = self.x;
    }

    fn tya(&mut self) {
        self.a = self.y;

        self.set_flag(Flags::Zero, self.a == 0);
        self.set_flag(Flags::Negative, (self.a as i8) < 0);
    }
}
