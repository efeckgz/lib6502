mod lookup;

use crate::bus::{Bus, BusDevice};
use lookup::{AddressingMode, IndexReg, LOOKUP, Nmeonic};

const STACK_BASE: u16 = 0x0100;

const FROM_BRANCH: bool = true;
const NOT_FROM_BRANCH: bool = false;

// T and N are generic parameters for the Bus. T ensures the devices the cpu drives in the Bus are BusDevice objects. N is the number of devices.
pub struct Cpu<T: BusDevice, const N: usize> {
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
    // pub bus: &'a mut dyn BusDevice, // The bus itself
    pub bus: Bus<T, N>, // The bus itself
    pub addr: u16,      // 16 bit address bus value
    pub data: u8,       // 8 bit data bus value
    pub read: bool,     // bus read/write mode control variable

    // Latches to hold temporary values
    latch_u8: u8,
    latch_u16: u16,
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

// pc, s, a, x, y, p
pub type RegisterState = (u16, u8, u8, u8, u8, u8);

// Inner state of the processor, used in state machine.
#[derive(PartialEq)]
enum State {
    // Reset states
    ResetHold,
    FirstStart,
    SecondStart,
    ThirdStart,
    FourthStart,
    FetchFirstVec(Vectors), // The fields here hold the vector to fetch
    FetchSecondVec(Vectors),

    DummyReadPc(bool), // Read on current pc val, dont increment pc and discard result

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
    FetchAbsLo(IndexReg),
    FetchAbsHi(IndexReg),
    IndexedStoreExtra(bool), // Store instructions in indexed mode take extra cycle to calculate effective address.

    ExecAbs,

    // Zero page states
    FetchZP(IndexReg),
    ExecZP,
    IndexedZPDummy(IndexReg), // Dummy read on base zp address

    // Read-Modify-Write states
    RmwIndexedExtra(bool), // Read from ea. Correct Hi byte if overflow on adding index register to ea.
    RmwRead,               // Read opcode from effective address
    RmwDummyWrite,         // Dummy write the value read to the effective address
    RmwExec,               // Excecute the rmw instruction and write the result back

    // JSR states - JSR is a 6 cycle absolute instruction but it works differently than others.
    JsrDummyStack, // A dummy stack read is done before storing the pc
    PushPcH,
    PushPcL,
    PullPcH,
    PullPcL,

    JmpFetchIAH, // Fetch indirect addr hi for jump indirect
    JmpFetchADL,
    JmpFetchADH,

    PushP, // Push processor status register to the stack

    FetchOffset, // Fetch relative branch offset

    IndirectAddX,             // Add x to the base address low indirect mode
    IndirectFetchLo,          // Fetch lo byte from 00, BAL+X
    IndirectFetchHi,          // Fetch hi byte from 00, BAL+X+1
    ExecIndirect,             // Execute indirect mode instruction
    IndirectYFetchBAL,        // Fetch Base address lo indirect y mode
    IndirectYFetchBAH,        // Fetch Base address hi indirect y mode
    IndirectYDummyRead(bool), // Do a dummy read on BAH, BAL+Y

    // Boolean flag indicates page crossed up.
    // Extra cycle due to page boundry cross.
    PageCrossed(bool),
}

#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq)]
enum Vectors {
    NmiLo = 0xFFFA,
    NmiHi = 0xFFFB,
    ResLo = 0xFFFC,
    ResHi = 0xFFFD,
    BrkLo = 0xFFFE,
    BrkHi = 0xFFFF,
}

impl<T: BusDevice, const N: usize> Cpu<T, N> {
    pub fn new(bus: Bus<T, N>) -> Self {
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

    // Reset back to initial state. Does not reset bus.
    pub fn reset(&mut self) {
        self.pc = 0;
        self.s = 255;
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.p = 0;
        self.state = State::ResetHold;
        self.cur_mode = AddressingMode::None;
        self.cur_nmeonic = Nmeonic::None;
        self.addr = 0;
        self.data = 0;
        self.read = false;
        self.latch_u8 = 0;
        self.latch_u16 = 0;
    }

    pub fn from_register_state((pc, s, a, x, y, p): RegisterState, bus: Bus<T, N>) -> Self {
        Self {
            a,
            x,
            y,
            pc,
            p,
            s,
            state: State::FetchOpcode(NOT_FROM_BRANCH),
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

    pub fn get_state(&self) -> RegisterState {
        (self.pc, self.s, self.a, self.x, self.y, self.p)
    }

    pub fn get_bus_pins(&self) -> (u16, u8, bool) {
        (self.addr, self.data, self.read)
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
            State::DummyReadPc(rts_t5) => self.dummy_read_pc(rts_t5),
            State::ExecImpl => self.exec_impl(),
            State::ExecImm => self.exec_imm(),
            State::ExecAcc => self.exec_acc(),
            State::ImplPush => self.impl_push(),
            State::ReadIncS => self.read_inc_s(),
            State::ImplPull => self.impl_pull(),
            State::FetchAbsLo(index_reg) => self.fetch_abs_lo(index_reg),
            State::FetchAbsHi(index_reg) => self.fetch_abs_hi(index_reg),
            State::IndexedStoreExtra(boundary_crossed) => {
                self.indexed_store_extra(boundary_crossed)
            }
            State::ExecAbs => self.exec_abs(),
            State::FetchZP(index_reg) => self.fetch_zp(index_reg),
            State::ExecZP => self.exec_zp(),
            State::IndexedZPDummy(index_reg) => self.indexed_zp_dummy(index_reg),
            State::RmwIndexedExtra(boundary_crossed) => self.rmw_indexed_extra(boundary_crossed),
            State::RmwRead => self.rmw_read(),
            State::RmwDummyWrite => self.rmw_dummy_write(),
            State::RmwExec => self.rmw_exec(),
            State::JsrDummyStack => self.jsr_dummy_stack(),
            State::PushPcH => self.push_pch(),
            State::PushPcL => self.push_pcl(),
            State::PullPcH => self.pull_pch(),
            State::PullPcL => self.pull_pcl(),
            State::JmpFetchIAH => self.jmp_fetch_iah(),
            State::JmpFetchADL => self.jmp_fetch_adl(),
            State::JmpFetchADH => self.jmp_fetch_adh(),
            State::PushP => self.push_p(),
            State::FetchOffset => self.fetch_offset(),
            State::IndirectAddX => self.indirect_add_x(),
            State::IndirectFetchLo => self.indirect_fetch_lo(),
            State::IndirectFetchHi => self.indirect_fetch_hi(),
            State::ExecIndirect => self.exec_indirect(),
            State::IndirectYFetchBAL => self.indirect_y_fetch_bal(),
            State::IndirectYFetchBAH => self.indirect_y_fetch_bah(),
            State::IndirectYDummyRead(boundary_cross) => self.indirect_y_dummy_read(boundary_cross),
            State::PageCrossed(page_up) => self.page_crossed(page_up),
        }
    }

    // Step the cpu forward 1 instruction
    pub fn instruction_step(&mut self) {
        self.cycle();
        while self.state != State::FetchOpcode(NOT_FROM_BRANCH) {
            self.cycle();
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
        // Brk instruction sets the I flag after pulling the status register from stack.
        // It is unclear exactly on which cycle this happens, and to my knowledge it doesn't matter anyway.
        if let Nmeonic::BRK = self.cur_nmeonic {
            self.set_flag(Flags::InterrputDisable, true);
        }

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

    // RTS instruction makes two dummy pc reads, 1 after fetching the opcode and 1 more after pulling the pc from stack.
    // The second read happens on the sixth cycle, pc pulled from stack is incremented. rts_t5 flag checks this situation.
    fn dummy_read_pc(&mut self, rts_t5: bool) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();

        // May adjust later to accomodate more states
        match self.cur_nmeonic {
            Nmeonic::PHP | Nmeonic::PHA => self.state = State::ImplPush,
            Nmeonic::PLP | Nmeonic::PLA | Nmeonic::RTI => self.state = State::ReadIncS,
            Nmeonic::RTS => {
                self.state = if rts_t5 {
                    self.pc = self.pc.wrapping_add(1);
                    State::FetchOpcode(NOT_FROM_BRANCH)
                } else {
                    State::ReadIncS
                };
            }
            Nmeonic::BRK => {
                // Look here if there is a pc related issue
                self.pc = self.pc.wrapping_add(1);
                self.state = State::PushPcH
            }
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
            let pch = self.pc & 0xFF00;
            let old_pcl = (self.pc & 0x00FF) as u8;
            let (new_pcl, boundry_crossed) = old_pcl.overflowing_add_signed(offset);

            if boundry_crossed {
                self.latch_u16 = pch + new_pcl as u16;
                self.state = State::PageCrossed(offset > 0);
            } else {
                self.pc = pch + new_pcl as u16;
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
                    self.state = State::DummyReadPc(false)
                }

                Nmeonic::BRK => {
                    self.state = State::DummyReadPc(false);
                }
                Nmeonic::RTI => self.state = State::DummyReadPc(false),
                _ => panic!("Unrecognized nmeonic for implied mode instruction"),
            },
            AddressingMode::Accumulator => self.state = State::ExecAcc,
            AddressingMode::Immediate => self.state = State::ExecImm,
            AddressingMode::Absolute(index_reg) => self.state = State::FetchAbsLo(index_reg),
            AddressingMode::ZeroPage(index_reg) => self.state = State::FetchZP(index_reg),
            AddressingMode::Indirect(index_reg) => self.state = State::FetchZP(index_reg), // Fetch a zero page address like normal
            AddressingMode::Relative => self.state = State::FetchOffset,
            _ => todo!("Unimplemented addressing mode!"),
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
        self.latch_u8 = self.data;
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
            Nmeonic::SBC => {
                self.latch_u8 ^= 0xFF;
                self.adc();
            }
            _ => panic!("Unrecognized opcode-addressing mode-nmeonic combination!"),
        }

        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn exec_acc(&mut self) {
        // Perform a dummy bus access. No actual value is read from or written to the bus.
        // But the 6502 performs a bus access at each cycle - even if it is useless.
        self.addr = self.pc;
        self.read = true;
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

        if let Nmeonic::RTS = self.cur_nmeonic {
            self.state = State::PullPcL;
        } else {
            self.state = State::ImplPull;
        }
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

    fn fetch_abs_lo(&mut self, index_reg: IndexReg) {
        // Fetch the low byte of the effective address
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.latch_u8 = self.data;
        self.pc = self.pc.wrapping_add(1);

        match self.cur_nmeonic {
            Nmeonic::JSR => self.state = State::JsrDummyStack,
            _ => self.state = State::FetchAbsHi(index_reg),
        }
    }

    fn fetch_abs_hi(&mut self, index_reg: IndexReg) {
        // Determine the index register to add, or add 0 for non-indexed.
        let ir = match index_reg {
            IndexReg::X => self.x,
            IndexReg::Y => self.y,
            IndexReg::None => 0,
        };

        // Fetch the high byte of the effective address
        self.addr = self.pc;
        self.read = true;
        self.access_bus();
        self.pc = self.pc.wrapping_add(1);

        let (lo, boundary_crossed) = self.latch_u8.overflowing_add(ir);
        let hi = self.data as u16;
        self.latch_u16 = (hi << 8) | (lo as u16);

        match self.cur_nmeonic {
            // JMP and JSR do not have indexed absolute mode.
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
            // These instructions take 6 cycles in non indexed mode, 7 in indexed mode.
            Nmeonic::ASL
            | Nmeonic::DEC
            | Nmeonic::INC
            | Nmeonic::LSR
            | Nmeonic::ROL
            | Nmeonic::ROR => {
                if let IndexReg::None = index_reg {
                    self.state = State::RmwRead; // Not in indexed mode
                } else {
                    self.state = State::RmwIndexedExtra(boundary_crossed)
                }
            }
            Nmeonic::STA | Nmeonic::STX | Nmeonic::STY => {
                if let IndexReg::None = index_reg {
                    self.state = State::ExecAbs;
                } else {
                    self.state = State::IndexedStoreExtra(boundary_crossed);
                }
            }
            _ => {
                self.state = if boundary_crossed {
                    State::PageCrossed(true) // Pass true to go page up
                } else {
                    State::ExecAbs
                };
            }
        }
    }

    fn indexed_store_extra(&mut self, boundary_crossed: bool) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();

        if boundary_crossed {
            self.latch_u16 = self.latch_u16.wrapping_add(0x0100);
        }

        self.state = State::ExecAbs; // Only store instructions will run in the match arm for exec_abs
    }

    fn exec_abs(&mut self) {
        match self.cur_nmeonic {
            Nmeonic::ADC => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.latch_u8 = self.data;
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
                self.latch_u8 = self.data ^ 0xFF;
                self.adc();
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

    fn fetch_zp(&mut self, index_reg: IndexReg) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus();

        self.latch_u8 = self.data;
        self.latch_u16 = self.data as u16;

        self.pc = self.pc.wrapping_add(1);

        if let AddressingMode::Indirect(ir) = self.cur_mode {
            match ir {
                IndexReg::X => self.state = State::IndirectAddX,
                IndexReg::Y => self.state = State::IndirectYFetchBAL,
                IndexReg::None => self.state = State::JmpFetchIAH,
                // _ => todo!("Complete whatever this will be"),
            }
            return;
        }

        match self.cur_nmeonic {
            Nmeonic::ASL
            | Nmeonic::DEC
            | Nmeonic::INC
            | Nmeonic::LSR
            | Nmeonic::ROL
            | Nmeonic::ROR => {
                if let IndexReg::None = index_reg {
                    self.state = State::RmwRead;
                } else {
                    self.state = State::IndexedZPDummy(index_reg)
                }
            }
            _ => {
                if let IndexReg::None = index_reg {
                    self.state = State::ExecZP;
                } else {
                    self.state = State::IndexedZPDummy(index_reg)
                }
            }
        }
    }

    fn exec_zp(&mut self) {
        match self.cur_nmeonic {
            Nmeonic::STA => self.sta(),
            Nmeonic::STX => self.stx(),
            Nmeonic::STY => self.sty(),
            Nmeonic::ADC => {
                self.addr = self.latch_u16;
                self.read = true;
                self.access_bus();
                self.latch_u8 = self.data;
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
                self.latch_u8 = self.data ^ 0xFF;
                self.adc();
            }
            _ => todo!("Remaining zero page mode instructions"),
        }

        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn indexed_zp_dummy(&mut self, index_reg: IndexReg) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();

        let irval = match index_reg {
            IndexReg::X => self.x,
            IndexReg::Y => self.y,
            IndexReg::None => 0,
        };

        // Address always stays in page zero.
        // self.latch_u16 = self.latch_u16.wrapping_add(irval as u16);
        self.latch_u8 = self.latch_u8.wrapping_add(irval);
        self.latch_u16 = self.latch_u8 as u16;

        self.state = match self.cur_nmeonic {
            Nmeonic::ASL
            | Nmeonic::DEC
            | Nmeonic::INC
            | Nmeonic::LSR
            | Nmeonic::ROL
            | Nmeonic::ROR => State::RmwRead,
            _ => State::ExecZP,
        };
    }

    fn rmw_indexed_extra(&mut self, boundary_crossed: bool) {
        // At this point self.latch_u16 holds BAH, BAL + ir
        // Read on this address, then correct the hi byte before continuing on regular RMW.
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();

        if boundary_crossed {
            self.latch_u16 = self.latch_u16.wrapping_add(0x0100);
        }

        self.state = State::RmwRead;
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
        self.data = (self.pc >> 8) as u8;
        self.read = false;
        self.access_bus();
        self.s = self.s.wrapping_sub(1);
        self.state = State::PushPcL;
    }

    fn push_pcl(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.data = (self.pc & 0xFF) as u8;
        self.read = false;
        self.access_bus();
        self.s = self.s.wrapping_sub(1);

        if let Nmeonic::BRK = self.cur_nmeonic {
            self.state = State::PushP;
        } else {
            self.state = State::FetchAbsHi(IndexReg::None);
        }
    }

    fn pull_pch(&mut self) {
        let lo = self.latch_u8 as u16;
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus(); // Program counter hi in data bus
        let hi = self.data as u16;

        self.pc = (hi << 8) | lo;

        if let Nmeonic::RTS = self.cur_nmeonic {
            self.state = State::DummyReadPc(true);
        } else {
            self.state = State::FetchOpcode(NOT_FROM_BRANCH);
        }
    }

    fn pull_pcl(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.read = true;
        self.access_bus(); // Program counter lo in data bus
        self.s = self.s.wrapping_add(1);
        self.latch_u8 = self.data;
        self.state = State::PullPcH;
    }

    fn jmp_fetch_iah(&mut self) {
        self.addr = self.pc;
        self.read = true;
        self.access_bus(); // hi in self.data

        self.pc = self.pc.wrapping_add(1);

        let lo = self.latch_u8 as u16;
        let hi = self.data as u16;
        self.latch_u16 = (hi << 8) | lo;

        self.state = State::JmpFetchADL;
    }

    fn jmp_fetch_adl(&mut self) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus(); // adl in self.data

        let hi = self.latch_u16 & 0xFF00;
        let mut lo = (self.latch_u16 & 0x00FF) as u8;

        lo = lo.wrapping_add(1);
        self.latch_u16 = hi | (lo as u16);

        self.latch_u8 = self.data; // adl in self.latch_u8
        self.state = State::JmpFetchADH;
    }

    fn jmp_fetch_adh(&mut self) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus(); // adh in self.data

        let lo = self.latch_u8 as u16;
        let hi = self.data as u16;
        self.latch_u16 = (hi << 8) | lo;

        // self.state = State::ExecAbs;
        self.pc = self.latch_u16;
        self.state = State::FetchOpcode(NOT_FROM_BRANCH);
    }

    fn push_p(&mut self) {
        self.addr = STACK_BASE + self.s as u16;
        self.data = self.p | 0x10; // Set the B flag when pushing p
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

    fn indirect_add_x(&mut self) {
        // Add X register to base address lo, dont cross page boundry.
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();

        self.latch_u8 = self.latch_u8.wrapping_add(self.x);
        self.latch_u16 = self.latch_u8 as u16;

        self.state = State::IndirectFetchLo;
    }

    fn indirect_fetch_lo(&mut self) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();

        // To not cross page boundry when advancing the address, wrapping increment the lo byte and cast to bigger type
        // to get the hi byte as 0.
        self.latch_u8 = self.latch_u8.wrapping_add(1);
        self.latch_u16 = self.latch_u8 as u16;

        self.latch_u8 = self.data; // Save the lo byte in latch
        self.state = State::IndirectFetchHi;
    }

    fn indirect_fetch_hi(&mut self) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();

        let lo = self.latch_u8 as u16;
        let hi = self.data as u16;
        self.latch_u16 = (hi << 8) | lo;

        self.state = State::ExecIndirect;
    }

    fn exec_indirect(&mut self) {
        if let Nmeonic::STA = self.cur_nmeonic {
            self.sta();
        } else {
            self.addr = self.latch_u16;
            self.read = true;
            self.access_bus();

            self.latch_u8 = self.data;

            match self.cur_nmeonic {
                Nmeonic::ADC => self.adc(),
                Nmeonic::AND => self.and(),
                Nmeonic::CMP => self.cmp(),
                Nmeonic::EOR => self.eor(),
                Nmeonic::LDA => self.lda(),
                Nmeonic::ORA => self.ora(),
                Nmeonic::SBC => {
                    self.latch_u8 = self.data ^ 0xFF;
                    self.adc();
                }
                _ => panic!("Unrecognized nmeonic for indirectX"),
            }
        }
        self.state = State::FetchOpcode(NOT_FROM_BRANCH)
    }

    fn indirect_y_fetch_bal(&mut self) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus(); // bal in self.data

        self.latch_u8 = self.latch_u8.wrapping_add(1);
        self.latch_u16 = self.latch_u8 as u16;

        self.latch_u8 = self.data; // bal in latch
        self.state = State::IndirectYFetchBAH;
    }

    fn indirect_y_fetch_bah(&mut self) {
        self.addr = self.latch_u16; // bah in self.data
        self.read = true;
        self.access_bus();

        let (lo, boundary_cross) = self.latch_u8.overflowing_add(self.y);
        let hi = self.data as u16;

        self.latch_u16 = (hi << 8) | (lo as u16);

        self.state = if let Nmeonic::STA = self.cur_nmeonic {
            State::IndirectYDummyRead(boundary_cross)
        } else if boundary_cross {
            State::PageCrossed(true) // Pass true to go page up
        } else {
            State::ExecIndirect
        };
    }

    fn indirect_y_dummy_read(&mut self, boundary_crossed: bool) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();

        if boundary_crossed {
            self.latch_u16 = self.latch_u16.wrapping_add(0x0100);
        }

        self.state = State::ExecIndirect;
    }

    fn page_crossed(&mut self, page_up: bool) {
        self.addr = self.latch_u16;
        self.read = true;
        self.access_bus();

        self.latch_u16 = if page_up {
            self.latch_u16.wrapping_add(0x0100)
        } else {
            self.latch_u16.wrapping_sub(0x0100)
        };

        self.state = match self.cur_mode {
            AddressingMode::Relative => {
                self.pc = self.latch_u16;
                State::FetchOpcode(NOT_FROM_BRANCH)
            }
            AddressingMode::Indirect(_) => State::ExecIndirect,
            _ => State::ExecAbs,
        };
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
        let val = self.latch_u8;

        let (mut res, mut of) = a_prev.overflowing_add(val);
        if self.flag_set(Flags::Carry) {
            let (fin, of2) = res.overflowing_add(1);
            res = fin;
            of = of || of2;
        }

        self.a = res;

        self.set_flag(Flags::Carry, of);
        self.set_flag(Flags::Zero, res == 0);
        self.set_flag(Flags::Negative, (res & 0x80) != 0);
        self.set_flag(Flags::Overflow, ((a_prev ^ res) & (val ^ res) & 0x80) != 0);
    }

    fn and(&mut self) {
        self.a &= self.data;

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
            AddressingMode::Absolute(_) | AddressingMode::ZeroPage(_) => {
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
            AddressingMode::Absolute(_) | AddressingMode::ZeroPage(_) => {
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
        // No operation.
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
        self.data = self.p | 0x10;
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
        self.p = (self.data & 0xCF) | 0x20; // Ignore the B flag and bit 5
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
            AddressingMode::Absolute(_) | AddressingMode::ZeroPage(_) => {
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
            AddressingMode::Absolute(_) | AddressingMode::ZeroPage(_) => {
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
        self.p = (self.data & 0xCF) | 0x20;
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
