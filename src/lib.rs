#![no_std]

pub mod bus;
pub mod cpu;

use bus::BusDevice;

pub struct Memory {
    bytes: [u8; 65536],
}

impl BusDevice for Memory {
    fn read(&mut self, addr: u16) -> u8 {
        self.bytes[addr as usize]
    }

    fn write(&mut self, addr: u16, data: u8) {
        self.bytes[addr as usize] = data;
    }
}

impl Memory {
    pub fn new() -> Self {
        Self { bytes: [0; 65536] }
    }

    pub fn load_program(&mut self, program: &[u8]) {
        for (i, byte) in program.iter().enumerate() {
            self.bytes[i] = *byte;
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::cpu::Flags;

    use super::*;

    use bus::Bus;
    use cpu::Cpu;

    #[test]
    fn it_works() {
        let mut bus: Bus<1> = Bus::new();
        let mut memory = Memory::new();

        // LDA #10
        memory.bytes[0] = 0xA9; // LDA immediate
        memory.bytes[1] = 0x0A; // 10 in hex
        memory.bytes[2] = 0xA9; // LDA immediate
        memory.bytes[3] = 0x00; // 0
        memory.bytes[4] = 0xA9;
        memory.bytes[5] = 0x08;

        bus.map_device(0x0000, 0xFFFF, &mut memory).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        // Run for 2 cycles
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x0A);
        assert!(!cpu.flag_set(cpu::Flags::Zero));

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x00);
        assert!(cpu.flag_set(cpu::Flags::Zero));

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x08);
        assert!(!cpu.flag_set(cpu::Flags::Zero));
    }

    #[test]
    fn lda_imm_abs() {
        // Check the lda instruction with immediate and absolute addressing modes.
        let mut bus: Bus<1> = Bus::new();
        let mut mem = Memory::new();

        // LDA 0x17 immediate
        mem.bytes[0] = 0xA9;
        mem.bytes[1] = 0x17;

        // LDA 0x11 absolute
        // value 0x11 will be at location 0x1234
        mem.bytes[2] = 0xAD; // Absolute addressing lda
        mem.bytes[3] = 0x34; // Lo byte of 0x1234
        mem.bytes[4] = 0x12; // Hi byte of 0x1234
        mem.bytes[0x1234] = 0x11; // Place the operand in the effective address

        bus.map_device(0x0000, 0xFFFF, &mut mem).unwrap();
        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        // Run 2 cycles for lda immediate
        cpu.cycle();
        assert_eq!(cpu.data, 0xA9);
        assert_eq!(cpu.addr, 0);

        cpu.cycle();
        assert_eq!(cpu.data, 0x17);
        assert_eq!(cpu.addr, 1);

        assert_eq!(cpu.a, 0x17);

        // Run 4 cycles for lda absolute
        cpu.cycle();
        assert_eq!(cpu.data, 0xAD);
        assert_eq!(cpu.addr, 2);

        cpu.cycle();
        assert_eq!(cpu.data, 0x34);
        assert_eq!(cpu.addr, 3);

        cpu.cycle();
        assert_eq!(cpu.data, 0x12);
        assert_eq!(cpu.addr, 4);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x1234);
        assert_eq!(cpu.data, 0x11);

        assert_eq!(cpu.a, 0x11);
    }

    #[test]
    fn adc_imm_abs() {
        let mut bus: Bus<1> = Bus::new();
        let mut mem = Memory::new();

        // When the program runs the accumulator will be added 10 twice, a should be 20.
        let program = [
            0x69, // ADC immediate
            0x0A, // Immediate value 10
            0x6D, // ADC absolute
            0x05, // ea lo byte
            0x00, // ea hi byte
            0x0A, // Value at ea is also 10
        ];

        mem.load_program(&program);

        bus.map_device(0x0000, 0xFFFF, &mut mem).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        let mut cycles = 0;
        while cycles < 6 {
            cpu.cycle();
            cycles += 1;
        }

        assert_eq!(cpu.a, 20);
    }

    #[test]
    fn load_shift_store() {
        let mut bus: Bus<1> = Bus::new();
        let mut mem = Memory::new();

        let mut program: [u8; 65536] = [0; 65536];

        // Test 1: Load A 0x40, shift left
        // A becomes 0x80, carry 0, zero 0, negative 1
        // Should take 4 cycles
        program[0] = 0xA9;
        program[1] = 0x40;
        program[2] = 0x0A;

        // Test 2: Store A at 0x1234, shift left at 0x1234, Load A at 0x1234
        // A becomes 0x00, carry 1, zero 1, negative 0
        // 8d 34 12 0e 34 12 ad 34 12
        // Should take 14 cycles
        program[3] = 0x8D;
        program[4] = 0x34;
        program[5] = 0x12;
        program[6] = 0x0E;
        program[7] = 0x34;
        program[8] = 0x12;
        program[9] = 0xAD;
        program[10] = 0x34;
        program[11] = 0x12;

        mem.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut mem).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x80);
        assert!(!cpu.flag_set(cpu::Flags::Carry));
        assert!(!cpu.flag_set(cpu::Flags::Zero));
        assert!(cpu.flag_set(cpu::Flags::Negative));

        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x00);
    }

    #[test]
    fn increment_decrement_abs() {
        let mut bus: Bus<1> = Bus::new();
        let mut mem = Memory::new();

        let program = [
            // Test 1: Increment
            // Load the A register with immediate value 0x42
            // Store the A register at 0x1234
            // Increment memory at address 0x1234
            // Load the A register from address 0x1234
            // A should hold the value 0x42 + 1 = 0x43
            0xA9, 0x42, // lda #$42
            0x8D, 0x34, 0x12, // sta $1234
            0xEE, 0x34, 0x12, // inc $1234
            0xAD, 0x34, 0x12, // lda $1234
            // Test 2:
            // Decrement memory at 0x1234 - Address now contains 0x43 - 1 = 0x42
            // Load the A register from 0x1234
            0xCE, 0x34, 0x12, // dec $1234
            0xAD, 0x34, 0x12, // lda $1234
        ];

        mem.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut mem).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        let mut cycles = 0;
        while cycles < 16 {
            cpu.cycle();
            cycles += 1;
        }

        assert_eq!(cpu.a, 0x43);

        cycles = 0;
        while cycles < 10 {
            cpu.cycle();
            cycles += 1;
        }

        assert_eq!(cpu.a, 0x42);
    }

    #[test]
    fn test_reset() {
        // Start the cpu from the reset state and load the accumulator a value.
        // Accumulator will hold the value after 9 cycles - 7 for reset sequence and 2 for lda.
        let mut bus: Bus<1> = Bus::new();
        let mut ram = Memory::new();

        let program = [0xA9, 0x42];

        ram.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x42);
    }

    #[test]
    fn test_init_pc() {
        // This test loads the initialization vector a program counter value,
        // and tests if the program execution starts from the correct memory address.
        let mut bus: Bus<1> = Bus::new();
        let mut ram = Memory::new();

        let mut program = [0_u8; 65536];

        // Load the initialization vector the initial pc value 0x0600
        program[0xFFFC] = 0x00;
        program[0xFFFD] = 0x06;

        // Load the program from 0x0600 - lda #42
        program[0x0600] = 0xA9;
        program[0x0601] = 0x42;

        ram.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        // The correct pc value is loaded
        assert_eq!(cpu.pc, 0x0600);

        cpu.cycle();
        cpu.cycle();

        // Program executed successfully
        assert_eq!(cpu.a, 0x42);
    }

    #[test]
    fn jsr_works() {
        // Start at 0x0600, jump to a subroutine at 0x1234
        let mut bus: Bus<1> = Bus::new();
        let mut ram = Memory::new();
        let mut program = [0_u8; 65536];

        program[0xFFFC] = 0x00;
        program[0xFFFD] = 0x06;

        // lda #42 to test startup
        program[0x0600] = 0xA9;
        program[0x0601] = 0x42;

        // jsr $1234
        program[0x0602] = 0x20;
        program[0x0603] = 0x34;
        program[0x0604] = 0x12;

        // lda #15 on the subroutine address
        program[0x1234] = 0xA9;
        program[0x1235] = 0x15;

        ram.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        assert_eq!(cpu.pc, 0x0600);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x42);

        // 6 cycles for jsr
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.pc, 0x1234); // New pc is set
        assert_eq!(cpu.s, 0xFD); // Stack decremented 2 bytes

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x15);
    }

    #[test]
    fn implied_tests() {
        // This test tests various 1 byte 2 cycle implied addressing mode instructions.
        let mut bus: Bus<1> = Bus::new();
        let mut ram = Memory::new();

        let program = [
            // Program to set the overflow flag
            0xA9, 0x50, // lda #$50
            0x69, 0x50, // adc #$50
            0xB8, // clv
            0x38, // sec
            0x18, // clc
            0xF8, // sed
            0xD8, // cld
            0x78, // sei
            0x58, // cli
            // Load values to index registers to test increment/decrement
            0xA2, 0x05, // ldx #$05
            0xA0, 0x05, // ldy #$05
            0xE8, // inx
            0xC8, // iny
            0xCA, // dex
            0x88, // dey
            // Load accumulator to test accumulator transfer instructions.
            0xA9, 0x42, // lda #$42
            0xAA, // tax
            0xA8, // tay
            // Load index registers to test transfer instructions.
            0xA2, 0x05, // ldx #$05
            0xA0, 0x06, // ldy #$06
            0x8A, // txa
            0x98, // tya
            0xBA, // tsx
            0xA2, 0x05, // ldx #$05
            0x9A, // txs
        ];

        ram.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x50);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0xA0);
        assert!(cpu.flag_set(cpu::Flags::Overflow));

        cpu.cycle();
        cpu.cycle();

        assert!(!cpu.flag_set(cpu::Flags::Overflow));

        cpu.cycle();
        cpu.cycle();

        assert!(cpu.flag_set(cpu::Flags::Carry));

        cpu.cycle();
        cpu.cycle();

        assert!(!cpu.flag_set(cpu::Flags::Carry));

        cpu.cycle();
        cpu.cycle();

        assert!(cpu.flag_set(cpu::Flags::Decimal));

        cpu.cycle();
        cpu.cycle();

        assert!(!cpu.flag_set(cpu::Flags::Decimal));

        cpu.cycle();
        cpu.cycle();

        assert!(cpu.flag_set(cpu::Flags::InterrputDisable));

        cpu.cycle();
        cpu.cycle();

        assert!(!cpu.flag_set(cpu::Flags::InterrputDisable));

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 5);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.y, 5);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 6);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.y, 6);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 5);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.y, 5);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x42);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 0x42);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.y, 0x42);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 5);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.y, 6);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 5);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 6);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 0xFF);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 5);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.s, 5);
    }

    #[test]
    fn implied_stack() {
        // Test the implied mode stack operations
        let mut bus: Bus<1> = Bus::new();
        let mut ram = Memory::new();

        let program = [
            0xA9, 0x42, // lda #$42
            0x48, // pha
            // To test php, will load 0x80 to A to set neagtive flag which is bit 7 of p.
            // p will become 0x80, so 0x80 should be pushed.
            0xA9, 0x80, // lda #$80
            0x08, // php
            // Stack has 0x80, pla would set A 0x80
            0x68, // pla
            // plp, p would have 0x42
            0x28,
        ];

        ram.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x42);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.pc, 3);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x01FF);
        assert_eq!(cpu.data, 0x42);
        assert_eq!(cpu.pc, 3);
        assert_eq!(cpu.s, 0xFE);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x80);
        assert!(cpu.flag_set(cpu::Flags::Negative));
        assert_eq!(cpu.p, 0x80);

        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.addr, 0x01FE);
        assert_eq!(cpu.data, 0x80);
        assert_eq!(cpu.s, 0xFD);

        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x80);
        assert_eq!(cpu.s, 0xFE);

        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.p, 0x42);
        assert_eq!(cpu.s, 0xFF);
    }

    #[test]
    fn brk_rti_works() {
        // Load A, verify that it works.
        // brk, interrupt routine Loads A something else, verify it works.
        let mut bus: Bus<1> = Bus::new();
        let mut ram = Memory::new();
        let mut program = [0_u8; 65536];

        // Reset vectors
        program[0xFFFC] = 0x00;
        program[0xFFFD] = 0x06;

        // brk vectors
        program[0xFFFE] = 0x00;
        program[0xFFFF] = 0x07;

        // lda #$42
        program[0x0600] = 0xA9;
        program[0x0601] = 0x42;

        // brk
        program[0x0602] = 0x00;

        // lda #$50
        program[0x0603] = 0xA9;
        program[0x0604] = 0x50;

        // lda #$80
        program[0x0700] = 0xA9;
        program[0x0701] = 0x80;

        // rti
        program[0x0702] = 0x40;

        ram.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        assert_eq!(cpu.pc, 0x0600);

        // lda #$42
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x42);
        assert_eq!(cpu.pc, 0x0602);
        assert_eq!(cpu.p, 0); // No flags set

        // brk
        // Fetch the opcode
        cpu.cycle();
        assert_eq!(cpu.addr, 0x0602);
        assert_eq!(cpu.data, 0x00);
        assert_eq!(cpu.pc, 0x0603);
        assert!(cpu.flag_set(Flags::Break)); // Is the break flag supposed to be set this cycle?

        let p_before = cpu.p;

        // Read pc, dont inc, discard data
        cpu.cycle();
        assert_eq!(cpu.addr, 0x0603);
        assert_eq!(cpu.data, 0xA9);
        assert_eq!(cpu.pc, 0x0603);

        // Push pc hi to the stack
        cpu.cycle();
        assert_eq!(cpu.addr, 0x01FF); // Address bus should containg s + 1, s is decremented this cycle
        assert_eq!(cpu.data, 0x06);
        assert!(!cpu.read);
        assert_eq!(cpu.pc, 0x0603);
        assert_eq!(cpu.s, 0xFE); // s decremented

        // Push pc lo to the stack
        cpu.cycle();
        assert_eq!(cpu.addr, 0x01FE);
        assert_eq!(cpu.data, 0x03);
        assert!(!cpu.read);
        assert_eq!(cpu.pc, 0x0603);
        assert_eq!(cpu.s, 0xFD);

        // Push status register to the stack
        cpu.cycle();
        assert_eq!(cpu.addr, 0x01FD);
        assert_eq!(cpu.data, cpu.p);
        assert!(!cpu.read);
        assert_eq!(cpu.pc, 0x0603);
        assert_eq!(cpu.s, 0xFC);

        // Fetch low byte of interrupt vector from FFFE
        cpu.cycle();
        assert_eq!(cpu.addr, 0xFFFE);
        assert_eq!(cpu.data, 0x00);
        assert!(cpu.read);
        assert_eq!(cpu.pc, 0x0603);

        // Fetch high byte of interrupt vector from FFFF
        cpu.cycle();
        assert_eq!(cpu.addr, 0xFFFF);
        assert_eq!(cpu.data, 0x07);
        assert!(cpu.read);
        assert_eq!(cpu.pc, 0x0700);

        // lda #$80
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x80);
        assert!(cpu.flag_set(Flags::Negative));
        assert_eq!(cpu.pc, 0x0702);
        assert_ne!(cpu.p, 0); // status register changed

        // rti
        // Fetch opcode
        cpu.cycle();
        assert_eq!(cpu.addr, 0x0702);
        assert_eq!(cpu.data, 0x40);
        assert_eq!(cpu.pc, 0x0703);

        // Read and discard pc, dont increment
        cpu.cycle();
        assert_eq!(cpu.addr, 0x0703);
        assert_eq!(cpu.data, 0x00);
        assert_eq!(cpu.pc, 0x0703); // pc is not incremented in this cycle

        // Read and discard stack, inc s
        cpu.cycle();
        assert_eq!(cpu.addr, 0x01FC);
        assert_eq!(cpu.data, 0x00);
        assert_eq!(cpu.s, 0xFD);

        // Pull p from stack
        cpu.cycle();
        assert_eq!(cpu.addr, 0x01FD);
        assert!(cpu.read);
        assert_eq!(cpu.data, p_before);
        assert_eq!(cpu.s, 0xFE);

        // Pull pcl from stack
        cpu.cycle();
        assert_eq!(cpu.addr, 0x01FE);
        assert!(cpu.read);
        assert_eq!(cpu.data, 0x03);
        assert_eq!(cpu.s, 0xFF);
        assert_eq!(cpu.pc, 0x0703);

        // Pull pchi from stack
        cpu.cycle();
        assert_eq!(cpu.addr, 0x01FF);
        assert!(cpu.read);
        assert_eq!(cpu.data, 0x06);
        assert_eq!(cpu.s, 0x00); // Stack underflow?
        assert_eq!(cpu.pc, 0x0603);
        assert_eq!(cpu.p, p_before); // Status register pulled

        // lda #$50
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x50);
    }

    #[test]
    fn zp_ops() {
        let mut bus: Bus<1> = Bus::new();
        let mut ram = Memory::new();
        let mut program = [0x00_u8; 65536]; // nop padding

        // lda $FF
        program[0] = 0xA5;
        program[1] = 0xFF;

        // ldx $FF
        program[2] = 0xA6;
        program[3] = 0xFF;

        // ldy $FF
        program[4] = 0xA4;
        program[5] = 0xFF;

        // inx
        program[6] = 0xE8;

        // stx $AA
        program[7] = 0x86;
        program[8] = 0xAA;

        // lda $AA
        program[9] = 0xA5;
        program[10] = 0xAA;

        // asl $FF
        program[11] = 0x06;
        program[12] = 0xFF;

        // lda $FF
        program[13] = 0xA5;
        program[14] = 0xFF;

        program[0x00FF] = 0x42;

        ram.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();

        assert_eq!(cpu.pc, 0x00);

        // lda $FF
        cpu.cycle();
        assert_eq!(cpu.addr, 0);
        assert_eq!(cpu.data, 0xA5);
        assert!(cpu.read);

        cpu.cycle();
        assert_eq!(cpu.addr, 1);
        assert_eq!(cpu.data, 0xFF);
        assert!(cpu.read);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x00FF);
        assert_eq!(cpu.data, 0x42);
        assert!(cpu.read);
        assert_eq!(cpu.a, 0x42);

        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 0x42);

        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.y, 0x42);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.x, 0x43);

        // stx $AA
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        // lda $AA
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x43);

        // asl $FF
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        // lda $FF
        cpu.cycle();
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x84);
    }

    #[test]
    fn test_branch() {
        let mut bus: Bus<1> = Bus::new();
        let mut ram = Memory::new();
        let mut program = [0_u8; 65536];

        // lda #00
        program[0x0000] = 0xA9;
        program[0x0001] = 0x00;

        // bne $10 - should not take the branch
        program[0x0002] = 0xD0;
        program[0x0003] = 0x10;

        // lda #$80 - set the negative flag
        program[0x0004] = 0xA9;
        program[0x0005] = 0x80;

        // bmi $10 - sould run for 3 cycles to take the branch
        program[0x0006] = 0x30;
        program[0x0007] = 0x10;

        // lda #10 - should not run
        program[0x0008] = 0xA9;
        program[0x0009] = 0xA0;

        // lda #$42 - should run
        program[0x0018] = 0xA9;
        program[0x0019] = 0x42;

        // lda #0 - set the zero flag
        program[0x001A] = 0xA9;
        program[0x001B] = 0x00;

        // beq $FF - take the branch in 4 cycles for boundry cross
        program[0x001C] = 0xF0;
        program[0x001D] = 0xFF;

        // lda #$17 - should run
        program[0x011D] = 0xA9;
        program[0x011E] = 0x17;

        ram.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

        let mut cpu = Cpu::new(&mut bus);
        cpu.start_sequence();
        assert_eq!(cpu.pc, 0);

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0);
        assert!(cpu.flag_set(Flags::Zero));

        // Branch not taken
        cpu.cycle();
        cpu.cycle();

        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x80);
        assert!(cpu.flag_set(Flags::Negative));

        // Branch taken without page cross - 3 cycles
        cpu.cycle();
        assert_eq!(cpu.addr, 0x0006);
        assert_eq!(cpu.data, 0x30);
        assert_eq!(cpu.pc, 0x0007);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x0007);
        assert_eq!(cpu.data, 0x10);
        assert_eq!(cpu.pc, 0x0008);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x0008);

        // lda #$42
        cpu.cycle();
        assert_eq!(cpu.addr, 0x0018);
        assert_eq!(cpu.data, 0xA9);
        assert_eq!(cpu.pc, 0x0019);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x0019);
        assert_eq!(cpu.data, 0x42);
        assert_eq!(cpu.pc, 0x001A);
        assert_eq!(cpu.a, 0x42);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x001A);
        assert_eq!(cpu.data, 0xA9);
        assert_eq!(cpu.pc, 0x001B);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x001B);
        assert_eq!(cpu.data, 0x00);
        assert_eq!(cpu.pc, 0x001C);
        assert_eq!(cpu.a, 0x00);
        assert!(cpu.flag_set(Flags::Zero));

        // beq $FF
        cpu.cycle();
        assert_eq!(cpu.addr, 0x001C);
        assert_eq!(cpu.data, 0xF0);
        assert_eq!(cpu.pc, 0x001D);

        cpu.cycle();
        assert_eq!(cpu.addr, 0x001D);
        assert_eq!(cpu.data, 0xFF);
        assert_eq!(cpu.pc, 0x001E);

        cpu.cycle();
    }
}
