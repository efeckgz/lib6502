#![no_std]

use bus::{Bus, BusDevice};
use cpu::Cpu;

mod bus;
mod cpu;

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
    use super::*;

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

        bus.map_device(0x00, 0xFF, &mut memory).unwrap();

        let mut cpu = Cpu::new(&mut bus);

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

        let mut cycles = 0;
        while cycles < 6 {
            cpu.cycle();
            cycles += 1;
        }

        assert_eq!(cpu.a, 20);
    }

    #[test]
    fn asl_works() {
        let mut bus: Bus<1> = Bus::new();
        let mut mem = Memory::new();

        let mut program: [u8; 65536] = [0; 65536];

        // Test 1:
        // Load the accumulator 0x40 using absolute lda instruction.
        // Shift it left - should become 0x80
        // carry and zero unset, negative set
        program[0] = 0xAD; // Lda absolute
        program[1] = 0x34; // Lo byte
        program[2] = 0x12; // Hi byte
        program[0x1234] = 0x40; // value to load
        program[3] = 0x0A; // Asl accumulator

        // Test 2:
        // Store accumulator at the same address. (sta $1234)
        // Shift $1234 left - should become 0. (asl $1234)
        // Load accumulator the address 1234. (lda $1234)
        // carry set. zero and negative unset.
        program[4] = 0x8D; // Absolute addressing sta
        program[5] = 0x34; // Lo byte
        program[6] = 0x12; // Hi byte
        program[7] = 0x0E; // Absolute addressing asl
        program[8] = 0x34; // Lo byte
        program[9] = 0x12; // Hi byte
        program[10] = 0xAD;
        program[11] = 0x34;
        program[12] = 0x12;

        mem.load_program(&program);
        bus.map_device(0x0000, 0xFFFF, &mut mem).unwrap();

        let mut cpu = Cpu::new(&mut bus);

        let mut cycles = 0;
        while cycles < 6 {
            cpu.cycle();
            cycles += 1;
        }

        assert_eq!(cpu.a, 0x80);
        assert!(!cpu.flag_set(cpu::Flags::Carry));
        assert!(!cpu.flag_set(cpu::Flags::Zero));
        assert!(cpu.flag_set(cpu::Flags::Negative));

        cycles = 0;
        while cycles < 14 {
            cpu.cycle();
            cycles += 1;
        }

        assert_eq!(cpu.a, 0x00);
        // assert!(cpu.flag_set(cpu::Flags::Carry));
        assert!(cpu.flag_set(cpu::Flags::Zero));
        assert!(!cpu.flag_set(cpu::Flags::Negative));
    }
}
