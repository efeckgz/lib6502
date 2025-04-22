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

        bus.map_device(0x00, 0xFF, &mut memory).unwrap();

        let mut cpu = Cpu::new(&mut bus);

        // Run for 2 cycles
        cpu.cycle();
        cpu.cycle();

        assert_eq!(cpu.a, 0x0A);
    }
}
