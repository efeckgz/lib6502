use memory::Memory;

mod cpu;
mod memory;

struct Mem {
    pub bytes: [u8; 4096],
}

impl Memory for Mem {
    fn read(&self, addr: u16) -> u8 {
        self.bytes[addr as usize]
    }

    fn write(&mut self, addr: u16, data: u8) {
        self.bytes[addr as usize] = data;
    }
}

#[cfg(test)]
mod tests {
    use std::mem;

    use self::cpu::CPU;
    use super::*;

    #[test]
    fn it_works() {
        let mut memory = Mem { bytes: [0; 4096] };
        memory.bytes[0x10] = 0x69; // immidiate addressed adc instruction
        memory.bytes[0x11] = 0x15; // value in the second byte of the instruction.
                                   // This should add 0x15 to the a register. The value of a register after the cpu has run should be 0x15.

        let mut cpu = CPU::new(memory);
        cpu.run_for(10);
        assert_eq!(cpu.a, 0x15);
    }
}
