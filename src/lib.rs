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
    use self::cpu::CPU;
    use super::*;

    #[test]
    fn adc_works() {
        let memory = Mem { bytes: [0; 4096] };
        let mut cpu = CPU::new(memory);

        let add_1: u8 = 0xFA;
        let add_2: u8 = 0x18;
        let (result, _) = add_1.overflowing_add(add_2);

        let program = vec![0x69, add_1, 0x69, add_2];
        cpu.load_program(program);

        cpu.run_for(2);
        assert_eq!(cpu.a, result); // Test if the result is correct
        assert!(cpu.flag_raised(0)); // Test if the carry flag is set correct
    }
}
