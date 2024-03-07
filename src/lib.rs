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
        let mut cpu = CPU::new(memory, 0x600);

        let add_1: u8 = 0xFA;
        let add_2: u8 = 0x18;
        let (result, did_overflow) = add_1.overflowing_add(add_2); // must be 0x02

        let program = vec![0x69, add_1, 0x69, add_2];
        let inst_count = (program.len() / 2) as i32;
        cpu.load_program(program);

        cpu.run_for(inst_count);
        assert_eq!(cpu.a, result); // Test if the result is correct
        assert!(cpu.flag_raised(0) == did_overflow); // Test if the carry flag is set correct
    }

    #[test]
    fn and_works() {
        let memory = Mem { bytes: [0; 4096] };
        let mut cpu = CPU::new(memory, 0x600);

        // A test value to load to the a register.
        // Since lda is not implemented we will add this number to A when A is 0.
        let test_a: u8 = 0x15;
        let test_val: u8 = 0x01; // we will and accumulator with this value.

        let program = vec![0x69, test_a, 0x29, test_val];
        cpu.load_program(program);

        cpu.run_for(2);
        assert_eq!(cpu.a, test_a & test_val);
    }
}
