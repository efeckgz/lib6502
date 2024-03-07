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

    fn reset(&mut self) {
        self.bytes = [0; 4096];
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

        let tests = vec![
            (0x69_u8, 0xFF_u8, 0x18_u8),
            (0x69_u8, 0x1A_u8, 0xBD_u8),
            (0x69_u8, 0xFF_u8, 0x01_u8),
        ];

        let mut test_index = 1;
        for (opcode, add_1, add_2) in tests {
            cpu.reset();
            let (result, did_overflow) = add_1.overflowing_add(add_2);
            let program = vec![opcode, add_1, opcode, add_2];
            cpu.load_program(program);
            cpu.run_for(2);

            assert_eq!(cpu.a, result);
            assert!(cpu.flag_raised(cpu::CARRY) == did_overflow);
            println!("Test {test_index} passed.");
            test_index += 1;
        }
    }

    #[test]
    fn and_works() {
        let memory = Mem { bytes: [0; 4096] };
        let mut cpu = CPU::new(memory);

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
