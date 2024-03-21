use memory::Memory;

pub mod cpu;
pub mod memory;

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
            // let program = vec![opcode, add_1, opcode, add_2];
            let program: [u8; 4] = [opcode, add_1, opcode, add_2];
            cpu.load_program(&program);
            cpu.run_for(2);

            assert_eq!(cpu.a, result);
            assert!(cpu.flag_raised(cpu::FlagBitPos::Carry) == did_overflow);
            println!("Test {test_index} passed.");
            test_index += 1;
        }
    }

    #[test]
    fn abs_adc_works() {
        let memory = Mem { bytes: [0; 4096] };
        let mut cpu = CPU::new(memory);

        let lo = 0x10_u8;
        let hi = 0x06_u8;
        let addr: u16 = ((hi as u16) << 8) | (lo as u16); // should be 0x610
        println!("addr: {addr:#04x}");

        let mut program: [u8; 2048] = [0; 2048]; // 2k bytes of 0.
        program[0] = 0x6D_u8; // Absolute addressing adc instruction
        program[1] = lo;
        program[2] = hi;
        program[0x10] = 0x19_u8; // memory address 0x610 will have the value 0x19

        cpu.load_program(&program);

        cpu.run_for(1);
        assert_eq!(cpu.a, 0x19);
    }

    #[test]
    fn and_works() {
        let memory = Mem { bytes: [0; 4096] };
        let mut cpu = CPU::new(memory);

        // A test value to load to the a register.
        // lda with test_a, then and a with test_val
        let test_a: u8 = 0x15;
        let test_val: u8 = 0x01; // we will and accumulator with this value.
        let test_result = test_a & test_val;

        let program: [u8; 4] = [0xA9, test_a, 0x29, test_val];
        cpu.load_program(&program);

        cpu.run_for(2);
        assert_eq!(cpu.a, test_result);
    }

    #[test]
    fn lda_works() {
        let memory = Mem { bytes: [0; 4096] };
        let mut cpu = CPU::new(memory);

        let mut program = [0; 2048];
        program[0] = 0xA9_u8;
        program[1] = 0x00_u8;

        // program[0] = 0xA5_u8; // at 0x600 - zero page addressing
        // program[1] = 0xFA_u8; // at 0x601
        // program[0x00FA - 0x600] = 0x15;

        cpu.load_program(&program);
        cpu.run_for(1);

        assert_eq!(cpu.a, 0x00);
        assert!(!cpu.flag_raised(cpu::FlagBitPos::Negative));
        assert!(cpu.flag_raised(cpu::FlagBitPos::Zero));
    }

    // #[test]
    // fn ldx_works() {
    //     let memory = Mem { bytes: [0; 4096] };
    //     let mut cpu = CPU::new(memory);

    //     let mut program = [0; 2048];
    //     program[0] = 0xA2_u8;
    //     program[1] = 0x24_u8;

    //     cpu.load_program(&program);
    //     cpu.run_for(1);

    //     assert_eq!(cpu.x, 0x24);
    //     assert!(!cpu.flag_raised(cpu::FlagBitPos::Negative));
    //     assert!(!cpu.flag_raised(cpu::FlagBitPos::Zero));
    // }

    // #[test]
    // fn ldy_works() {
    //     let memory = Mem { bytes: [0; 4096] };
    //     let mut cpu = CPU::new(memory);

    //     let mut program = [0; 2048];
    //     program[0] = 0xA0_u8;
    //     program[1] = 0x24_u8;

    //     cpu.load_program(&program);
    //     cpu.run_for(1);

    //     assert_eq!(cpu.y, 0x24);
    //     assert!(!cpu.flag_raised(cpu::FlagBitPos::Negative));
    //     assert!(!cpu.flag_raised(cpu::FlagBitPos::Zero));
    // }
}
