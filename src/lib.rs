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
    fn it_works() {
        let memory = Mem { bytes: [0; 4096] };
        let mut cpu = CPU::new(memory);

        let program = vec![0x69, 0x15, 0x69, 0x02];
        cpu.load_program(program);

        cpu.run_for(2);
        assert_eq!(cpu.a, 0x17);
    }
}
