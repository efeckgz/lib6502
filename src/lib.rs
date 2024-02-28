use memory::Memory;

mod cpu;
mod memory;

struct mem {
    m: [u8; 4096],
}

impl Memory for mem {
    fn read(&self, addr: u16) -> u8 {
        self.m[addr as usize]
    }

    fn write(&mut self, addr: u16, data: u8) {
        self.m[addr as usize] = data;
    }
}

#[cfg(test)]
mod tests {
    use self::cpu::CPU;
    use super::*;

    #[test]
    fn it_works() {
        let memory = mem { m: [0; 4096] };

        let cpu = CPU::new(memory);
    }
}
