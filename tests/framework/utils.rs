use lib6502::{bus::BusDevice, cpu::RegisterState};

use serde::{Deserialize, Serialize};

// Works on my mac
pub const TESTS_DIR: &str = "./65x02/nes6502/v1";

#[derive(Serialize, Deserialize, Debug)]
pub struct Test {
    pub name: String,

    #[serde(rename(deserialize = "initial"))]
    pub initial_state: State,

    #[serde(rename(deserialize = "final"))]
    pub final_state: State,

    pub cycles: Vec<(u16, u8, String)>, // address, value, read/write
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct State {
    pub pc: u16,
    pub s: u8,
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub p: u8,
    pub ram: Vec<(u16, u8)>,
}

#[derive(Clone)]
pub struct Ram {
    pub bytes: [u8; 65536],
}

impl State {
    pub fn new((pc, s, a, x, y, p): RegisterState, ram: Vec<(u16, u8)>) -> Self {
        Self {
            pc,
            s,
            a,
            x,
            y,
            p,
            ram,
        }
    }

    pub fn to_registers(&self) -> RegisterState {
        (self.pc, self.s, self.a, self.x, self.y, self.p)
    }
}

impl Ram {
    pub fn new() -> Self {
        Self {
            bytes: [0_u8; 65536],
        }
    }

    pub fn load_from_state(&mut self, st: State) {
        for (addr, byte) in st.ram {
            self.write(addr, byte);
        }
    }

    pub fn get_state(&mut self, st: &State) -> Vec<(u16, u8)> {
        let mut res: Vec<(u16, u8)> = vec![];
        let supposed = st.ram.clone();
        for (addr, _) in supposed.iter() {
            let data = self.read(*addr);
            res.push((*addr as u16, data));
        }
        res
    }
}

impl BusDevice for Ram {
    fn read(&mut self, addr: u16) -> u8 {
        self.bytes[addr as usize]
    }

    fn write(&mut self, addr: u16, data: u8) {
        self.bytes[addr as usize] = data;
    }
}
