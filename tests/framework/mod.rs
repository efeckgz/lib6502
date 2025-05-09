use lib6502::{
    bus::{Bus, BusDevice},
    cpu::{Cpu, RegisterState},
};
use serde::{Deserialize, Serialize};

const TESTS_DIR: &str = "./65x02/6502/v1";

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
    pub fn from(cpu: &Cpu, ram: Ram) -> Self {
        let mut memdump: Vec<(u16, u8)> = vec![];
        for (addr, byte) in ram.bytes.iter().enumerate() {
            if *byte == 0 {
                continue;
            }

            memdump.push((addr as u16, *byte));
        }

        let (pc, s, a, x, y, p) = cpu.to_state();

        Self {
            pc,
            s,
            a,
            x,
            y,
            p,
            ram: memdump,
        }
    }
    // pub fn from_registers((pc, s, a, x, y, p): RegisterState) -> Self {
    //     Self {
    //         pc,
    //         s,
    //         a,
    //         x,
    //         y,
    //         p,
    //         ram: vec![],
    //     }
    // }

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
}

impl BusDevice for Ram {
    fn read(&mut self, addr: u16) -> u8 {
        self.bytes[addr as usize]
    }

    fn write(&mut self, addr: u16, data: u8) {
        self.bytes[addr as usize] = data;
    }
}

pub fn load_test(name: &str) -> Vec<Test> {
    let test_name = TESTS_DIR.to_owned() + &format!("/{}.json", name);
    let bytes = std::fs::read(test_name).unwrap();
    serde_json::from_slice(&bytes).unwrap()
}

pub fn run_test(test_name: &str) {
    let tests = load_test(test_name);
    let t = &tests[0];

    let init = &t.initial_state;
    let final_state = &t.final_state;

    let mut bus: Bus<1> = Bus::new();
    let mut ram = Ram::new();

    ram.load_from_state(init.clone());
    bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

    let mut cpu = Cpu::from_register_state(init.to_registers(), &mut bus);

    for c in &t.cycles {
        let (addr, data, rw) = c;
        let read = if rw == "read" { true } else { false };

        cpu.cycle();
        assert_eq!(cpu.addr, *addr);
        assert_eq!(cpu.data, *data);
        assert_eq!(cpu.read, read);
    }
}
