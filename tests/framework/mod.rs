use lib6502::cpu::RegisterState;
use serde::{Deserialize, Serialize};

const TESTS_DIR: &str = "./65x02/6502/v1";

#[derive(Serialize, Deserialize, Debug)]
pub struct Test {
    name: String,

    #[serde(rename(deserialize = "initial"))]
    initial_state: State,

    #[serde(rename(deserialize = "final"))]
    final_state: State,

    cycles: Vec<(u16, u8, String)>, // address, value, read/write
}

#[derive(Serialize, Deserialize, Debug)]
pub struct State {
    pc: u16,
    s: u8,
    a: u8,
    x: u8,
    y: u8,
    p: u8,
    ram: Vec<(u16, u8)>,
}

impl State {
    pub fn from_registers((pc, s, a, x, y, p): RegisterState) -> Self {
        Self {
            pc,
            s,
            a,
            x,
            y,
            p,
            ram: vec![],
        }
    }
}

pub fn load_test(name: &str) -> Vec<Test> {
    let test_name = TESTS_DIR.to_owned() + &format!("/{}.json", name);
    let bytes = std::fs::read(test_name).unwrap();
    serde_json::from_slice(&bytes).unwrap()
}
