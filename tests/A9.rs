use lib6502::Memory;
use lib6502::bus::Bus;
use lib6502::cpu::{Cpu, RegisterState};

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
struct Test {
    initial_state: State,
    final_state: State,
    cycles: (u16, u8, bool), // address, value, read/write
}

#[derive(Serialize, Deserialize)]
struct State {
    registers: RegisterState,
    ram: Vec<(u16, u8)>,
}

#[test]
fn it_works() {
    assert_eq!(42, 42);
}
