use std::fmt::{format, write};

use lib6502::{bus::BusAdapter, cpu::Flags, cpu::RegisterState};
use serde::{Deserialize, Serialize};
pub const TESTS_DIR: &str = "./65x02/6502/v1";

#[derive(Serialize, Deserialize, Debug)]
pub struct Test {
    pub name: String,

    #[serde(rename(deserialize = "initial"))]
    pub initial_state: State,

    #[serde(rename(deserialize = "final"))]
    pub final_state: State,

    pub cycles: Vec<(u16, u8, String)>, // address, value, read/write
}

#[derive(Serialize, Deserialize, Clone, PartialEq)]
pub struct State {
    pub pc: u16,
    pub s: u8,
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub p: u8,
    pub ram: Vec<(u16, u8)>,
}

pub struct Bus {
    pub addresses: Vec<(u16, u8)>,
}

impl Bus {
    pub fn from_initial_state(st: &State) -> Self {
        Self {
            addresses: st.ram.clone(),
        }
    }
}

impl BusAdapter for Bus {
    fn read(&mut self, addr: u16) -> u8 {
        for (_addr, data) in &self.addresses {
            if *_addr == addr {
                return *data;
            }
        }

        0
    }

    fn write(&mut self, addr: u16, data: u8) {
        for (_addr, _data) in &mut self.addresses {
            if *_addr == addr {
                *_data = data;
                return;
            }
        }
        self.addresses.push((addr, data));
    }
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

    fn flag_set(&self, flag: Flags) -> bool {
        (self.p & (1 << flag as u8)) != 0
    }
}

impl core::fmt::Debug for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let carry = if self.flag_set(Flags::Carry) { 1 } else { 0 };
        let zero = if self.flag_set(Flags::Zero) { 1 } else { 0 };
        let interrupt_disable = if self.flag_set(Flags::InterrputDisable) {
            1
        } else {
            0
        };
        let decimal = if self.flag_set(Flags::Decimal) { 1 } else { 0 };
        let brk = if self.flag_set(Flags::Break) { 1 } else { 0 };
        let overflow = if self.flag_set(Flags::Overflow) { 1 } else { 0 };
        let negative = if self.flag_set(Flags::Negative) { 1 } else { 0 };

        let flags_string = format!(
            "Carry: {} Zero: {} Interrupt Disable: {} Decimal: {} Break: {} Overflow: {} Negative: {}",
            carry, zero, interrupt_disable, decimal, brk, overflow, negative
        );

        write!(
            f,
            "PC: {} S: {} A: {} X: {} Y: {} {} Ram: {:?}",
            self.pc, self.s, self.a, self.x, self.y, flags_string, self.ram
        )
    }
}
