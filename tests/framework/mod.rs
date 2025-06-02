mod utils;

use std::path::PathBuf;

use lib6502::bus::{Bus, BusDevice};
use lib6502::cpu::Cpu;
use utils::{Devices, Ram, State, TESTS_DIR, Test};

pub fn run_tests(opcode: &str) {
    let tests = load_tests(opcode);
    for test in tests {
        run_single_test(test);
    }
}

fn load_tests(name: &str) -> Vec<Test> {
    let mut p = PathBuf::from(TESTS_DIR);
    p.push(&format!("{}.json", name));

    let bytes = std::fs::read(p.as_path()).unwrap();
    serde_json::from_slice(&bytes).unwrap()
}

fn run_single_test(t: Test) {
    print!("Running test {}", t.name);
    let init = &t.initial_state;
    let final_state = t.final_state;

    let mut bus: Bus<Devices, 1> = Bus::new();

    let mut _ram = Ram::new();
    _ram.load_from_state(init.clone());

    let ram_device = Devices::Ram(_ram);
    bus.map_device(0x0000, 0xFFFF, ram_device, 1).unwrap();

    let mut cpu = Cpu::from_register_state(init.to_registers(), bus);

    let mut i = 1;
    print!("\tCyles: ");
    for c in &t.cycles {
        let (addr, data, rw) = c;
        let read = if rw == "read" { true } else { false };

        cpu.cycle();
        let (cpu_addr, cpu_data, cpu_rw) = cpu.get_bus_pins();

        assert_eq!(cpu_addr, *addr);
        assert_eq!(cpu_data, *data);
        assert_eq!(cpu_rw, read);
        print!("{} \u{2714} ", i);
        i += 1;
    }

    let final_cpu = State::new(cpu.get_state(), get_final_mem(&mut cpu.bus, &final_state));
    assert_eq!(final_cpu, final_state);
    println!("\t{}", text_green("PASS!"));
}

fn get_final_mem(bus: &mut Bus<Devices, 1>, st: &State) -> Vec<(u16, u8)> {
    let mut result = vec![];
    for (addr, _) in st.clone().ram {
        result.push((addr, bus.read(addr)));
    }
    result
}

fn text_green(s: &str) -> String {
    format!("\x1b[92m{}\x1b[0m", s)
}
