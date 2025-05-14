mod utils;

use lib6502::bus::Bus;
use lib6502::cpu::Cpu;
use utils::{Ram, State, TESTS_DIR, Test};

pub fn run_tests(opcode: &str) {
    let tests = load_tests(opcode);
    for test in tests {
        run_single_test(test);
    }
}

fn load_tests(name: &str) -> Vec<Test> {
    let test_name = TESTS_DIR.to_owned() + &format!("/{}.json", name);
    let bytes = std::fs::read(test_name).unwrap();
    serde_json::from_slice(&bytes).unwrap()
}

fn run_single_test(t: Test) {
    print!("Running test {}\t", t.name);
    let init = &t.initial_state;
    let final_state = &t.final_state;

    let mut bus: Bus<1> = Bus::new();
    let mut ram = Ram::new();

    ram.load_from_state(init.clone());
    bus.map_device(0x0000, 0xFFFF, &mut ram).unwrap();

    let mut cpu = Cpu::from_register_state(init.to_registers(), &mut bus);

    let mut i = 0;
    for c in &t.cycles {
        let (addr, data, rw) = c;
        let read = if rw == "read" { true } else { false };

        cpu.cycle();
        let (cpu_addr, cpu_data, cpu_rw) = cpu.get_bus_pins();

        assert_eq!(cpu_addr, *addr);
        assert_eq!(cpu_data, *data);
        assert_eq!(cpu_rw, read);
        println!("Cycle {} works!", i);
        i += 1;
    }

    let final_cpu = State::new(cpu.get_state(), ram.get_state(final_state));
    assert_eq!(final_cpu, *final_state);
    println!("{}", text_green("PASS!"));
}

fn text_green(s: &str) -> String {
    format!("\x1b[92m{}\x1b[0m", s)
}
