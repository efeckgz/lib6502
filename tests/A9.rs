mod framework;

use lib6502::{bus::Bus, cpu::Cpu};

use framework::{Ram, load_test};

#[test]
fn it_works() {
    let tests = load_test("A9");
    let t = &tests[0];

    let init = &t.initial_state;

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

    assert_eq!(42, 42);
}
