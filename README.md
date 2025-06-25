# lib6502

lib6502 is a Rust crate that provides a cycle-accurate emulator for the Mos Technology 6502 microprocessor. The core emulator is complete, but there are many improvements to be done.

## Usage

The goal of this library is to be used as a working CPU for builidng an emulator for a 6502 based system. To achieve this, it exposes a trait `BusAdapter`. Implementors should build the Bus made up of different devices depending on the specific system being emulated. The Bus can then implement this trait and passed to `Cpu`.

The main part of the `Cpu` is the `cycle` function. When called, this function executes 1 cpu cycle. Calling this function in a loop achieves a working Cpu. The `instruction_step` function can be used to execute 1 instruction. This function calls `cycle` until the current instruction is finished. The cpu is configured to emulate the 7 cycle initialization sequence. This means creating a new cpu with `Cpu::new()` will not have the correct PC value until after calling `cycle` 7 times. Calling `cpu.start_sequence()` runs 7 cycles to make this easier.

The library is currently not published on crates.io. If you wish to use it, you must clone this repository to your computer and add a local dependency to your `Cargo.toml` file.

## Features

- [X] Dependency free & `no_std` support in release builds (Uses serde to parse json tests in dev builds)
- [X] Cycle accuracy
- [X] All legal opcodes
- [ ] Illegal opcodes
- [ ] All 6502 variants (Currently only NES variant is added)
- [ ] Proper error handling (Currently calls `panic!` on undefined state)
- [ ] Proper library documentation

## Testing

1. Git clone [Single Step Tests](https://github.com/SingleStepTests/65x02.git) into the project root.
2. Run cargo test. This will run the SST json tests. Use `-- --nocapture` option to see the test output.
