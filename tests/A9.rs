mod framework;

use lib6502::Memory;
use lib6502::bus::Bus;

use framework::load_test;

#[test]
fn it_works() {
    let test = load_test("A9");
    println!("{:?}", test);
    assert_eq!(42, 42);
}
