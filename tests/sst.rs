mod framework;

use framework::run_test;

#[test]
fn sst() {
    run_test("A9");
    run_test("A5");
    run_test("AD");
    run_test("20");
}
