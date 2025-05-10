mod framework;

use framework::run_test;

#[test]
fn it_works() {
    run_test("A9");
    run_test("A5");
    run_test("AD");
}
