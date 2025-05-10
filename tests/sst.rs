mod framework;

use framework::run_test;

#[test]
fn sst() {
    // lda
    run_test("A9");
    run_test("A5");
    run_test("AD");

    run_test("90"); // bcc
    run_test("B0"); // bcs
    run_test("F0"); // beq
    run_test("30"); // bmi
    run_test("D0"); // bne
    run_test("10"); // bpl
    run_test("50"); // bvc
    run_test("70"); // bvs
}
