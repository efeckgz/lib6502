mod framework;

use framework::run_tests;

#[test]
fn sst() {
    // lda
    run_tests("A9");
    run_tests("A5");
    run_tests("AD");

    run_tests("90"); // bcc
    run_tests("B0"); // bcs
    run_tests("F0"); // beq
    run_tests("30"); // bmi
    run_tests("D0"); // bne
    run_tests("10"); // bpl
    run_tests("50"); // bvc
    run_tests("70"); // bvs
}
