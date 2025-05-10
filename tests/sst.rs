mod framework;

use framework::run_tests;

#[test]
fn sst() {
    // lda
    run_tests("A9");
    run_tests("A5");
    run_tests("AD");

    // implied 2 cycle
    run_tests("18");
    run_tests("D8");
    run_tests("58");
    run_tests("B8");
    run_tests("CA");
    run_tests("88");
    run_tests("E8");
    run_tests("C8");
    run_tests("EA");
    run_tests("38");
    run_tests("F8");
    run_tests("78");
    run_tests("AA");
    run_tests("A8");
    run_tests("BA");
    run_tests("8A");
    run_tests("9A");
    run_tests("98");

    run_tests("90"); // bcc
    run_tests("B0"); // bcs
    run_tests("F0"); // beq
    run_tests("30"); // bmi
    run_tests("D0"); // bne
    run_tests("10"); // bpl
    run_tests("50"); // bvc
    run_tests("70"); // bvs
}
