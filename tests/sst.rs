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

    // implied other
    // run_tests("00"); CHECK
    run_tests("48");
    // run_tests("08"); CHECK
    run_tests("68");
    // run_tests("28"); CHECK
    // run_tests("40");
    // run_tests("60");

    // Absolute
    run_tests("6D");
    run_tests("2D");
    run_tests("0E");
    run_tests("2C");
    run_tests("CD");
    run_tests("EC");
    run_tests("CC");
    run_tests("CE");
    run_tests("4D");
    run_tests("EE");
    run_tests("4C");
    run_tests("20");
    run_tests("AD");
    run_tests("AE");
    run_tests("AC");
    run_tests("4E");
    run_tests("0D");
    run_tests("2E");
    run_tests("6E");
    // run_tests("ED");
    run_tests("8D");
    run_tests("8E");
    run_tests("8C");

    run_tests("90"); // bcc
    run_tests("B0"); // bcs
    run_tests("F0"); // beq
    run_tests("30"); // bmi
    run_tests("D0"); // bne
    run_tests("10"); // bpl
    run_tests("50"); // bvc
    run_tests("70"); // bvs
}
