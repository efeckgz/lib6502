mod framework;

use framework::run_tests;

#[test]
fn sst() {
    // Implied
    run_tests("00"); // brk
    run_tests("18"); // clc
    run_tests("D8"); // cld
    run_tests("58"); // cli
    run_tests("B8"); // clv
    run_tests("CA"); // dex
    run_tests("88"); // dey
    run_tests("E8"); // inx
    run_tests("C8"); // iny
    run_tests("EA"); // nop
    run_tests("48"); // pha
    run_tests("08"); // php
    run_tests("68"); // pla
    run_tests("28"); // plp
    run_tests("40"); // rti
    run_tests("60"); // rts
    run_tests("38"); // sec
    run_tests("F8"); // sed
    run_tests("78"); // sei
    run_tests("AA"); // tax
    run_tests("A8"); // tay
    run_tests("BA"); // tsx
    run_tests("8A"); // txa
    run_tests("9A"); // txs
    run_tests("98"); // tya

    // Immediate
    run_tests("69"); // adc
    run_tests("29"); // and
    run_tests("C9"); // cmp
    run_tests("E0"); // cpx
    run_tests("C0"); // cpy
    run_tests("49"); // eor
    run_tests("A9"); // lda
    run_tests("A2"); // ldx
    run_tests("A0"); // ldy
    run_tests("09"); // ora
    run_tests("E9"); // sbc

    // Accumulator
    run_tests("0A"); // asl
    run_tests("4A"); // lsr
    run_tests("2A"); // rol
    run_tests("6A"); // ror

    // Absolute
    run_tests("6D"); // adc
    run_tests("2D"); // and
    run_tests("0E"); // asl
    run_tests("2C"); // bit
    run_tests("CD"); // cmp
    run_tests("EC"); // cpx
    run_tests("CC"); // cpy
    run_tests("CE"); // dec
    run_tests("4D"); // eor
    run_tests("EE"); // inc
    run_tests("4C"); // jmp
    run_tests("20"); // jsr
    run_tests("AD"); // lda
    run_tests("AE"); // ldx
    run_tests("AC"); // ldy
    run_tests("4E"); // lsr
    run_tests("0D"); // ora
    run_tests("2E"); // rol
    run_tests("6E"); // ror
    run_tests("ED"); // sbc
    run_tests("8D"); // sta
    run_tests("8E"); // stx
    run_tests("8C"); // sty

    // Absolute X
    run_tests("7D"); // adc
    run_tests("3D"); // and
    run_tests("1E"); // asl
    run_tests("DD"); // cmp
    run_tests("DE"); // dec
    run_tests("5D"); // eor
    run_tests("FE"); // inc
    run_tests("BD"); // lda
    run_tests("BC"); // ldy
    run_tests("5E"); // lsr
    run_tests("1D"); // ora
    run_tests("3E"); // rol
    run_tests("7E"); // ror
    run_tests("FD"); // sbc
    run_tests("9D"); // sta

    // Absolute Y
    run_tests("79"); // adc
    run_tests("39"); // and
    run_tests("D9"); // cmp
    run_tests("59"); // eor
    run_tests("B9"); // lda
    run_tests("BE"); // ldx
    run_tests("19"); // ora
    run_tests("F9"); // sbc
    run_tests("99"); // sta

    // Zero Page
    run_tests("65"); // adc
    run_tests("25"); // and
    run_tests("06"); // asl
    run_tests("24"); // bit
    run_tests("C5"); // cmp
    run_tests("E4"); // cpx
    run_tests("C4"); // cpy
    run_tests("C6"); // dec
    run_tests("45"); // eor
    run_tests("E6"); // inc
    run_tests("A5"); // lda
    run_tests("A6"); // ldx
    run_tests("A4"); // ldy
    run_tests("46"); // lsr
    run_tests("05"); // ora
    run_tests("26"); // rol
    run_tests("66"); // ror
    run_tests("E5"); // sbc
    run_tests("85"); // sta
    run_tests("86"); // stx
    run_tests("84"); // sty

    // Zero Page X
    run_tests("75"); // adc
    run_tests("35"); // and
    run_tests("16"); // asl
    run_tests("D5"); // cmp
    run_tests("D6"); // dec
    run_tests("55"); // eor
    run_tests("F6"); // inc
    run_tests("B5"); // lda
    run_tests("B4"); // ldy
    run_tests("56"); // lsr
    run_tests("15"); // ora
    run_tests("36"); // rol
    run_tests("76"); // ror
    run_tests("F5"); // sbc
    run_tests("95"); // sta
    run_tests("94"); // sty

    // Zero Page Y
    run_tests("B6"); // ldx
    run_tests("96"); // stx

    // Relative
    run_tests("90"); // bcc
    run_tests("B0"); // bcs
    run_tests("F0"); // beq
    run_tests("30"); // bmi
    run_tests("D0"); // bne
    run_tests("10"); // bpl
    run_tests("50"); // bvc
    run_tests("70"); // bvs
}
