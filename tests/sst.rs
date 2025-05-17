mod framework;

use framework::run_tests;

#[test]
fn sst() {
    // Implied
    run_tests("00"); // brk
    run_tests("18"); // clc
    run_tests("d8"); // cld
    run_tests("58"); // cli
    run_tests("b8"); // clv
    run_tests("ca"); // dex
    run_tests("88"); // dey
    run_tests("e8"); // inx
    run_tests("c8"); // iny
    run_tests("ea"); // nop
    run_tests("48"); // pha
    run_tests("08"); // php
    run_tests("68"); // pla
    run_tests("28"); // plp
    run_tests("40"); // rti
    run_tests("60"); // rts
    run_tests("38"); // sec
    run_tests("f8"); // sed
    run_tests("78"); // sei
    run_tests("aa"); // tax
    run_tests("a8"); // tay
    run_tests("ba"); // tsx
    run_tests("8a"); // txa
    run_tests("9a"); // txs
    run_tests("98"); // tya

    // Immediate
    run_tests("69"); // adc
    run_tests("29"); // and
    run_tests("c9"); // cmp
    run_tests("e0"); // cpx
    run_tests("c0"); // cpy
    run_tests("49"); // eor
    run_tests("a9"); // lda
    run_tests("a2"); // ldx
    run_tests("a0"); // ldy
    run_tests("09"); // ora
    run_tests("e9"); // sbc

    // Accumulator
    run_tests("0a"); // asl
    run_tests("4a"); // lsr
    run_tests("2a"); // rol
    run_tests("6a"); // ror

    // Absolute
    run_tests("6d"); // adc
    run_tests("2d"); // and
    run_tests("0e"); // asl
    run_tests("2c"); // bit
    run_tests("cd"); // cmp
    run_tests("ec"); // cpx
    run_tests("cc"); // cpy
    run_tests("ce"); // dec
    run_tests("4d"); // eor
    run_tests("ee"); // inc
    run_tests("4c"); // jmp
    run_tests("20"); // jsr
    run_tests("ad"); // lda
    run_tests("ae"); // ldx
    run_tests("ac"); // ldy
    run_tests("4e"); // lsr
    run_tests("0d"); // ora
    run_tests("2e"); // rol
    run_tests("6e"); // ror
    run_tests("ed"); // sbc
    run_tests("8d"); // sta
    run_tests("8e"); // stx
    run_tests("8c"); // sty

    // Absolute X
    run_tests("7d"); // adc
    run_tests("3d"); // and
    run_tests("1e"); // asl
    run_tests("dd"); // cmp
    run_tests("de"); // dec
    run_tests("5d"); // eor
    run_tests("fe"); // inc
    run_tests("bd"); // lda
    run_tests("bc"); // ldy
    run_tests("5e"); // lsr
    run_tests("1d"); // ora
    run_tests("3e"); // rol
    run_tests("7e"); // ror
    run_tests("fd"); // sbc
    run_tests("9d"); // sta

    // Absolute Y
    run_tests("79"); // adc
    run_tests("39"); // and
    run_tests("d9"); // cmp
    run_tests("59"); // eor
    run_tests("b9"); // lda
    run_tests("be"); // ldx
    run_tests("19"); // ora
    run_tests("f9"); // sbc
    run_tests("99"); // sta

    // Zero Page
    run_tests("65"); // adc
    run_tests("25"); // and
    run_tests("06"); // asl
    run_tests("24"); // bit
    run_tests("c5"); // cmp
    run_tests("e4"); // cpx
    run_tests("c4"); // cpy
    run_tests("c6"); // dec
    run_tests("45"); // eor
    run_tests("e6"); // inc
    run_tests("a5"); // lda
    run_tests("a6"); // ldx
    run_tests("a4"); // ldy
    run_tests("46"); // lsr
    run_tests("05"); // ora
    run_tests("26"); // rol
    run_tests("66"); // ror
    run_tests("e5"); // sbc
    run_tests("85"); // sta
    run_tests("86"); // stx
    run_tests("84"); // sty

    // Zero Page X
    run_tests("75"); // adc
    run_tests("35"); // and
    run_tests("16"); // asl
    run_tests("d5"); // cmp
    run_tests("d6"); // dec
    run_tests("55"); // eor
    run_tests("f6"); // inc
    run_tests("b5"); // lda
    run_tests("b4"); // ldy
    run_tests("56"); // lsr
    run_tests("15"); // ora
    run_tests("36"); // rol
    run_tests("76"); // ror
    run_tests("f5"); // sbc
    run_tests("95"); // sta
    run_tests("94"); // sty

    // Zero Page Y
    run_tests("b6"); // ldx
    run_tests("96"); // stx

    // Indirect
    run_tests("6c"); // jmp

    // Indirect X
    run_tests("61"); // adc
    run_tests("21"); // and
    run_tests("c1"); // cmp
    run_tests("41"); // eor
    run_tests("a1"); // lda
    run_tests("01"); // ora
    run_tests("e1"); // sbc
    run_tests("81"); // sta

    // Indirect Y
    run_tests("71"); // adc
    run_tests("31"); // and
    run_tests("d1"); // cmp
    run_tests("51"); // eor
    run_tests("b1"); // lda
    run_tests("11"); // ora
    run_tests("f1"); // sbc
    run_tests("91"); // sta

    // Relative
    run_tests("90"); // bcc
    run_tests("b0"); // bcs
    run_tests("f0"); // beq
    run_tests("30"); // bmi
    run_tests("d0"); // bne
    run_tests("10"); // bpl
    run_tests("50"); // bvc
    run_tests("70"); // bvs
}
