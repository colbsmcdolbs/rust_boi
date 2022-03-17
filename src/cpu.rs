use std::ops::{Shl, Shr};

pub enum Register {
    AF,
    BC,
    DE,
    HL
}

pub enum Flag {
    Z,
    N,
    H,
    C
}

pub (crate) struct Cpu {
    a: u8,
    b: u8, c: u8,
    d: u8, e: u8,
    h: u8, l: u8,
    sp: usize,
    pc: usize,
    zf: bool, // Zero Flag
    nf: bool, // Subtraction Flag (BCD)
    hf: bool, // Half Carry Flag (BCD)
    cf: bool, // Carry Flag
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            a: 0,
            b: 0, c: 0,
            d: 0, e: 0,
            h: 0, l: 0,
            sp: 0,
            pc: 0,
            zf: false,
            nf: false,
            hf: false,
            cf: false
        }
    }

    pub fn get_16_register(& self, register: Register) -> u16 {
        match register {
            Register::AF => {
                let f: u8 = ((self.zf as u8) << 7) | ((self.nf as u8) << 6) | ((self.hf as u8) << 5) | ((self.cf as u8) << 4);
                ((self.a as u16) << 8) | f as u16
            },
            Register::BC => { ((self.b as u16) << 8) | self.c as u16 },
            Register::DE => { ((self.d as u16) << 8) | self.e as u16 },
            Register::HL => { ((self.h as u16) << 8) | self.l as u16 },
        }
    }

    pub fn set_16_register(& mut self, register: Register, value: u16) {
        match register {
            Register::AF => {
                self.a = (value >> 8) as u8;
                self.zf = (((value & 0b10000000) as u8) >> 7) != 0;
                self.nf = (((value & 0b01000000) as u8) >> 6) != 0;
                self.hf = (((value & 0b00100000) as u8) >> 5) != 0;
                self.cf = (((value & 0b00010000) as u8) >> 4) != 0;
            },
            Register::BC => {
                self.b = (value >> 8) as u8;
                self.c = (value & 0x00FF) as u8;
            },
            Register::DE => {
                self.d = (value >> 8) as u8;
                self.e = (value & 0x00FF) as u8;
            },
            Register::HL => {
                self.h = (value >> 8) as u8;
                self.l = (value & 0x00FF) as u8;
            }
        }
    }

    pub fn set_flag(& mut self, flag: Flag, value: bool) {
        match flag {
            Flag::Z => { self.zf = value },
            Flag::N => { self.nf = value },
            Flag::H => { self.hf = value },
            Flag::C => { self.cf = value }
        }
    }

    ////////////////////////////////////////////////////////////
    /// Implemented Instructions
    ////////////////////////////////////////////////////////////
    
    // ADC A,r8
    // ADC A,[HL]
    // ADC a,n8
    fn adc(& mut self, value: u8) -> u8 {
        let a_with_carry: u8 = self.a + (self.cf as u8);
        let (result, overflow) = a_with_carry.overflowing_add(value);

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, add_half_carry(a_with_carry, value, self.cf as u8));
        self.set_flag(Flag::C, overflow);

        result
    }

    // ADD A,r8
    // ADD A,[HL]
    // ADD A,n8
    fn add_8(& mut self, value: u8) -> u8 {
        let (result, overflow) = self.a.overflowing_add(value);

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, add_half_carry(self.a, value, 0));
        self.set_flag(Flag::C, overflow);

        result
    }

    // AND A,r8
    // AND A,[HL]
    // AND A,n8
    // TODO: Should this set the A register??
    fn and(& mut self, value: u8) {
        let result: u8 = self.a & value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, true);
        self.set_flag(Flag::C, false);
    }

    // CP A,r8
    // CP A,[HL]
    // CP A,n8
    fn cp(& mut self, value: u8) {
        let result: u8 = self.a - value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, sub_half_carry(self.a, value, 0));
        self.set_flag(Flag::C, value > self.a);
    }

    // DEC r8
    // DEC [HL]
    fn dec(& mut self, value: u8) -> u8 {
        let result: u8 = value - 1;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, sub_half_carry(value, 1, 0));

        result
    }

    // INC r8
    // INC [HL]
    fn inc(& mut self, value: u8) -> u8 {
        let result: u8 = value + 1;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, add_half_carry(value, 1, 0));

        result
    }

    // OR A,r8
    // OR A,[HL]
    // OR A, n8
    fn or(& mut self, value: u8) -> u8 {
        let result: u8 = self.a | value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, false);

        result
    }

    // SBC A,r8
    // SBC A,[HL]
    // SBC A,n8
    fn sbc(& mut self, value: u8) {

    }

    // SUB A,r8
    // SUB A,[HL]
    // SUB A,n8
    fn sub(& mut self, value: u8) -> u8 {
        let result: u8 = self.a - value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, sub_half_carry(self.a, value, 0));
        self.set_flag(Flag::C, value > self.a);

        result
    }

    // XOR A,r8
    // XOR A,[HL]
    // XOR A,n8
    fn xor(& mut self, value: u8) -> u8 {
        let result: u8 = self.a ^ value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, false);

        result
    }

    // ADD HL,r16
    // ADD HL,SP
    fn add_16(& mut self, value: u16) -> u16 {
        let (result, overflow) = self.get_16_register(Register::HL).overflowing_add(value);

        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, add_u16_half_carry(self.get_16_register(Register::HL), value));
        self.set_flag(Flag::C, overflow);

        result
    }

    // DEC r16
    fn dec_16(& mut self, value: u16) -> u16 {
        value - 1
    }

    // INC r16
    fn inc_16(& mut self, value: u16) -> u16 {
        value + 1
    }

    // BIT u3,r8
    // BIT u3,[HL]
    fn bit(& mut self, value: u8, bit: usize) {
        self.set_flag(Flag::Z, ((value >> bit) & 1) == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, true);
    }

    // RES u3,r8
    // RES u3, [HL]
    fn res(& self, value: u8, bit: usize) -> u8 {
        let mask: u8 = 1 << bit;
        value ^ mask
    }

    // SET u3,r8
    // SET u3,[HL]
    fn set(& self, value: u8, bit: usize) -> u8 {
        let mask: u8 = 1 << bit;
        value | mask
    }

    // SWAP r8
    // SWAP [HL]
    fn swap(& self, value: u8) -> u8 {
        value.rotate_left(4)
    }

    // RL r8
    // RL [HL]
    // RLA
    fn rl(& mut self, value: u8, is_rla: bool) -> u8 {
        let overflow: bool = (value >> 7) == 1;
        let mut result: u8 = value.shl(1);

        if self.cf {
            result = result | 1;
        }

        self.set_flag(Flag::Z, !is_rla && result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, overflow);

        result
    }

    // RLC r8
    // RCL [HL]
    // RLCA
    fn rlc(& mut self, value: u8, is_rlca: bool) -> u8 {
        let overflow: bool = (value >> 7) == 1;
        let mut result: u8 = value.shl(1);

        if overflow {
            result = result | 1;
        }

        self.set_flag(Flag::Z, !is_rlca && result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, overflow);

        result
    }

    // RR r8
    // RR [HL]
    // RRA
    fn rr(& mut self, value: u8, is_rr: bool) -> u8 {
        let overflow: bool = (value & 1) == 1;
        let mut result: u8 = value.shr(1);

        if self.cf {
            result = result | 0x80;
        }

        self.set_flag(Flag::Z, !is_rr && result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, overflow);

        result
    }

    // RRC r8
    // RRC [HL]
    // RRCA
    fn rrc(& mut self, value: u8, is_rrca: bool) -> u8 {
        let overflow: bool = (value & 1) == 1;
        let mut result: u8 = value.shr(1);

        if overflow {
            result = result | 0x80;
        }

        self.set_flag(Flag::Z, !is_rrca && result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, overflow);

        result
    }

    // SLA r8
    // SLA [HL]
    fn sla(& mut self, value: u8) -> u8 {
        let overflow: bool = (value >> 7) == 1;
        let result: u8 = value.shl(1);

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, overflow);

        result
    }

    // SRA r8
    // SRA [HL]
    fn sra(& mut self, value: u8) -> u8 {
        let msb: bool = (value >> 7) == 1;
        let overflow: bool = (value & 1) == 1;
        let mut result: u8 = value.shr(1);

        if msb {
            result = result | 0x80;
        }

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, overflow);

        result
    }

    // SRL r8
    // SRL [HL]
    fn srl(& mut self, value: u8) -> u8 {
        let overflow: bool = (value & 1) == 1;
        let result: u8 = value.shr(1);

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, overflow);

        result
    }
}

// Utils
fn add_half_carry(a: u8, b: u8, c: u8) -> bool {
    (((a & 0xF) + (b & 0xF) + (c & 0xF)) & 0x10) == 0x10
}

fn add_u16_half_carry(a: u16, b: u16) -> bool {
    ((a & 0xFFF) + (b & 0xFFF) & 0x1000) == 0x1000
}

fn sub_half_carry(a: u8, b: u8, c: u8) -> bool {
    ((a & 0xF) - (b & 0xF) - (c & 0xF)) < 0
}

#[cfg(test)]
mod cpu_tests {
    use super::*;

    #[test]
    fn successfully_sets_flag() {
        let mut cpu = Cpu::new();
        cpu.set_flag(Flag::Z, true);
        assert!(cpu.zf == true)
    }

    #[test]
    fn correctly_parses_flags() {
        let mut cpu = Cpu::new();
        cpu.set_flag(Flag::Z, true);
        cpu.set_flag(Flag::N, true);
        cpu.set_flag(Flag::H, true);
        cpu.set_flag(Flag::C, true);
        assert!(cpu.get_16_register(Register::AF) == 0x00F0)
    }

    #[test]
    fn set_af_correctly_sets_flags() {
        let mut cpu = Cpu::new();
        cpu.set_16_register(Register::AF, 0xFFFF);
        assert!(cpu.a == 0xFF);
        assert!(cpu.zf == true);
        assert!(cpu.nf == true);
        assert!(cpu.hf == true);
        assert!(cpu.cf == true);
    }

    #[test]
    fn set_af_correctly_sets_flags_to_off() {
        let mut cpu = Cpu::new();
        cpu.set_16_register(Register::AF, 0xFF00);
        assert!(cpu.a == 0xFF);
        assert!(cpu.zf == false);
        assert!(cpu.nf == false);
        assert!(cpu.hf == false);
        assert!(cpu.cf == false);
    }

    #[test]
    fn res_correctly_turns_bit_to_zero() {
        let cpu = Cpu::new();
        let result: u8 = cpu.res(0xFF, 5);
        assert!(result == 0b1101_1111)
    }

    #[test]
    fn set_correctly_turns_bit_to_one() {
        let cpu = Cpu::new();
        let result: u8 = cpu.res(0x00, 5);
        assert!(result == 0b0010_0000)
    }

    #[test]
    fn swap_correctly_rotates_byte() {
        let cpu = Cpu::new();
        let result: u8 = cpu.swap(0x0F);
        assert!(result == 0xF0)
    }

    #[test]
    fn temp_test() {
        let temp: u8 = 0xFF;
        let overflow: bool = (temp >> 7) == 1;
        assert!(overflow);

        let r_overflow: bool = (temp & 1) == 1;
        assert!(r_overflow);
    }
}
