use std::ops::{Shl, Shr};

use crate::memory_map::MemoryMap;

pub enum Register {
    AF,
    BC,
    DE,
    HL,
}

pub enum Flag {
    Z,
    N,
    H,
    C,
}

pub(crate) struct Cpu {
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
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
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
            sp: 0,
            pc: 0,
            zf: false,
            nf: false,
            hf: false,
            cf: false,
        }
    }

    pub fn get_16_register(&self, register: Register) -> u16 {
        match register {
            Register::AF => {
                let f: u8 =
                    ((self.zf as u8) << 7) | ((self.nf as u8) << 6) | ((self.hf as u8) << 5) | ((self.cf as u8) << 4);
                ((self.a as u16) << 8) | f as u16
            }
            Register::BC => ((self.b as u16) << 8) | self.c as u16,
            Register::DE => ((self.d as u16) << 8) | self.e as u16,
            Register::HL => ((self.h as u16) << 8) | self.l as u16,
        }
    }

    pub fn set_16_register(&mut self, register: Register, value: u16) {
        match register {
            Register::AF => {
                self.a = (value >> 8) as u8;
                self.zf = (((value & 0b10000000) as u8) >> 7) != 0;
                self.nf = (((value & 0b01000000) as u8) >> 6) != 0;
                self.hf = (((value & 0b00100000) as u8) >> 5) != 0;
                self.cf = (((value & 0b00010000) as u8) >> 4) != 0;
            }
            Register::BC => {
                self.b = (value >> 8) as u8;
                self.c = (value & 0x00FF) as u8;
            }
            Register::DE => {
                self.d = (value >> 8) as u8;
                self.e = (value & 0x00FF) as u8;
            }
            Register::HL => {
                self.h = (value >> 8) as u8;
                self.l = (value & 0x00FF) as u8;
            }
        }
    }

    pub fn execute(&mut self, mut mem_map: MemoryMap) {
        let opcode: u8 = mem_map.get(self.get_pc());
        match opcode {
            0x00 => {} // NOP
            0x01 => { // LD BC,u16
                let value: u16 = self.get_pc_u16(&mut mem_map);
                self.set_16_register(Register::BC, value)
            }
            0x02 => { // LD (BC),A
                mem_map.set(self.get_16_register(Register::BC) as usize, self.a)
            }
            0x03 => { // INC BC
                self.set_16_register(Register::BC, self.inc_16(self.get_16_register(Register::BC)))
            }
            0x04 => { // INC B
                self.b = self.inc(self.b)
            }
            0x05 => { // DEC B
                self.b = self.dec(self.b)
            }
            0x06 => { // LD B, u8
                self.b = mem_map.get(self.get_pc())
            }
            0x07 => panic!("Instruction not implemented! RLCA"),
            0x08 => { // LD (u16),SP
                let index: u16 = self.get_pc_u16(&mut mem_map);
                mem_map.set_u16(index as usize, self.sp as u16)
            }
            0x09 => { // ADD HL, BC
                self.add_hl(self.get_16_register(Register::BC))
            }
            0x0A => { // LD A, (BC)
                self.a = mem_map.get(self.get_16_register(Register::BC) as usize)
            }
            0x0B => { // DEC BC (TODO possible micro-bottlneck here)
                let value: u16 = self.dec_16(self.get_16_register(Register::BC));
                self.set_16_register(Register::BC, value)
            }
            0x0C => { // INC C
                self.c = self.inc(self.c)
            }
            0x0D => { // DEC C
                self.c = self.dec(self.c)
            }
            0x0E => { // LD C, u8
                self.c = mem_map.get(self.get_pc())
            }
            0x0F => { // RRCA
                panic!("Instruction not implemented! RRCA")
            }
            0x10 => { // STOP
                panic!("Instruction not implemented! STOP")
            }
            0x11 => { // LD DE,u16
                let value: u16 = self.get_pc_u16(&mut mem_map);
                self.set_16_register(Register::DE, value)
            }
            0x12 => { // LD (DE),A
                mem_map.set(self.get_16_register(Register::DE) as usize, self.a)
            }
            0x13 => { // INC DE
                self.set_16_register(Register::DE, self.inc_16(self.get_16_register(Register::DE)))
            }
            0x14 => { // INC D
                self.d = self.inc(self.d)
            }
            0x15 => { // DEC D
                self.d = self.dec(self.d)
            }
            0x16 => { // LD D, u8
                self.d = mem_map.get(self.get_pc())
            }
            0x17 => { // RLA
                panic!("Instruction not implemented! RLA")
            }
            0x18 => {
                panic!("Instruction not implemented! JR i8")
            }
            0x19 => { // ADD HL, DE
                self.add_hl(self.get_16_register(Register::DE))
            }
            0x1A => { // LD A, (DE)
                self.a = mem_map.get(self.get_16_register(Register::DE) as usize)
            }
            0x1B => { // DEC DE (TODO possible micro-bottlneck here)
                let value: u16 = self.dec_16(self.get_16_register(Register::DE));
                self.set_16_register(Register::DE, value)
            }
            0x1C => { // INC E
                self.e = self.inc(self.e)
            }
            0x1D => { // DEC E
                self.e = self.dec(self.e)
            }
            0x1E => { // LD E, u8
                self.e = mem_map.get(self.get_pc())
            }
            0x1F => { // RRA
                panic!("Instruction not implemented! RRA")
            }
            0x20 => { // JR NZ, i8
                panic!("Instruction not implemented! JR NZ, i8")
            }
            0x21 => { // LD HL,u16
                let value: u16 = self.get_pc_u16(&mut mem_map);
                self.set_16_register(Register::HL, value)
            }
            0x22 => { // LD (HL+),A
                mem_map.set((self.get_16_register(Register::HL) + 1) as usize, self.a)
            }
            0x23 => { // INC HL
                self.set_16_register(Register::HL, self.inc_16(self.get_16_register(Register::HL)))
            }
            0x24 => { // INC H
                self.h = self.inc(self.h)
            }
            0x25 => { // DEC H
                self.h = self.dec(self.h)
            }
            0x26 => { // LD H, u8
                self.h = mem_map.get(self.get_pc())
            }
            0x27 => { // DAA
                panic!("Instruction not implemented! DAA")
            }
            0x28 => {
                panic!("Instruction not implemented! JR Z, i8")
            }
            0x29 => { // ADD HL, HL
                self.add_hl(self.get_16_register(Register::HL))
            }
            0x2A => { // LD A, (HL+)
                self.a = mem_map.get((self.get_16_register(Register::HL) + 1) as usize)
            }
            0x2B => { // DEC HL (TODO possible micro-bottlneck here)
                let value: u16 = self.dec_16(self.get_16_register(Register::HL));
                self.set_16_register(Register::HL, value)
            }
            0x2C => { // INC L
                self.l = self.inc(self.l)
            }
            0x2D => { // DEC L
                self.l = self.dec(self.l)
            }
            0x2E => { // LD L, u8
                self.l = mem_map.get(self.get_pc())
            }
            0x2F => { // CPL
                panic!("Instruction not implemented! CPL")
            }
            0x30 => { // JR NC, i8
                panic!("Instruction not implemented! JR NC, i8")
            }
            0x31 => { // LD SP,u16
                self.sp = self.get_pc_u16(&mut mem_map) as usize
            }
            0x32 => { // LD (HL-),A
                mem_map.set((self.get_16_register(Register::HL) - 1) as usize, self.a)
            }
            0x33 => { // INC SP
                self.sp = self.sp + 1
            }
            _ => panic!("Instruction not implemented!"),
        }
    }

    pub fn set_flag(&mut self, flag: Flag, value: bool) {
        match flag {
            Flag::Z => self.zf = value,
            Flag::N => self.nf = value,
            Flag::H => self.hf = value,
            Flag::C => self.cf = value,
        }
    }

    fn get_pc(&mut self) -> usize {
        let original: usize = self.pc;
        self.pc = self.pc + 1;
        original
    }

    // TODO - Possibly misleading function name
    fn get_pc_u16(&mut self, mem_map: &mut MemoryMap) -> u16 {
        let lower: u8 = mem_map.get(self.get_pc());
        let upper: u8 = mem_map.get(self.get_pc());
        ((upper as u16) << 8) | lower as u16
    }

    ////////////////////////////////////////////////////////////
    /// Implemented Instructions
    ////////////////////////////////////////////////////////////

    // ADC A,r8
    // ADC A,[HL]
    // ADC a,n8
    fn adc(&mut self, value: u8) {
        let cf: u8 = self.cf as u8;
        let result = self.a.wrapping_add(value).wrapping_add(cf);

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, add_half_carry(self.a, value, cf as u8));
        self.set_flag(Flag::C, (self.a as u16) + (value as u16) + (cf as u16) > 0xFF);

        self.a = result;
    }

    // ADD A,r8
    // ADD A,[HL]
    // ADD A,n8
    fn add_8(&mut self, value: u8) {
        let (result, overflow) = self.a.overflowing_add(value);

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, add_half_carry(self.a, value, 0));
        self.set_flag(Flag::C, overflow);

        self.a = result;
    }

    // AND A,r8
    // AND A,[HL]
    // AND A,n8
    fn and(&mut self, value: u8) {
        let result: u8 = self.a & value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, true);
        self.set_flag(Flag::C, false);

        self.a = result;
    }

    // CP A,r8
    // CP A,[HL]
    // CP A,n8
    fn cp(&mut self, value: u8) {
        let result: u8 = self.a - value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, sub_half_carry(self.a, value, 0));
        self.set_flag(Flag::C, value > self.a);
    }

    // DEC r8
    // DEC [HL]
    fn dec(&mut self, value: u8) -> u8 {
        let result: u8 = value - 1;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, sub_half_carry(value, 1, 0));

        result
    }

    // INC r8
    // INC [HL]
    fn inc(&mut self, value: u8) -> u8 {
        let result: u8 = value + 1;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, add_half_carry(value, 1, 0));

        result
    }

    // OR A,r8
    // OR A,[HL]
    // OR A, n8
    fn or(&mut self, value: u8) {
        let result: u8 = self.a | value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, false);

        self.a = result;
    }

    // SBC A,r8
    // SBC A,[HL]
    // SBC A,n8
    fn sbc(&mut self, value: u8) {
        let cf: u8 = self.cf as u8;
        let result = self.a.wrapping_sub(value).wrapping_sub(cf);

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, (self.a & 0x0F) < (value & 0x0F) + cf);
        self.set_flag(Flag::C, (self.a as u16) < (value as u16) + (cf as u16));

        self.a = result;
    }

    // SUB A,r8
    // SUB A,[HL]
    // SUB A,n8
    fn sub(&mut self, value: u8) {
        let result: u8 = self.a.wrapping_sub(value);

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, (self.a & 0x0F) < (value & 0x0F));
        self.set_flag(Flag::C, value > self.a);

        self.a = result;
    }

    // XOR A,r8
    // XOR A,[HL]
    // XOR A,n8
    fn xor(&mut self, value: u8) {
        let result: u8 = self.a ^ value;

        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::C, false);

        self.a = result;
    }

    // ADD HL,r16
    // ADD HL,SP
    fn add_hl(&mut self, value: u16) {
        let (result, overflow) = self.get_16_register(Register::HL).overflowing_add(value);

        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, add_u16_half_carry(self.get_16_register(Register::HL), value));
        self.set_flag(Flag::C, overflow);

        self.set_16_register(Register::HL, result);
    }

    // DEC r16
    fn dec_16(&self, value: u16) -> u16 {
        value - 1
    }

    // INC r16
    fn inc_16(&self, value: u16) -> u16 {
        value + 1
    }

    // BIT u3,r8
    // BIT u3,[HL]
    fn bit(&mut self, value: u8, bit: usize) {
        self.set_flag(Flag::Z, ((value >> bit) & 1) == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, true);
    }

    // RES u3,r8
    // RES u3, [HL]
    fn res(&self, value: u8, bit: usize) -> u8 {
        let mask: u8 = 1 << bit;
        value ^ mask
    }

    // SET u3,r8
    // SET u3,[HL]
    fn set(&self, value: u8, bit: usize) -> u8 {
        let mask: u8 = 1 << bit;
        value | mask
    }

    // SWAP r8
    // SWAP [HL]
    fn swap(&self, value: u8) -> u8 {
        value.rotate_left(4)
    }

    // RL r8
    // RL [HL]
    // RLA
    fn rl(&mut self, value: u8, is_rla: bool) -> u8 {
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
    fn rlc(&mut self, value: u8, is_rlca: bool) -> u8 {
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
    fn rr(&mut self, value: u8, is_rr: bool) -> u8 {
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
    fn rrc(&mut self, value: u8, is_rrca: bool) -> u8 {
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
    fn sla(&mut self, value: u8) -> u8 {
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
    fn sra(&mut self, value: u8) -> u8 {
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
    fn srl(&mut self, value: u8) -> u8 {
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
