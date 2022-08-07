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
            0x34 => { // INC (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                mem_map.set(self.get_hl_as_usize(), self.inc(value))
            }
            0x35 => { // DEC (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                mem_map.set(self.get_hl_as_usize(), self.dec(value))
            }
            0x36 => { // LD (HL), u8
                let value: u8 = mem_map.get(self.get_pc());
                mem_map.set(self.get_hl_as_usize(), value)
            }
            0x37 => { // SCF
                panic!("Instruction not implemented! SCF")
            }
            0x38 => {
                panic!("Instruction not implemented! JR C, i8")
            }
            0x39 => { // ADD HL, HL
                self.add_hl(self.sp as u16)
            }
            0x3A => { // LD A, (HL-)
                self.a = mem_map.get((self.get_16_register(Register::HL) - 1) as usize)
            }
            0x3B => { // DEC SP (TODO possible micro-bottlneck here)
                let value: u16 = self.dec_16(self.sp as u16);
                self.sp = value as usize
            }
            0x3C => { // INC A
                self.a = self.inc(self.a)
            }
            0x3D => { // DEC A
                self.a = self.dec(self.a)
            }
            0x3E => { // LD A, u8
                self.a = mem_map.get(self.get_pc())
            }
            0x3F => { // CCF
                panic!("Instruction not implemented! CCF")
            }
            0x40 => { // LD B, B (TODO: NOOP Instead??)
                self.b = self.b
            }
            0x41 => { // LD B, C
                self.b = self.c
            }
            0x42 => { // LD B, D
                self.b = self.d
            }
            0x43 => { // LD B, E
                self.b = self.e
            }
            0x44 => { // LD B, H
                self.b = self.h
            }
            0x45 => { // LD B, L
                self.b = self.l
            }
            0x46 => { // LD B, (HL)
                self.b = mem_map.get(self.get_hl_as_usize())
            }
            0x47 => { // LD B, A
                self.b = self.a
            }
            0x48 => { // LD C, B
                self.c = self.b
            }
            0x49 => { // LD C, C
                self.c = self.c
            }
            0x4A => { // LD C, D
                self.c = self.d
            }
            0x4B => { // LD C, E
                self.c = self.e
            }
            0x4C => { // LD C, H
                self.c = self.h
            }
            0x4D => { // LD C, L
                self.c = self.l
            }
            0x4E => { // LD C, (HL)
                self.c = mem_map.get(self.get_hl_as_usize())
            }
            0x4F => { // LD C, A
                self.c = self.a
            }
            0x50 => { // LD D, B
                self.d = self.b
            }
            0x51 => { // LD D, C
                self.d = self.c
            }
            0x52 => { // LD D, D
                self.d = self.d
            }
            0x53 => { // LD D, E
                self.d = self.e
            }
            0x54 => { // LD D, H
                self.d = self.h
            }
            0x55 => { // LD D, L
                self.d = self.l
            }
            0x56 => { // LD D, (HL)
                self.d = mem_map.get(self.get_hl_as_usize())
            }
            0x57 => { // LD D, A
                self.d = self.a
            }
            0x58 => { // LD E, B
                self.e = self.b
            }
            0x59 => { // LD E, C
                self.e = self.c
            }
            0x5A => { // LD E, D
                self.e = self.d
            }
            0x5B => { // LD E, E
                self.e = self.e
            }
            0x5C => { // LD E, H
                self.e = self.h
            }
            0x5D => { // LD E, L
                self.e = self.l
            }
            0x5E => { // LD E, (HL)
                self.e = mem_map.get(self.get_hl_as_usize())
            }
            0x5F => { // LD E, A
                self.e = self.a
            }
            0x60 => { // LD H, B
                self.h = self.b
            }
            0x61 => { // LD H, C
                self.h = self.c
            }
            0x62 => { // LD H, D
                self.h = self.d
            }
            0x63 => { // LD H, E
                self.h = self.e
            }
            0x64 => { // LD H, H
                self.h = self.h
            }
            0x65 => { // LD H, L
                self.h = self.l
            }
            0x66 => { // LD H, (HL)
                self.h = mem_map.get(self.get_hl_as_usize())
            }
            0x67 => { // LD H, A
                self.h = self.a
            }
            0x68 => { // LD L, B
                self.l = self.b
            }
            0x69 => { // LD L, C
                self.l = self.c
            }
            0x6A => { // LD L, D
                self.l = self.d
            }
            0x6B => { // LD L, E
                self.l = self.e
            }
            0x6C => { // LD L, H
                self.l = self.h
            }
            0x6D => { // LD L, L
                self.l = self.l
            }
            0x6E => { // LD L, (HL)
                self.l = mem_map.get(self.get_hl_as_usize())
            }
            0x6F => { // LD L, A
                self.l = self.a
            }
            0x70 => { // LD (HL), B
                mem_map.set(self.get_hl_as_usize(), self.b)
            }
            0x71 => { // LD (HL), C
                mem_map.set(self.get_hl_as_usize(), self.c)
            }
            0x72 => { // LD (HL), D
                mem_map.set(self.get_hl_as_usize(), self.d)
            }
            0x73 => { // LD (HL), E
                mem_map.set(self.get_hl_as_usize(), self.e)
            }
            0x74 => { // LD (HL), H
                mem_map.set(self.get_hl_as_usize(), self.h)
            }
            0x75 => { // LD (HL), L
                mem_map.set(self.get_hl_as_usize(), self.l)
            }
            0x76 => { // HALT
                panic!("Instruction not implemented! HALT")
            }
            0x77 => { // LD (HL), A
                mem_map.set(self.get_hl_as_usize(), self.a)
            }
            0x78 => { // LD A, B
                self.a = self.b
            }
            0x79 => { // LD A, C
                self.a = self.c
            }
            0x7A => { // LD A, D
                self.a = self.d
            }
            0x7B => { // LD A, E
                self.a = self.e
            }
            0x7C => { // LD A, H
                self.a = self.h
            }
            0x7D => { // LD A, L
                self.a = self.l
            }
            0x7E => { // LD A, (HL)
                self.a = mem_map.get(self.get_hl_as_usize())
            }
            0x7F => { // LD A, A
                self.a = self.a
            }
            0x80 => { // ADD A, B
                self.add_8(self.b)
            }
            0x81 => { // ADD A, C
                self.add_8(self.c)
            }
            0x82 => { // ADD A, D
                self.add_8(self.d)
            }
            0x83 => { // ADD A, E
                self.add_8(self.e)
            }
            0x84 => { // ADD A, H
                self.add_8(self.h)
            }
            0x85 => { // ADD A, L
                self.add_8(self.l)
            }
            0x86 => { // ADD A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.add_8(value)
            }
            0x87 => { // ADD A, A
                self.add_8(self.a)
            }
            0x88 => { // ADC A, B
                self.adc(self.b)
            }
            0x89 => { // ADC A, C
                self.adc(self.c)
            }
            0x8A => { // ADC A, D
                self.adc(self.d)
            }
            0x8B => { // ADC A, E
                self.adc(self.e)
            }
            0x8C => { // ADC A, H
                self.adc(self.h)
            }
            0x8D => { // ADC A, L
                self.adc(self.l)
            }
            0x8E => { // ADC A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.adc(value)
            }
            0x8F => { // ADC A, A
                self.adc(self.a)
            }
            0x90 => { // SUB A, B
                self.sub(self.b)
            }
            0x91 => { // SUB A, C
                self.sub(self.c)
            }
            0x92 => { // SUB A, D
                self.sub(self.d)
            }
            0x93 => { // SUB A, E
                self.sub(self.e)
            }
            0x94 => { // SUB A, H
                self.sub(self.h)
            }
            0x95 => { // SUB A, L
                self.sub(self.l)
            }
            0x96 => { // SUB A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.sub(value)
            }
            0x97 => { // SUB A, A
                self.sub(self.a)
            }
            0x98 => { // SBC A, B
                self.sbc(self.b)
            }
            0x99 => { // SBC A, C
                self.sbc(self.c)
            }
            0x9A => { // SBC A, D
                self.sbc(self.d)
            }
            0x9B => { // SBC A, E
                self.sbc(self.e)
            }
            0x9C => { // SBC A, H
                self.sbc(self.h)
            }
            0x9D => { // SBC A, L
                self.sbc(self.l)
            }
            0x9E => { // SBC A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.sbc(value)
            }
            0x9F => { // SBC A, A
                self.sbc(self.a)
            }
            0xA0 => { // AND A, B
                self.and(self.b)
            }
            0xA1 => { // AND A, C
                self.and(self.c)
            }
            0xA2 => { // AND A, D
                self.and(self.d)
            }
            0xA3 => { // AND A, E
                self.and(self.e)
            }
            0xA4 => { // AND A, H
                self.and(self.h)
            }
            0xA5 => { // AND A, L
                self.and(self.l)
            }
            0xA6 => { // AND A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.and(value)
            }
            0xA7 => { // AND A, A
                self.and(self.a)
            }
            0xA8 => { // XOR A, B
                self.xor(self.b)
            }
            0xA9 => { // XOR A, C
                self.xor(self.c)
            }
            0xAA => { // XOR A, D
                self.xor(self.d)
            }
            0xAB => { // XOR A, E
                self.xor(self.e)
            }
            0xAC => { // XOR A, H
                self.xor(self.h)
            }
            0xAD => { // XOR A, L
                self.xor(self.l)
            }
            0xAE => { // XOR A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.xor(value)
            }
            0xAF => { // XOR A, A
                self.xor(self.a)
            }
            0xB0 => { // OR A, B
                self.or(self.b)
            }
            0xB1 => { // OR A, C
                self.or(self.c)
            }
            0xB2 => { // OR A, D
                self.or(self.d)
            }
            0xB3 => { // OR A, E
                self.or(self.e)
            }
            0xB4 => { // OR A, H
                self.or(self.h)
            }
            0xB5 => { // OR A, L
                self.or(self.l)
            }
            0xB6 => { // OR A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.or(value)
            }
            0xB7 => { // OR A, A
                self.or(self.a)
            }
            0xB8 => { // CP A, B
                self.cp(self.b)
            }
            0xB9 => { // CP A, C
                self.cp(self.c)
            }
            0xBA => { // CP A, D
                self.cp(self.d)
            }
            0xBB => { // CP A, E
                self.cp(self.e)
            }
            0xBC => { // CP A, H
                self.cp(self.h)
            }
            0xBD => { // CP A, L
                self.cp(self.l)
            }
            0xBE => { // CP A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.cp(value)
            }
            0xBF => { // CP A, A
                self.cp(self.a)
            }
            0xC0 => {
                panic!("Instruction not implemented! RET NZ")
            }
            0xC1 => {
                panic!("Instruction not implemented! POP BC")
            }
            0xC2 => {
                panic!("Instruction not implemented! JP NZ, u16")
            }
            0xC3 => {
                panic!("Instruction not implemented! JP u16")
            }
            0xC4 => {
                panic!("Instruction not implemented! CALL NZ, u16")
            }
            0xC5 => {
                panic!("Instruction not implemented! PUSH BC")
            }
            0xC6 => {
                let value: u8 = mem_map.get(self.get_pc());
                self.add_8(value)
            }
            0xC7 => {
                panic!("Instruction not implemented! RST 00h")
            }
            0xC8 => {
                panic!("Instruction not implemented! RET Z")
            }
            0xC9 => {
                panic!("Instruction not implemented! RET")
            }
            0xCA => {
                panic!("Instruction not implemented! JP Z, u16")
            }
            0xCB => {
                let opcode: u8 = mem_map.get(self.get_pc());
                match opcode {
                    0x00 => { // RLC B
                        self.b = self.rlc(self.b, false)
                    }
                    0x01 => { // RLC C
                        self.c = self.rlc(self.c, false)
                    }
                    0x02 => { // RLC D
                        self.d = self.rlc(self.d, false)
                    }
                    0x03 => { // RLC E 
                        self.e = self.rlc(self.e, false)
                    }
                    0x04 => { // RLC H
                        self.h = self.rlc(self.h, false)
                    }
                    0x05 => { // RLC L
                        self.l = self.rlc(self.l, false)
                    }
                    0x06 => { // RLC (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.rlc(value, false);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x07 => { // RLC A
                        self.a = self.rlc(self.a, false)
                    }
                    0x08 => { // RRC B
                        self.b = self.rrc(self.b, false)
                    }
                    0x09 => { // RRC C
                        self.c = self.rrc(self.c, false)
                    }
                    0x0A => { // RRC D
                        self.d = self.rrc(self.d, false)
                    }
                    0x0B => { // RRC E 
                        self.e = self.rrc(self.e, false)
                    }
                    0x0C => { // RRC H
                        self.h = self.rrc(self.h, false)
                    }
                    0x0D => { // RRC L
                        self.l = self.rrc(self.l, false)
                    }
                    0x0E => { // RRC (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.rrc(value, false);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x0F => { // RRC A
                        self.a = self.rrc(self.a, false)
                    }
                    0x10 => { // RL B
                        self.b = self.rl(self.b, false)
                    }
                    0x11 => { // RL C
                        self.c = self.rl(self.c, false)
                    }
                    0x12 => { // RL D
                        self.d = self.rl(self.d, false)
                    }
                    0x13 => { // RL E 
                        self.e = self.rl(self.e, false)
                    }
                    0x14 => { // RL H
                        self.h = self.rl(self.h, false)
                    }
                    0x15 => { // RL L
                        self.l = self.rl(self.l, false)
                    }
                    0x16 => { // RL (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.rl(value, false);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x17 => { // RL A
                        self.a = self.rl(self.a, false)
                    }
                    0x18 => { // RR B
                        self.b = self.rr(self.b, false)
                    }
                    0x19 => { // RR C
                        self.c = self.rr(self.c, false)
                    }
                    0x1A => { // RR D
                        self.d = self.rr(self.d, false)
                    }
                    0x1B => { // RR E 
                        self.e = self.rr(self.e, false)
                    }
                    0x1C => { // RR H
                        self.h = self.rr(self.h, false)
                    }
                    0x1D => { // RR L
                        self.l = self.rr(self.l, false)
                    }
                    0x1E => { // RR (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.rr(value, false);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x1F => { // RR A
                        self.a = self.rr(self.a, false)
                    }
                    0x20 => { // SLA B
                        self.b = self.sla(self.b)
                    }
                    0x21 => { // SLA C
                        self.c = self.sla(self.c)
                    }
                    0x22 => { // SLA D
                        self.d = self.sla(self.d)
                    }
                    0x23 => { // SLA E 
                        self.e = self.sla(self.e)
                    }
                    0x24 => { // SLA H
                        self.h = self.sla(self.h)
                    }
                    0x25 => { // SLA L
                        self.l = self.sla(self.l)
                    }
                    0x26 => { // SLA (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.sla(value);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x27 => { // SLA A
                        self.a = self.sla(self.a)
                    }
                    0x28 => { // SRA B
                        self.b = self.sra(self.b)
                    }
                    0x29 => { // SRA C
                        self.c = self.sra(self.c)
                    }
                    0x2A => { // SRA D
                        self.d = self.sra(self.d)
                    }
                    0x2B => { // SRA E 
                        self.e = self.sra(self.e)
                    }
                    0x2C => { // SRA H
                        self.h = self.sra(self.h)
                    }
                    0x2D => { // SRA L
                        self.l = self.sra(self.l)
                    }
                    0x2E => { // SRA (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.sra(value);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x2F => { // SRA A
                        self.a = self.sra(self.a)
                    }
                    0x30 => { // SWAP B
                        self.b = self.swap(self.b)
                    }
                    0x31 => { // SWAP C
                        self.c = self.swap(self.c)
                    }
                    0x32 => { // SWAP D
                        self.d = self.swap(self.d)
                    }
                    0x33 => { // SWAP E 
                        self.e = self.swap(self.e)
                    }
                    0x34 => { // SWAP H
                        self.h = self.swap(self.h)
                    }
                    0x35 => { // SWAP L
                        self.l = self.swap(self.l)
                    }
                    0x36 => { // SWAP (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.swap(value);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x37 => { // SWAP A
                        self.a = self.swap(self.a)
                    }
                    0x38 => { // SRL B
                        self.b = self.srl(self.b)
                    }
                    0x39 => { // SRL C
                        self.c = self.srl(self.c)
                    }
                    0x3A => { // SRL D
                        self.d = self.srl(self.d)
                    }
                    0x3B => { // SRL E 
                        self.e = self.srl(self.e)
                    }
                    0x3C => { // SRL H
                        self.h = self.srl(self.h)
                    }
                    0x3D => { // SRL L
                        self.l = self.srl(self.l)
                    }
                    0x3E => { // SRL (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.srl(value);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x3F => { // SRL A
                        self.a = self.srl(self.a)
                    }
                    0x40 => { // BIT 0, B
                        self.bit(self.b, 0)
                    }
                    0x41 => { // BIT 0, C
                        self.bit(self.c, 0)
                    }
                    0x42 => { // BIT 0, D
                        self.bit(self.d, 0)
                    }
                    0x43 => { // BIT 0, E 
                        self.bit(self.e, 0)
                    }
                    0x44 => { // BIT 0, H
                        self.bit(self.h, 0)
                    }
                    0x45 => { // BIT 0, L
                        self.bit(self.l, 0)
                    }
                    0x46 => { // BIT 0, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 0)
                    }
                    0x47 => { // BIT 0, A
                        self.bit(self.a, 0)
                    }
                    0x48 => { // BIT 1, B
                        self.bit(self.b, 1)
                    }
                    0x49 => { // BIT 1, C
                        self.bit(self.c, 1)
                    }
                    0x4A => { // BIT 1, D
                        self.bit(self.d, 1)
                    }
                    0x4B => { // BIT 1, E 
                        self.bit(self.e, 1)
                    }
                    0x4C => { // BIT 1, H
                        self.bit(self.h, 1)
                    }
                    0x4D => { // BIT 1, L
                        self.bit(self.l, 1)
                    }
                    0x4E => { // BIT 1, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 1)
                    }
                    0x4F => { // BIT 1, A
                        self.bit(self.a, 1)
                    }
                    0x50 => { // BIT 2, B
                        self.bit(self.b, 2)
                    }
                    0x51 => { // BIT 2, C
                        self.bit(self.c, 2)
                    }
                    0x52 => { // BIT 2, D
                        self.bit(self.d, 2)
                    }
                    0x53 => { // BIT 2, E 
                        self.bit(self.e, 2)
                    }
                    0x54 => { // BIT 2, H
                        self.bit(self.h, 2)
                    }
                    0x55 => { // BIT 2, L
                        self.bit(self.l, 2)
                    }
                    0x56 => { // BIT 2, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 2)
                    }
                    0x57 => { // BIT 2, A
                        self.bit(self.a, 2)
                    }
                    0x58 => { // BIT 3, B
                        self.bit(self.b, 3)
                    }
                    0x59 => { // BIT 3, C
                        self.bit(self.c, 3)
                    }
                    0x5A => { // BIT 3, D
                        self.bit(self.d, 3)
                    }
                    0x5B => { // BIT 3, E 
                        self.bit(self.e, 3)
                    }
                    0x5C => { // BIT 3, H
                        self.bit(self.h, 3)
                    }
                    0x5D => { // BIT 3, L
                        self.bit(self.l, 3)
                    }
                    0x5E => { // BIT 3, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 3)
                    }
                    0x5F => { // BIT 3, A
                        self.bit(self.a, 3)
                    }
                    0x60 => { // BIT 4, B
                        self.bit(self.b, 4)
                    }
                    0x61 => { // BIT 4, C
                        self.bit(self.c, 4)
                    }
                    0x62 => { // BIT 4, D
                        self.bit(self.d, 4)
                    }
                    0x63 => { // BIT 4, E 
                        self.bit(self.e, 4)
                    }
                    0x64 => { // BIT 4, H
                        self.bit(self.h, 4)
                    }
                    0x65 => { // BIT 4, L
                        self.bit(self.l, 4)
                    }
                    0x66 => { // BIT 4, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 4)
                    }
                    0x67 => { // BIT 4, A
                        self.bit(self.a, 4)
                    }
                    0x68 => { // BIT 5, B
                        self.bit(self.b, 5)
                    }
                    0x69 => { // BIT 5, C
                        self.bit(self.c, 5)
                    }
                    0x6A => { // BIT 5, D
                        self.bit(self.d, 5)
                    }
                    0x6B => { // BIT 5, E 
                        self.bit(self.e, 5)
                    }
                    0x6C => { // BIT 5, H
                        self.bit(self.h, 5)
                    }
                    0x6D => { // BIT 5, L
                        self.bit(self.l, 5)
                    }
                    0x6E => { // BIT 5, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 5)
                    }
                    0x6F => { // BIT 5, A
                        self.bit(self.a, 5)
                    }
                    0x70 => { // BIT 6, B
                        self.bit(self.b, 6)
                    }
                    0x71 => { // BIT 6, C
                        self.bit(self.c, 6)
                    }
                    0x72 => { // BIT 6, D
                        self.bit(self.d, 6)
                    }
                    0x73 => { // BIT 6, E 
                        self.bit(self.e, 6)
                    }
                    0x74 => { // BIT 6, H
                        self.bit(self.h, 6)
                    }
                    0x75 => { // BIT 6, L
                        self.bit(self.l, 6)
                    }
                    0x76 => { // BIT 6, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 6)
                    }
                    0x77 => { // BIT 6, A
                        self.bit(self.a, 6)
                    }
                    0x78 => { // BIT 7, B
                        self.bit(self.b, 7)
                    }
                    0x79 => { // BIT 7, C
                        self.bit(self.c, 7)
                    }
                    0x7A => { // BIT 7, D
                        self.bit(self.d, 7)
                    }
                    0x7B => { // BIT 7, E 
                        self.bit(self.e, 7)
                    }
                    0x7C => { // BIT 7, H
                        self.bit(self.h, 7)
                    }
                    0x7D => { // BIT 7, L
                        self.bit(self.l, 7)
                    }
                    0x7E => { // BIT 7, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 7)
                    }
                    0x7F => { // BIT 7, A
                        self.bit(self.a, 7)
                    }
                    0x80 => { // RES 0, B
                        self.b = self.res(self.b, 0)
                    }
                    0x81 => { // RES 0, C
                        self.c = self.res(self.c, 0)
                    }
                    0x82 => { // RES 0, D
                        self.d = self.res(self.d, 0)
                    }
                    0x83 => { // RES 0, E 
                        self.e = self.res(self.e, 0)
                    }
                    0x84 => { // RES 0, H
                        self.h = self.res(self.h, 0)
                    }
                    0x85 => { // RES 0, L
                        self.l = self.res(self.l, 0)
                    }
                    0x86 => { // RES 0, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 0);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x87 => { // RES 0, A
                        self.a = self.res(self.a, 0)
                    }
                    0x88 => { // RES 1, B
                        self.b = self.res(self.b, 1)
                    }
                    0x89 => { // RES 1, C
                        self.c = self.res(self.c, 1)
                    }
                    0x8A => { // RES 1, D
                        self.d = self.res(self.d, 1)
                    }
                    0x8B => { // RES 1, E 
                        self.e = self.res(self.e, 1)
                    }
                    0x8C => { // RES 1, H
                        self.h = self.res(self.h, 1)
                    }
                    0x8D => { // RES 1, L
                        self.l = self.res(self.l, 1)
                    }
                    0x8E => { // RES 1, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 1);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x8F => { // RES 1, A
                        self.a = self.res(self.a, 1)
                    }
                    0x90 => { // RES 2, B
                        self.b = self.res(self.b, 2)
                    }
                    0x91 => { // RES 2, C
                        self.c = self.res(self.c, 2)
                    }
                    0x92 => { // RES 2, D
                        self.d = self.res(self.d, 2)
                    }
                    0x93 => { // RES 2, E 
                        self.e = self.res(self.e, 2)
                    }
                    0x94 => { // RES 2, H
                        self.h = self.res(self.h, 2)
                    }
                    0x95 => { // RES 2, L
                        self.l = self.res(self.l, 2)
                    }
                    0x96 => { // RES 2, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 2);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x97 => { // RES 2, A
                        self.a = self.res(self.a, 2)
                    }
                    0x98 => { // RES 3, B
                        self.b = self.res(self.b, 3)
                    }
                    0x99 => { // RES 3, C
                        self.c = self.res(self.c, 3)
                    }
                    0x9A => { // RES 3, D
                        self.d = self.res(self.d, 3)
                    }
                    0x9B => { // RES 3, E 
                        self.e = self.res(self.e, 3)
                    }
                    0x9C => { // RES 3, H
                        self.h = self.res(self.h, 3)
                    }
                    0x9D => { // RES 3, L
                        self.l = self.res(self.l, 3)
                    }
                    0x9E => { // RES 3, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 3);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x9F => { // RES 3, A
                        self.a = self.res(self.a, 3)
                    }
                    0xA0 => { // RES 4, B
                        self.b = self.res(self.b, 4)
                    }
                    0xA1 => { // RES 4, C
                        self.c = self.res(self.c, 4)
                    }
                    0xA2 => { // RES 4, D
                        self.d = self.res(self.d, 4)
                    }
                    0xA3 => { // RES 4, E 
                        self.e = self.res(self.e, 4)
                    }
                    0xA4 => { // RES 4, H
                        self.h = self.res(self.h, 4)
                    }
                    0xA5 => { // RES 4, L
                        self.l = self.res(self.l, 4)
                    }
                    0xA6 => { // RES 4, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 4);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xA7 => { // RES 4, A
                        self.a = self.res(self.a, 4)
                    }
                    0xA8 => { // RES 5, B
                        self.b = self.res(self.b, 5)
                    }
                    0xA9 => { // RES 5, C
                        self.c = self.res(self.c, 5)
                    }
                    0xAA => { // RES 5, D
                        self.d = self.res(self.d, 5)
                    }
                    0xAB => { // RES 5, E 
                        self.e = self.res(self.e, 5)
                    }
                    0xAC => { // RES 5, H
                        self.h = self.res(self.h, 5)
                    }
                    0xAD => { // RES 5, L
                        self.l = self.res(self.l, 5)
                    }
                    0xAE => { // RES 5, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 5);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xAF => { // RES 5, A
                        self.a = self.res(self.a, 5)
                    }
                    0xB0 => { // RES 6, B
                        self.b = self.res(self.b, 6)
                    }
                    0xB1 => { // RES 6, C
                        self.c = self.res(self.c, 6)
                    }
                    0xB2 => { // RES 6, D
                        self.d = self.res(self.d, 6)
                    }
                    0xB3 => { // RES 6, E 
                        self.e = self.res(self.e, 6)
                    }
                    0xB4 => { // RES 6, H
                        self.h = self.res(self.h, 6)
                    }
                    0xB5 => { // RES 6, L
                        self.l = self.res(self.l, 6)
                    }
                    0xB6 => { // RES 6, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 6);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xB7 => { // RES 6, A
                        self.a = self.res(self.a, 6)
                    }
                    0xB8 => { // RES 7, B
                        self.b = self.res(self.b, 7)
                    }
                    0xB9 => { // RES 7, C
                        self.c = self.res(self.c, 7)
                    }
                    0xBA => { // RES 7, D
                        self.d = self.res(self.d, 7)
                    }
                    0xBB => { // RES 7, E 
                        self.e = self.res(self.e, 7)
                    }
                    0xBC => { // RES 7, H
                        self.h = self.res(self.h, 7)
                    }
                    0xBD => { // RES 7, L
                        self.l = self.res(self.l, 7)
                    }
                    0xBE => { // RES 7, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 7);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xBF => { // RES 7, A
                        self.a = self.res(self.a, 7)
                    }
                    0xC0 => { // SET 0, B
                        self.b = self.set(self.b, 0)
                    }
                    0xC1 => { // SET 0, C
                        self.c = self.set(self.c, 0)
                    }
                    0xC2 => { // SET 0, D
                        self.d = self.set(self.d, 0)
                    }
                    0xC3 => { // SET 0, E 
                        self.e = self.set(self.e, 0)
                    }
                    0xC4 => { // SET 0, H
                        self.h = self.set(self.h, 0)
                    }
                    0xC5 => { // SET 0, L
                        self.l = self.set(self.l, 0)
                    }
                    0xC6 => { // SET 0, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 0);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xC7 => { // SET 0, A
                        self.a = self.set(self.a, 0)
                    }
                    0xC8 => { // SET 1, B
                        self.b = self.set(self.b, 1)
                    }
                    0xC9 => { // SET 1, C
                        self.c = self.set(self.c, 1)
                    }
                    0xCA => { // SET 1, D
                        self.d = self.set(self.d, 1)
                    }
                    0xCB => { // SET 1, E 
                        self.e = self.set(self.e, 1)
                    }
                    0xCC => { // SET 1, H
                        self.h = self.set(self.h, 1)
                    }
                    0xCD => { // SET 1, L
                        self.l = self.set(self.l, 1)
                    }
                    0xCE => { // SET 1, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 1);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xCF => { // SET 1, A
                        self.a = self.set(self.a, 1)
                    }
                    0xD0 => { // SET 2, B
                        self.b = self.set(self.b, 2)
                    }
                    0xD1 => { // SET 2, C
                        self.c = self.set(self.c, 2)
                    }
                    0xD2 => { // SET 2, D
                        self.d = self.set(self.d, 2)
                    }
                    0xD3 => { // SET 2, E 
                        self.e = self.set(self.e, 2)
                    }
                    0xD4 => { // SET 2, H
                        self.h = self.set(self.h, 2)
                    }
                    0xD5 => { // SET 2, L
                        self.l = self.set(self.l, 2)
                    }
                    0xD6 => { // SET 2, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 2);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xD7 => { // SET 2, A
                        self.a = self.set(self.a, 2)
                    }
                    0xD8 => { // SET 3, B
                        self.b = self.set(self.b, 3)
                    }
                    0xD9 => { // SET 3, C
                        self.c = self.set(self.c, 3)
                    }
                    0xDA => { // SET 3, D
                        self.d = self.set(self.d, 3)
                    }
                    0xDB => { // SET 3, E 
                        self.e = self.set(self.e, 3)
                    }
                    0xDC => { // SET 3, H
                        self.h = self.set(self.h, 3)
                    }
                    0xDD => { // SET 3, L
                        self.l = self.set(self.l, 3)
                    }
                    0xDE => { // SET 3, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 3);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xDF => { // SET 3, A
                        self.a = self.set(self.a, 3)
                    }
                    0xE0 => { // SET 4, B
                        self.b = self.set(self.b, 4)
                    }
                    0xE1 => { // SET 4, C
                        self.c = self.set(self.c, 4)
                    }
                    0xE2 => { // SET 4, D
                        self.d = self.set(self.d, 4)
                    }
                    0xE3 => { // SET 4, E 
                        self.e = self.set(self.e, 4)
                    }
                    0xE4 => { // SET 4, H
                        self.h = self.set(self.h, 4)
                    }
                    0xE5 => { // SET 4, L
                        self.l = self.set(self.l, 4)
                    }
                    0xE6 => { // SET 4, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 4);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xE7 => { // SET 4, A
                        self.a = self.set(self.a, 4)
                    }
                    0xE8 => { // SET 5, B
                        self.b = self.set(self.b, 5)
                    }
                    0xE9 => { // SET 5, C
                        self.c = self.set(self.c, 5)
                    }
                    0xEA => { // SET 5, D
                        self.d = self.set(self.d, 5)
                    }
                    0xEB => { // SET 5, E 
                        self.e = self.set(self.e, 5)
                    }
                    0xEC => { // SET 5, H
                        self.h = self.set(self.h, 5)
                    }
                    0xED => { // SET 5, L
                        self.l = self.set(self.l, 5)
                    }
                    0xEE => { // SET 5, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 5);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xEF => { // SET 5, A
                        self.a = self.set(self.a, 5)
                    }
                    0xF0 => { // SET 6, B
                        self.b = self.set(self.b, 6)
                    }
                    0xF1 => { // SET 6, C
                        self.c = self.set(self.c, 6)
                    }
                    0xF2 => { // SET 6, D
                        self.d = self.set(self.d, 6)
                    }
                    0xF3 => { // SET 6, E 
                        self.e = self.set(self.e, 6)
                    }
                    0xF4 => { // SET 6, H
                        self.h = self.set(self.h, 6)
                    }
                    0xF5 => { // SET 6, L
                        self.l = self.set(self.l, 6)
                    }
                    0xF6 => { // SET 6, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 6);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xF7 => { // SET 6, A
                        self.a = self.set(self.a, 6)
                    }
                    0xF8 => { // SET 7, B
                        self.b = self.set(self.b, 7)
                    }
                    0xF9 => { // SET 7, C
                        self.c = self.set(self.c, 7)
                    }
                    0xFA => { // SET 7, D
                        self.d = self.set(self.d, 7)
                    }
                    0xFB => { // SET 7, E 
                        self.e = self.set(self.e, 7)
                    }
                    0xFC => { // SET 7, H
                        self.h = self.set(self.h, 7)
                    }
                    0xFD => { // SET 7, L
                        self.l = self.set(self.l, 7)
                    }
                    0xFE => { // SET 7, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 7);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xFF => { // SET 7, A
                        self.a = self.set(self.a, 7)
                    }
                    _ => panic!("Instruction not implemented!"),
                }
            }
            _ => panic!("Instruction not implemented!"),
        }
    }

    fn set_flag(&mut self, flag: Flag, value: bool) {
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

    fn get_hl_as_usize(&mut self) -> usize {
        self.get_16_register(Register::HL) as usize
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
    fn rr(&mut self, value: u8, is_rra: bool) -> u8 {
        let overflow: bool = (value & 1) == 1;
        let mut result: u8 = value.shr(1);

        if self.cf {
            result = result | 0x80;
        }

        self.set_flag(Flag::Z, !is_rra && result == 0);
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
