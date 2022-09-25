use std::ops::{Shl, Shr};
use Flag::*;
use Register::*;

use crate::memory_map::MemoryMap;

pub enum Register {
    AF,
    BC,
    DE,
    HL,
}

pub enum Flag {
    Z, // Zero
    N, // Negative
    H, // Half-Carry
    C, // Carry
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
            AF => {
                let f: u8 =
                    ((self.zf as u8) << 7) | ((self.nf as u8) << 6) | ((self.hf as u8) << 5) | ((self.cf as u8) << 4);
                ((self.a as u16) << 8) | f as u16
            }
            BC => ((self.b as u16) << 8) | self.c as u16,
            DE => ((self.d as u16) << 8) | self.e as u16,
            HL => ((self.h as u16) << 8) | self.l as u16,
        }
    }

    pub fn set_16_register(&mut self, register: Register, value: u16) {
        match register {
            AF => {
                self.a = (value >> 8) as u8;
                self.zf = (((value & 0b10000000) as u8) >> 7) != 0;
                self.nf = (((value & 0b01000000) as u8) >> 6) != 0;
                self.hf = (((value & 0b00100000) as u8) >> 5) != 0;
                self.cf = (((value & 0b00010000) as u8) >> 4) != 0;
            }
            BC => {
                self.b = (value >> 8) as u8;
                self.c = (value & 0x00FF) as u8;
            }
            DE => {
                self.d = (value >> 8) as u8;
                self.e = (value & 0x00FF) as u8;
            }
            HL => {
                self.h = (value >> 8) as u8;
                self.l = (value & 0x00FF) as u8;
            }
        }
    }

    fn set_f(&mut self, value: u8) {
        self.zf = (((value & 0b10000000) as u8) >> 7) != 0;
        self.nf = (((value & 0b01000000) as u8) >> 6) != 0;
        self.hf = (((value & 0b00100000) as u8) >> 5) != 0;
        self.cf = (((value & 0b00010000) as u8) >> 4) != 0;
    }

    fn get_f(&mut self) -> u8 {
        (self.zf as u8) << 7 | (self.nf as u8) << 6 | (self.hf as u8) << 5 | (self.cf as u8) << 4
    }

    pub fn execute(&mut self, mut mem_map: MemoryMap) {
        let opcode: u8 = mem_map.get(self.get_pc());
        match opcode {
            0x00 => {} // NOOP
            0x01 => {
                // LD BC,u16
                let value: u16 = self.get_pc_u16(&mut mem_map);
                self.set_16_register(BC, value)
            }
            0x02 => {
                // LD (BC),A
                mem_map.set(self.get_16_register(BC) as usize, self.a)
            }
            0x03 => {
                // INC BC
                self.set_16_register(BC, self.inc_16(self.get_16_register(BC)))
            }
            0x04 => self.b = self.inc(self.b),           // INC B
            0x05 => self.b = self.dec(self.b),           // DEC B
            0x06 => self.b = mem_map.get(self.get_pc()), // LD B, u8
            0x07 => self.a = self.rlc(self.a, true),     // RLCA
            0x08 => {
                // LD (u16),SP
                let index: u16 = self.get_pc_u16(&mut mem_map);
                mem_map.set_u16(index as usize, self.sp as u16)
            }
            0x09 => self.add_hl(self.get_16_register(BC)), // ADD HL, BC
            0x0A => self.a = mem_map.get(self.get_16_register(BC) as usize), // LD A, (BC)
            0x0B => {
                // DEC BC (TODO possible micro-bottlneck here)
                let value: u16 = self.dec_16(self.get_16_register(BC));
                self.set_16_register(BC, value)
            }
            0x0C => self.c = self.inc(self.c),                   // INC C
            0x0D => self.c = self.dec(self.c),                   // DEC C
            0x0E => self.c = mem_map.get(self.get_pc()),         // LD C, u8
            0x0F => self.a = self.rrc(self.a, true),             // RRCA
            0x10 => panic!("Instruction not implemented! STOP"), // STOP
            0x11 => {
                // LD DE,u16
                let value: u16 = self.get_pc_u16(&mut mem_map);
                self.set_16_register(DE, value)
            }
            0x12 => {
                // LD (DE),A
                mem_map.set(self.get_16_register(DE) as usize, self.a)
            }
            0x13 => {
                // INC DE
                self.set_16_register(DE, self.inc_16(self.get_16_register(DE)))
            }
            0x14 => self.d = self.inc(self.d),                               // INC D
            0x15 => self.d = self.dec(self.d),                               // DEC D
            0x16 => self.d = mem_map.get(self.get_pc()),                     // LD D, u8
            0x17 => self.a = self.rl(self.a, true),                          // RLA
            0x18 => self.jr(&mut mem_map),                                   // JR i8R i8
            0x19 => self.add_hl(self.get_16_register(DE)),                   // ADD HL, DE
            0x1A => self.a = mem_map.get(self.get_16_register(DE) as usize), // LD A, (DE)
            0x1B => {
                // DEC DE (TODO possible micro-bottlneck here)
                let value: u16 = self.dec_16(self.get_16_register(DE));
                self.set_16_register(DE, value)
            }
            0x1C => self.e = self.inc(self.e),           // INC E
            0x1D => self.e = self.dec(self.e),           // DEC E
            0x1E => self.e = mem_map.get(self.get_pc()), // LD E, u8
            0x1F => self.a = self.rr(self.a, true),      // RRA
            0x20 => {
                // JR NZ, i8
                let value: i8 = mem_map.get(self.get_pc()) as i8;
                if !self.zf {
                    self.jr_cond(value)
                }
            }
            0x21 => {
                // LD HL,u16
                let value: u16 = self.get_pc_u16(&mut mem_map);
                self.set_16_register(HL, value)
            }
            0x22 => {
                // LD (HL+),A
                mem_map.set((self.get_16_register(HL) + 1) as usize, self.a)
            }
            0x23 => {
                // INC HL
                self.set_16_register(HL, self.inc_16(self.get_16_register(HL)))
            }
            0x24 => self.h = self.inc(self.h),                  // INC H
            0x25 => self.h = self.dec(self.h),                  // DEC H
            0x26 => self.h = mem_map.get(self.get_pc()),        // LD H, u8
            0x27 => panic!("Instruction not implemented! DAA"), // DAA
            0x28 => {
                // JR Z, i8
                let value: i8 = mem_map.get(self.get_pc()) as i8;
                if self.zf {
                    self.jr_cond(value)
                }
            }
            0x29 => self.add_hl(self.get_16_register(HL)), // ADD HL, HL
            0x2A => {
                // LD A, (HL+)
                self.a = mem_map.get((self.get_16_register(HL) + 1) as usize)
            }
            0x2B => {
                // DEC HL (TODO possible micro-bottlneck here)
                let value: u16 = self.dec_16(self.get_16_register(HL));
                self.set_16_register(HL, value)
            }
            0x2C => self.l = self.inc(self.l),           // INC L
            0x2D => self.l = self.dec(self.l),           // DEC L
            0x2E => self.l = mem_map.get(self.get_pc()), // LD L, u8
            0x2F => self.cpl(),                          // CPL
            0x30 => {
                // JR NC, i8
                let value: i8 = mem_map.get(self.get_pc()) as i8;
                if !self.cf {
                    self.jr_cond(value)
                }
            }
            0x31 => self.sp = self.get_pc_u16(&mut mem_map) as usize, // LD SP,u16
            0x32 => {
                // LD (HL-),A
                mem_map.set((self.get_16_register(HL) - 1) as usize, self.a)
            }
            0x33 => self.sp = self.sp + 1, // INC SP
            0x34 => {
                // INC (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                mem_map.set(self.get_hl_as_usize(), self.inc(value))
            }
            0x35 => {
                // DEC (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                mem_map.set(self.get_hl_as_usize(), self.dec(value))
            }
            0x36 => {
                // LD (HL), u8
                let value: u8 = mem_map.get(self.get_pc());
                mem_map.set(self.get_hl_as_usize(), value)
            }
            0x37 => self.scf(), // SCF
            0x38 => {
                // JR C, i8
                let value: i8 = mem_map.get(self.get_pc()) as i8;
                if self.cf {
                    self.jr_cond(value)
                }
            }
            0x39 => self.add_hl(self.sp as u16), // ADD HL, SP
            0x3A => {
                // LD A, (HL-)
                self.a = mem_map.get((self.get_16_register(HL) - 1) as usize)
            }
            0x3B => {
                // DEC SP (TODO possible micro-bottlneck here)
                let value: u16 = self.dec_16(self.sp as u16);
                self.sp = value as usize
            }
            0x3C => self.a = self.inc(self.a),                    // INC A
            0x3D => self.a = self.dec(self.a),                    // DEC A
            0x3E => self.a = mem_map.get(self.get_pc()),          // LD A, u8
            0x3F => self.ccf(),                                   // CCF
            0x40 => self.b = self.b,                              // LD B, B (TODO: NOOP Instead??)
            0x41 => self.b = self.c,                              // LD B, C
            0x42 => self.b = self.d,                              // LD B, D
            0x43 => self.b = self.e,                              // LD B, E
            0x44 => self.b = self.h,                              // LD B, H
            0x45 => self.b = self.l,                              // LD B, L
            0x46 => self.b = mem_map.get(self.get_hl_as_usize()), // LD B, (HL)
            0x47 => self.b = self.a,                              // LD B, A
            0x48 => self.c = self.b,                              // LD C, B
            0x49 => self.c = self.c,                              // LD C, C
            0x4A => self.c = self.d,                              // LD C, D
            0x4B => self.c = self.e,                              // LD C, E
            0x4C => self.c = self.h,                              // LD C, H
            0x4D => self.c = self.l,                              // LD C, L
            0x4E => self.c = mem_map.get(self.get_hl_as_usize()), // LD C, (HL)
            0x4F => self.c = self.a,                              // LD C, A
            0x50 => self.d = self.b,                              // LD D, B
            0x51 => self.d = self.c,                              // LD D, C
            0x52 => self.d = self.d,                              // LD D, D
            0x53 => self.d = self.e,                              // LD D, E
            0x54 => self.d = self.h,                              // LD D, H
            0x55 => self.d = self.l,                              // LD D, L
            0x56 => self.d = mem_map.get(self.get_hl_as_usize()), // LD D, (HL)
            0x57 => self.d = self.a,                              // LD D, A
            0x58 => self.e = self.b,                              // LD E, B
            0x59 => self.e = self.c,                              // LD E, C
            0x5A => self.e = self.d,                              // LD E, D
            0x5B => self.e = self.e,                              // LD E, E
            0x5C => self.e = self.h,                              // LD E, H
            0x5D => self.e = self.l,                              // LD E, L
            0x5E => self.e = mem_map.get(self.get_hl_as_usize()), // LD E, (HL)
            0x5F => self.e = self.a,                              // LD E, A
            0x60 => self.h = self.b,                              // LD H, B
            0x61 => self.h = self.c,                              // LD H, C
            0x62 => self.h = self.d,                              // LD H, D
            0x63 => self.h = self.e,                              // LD H, E
            0x64 => self.h = self.h,                              // LD H, H
            0x65 => self.h = self.l,                              // LD H, L
            0x66 => self.h = mem_map.get(self.get_hl_as_usize()), // LD H, (HL)
            0x67 => self.h = self.a,                              // LD H, A
            0x68 => self.l = self.b,                              // LD L, B
            0x69 => self.l = self.c,                              // LD L, C
            0x6A => self.l = self.d,                              // LD L, D
            0x6B => self.l = self.e,                              // LD L, E
            0x6C => self.l = self.h,                              // LD L, H
            0x6D => self.l = self.l,                              // LD L, L
            0x6E => self.l = mem_map.get(self.get_hl_as_usize()), // LD L, (HL)
            0x6F => self.l = self.a,                              // LD L, A
            0x70 => mem_map.set(self.get_hl_as_usize(), self.b),  // LD (HL), B
            0x71 => mem_map.set(self.get_hl_as_usize(), self.c),  // LD (HL), C
            0x72 => mem_map.set(self.get_hl_as_usize(), self.d),  // LD (HL), D
            0x73 => mem_map.set(self.get_hl_as_usize(), self.e),  // LD (HL), E
            0x74 => mem_map.set(self.get_hl_as_usize(), self.h),  // LD (HL), H
            0x75 => mem_map.set(self.get_hl_as_usize(), self.l),  // LD (HL), L
            0x76 => panic!("Instruction not implemented! HALT"),  // HALT
            0x77 => mem_map.set(self.get_hl_as_usize(), self.a),  // LD (HL), A
            0x78 => self.a = self.b,                              // LD A, B
            0x79 => self.a = self.c,                              // LD A, C
            0x7A => self.a = self.d,                              // LD A, D
            0x7B => self.a = self.e,                              // LD A, E
            0x7C => self.a = self.h,                              // LD A, H
            0x7D => self.a = self.l,                              // LD A, L
            0x7E => self.a = mem_map.get(self.get_hl_as_usize()), // LD A, (HL)
            0x7F => self.a = self.a,                              // LD A, A
            0x80 => self.add_8(self.b),                           // ADD A, B
            0x81 => self.add_8(self.c),                           // ADD A, C
            0x82 => self.add_8(self.d),                           // ADD A, D
            0x83 => self.add_8(self.e),                           // ADD A, E
            0x84 => self.add_8(self.h),                           // ADD A, H
            0x85 => self.add_8(self.l),                           // ADD A, L
            0x86 => {
                // ADD A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.add_8(value)
            }
            0x87 => self.add_8(self.a), // ADD A, A
            0x88 => self.adc(self.b),   // ADC A, B
            0x89 => self.adc(self.c),   // ADC A, C
            0x8A => self.adc(self.d),   // ADC A, D
            0x8B => self.adc(self.e),   // ADC A, E
            0x8C => self.adc(self.h),   // ADC A, H
            0x8D => self.adc(self.l),   // ADC A, L
            0x8E => {
                // ADC A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.adc(value)
            }
            0x8F => self.adc(self.a), // ADC A, A
            0x90 => self.sub(self.b), // SUB A, B
            0x91 => self.sub(self.c), // SUB A, C
            0x92 => self.sub(self.d), // SUB A, D
            0x93 => self.sub(self.e), // SUB A, E
            0x94 => self.sub(self.h), // SUB A, H
            0x95 => self.sub(self.l), // SUB A, L
            0x96 => {
                // SUB A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.sub(value)
            }
            0x97 => self.sub(self.a), // SUB A, A
            0x98 => self.sbc(self.b), // SBC A, B
            0x99 => self.sbc(self.c), // SBC A, C
            0x9A => self.sbc(self.d), // SBC A, D
            0x9B => self.sbc(self.e), // SBC A, E
            0x9C => self.sbc(self.h), // SBC A, H
            0x9D => self.sbc(self.l), // SBC A, L
            0x9E => {
                // SBC A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.sbc(value)
            }
            0x9F => self.sbc(self.a), // SBC A, A
            0xA0 => self.and(self.b), // AND A, B
            0xA1 => self.and(self.c), // AND A, C
            0xA2 => self.and(self.d), // AND A, D
            0xA3 => self.and(self.e), // AND A, E
            0xA4 => self.and(self.h), // AND A, H
            0xA5 => self.and(self.l), // AND A, L
            0xA6 => {
                // AND A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.and(value)
            }
            0xA7 => self.and(self.a), // AND A, A
            0xA8 => self.xor(self.b), // XOR A, B
            0xA9 => self.xor(self.c), // XOR A, C
            0xAA => self.xor(self.d), // XOR A, D
            0xAB => self.xor(self.e), // XOR A, E
            0xAC => self.xor(self.h), // XOR A, H
            0xAD => self.xor(self.l), // XOR A, L
            0xAE => {
                // XOR A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.xor(value)
            }
            0xAF => self.xor(self.a), // XOR A, A
            0xB0 => self.or(self.b),  // OR A, B
            0xB1 => self.or(self.c),  // OR A, C
            0xB2 => self.or(self.d),  // OR A, D
            0xB3 => self.or(self.e),  // OR A, E
            0xB4 => self.or(self.h),  // OR A, H
            0xB5 => self.or(self.l),  // OR A, L
            0xB6 => {
                // OR A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.or(value)
            }
            0xB7 => self.or(self.a), // OR A, A
            0xB8 => self.cp(self.b), // CP A, B
            0xB9 => self.cp(self.c), // CP A, C
            0xBA => self.cp(self.d), // CP A, D
            0xBB => self.cp(self.e), // CP A, E
            0xBC => self.cp(self.h), // CP A, H
            0xBD => self.cp(self.l), // CP A, L
            0xBE => {
                // CP A, (HL)
                let value: u8 = mem_map.get(self.get_hl_as_usize());
                self.cp(value)
            }
            0xBF => self.cp(self.a), // CP A, A
            0xC0 => {
                // RET NZ
                if !self.nf {
                    self.pop_pc(&mut mem_map)
                }
            },
            0xC1 => self.pop(&mut mem_map, BC), // POP BC
            0xC2 => {
                // JP NZ, u16
                let updated_pc: usize = self.get_pc_u16(&mut mem_map) as usize;
                if !self.zf {
                    self.pc = updated_pc
                }
            }
            0xC3 => {
                // JP u16
                self.pc = self.get_pc_u16(&mut mem_map) as usize
            }
            0xC4 => panic!("Instruction not implemented! CALL NZ, u16"),
            0xC5 => self.push(&mut mem_map, BC), // PUSH BC
            0xC6 => {
                let value: u8 = mem_map.get(self.get_pc());
                self.add_8(value)
            }
            0xC7 => panic!("Instruction not implemented! RST 00h"),
            0xC8 => {
                // RET Z
                if self.zf {
                    self.pop_pc(&mut mem_map)
                }
            }
            0xC9 => self.pop_pc(&mut mem_map), // RET
            0xCA => {
                // JP Z, u16
                let updated_pc: usize = self.get_pc_u16(&mut mem_map) as usize;
                if self.zf {
                    self.pc = updated_pc
                }
            }
            0xCB => {
                let opcode: u8 = mem_map.get(self.get_pc());
                match opcode {
                    0x00 => self.b = self.rlc(self.b, false), // RLC B
                    0x01 => self.c = self.rlc(self.c, false), // RLC C
                    0x02 => self.d = self.rlc(self.d, false), // RLC D
                    0x03 => self.e = self.rlc(self.e, false), // RLC E
                    0x04 => self.h = self.rlc(self.h, false), // RLC H
                    0x05 => self.l = self.rlc(self.l, false), // RLC L
                    0x06 => {
                        // RLC (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.rlc(value, false);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x07 => self.a = self.rlc(self.a, false), // RLC A
                    0x08 => self.b = self.rrc(self.b, false), // RRC B
                    0x09 => self.c = self.rrc(self.c, false), // RRC C
                    0x0A => self.d = self.rrc(self.d, false), // RRC D
                    0x0B => self.e = self.rrc(self.e, false), // RRC E
                    0x0C => self.h = self.rrc(self.h, false), // RRC H
                    0x0D => self.l = self.rrc(self.l, false), // RRC L
                    0x0E => {
                        // RRC (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.rrc(value, false);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x0F => self.a = self.rrc(self.a, false), // RRC A
                    0x10 => self.b = self.rl(self.b, false),  // RL B
                    0x11 => self.c = self.rl(self.c, false),  // RL C
                    0x12 => self.d = self.rl(self.d, false),  // RL D
                    0x13 => self.e = self.rl(self.e, false),  // RL E
                    0x14 => self.h = self.rl(self.h, false),  // RL H
                    0x15 => self.l = self.rl(self.l, false),  // RL L
                    0x16 => {
                        // RL (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.rl(value, false);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x17 => self.a = self.rl(self.a, false), // RL A
                    0x18 => self.b = self.rr(self.b, false), // RR B
                    0x19 => self.c = self.rr(self.c, false), // RR C
                    0x1A => self.d = self.rr(self.d, false), // RR D
                    0x1B => self.e = self.rr(self.e, false), // RR E
                    0x1C => self.h = self.rr(self.h, false), // RR H
                    0x1D => self.l = self.rr(self.l, false), // RR L
                    0x1E => {
                        // RR (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.rr(value, false);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x1F => self.a = self.rr(self.a, false), // RR A
                    0x20 => self.b = self.sla(self.b),       // SLA B
                    0x21 => self.c = self.sla(self.c),       // SLA C
                    0x22 => self.d = self.sla(self.d),       // SLA D
                    0x23 => self.e = self.sla(self.e),       // SLA E
                    0x24 => self.h = self.sla(self.h),       // SLA H
                    0x25 => self.l = self.sla(self.l),       // SLA L
                    0x26 => {
                        // SLA (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.sla(value);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x27 => self.a = self.sla(self.a), // SLA A
                    0x28 => self.b = self.sra(self.b), // SRA B
                    0x29 => self.c = self.sra(self.c), // SRA C
                    0x2A => self.d = self.sra(self.d), // SRA D
                    0x2B => self.e = self.sra(self.e), // SRA E
                    0x2C => self.h = self.sra(self.h), // SRA H
                    0x2D => self.l = self.sra(self.l), // SRA L
                    0x2E => {
                        // SRA (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.sra(value);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x2F => self.a = self.sra(self.a),  // SRA A
                    0x30 => self.b = self.swap(self.b), // SWAP B
                    0x31 => self.c = self.swap(self.c), // SWAP C
                    0x32 => self.d = self.swap(self.d), // SWAP D
                    0x33 => self.e = self.swap(self.e), // SWAP E
                    0x34 => self.h = self.swap(self.h), // SWAP H
                    0x35 => self.l = self.swap(self.l), // SWAP L
                    0x36 => {
                        // SWAP (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.swap(value);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x37 => self.a = self.swap(self.a), // SWAP A
                    0x38 => self.b = self.srl(self.b),  // SRL B
                    0x39 => self.c = self.srl(self.c),  // SRL C
                    0x3A => self.d = self.srl(self.d),  // SRL D
                    0x3B => self.e = self.srl(self.e),  // SRL E
                    0x3C => self.h = self.srl(self.h),  // SRL H
                    0x3D => self.l = self.srl(self.l),  // SRL L
                    0x3E => {
                        // SRL (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.srl(value);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x3F => self.a = self.srl(self.a), // SRL A
                    0x40 => self.bit(self.b, 0),       // BIT 0, B
                    0x41 => self.bit(self.c, 0),       // BIT 0, C
                    0x42 => self.bit(self.d, 0),       // BIT 0, D
                    0x43 => self.bit(self.e, 0),       // BIT 0, E
                    0x44 => self.bit(self.h, 0),       // BIT 0, H
                    0x45 => self.bit(self.l, 0),       // BIT 0, L
                    0x46 => {
                        // BIT 0, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 0)
                    }
                    0x47 => self.bit(self.a, 0), // BIT 0, A
                    0x48 => self.bit(self.b, 1), // BIT 1, B
                    0x49 => self.bit(self.c, 1), // BIT 1, C
                    0x4A => self.bit(self.d, 1), // BIT 1, D
                    0x4B => self.bit(self.e, 1), // BIT 1, E
                    0x4C => self.bit(self.h, 1), // BIT 1, H
                    0x4D => self.bit(self.l, 1), // BIT 1, L
                    0x4E => {
                        // BIT 1, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 1)
                    }
                    0x4F => self.bit(self.a, 1), // BIT 1, A
                    0x50 => self.bit(self.b, 2), // BIT 2, B
                    0x51 => self.bit(self.c, 2), // BIT 2, C
                    0x52 => self.bit(self.d, 2), // BIT 2, D
                    0x53 => self.bit(self.e, 2), // BIT 2, E
                    0x54 => self.bit(self.h, 2), // BIT 2, H
                    0x55 => self.bit(self.l, 2), // BIT 2, L
                    0x56 => {
                        // BIT 2, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 2)
                    }
                    0x57 => self.bit(self.a, 2), // BIT 2, A
                    0x58 => self.bit(self.b, 3), // BIT 3, B
                    0x59 => self.bit(self.c, 3), // BIT 3, C
                    0x5A => self.bit(self.d, 3), // BIT 3, D
                    0x5B => self.bit(self.e, 3), // BIT 3, E
                    0x5C => self.bit(self.h, 3), // BIT 3, H
                    0x5D => self.bit(self.l, 3), // BIT 3, L
                    0x5E => {
                        // BIT 3, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 3)
                    }
                    0x5F => self.bit(self.a, 3), // BIT 3, A
                    0x60 => self.bit(self.b, 4), // BIT 4, B
                    0x61 => self.bit(self.c, 4), // BIT 4, C
                    0x62 => self.bit(self.d, 4), // BIT 4, D
                    0x63 => self.bit(self.e, 4), // BIT 4, E
                    0x64 => self.bit(self.h, 4), // BIT 4, H
                    0x65 => self.bit(self.l, 4), // BIT 4, L
                    0x66 => {
                        // BIT 4, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 4)
                    }
                    0x67 => self.bit(self.a, 4), // BIT 4, A
                    0x68 => self.bit(self.b, 5), // BIT 5, B
                    0x69 => self.bit(self.c, 5), // BIT 5, C
                    0x6A => self.bit(self.d, 5), // BIT 5, D
                    0x6B => self.bit(self.e, 5), // BIT 5, E
                    0x6C => self.bit(self.h, 5), // BIT 5, H
                    0x6D => self.bit(self.l, 5), // BIT 5, L
                    0x6E => {
                        // BIT 5, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 5)
                    }
                    0x6F => self.bit(self.a, 5), // BIT 5, A
                    0x70 => self.bit(self.b, 6), // BIT 6, B
                    0x71 => self.bit(self.c, 6), // BIT 6, C
                    0x72 => self.bit(self.d, 6), // BIT 6, D
                    0x73 => self.bit(self.e, 6), // BIT 6, E
                    0x74 => self.bit(self.h, 6), // BIT 6, H
                    0x75 => self.bit(self.l, 6), // BIT 6, L
                    0x76 => {
                        // BIT 6, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 6)
                    }
                    0x77 => self.bit(self.a, 6), // BIT 6, A
                    0x78 => self.bit(self.b, 7), // BIT 7, B
                    0x79 => self.bit(self.c, 7), // BIT 7, C
                    0x7A => self.bit(self.d, 7), // BIT 7, D
                    0x7B => self.bit(self.e, 7), // BIT 7, E
                    0x7C => self.bit(self.h, 7), // BIT 7, H
                    0x7D => self.bit(self.l, 7), // BIT 7, L
                    0x7E => {
                        // BIT 7, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        self.bit(value, 7)
                    }
                    0x7F => self.bit(self.a, 7),          // BIT 7, A
                    0x80 => self.b = self.res(self.b, 0), // RES 0, B
                    0x81 => self.c = self.res(self.c, 0), // RES 0, C
                    0x82 => self.d = self.res(self.d, 0), // RES 0, D
                    0x83 => self.e = self.res(self.e, 0), // RES 0, E
                    0x84 => self.h = self.res(self.h, 0), // RES 0, H
                    0x85 => self.l = self.res(self.l, 0), // RES 0, L
                    0x86 => {
                        // RES 0, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 0);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x87 => self.a = self.res(self.a, 0), // RES 0, A
                    0x88 => self.b = self.res(self.b, 1), // RES 1, B
                    0x89 => self.c = self.res(self.c, 1), // RES 1, C
                    0x8A => self.d = self.res(self.d, 1), // RES 1, D
                    0x8B => self.e = self.res(self.e, 1), // RES 1, E
                    0x8C => self.h = self.res(self.h, 1), // RES 1, H
                    0x8D => self.l = self.res(self.l, 1), // RES 1, L
                    0x8E => {
                        // RES 1, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 1);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x8F => self.a = self.res(self.a, 1), // RES 1, A
                    0x90 => self.b = self.res(self.b, 2), // RES 2, B
                    0x91 => self.c = self.res(self.c, 2), // RES 2, C
                    0x92 => self.d = self.res(self.d, 2), // RES 2, D
                    0x93 => self.e = self.res(self.e, 2), // RES 2, E
                    0x94 => self.h = self.res(self.h, 2), // RES 2, H
                    0x95 => self.l = self.res(self.l, 2), // RES 2, L
                    0x96 => {
                        // RES 2, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 2);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x97 => self.a = self.res(self.a, 2), // RES 2, A
                    0x98 => self.b = self.res(self.b, 3), // RES 3, B
                    0x99 => self.c = self.res(self.c, 3), // RES 3, C
                    0x9A => self.d = self.res(self.d, 3), // RES 3, D
                    0x9B => self.e = self.res(self.e, 3), // RES 3, E
                    0x9C => self.h = self.res(self.h, 3), // RES 3, H
                    0x9D => self.l = self.res(self.l, 3), // RES 3, L
                    0x9E => {
                        // RES 3, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 3);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0x9F => self.a = self.res(self.a, 3), // RES 3, A
                    0xA0 => self.b = self.res(self.b, 4), // RES 4, B
                    0xA1 => self.c = self.res(self.c, 4), // RES 4, C
                    0xA2 => self.d = self.res(self.d, 4), // RES 4, D
                    0xA3 => self.e = self.res(self.e, 4), // RES 4, E
                    0xA4 => self.h = self.res(self.h, 4), // RES 4, H
                    0xA5 => self.l = self.res(self.l, 4), // RES 4, L
                    0xA6 => {
                        // RES 4, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 4);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xA7 => self.a = self.res(self.a, 4), // RES 4, A
                    0xA8 => self.b = self.res(self.b, 5), // RES 5, B
                    0xA9 => self.c = self.res(self.c, 5), // RES 5, C
                    0xAA => self.d = self.res(self.d, 5), // RES 5, D
                    0xAB => self.e = self.res(self.e, 5), // RES 5, E
                    0xAC => self.h = self.res(self.h, 5), // RES 5, H
                    0xAD => self.l = self.res(self.l, 5), // RES 5, L
                    0xAE => {
                        // RES 5, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 5);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xAF => self.a = self.res(self.a, 5), // RES 5, A
                    0xB0 => self.b = self.res(self.b, 6), // RES 6, B
                    0xB1 => self.c = self.res(self.c, 6), // RES 6, C
                    0xB2 => self.d = self.res(self.d, 6), // RES 6, D
                    0xB3 => self.e = self.res(self.e, 6), // RES 6, E
                    0xB4 => self.h = self.res(self.h, 6), // RES 6, H
                    0xB5 => self.l = self.res(self.l, 6), // RES 6, L
                    0xB6 => {
                        // RES 6, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 6);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xB7 => self.a = self.res(self.a, 6), // RES 6, A
                    0xB8 => self.b = self.res(self.b, 7), // RES 7, B
                    0xB9 => self.c = self.res(self.c, 7), // RES 7, C
                    0xBA => self.d = self.res(self.d, 7), // RES 7, D
                    0xBB => self.e = self.res(self.e, 7), // RES 7, E
                    0xBC => self.h = self.res(self.h, 7), // RES 7, H
                    0xBD => self.l = self.res(self.l, 7), // RES 7, L
                    0xBE => {
                        // RES 7, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.res(value, 7);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xBF => self.a = self.res(self.a, 7), // RES 7, A
                    0xC0 => self.b = self.set(self.b, 0), // SET 0, B
                    0xC1 => self.c = self.set(self.c, 0), // SET 0, C
                    0xC2 => self.d = self.set(self.d, 0), // SET 0, D
                    0xC3 => self.e = self.set(self.e, 0), // SET 0, E
                    0xC4 => self.h = self.set(self.h, 0), // SET 0, H
                    0xC5 => self.l = self.set(self.l, 0), // SET 0, L
                    0xC6 => {
                        // SET 0, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 0);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xC7 => self.a = self.set(self.a, 0), // SET 0, A
                    0xC8 => self.b = self.set(self.b, 1), // SET 1, B
                    0xC9 => self.c = self.set(self.c, 1), // SET 1, C
                    0xCA => self.d = self.set(self.d, 1), // SET 1, D
                    0xCB => self.e = self.set(self.e, 1), // SET 1, E
                    0xCC => self.h = self.set(self.h, 1), // SET 1, H
                    0xCD => self.l = self.set(self.l, 1), // SET 1, L
                    0xCE => {
                        // SET 1, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 1);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xCF => self.a = self.set(self.a, 1), // SET 1, A
                    0xD0 => self.b = self.set(self.b, 2), // SET 2, B
                    0xD1 => self.c = self.set(self.c, 2), // SET 2, C
                    0xD2 => self.d = self.set(self.d, 2), // SET 2, D
                    0xD3 => self.e = self.set(self.e, 2), // SET 2, E
                    0xD4 => self.h = self.set(self.h, 2), // SET 2, H
                    0xD5 => self.l = self.set(self.l, 2), // SET 2, L
                    0xD6 => {
                        // SET 2, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 2);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xD7 => self.a = self.set(self.a, 2), // SET 2, A
                    0xD8 => self.b = self.set(self.b, 3), // SET 3, B
                    0xD9 => self.c = self.set(self.c, 3), // SET 3, C
                    0xDA => self.d = self.set(self.d, 3), // SET 3, D
                    0xDB => self.e = self.set(self.e, 3), // SET 3, E
                    0xDC => self.h = self.set(self.h, 3), // SET 3, H
                    0xDD => self.l = self.set(self.l, 3), // SET 3, L
                    0xDE => {
                        // SET 3, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 3);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xDF => self.a = self.set(self.a, 3), // SET 3, A
                    0xE0 => self.b = self.set(self.b, 4), // SET 4, B
                    0xE1 => self.c = self.set(self.c, 4), // SET 4, C
                    0xE2 => self.d = self.set(self.d, 4), // SET 4, D
                    0xE3 => self.e = self.set(self.e, 4), // SET 4, E
                    0xE4 => self.h = self.set(self.h, 4), // SET 4, H
                    0xE5 => self.l = self.set(self.l, 4), // SET 4, L
                    0xE6 => {
                        // SET 4, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 4);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xE7 => self.a = self.set(self.a, 4), // SET 4, A
                    0xE8 => self.b = self.set(self.b, 5), // SET 5, B
                    0xE9 => self.c = self.set(self.c, 5), // SET 5, C
                    0xEA => self.d = self.set(self.d, 5), // SET 5, D
                    0xEB => self.e = self.set(self.e, 5), // SET 5, E
                    0xEC => self.h = self.set(self.h, 5), // SET 5, H
                    0xED => self.l = self.set(self.l, 5), // SET 5, L
                    0xEE => {
                        // SET 5, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 5);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xEF => self.a = self.set(self.a, 5), // SET 5, A
                    0xF0 => self.b = self.set(self.b, 6), // SET 6, B
                    0xF1 => self.c = self.set(self.c, 6), // SET 6, C
                    0xF2 => self.d = self.set(self.d, 6), // SET 6, D
                    0xF3 => self.e = self.set(self.e, 6), // SET 6, E
                    0xF4 => self.h = self.set(self.h, 6), // SET 6, H
                    0xF5 => self.l = self.set(self.l, 6), // SET 6, L
                    0xF6 => {
                        // SET 6, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 6);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xF7 => self.a = self.set(self.a, 6), // SET 6, A
                    0xF8 => self.b = self.set(self.b, 7), // SET 7, B
                    0xF9 => self.c = self.set(self.c, 7), // SET 7, C
                    0xFA => self.d = self.set(self.d, 7), // SET 7, D
                    0xFB => self.e = self.set(self.e, 7), // SET 7, E
                    0xFC => self.h = self.set(self.h, 7), // SET 7, H
                    0xFD => self.l = self.set(self.l, 7), // SET 7, L
                    0xFE => {
                        // SET 7, (HL)
                        let value: u8 = mem_map.get(self.get_hl_as_usize());
                        let result: u8 = self.set(value, 7);
                        mem_map.set(self.get_hl_as_usize(), result)
                    }
                    0xFF => self.a = self.set(self.a, 7), // SET 7, A
                    _ => panic!("Instruction not implemented!"),
                }
            }
            0xCC => panic!("Instruction not implemented! CALL Z, u16"),
            0xCD => {
                // CALL u16
                let lower: u8 = mem_map.get(self.get_pc());
                let upper: u8 = mem_map.get(self.get_pc());
                mem_map.set(self.get_sp_dec(), lower);
                mem_map.set(self.get_sp_dec(), upper);
            }
            0xCE => {
                let value: u8 = mem_map.get(self.get_pc());
                self.adc(value)
            }
            0xCF => panic!("Instruction not implemented! RST 08h"),
            0xD0 => {
                // RET NC
                if !self.cf {
                    self.pop_pc(&mut mem_map)
                }
            }
            0xD1 => self.pop(&mut mem_map, DE), // POP DE
            0xD2 => {
                // JP NC, u16
                let updated_pc: usize = self.get_pc_u16(&mut mem_map) as usize;
                if !self.cf {
                    self.pc = updated_pc
                }
            }
            0xD3 => panic!("No instruction exists for 0xD3"),
            0xD4 => panic!("Instruction not implemented! CALL NC, u16"),
            0xD5 => self.push(&mut mem_map, DE), // PUSH DE
            0xD6 => {
                // SUB A, u8
                let value: u8 = mem_map.get(self.get_pc());
                self.sub(value)
            }
            0xD7 => panic!("Instruction not implemented! RST 10h"),
            0xD8 => {
                // RET C
                if self.cf {
                    self.pop_pc(&mut mem_map)
                }
            }
            0xD9 => panic!("Instruction not implemented! RETI"),
            0xDA => {
                // JP C, u16
                let updated_pc: usize = self.get_pc_u16(&mut mem_map) as usize;
                if self.cf {
                    self.pc = updated_pc
                }
            }
            0xDB => panic!("No instruction exists for 0xDB"),
            0xDC => panic!("Instruction not implemented! CALL C, u16"),
            0xDD => panic!("No instruction exists for 0xDD"),
            0xDE => {
                // SBC A, u8
                let value: u8 = mem_map.get(self.get_pc());
                self.sbc(value)
            }
            0xDF => panic!("Instruction not implemented! RST 18h"),
            0xE0 => panic!("Instruction not implemented! LD (FF00+u8), A"),
            0xE1 => self.pop(&mut mem_map, HL), // POP HL
            0xE2 => panic!("Instruction not implemented! LD (FF00+C), A"),
            0xE3 => panic!("No instruction exists for 0xE3"),
            0xE4 => panic!("No instruction exists for 0xE4"),
            0xE5 => self.push(&mut mem_map, HL), // PUSH HL
            0xE6 => {
                // AND A, u8
                let value: u8 = mem_map.get(self.get_pc());
                self.and(value)
            }
            0xE7 => panic!("Instruction not implemented! RST 20h"),
            0xE8 => panic!("Instruction not implemented! ADD SP, i8"),
            0xE9 => self.pc = self.get_16_register(HL) as usize, // JP HL
            0xEA => panic!("Instruction not implemented! LD (u16), A"),
            0xEB => panic!("No instruction exists for 0xEB"),
            0xEC => panic!("No instruction exists for 0xEC"),
            0xED => panic!("No instruction exists for 0xED"),
            0xEE => {
                // XOR A, u8
                let value: u8 = mem_map.get(self.get_pc());
                self.xor(value)
            }
            0xEF => panic!("Instruction not implemented! RST 28h"),
            0xF0 => panic!("Instruction not implemented! LD A, (FF00+u8)"),
            0xF1 => self.pop(&mut mem_map, AF), // POP AF
            0xF2 => panic!("Instruction not implemented! LD A, (FF00+C)"),
            0xF3 => panic!("Instruction not implemented! DI"),
            0xF4 => panic!("No instruction exists for 0xF4"),
            0xF5 => self.push(&mut mem_map, AF), // PUSH AF
            0xF6 => {
                // OR A, u8
                let value: u8 = mem_map.get(self.get_pc());
                self.or(value)
            }
            0xF7 => panic!("Instruction not implemented! RST 30h"),
            0xF8 => panic!("Instruction not implemented! LD HL, SP + i8"),
            0xF9 => panic!("Instruction not implemented! LD SP, HL"),
            0xFA => panic!("Instruction not implemented! LD A, (u16)"),
            0xFB => panic!("Instruction not implemented! EI"),
            0xFC => panic!("No instruction exists for 0xFC"),
            0xFD => panic!("No instruction exists for 0xFD"),
            0xFE => {
                // CP A, u8
                let value: u8 = mem_map.get(self.get_pc());
                self.cp(value)
            }
            0xFF => panic!("Instruction not implemented! RST 38h"),
        }
    }

    fn set_flag(&mut self, flag: Flag, value: bool) {
        match flag {
            Z => self.zf = value,
            N => self.nf = value,
            H => self.hf = value,
            C => self.cf = value,
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
        self.get_16_register(HL) as usize
    }

    fn get_sp_inc(&mut self) -> usize {
        let original: usize = self.sp;
        self.sp = self.sp + 1;
        original
    }

    fn get_sp_dec(&mut self) -> usize {
        let original: usize = self.sp;
        self.sp = self.sp - 1;
        original
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

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, add_half_carry(self.a, value, cf as u8));
        self.set_flag(C, (self.a as u16) + (value as u16) + (cf as u16) > 0xFF);

        self.a = result;
    }

    // ADD A,r8
    // ADD A,[HL]
    // ADD A,n8
    fn add_8(&mut self, value: u8) {
        let (result, overflow) = self.a.overflowing_add(value);

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, add_half_carry(self.a, value, 0));
        self.set_flag(C, overflow);

        self.a = result;
    }

    // AND A,r8
    // AND A,[HL]
    // AND A,n8
    fn and(&mut self, value: u8) {
        let result: u8 = self.a & value;

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, true);
        self.set_flag(C, false);

        self.a = result;
    }

    // CP A,r8
    // CP A,[HL]
    // CP A,n8
    fn cp(&mut self, value: u8) {
        let result: u8 = self.a - value;

        self.set_flag(Z, result == 0);
        self.set_flag(N, true);
        self.set_flag(H, sub_half_carry(self.a, value, 0));
        self.set_flag(C, value > self.a);
    }

    // DEC r8
    // DEC [HL]
    fn dec(&mut self, value: u8) -> u8 {
        let result: u8 = value - 1;

        self.set_flag(Z, result == 0);
        self.set_flag(N, true);
        self.set_flag(H, sub_half_carry(value, 1, 0));

        result
    }

    // INC r8
    // INC [HL]
    fn inc(&mut self, value: u8) -> u8 {
        let result: u8 = value + 1;

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, add_half_carry(value, 1, 0));

        result
    }

    // OR A,r8
    // OR A,[HL]
    // OR A, n8
    fn or(&mut self, value: u8) {
        let result: u8 = self.a | value;

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, false);

        self.a = result;
    }

    // SBC A,r8
    // SBC A,[HL]
    // SBC A,n8
    fn sbc(&mut self, value: u8) {
        let cf: u8 = self.cf as u8;
        let result = self.a.wrapping_sub(value).wrapping_sub(cf);

        self.set_flag(Z, result == 0);
        self.set_flag(N, true);
        self.set_flag(H, (self.a & 0x0F) < (value & 0x0F) + cf);
        self.set_flag(C, (self.a as u16) < (value as u16) + (cf as u16));

        self.a = result;
    }

    // SUB A,r8
    // SUB A,[HL]
    // SUB A,n8
    fn sub(&mut self, value: u8) {
        let result: u8 = self.a.wrapping_sub(value);

        self.set_flag(Z, result == 0);
        self.set_flag(N, true);
        self.set_flag(H, (self.a & 0x0F) < (value & 0x0F));
        self.set_flag(C, value > self.a);

        self.a = result;
    }

    // XOR A,r8
    // XOR A,[HL]
    // XOR A,n8
    fn xor(&mut self, value: u8) {
        let result: u8 = self.a ^ value;

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, false);

        self.a = result;
    }

    // ADD HL,r16
    // ADD HL,SP
    fn add_hl(&mut self, value: u16) {
        let (result, overflow) = self.get_16_register(HL).overflowing_add(value);

        self.set_flag(N, false);
        self.set_flag(H, add_u16_half_carry(self.get_16_register(HL), value));
        self.set_flag(C, overflow);

        self.set_16_register(HL, result);
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
        self.set_flag(Z, ((value >> bit) & 1) == 0);
        self.set_flag(N, false);
        self.set_flag(H, true);
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

        self.set_flag(Z, !is_rla && result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, overflow);

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

        self.set_flag(Z, !is_rlca && result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, overflow);

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

        self.set_flag(Z, !is_rra && result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, overflow);

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

        self.set_flag(Z, !is_rrca && result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, overflow);

        result
    }

    // SLA r8
    // SLA [HL]
    fn sla(&mut self, value: u8) -> u8 {
        let overflow: bool = (value >> 7) == 1;
        let result: u8 = value.shl(1);

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, overflow);

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

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, overflow);

        result
    }

    // SRL r8
    // SRL [HL]
    fn srl(&mut self, value: u8) -> u8 {
        let overflow: bool = (value & 1) == 1;
        let result: u8 = value.shr(1);

        self.set_flag(Z, result == 0);
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, overflow);

        result
    }

    // JR i8
    fn jr(&mut self, mem_map: &mut MemoryMap) {
        let value: i8 = mem_map.get(self.get_pc()) as i8;
        self.pc = ((self.pc as i16) + (value as i16)) as usize
    }

    // JR cc, i8
    fn jr_cond(&mut self, value: i8) {
        self.pc = ((self.pc as i16) + (value as i16)) as usize
    }

    // CPL
    fn cpl(&mut self) {
        self.a = !self.a;
    }

    // SCF
    fn scf(&mut self) {
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, true);
    }

    // CCF
    fn ccf(&mut self) {
        self.set_flag(N, false);
        self.set_flag(H, false);
        self.set_flag(C, !self.cf);
    }

    fn pop(&mut self, mem_map: &mut MemoryMap, register: Register) {
        let lower: u8 = mem_map.get(self.get_sp_inc());
        let upper: u8 = mem_map.get(self.get_sp_inc());
        match register {
            AF => {
                self.a = upper;
                self.set_f(lower);
            }
            BC => {
                self.b = upper;
                self.c = lower;
            }
            DE => {
                self.d = upper;
                self.e = lower;
            }
            HL => {
                self.h = upper;
                self.l = lower;
            }
        }
    }

    fn pop_pc(&mut self, mem_map: &mut MemoryMap) {
        let lower: u8 = mem_map.get(self.get_sp_inc());
        let upper: u8 = mem_map.get(self.get_sp_inc());

        self.pc = (((upper as u16) << 8) + (lower as u16)) as usize
    }

    fn push(&mut self, mem_map: &mut MemoryMap, register: Register) {
        let upper_address: usize = self.get_sp_dec();
        let lower_address: usize = self.get_sp_dec();
        match register {
            AF => {
                mem_map.set(upper_address, self.a);
                mem_map.set(lower_address, self.get_f());
            }
            BC => {
                mem_map.set(upper_address, self.b);
                mem_map.set(lower_address, self.c);
            }
            DE => {
                mem_map.set(upper_address, self.d);
                mem_map.set(lower_address, self.e);
            }
            HL => {
                mem_map.set(upper_address, self.h);
                mem_map.set(lower_address, self.l);
            }
        }
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
        cpu.set_flag(Z, true);
        assert!(cpu.zf == true)
    }

    #[test]
    fn correctly_parses_flags() {
        let mut cpu = Cpu::new();
        cpu.set_flag(Z, true);
        cpu.set_flag(N, true);
        cpu.set_flag(H, true);
        cpu.set_flag(C, true);
        assert!(cpu.get_16_register(AF) == 0x00F0)
    }

    #[test]
    fn set_af_correctly_sets_flags() {
        let mut cpu = Cpu::new();
        cpu.set_16_register(AF, 0xFFFF);
        assert!(cpu.a == 0xFF);
        assert!(cpu.zf == true);
        assert!(cpu.nf == true);
        assert!(cpu.hf == true);
        assert!(cpu.cf == true);
    }

    #[test]
    fn set_af_correctly_sets_flags_to_off() {
        let mut cpu = Cpu::new();
        cpu.set_16_register(AF, 0xFF00);
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
