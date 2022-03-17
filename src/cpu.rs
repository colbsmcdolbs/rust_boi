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
}