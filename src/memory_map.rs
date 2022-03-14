/*
0000	3FFF	16 KiB ROM bank 00	From cartridge, usually a fixed bank
4000	7FFF	16 KiB ROM Bank 01~NN	From cartridge, switchable bank via mapper (if any)
8000	9FFF	8 KiB Video RAM (VRAM)	In CGB mode, switchable bank 0/1
A000	BFFF	8 KiB External RAM	From cartridge, switchable bank if any
C000	CFFF	4 KiB Work RAM (WRAM)	
D000	DFFF	4 KiB Work RAM (WRAM)	In CGB mode, switchable bank 1~7
E000	FDFF	Mirror of C000~DDFF (ECHO RAM)	Nintendo says use of this area is prohibited.
FE00	FE9F	Sprite attribute table (OAM)	
FEA0	FEFF	Not Usable	Nintendo says use of this area is prohibited
FF00	FF7F	I/O Registers	
FF80	FFFE	High RAM (HRAM)	
FFFF	FFFF	Interrupt Enable register (IE)
*/
use crate::{switchable_memory_bank::SwitchableMemoryBank};

pub(crate) struct MemoryMap {
	rom_bank_00: [u8; 16384],
    rom_bank_0n: SwitchableMemoryBank, //Switchable bank via mapper
    vram: SwitchableMemoryBank,        //Switchable bank in CGB mode 0/1
    external: SwitchableMemoryBank,    //Switchable bank from cartridge
    wram_00: [u8; 4096],
    wram_0n: SwitchableMemoryBank,     //Switchable bank in CGB mode 1~7
    sprite: [u8; 160],
    io: [u8; 128],
    hram: [u8; 127],
    interrupt: u8
}

pub enum SwitchableBankType {
    RomBank0N,
    VRam,
    External,
    Wram0N
}

impl MemoryMap {
    pub fn new(rom_bank_size: usize, external_ram_size: usize) -> MemoryMap {
        MemoryMap {
            rom_bank_00: [0; 16384],
            rom_bank_0n: SwitchableMemoryBank::new(rom_bank_size, 16384),
            vram: SwitchableMemoryBank::new(2, 8192),
            external: SwitchableMemoryBank::new(external_ram_size, 8192),
            wram_00: [0; 4096],
            wram_0n: SwitchableMemoryBank::new(7, 4096),
            sprite:[0; 160],
            io:[0; 128],
            hram:[0; 127],
            interrupt: 0
        }
    }

    pub fn get(&mut self, index: usize) -> u8 {
        match index {
            0x0000 ..= 0x3FFF => self.rom_bank_00[index],
            0x4000 ..= 0x7FFF => self.rom_bank_0n.get_value(index - 0x4000),
            0x8000 ..= 0x9FFF => self.vram.get_value(index - 0x8000),
            0xA000 ..= 0xBFFF => self.external.get_value(index - 0xA000),
            0xC000 ..= 0xCFFF => self.wram_00[index - 0xC000],
            0xD000 ..= 0xDFFF => self.wram_0n.get_value(index - 0xD000),
            0xFE00 ..= 0xFE9F => self.sprite[index - 0xFE00],
            0xFF00 ..= 0xFF7F => self.io[index - 0xFF00],
            0xFF80 ..= 0xFFFE => self.hram[index - 0xFF80],
            0xFFFF => self.interrupt,
            _ => panic!("Invalid address!")
        }
    }

    pub fn set(&mut self, index: usize, value: u8) {
        match index {
            0x0000 ..= 0x3FFF => self.rom_bank_00[index] = value,
            0x4000 ..= 0x7FFF => self.rom_bank_0n.set_value(index - 0x4000, value),
            0x8000 ..= 0x9FFF => self.vram.set_value(index - 0x8000, value),
            0xA000 ..= 0xBFFF => self.external.set_value(index - 0xA000, value),
            0xC000 ..= 0xCFFF => self.wram_00[index - 0xC000] = value,
            0xD000 ..= 0xDFFF => self.wram_0n.set_value(index - 0xD000, value),
            0xFE00 ..= 0xFE9F => self.sprite[index - 0xFE00] = value,
            0xFF00 ..= 0xFF7F => self.io[index - 0xFF00] = value,
            0xFF80 ..= 0xFFFE => self.hram[index - 0xFF80] = value,
            0xFFFF => self.interrupt = value,
            _ => panic!("Invalid address!")
        } 
    }

    pub fn update_switchable_bank_index(&mut self, bank_index: usize, switchable_bank_type: SwitchableBankType) {
        match switchable_bank_type {
            SwitchableBankType::RomBank0N => {
                self.rom_bank_0n.set_bank(bank_index);
            },
            SwitchableBankType::VRam => {
                self.vram.set_bank(bank_index);
            },
            SwitchableBankType::External => {
                self.external.set_bank(bank_index);
            },
            SwitchableBankType::Wram0N => {
                self.wram_0n.set_bank(bank_index);
            },
        }
    }
}
