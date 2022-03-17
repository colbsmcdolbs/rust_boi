pub (crate) struct SwitchableMemoryBank {
    current_index: usize,
    banks: Vec<Vec<u8>>
}

impl SwitchableMemoryBank {
    pub fn new(size: usize, bank_size: usize) -> SwitchableMemoryBank {
        if size < 1 {
            panic!("Must have at least 2 banks to be switchable")
        }
        
        SwitchableMemoryBank {
            current_index: 0,
            banks: vec![vec![0; bank_size]; size]
        }
    }

    pub fn get_value(& self, index: usize) -> u8 {
        self.banks[self.current_index][index]
    }

    pub fn set_value(&mut self, index: usize, value: u8) {
        self.banks[self.current_index][index] = value
    }

    pub fn set_bank(&mut self, bank_index: usize) {
        self.current_index = bank_index
    }
}

#[cfg(test)]
mod switchable_memory_bank_tests {
    use super::*;

    #[test]
    fn successfully_creates() {
        let switch_bank = SwitchableMemoryBank::new(2, 4096);
        assert!(switch_bank.current_index == 0)
    }

    #[test]
    #[should_panic]
    fn fails_to_create_with_size_0() {
        let _switch_bank = SwitchableMemoryBank::new(0, 4096);
    }

    #[test]
    #[should_panic]
    fn setting_at_invalid_bank_index_and_getting_should_panic() {
        let mut switch_bank = SwitchableMemoryBank::new(2, 4096);
        switch_bank.set_bank(2);
        switch_bank.get_value(0);
        switch_bank.set_value(0, 0);
    }
}