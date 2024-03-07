pub trait Memory {
    fn read(&self, addr: u16) -> u8; // Read a byte from the given address.
    fn write(&mut self, addr: u16, data: u8); // Write a byte to a given address.
    fn reset(&mut self); // Reset the contents of the entire memory.
}
