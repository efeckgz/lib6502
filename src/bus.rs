use core::mem::MaybeUninit;
use core::ptr;

// Anything that implements this trait can be used as a device in the bus.
pub trait BusDevice {
    fn read(&mut self, addr: u16) -> u8;
    fn write(&mut self, addr: u16, data: u8);
}

// A device slot in the bus will be a 3-tuple of the device start address, end address and the device itself.
// type DeviceSlot<'a> = Option<(u16, u16, &'a mut dyn BusDevice)>;

pub struct DeviceSlot<T: BusDevice> {
    start_addr: u16,
    end_addr: u16,
    device: T,
}

impl<T: BusDevice> DeviceSlot<T> {
    pub fn new(start_addr: u16, end_addr: u16, device: T) -> Self {
        Self {
            start_addr,
            end_addr,
            device,
        }
    }
}

impl<T: BusDevice> BusDevice for DeviceSlot<T> {
    fn read(&mut self, addr: u16) -> u8 {
        if addr >= self.start_addr && addr <= self.end_addr {
            self.device.read(addr)
        } else {
            0xFF // Address not in range
        }
    }

    fn write(&mut self, addr: u16, data: u8) {
        if addr >= self.start_addr && addr <= self.end_addr {
            self.device.write(addr, data);
        }
    }
}

// The Bus will be a collection of devices mapped to their respective addresses.
// pub struct Bus<'a, const N: usize> {
//     slots: [DeviceSlot<'a>; N],
// }

pub struct Bus<Devices> {
    devices: Devices,
}

impl<Devices: BusDevice> Bus<Devices> {
    pub fn new(devices: Devices) -> Self {
        Self { devices }
    }
}

impl<Devices: BusDevice> BusDevice for Bus<Devices> {
    fn read(&mut self, addr: u16) -> u8 {
        self.devices.read(addr)
    }

    fn write(&mut self, addr: u16, data: u8) {
        self.devices.write(addr, data);
    }
}

macro_rules! impl_bus_device_for_tuple {
    ($($T:ident),*) => {
        impl<$($T: BusDevice),*> BusDevice for ($($T,)*) {
            fn read(&mut self, addr: u16) -> u8 {
                #[allow(non_snake_case)]
                let ($($T,)*) = self;
                $(
                    if let result @ 0..=0xFE = $T.read(addr) {
                        return result;
                    }
                )*
                0xFF // No device responded
            }

            fn write(&mut self, addr: u16, data: u8) {
                #[allow(non_snake_case)]
                let ($($T,)*) = self;
                $(
                    $T.write(addr, data);
                )*
            }
        }
    };
}

// impl<'a, const N: usize> Bus<'a, N> {
//     pub fn new() -> Self {
//         // Create an uninitialized array of DeviceSlots, with the promise of initializing them later.
//         let mut slots: [MaybeUninit<DeviceSlot<'a>>; N] =
//             unsafe { MaybeUninit::uninit().assume_init() };

//         // Manually initialize each device slot to None.
//         for device in &mut slots {
//             *device = MaybeUninit::new(None);
//         }

//         // Cast slots to an array of DeviceSlots for use in struct Bus.
//         let slots = unsafe { ptr::read(&slots as *const _ as *const [Option<_>; N]) };

//         Self { slots }
//     }

//     // Add a new device to the bus
//     pub fn map_device(
//         &mut self,
//         startaddr: u16,
//         endaddr: u16,
//         device: &'a mut dyn BusDevice,
//     ) -> Result<(), ()> {
//         for slot in &mut self.slots {
//             if (*slot).is_none() {
//                 *slot = Some((startaddr, endaddr, device));
//                 return Ok(());
//             }
//         }
//         Err(()) // No space left on the bus
//     }
// }

// impl<'a, const N: usize> BusDevice for Bus<'a, N> {
//     // Perform a bus read on the given device
//     fn read(&mut self, addr: u16) -> u8 {
//         for slot in self.slots.iter_mut().flatten() {
//             let (start, end, device) = slot;
//             if addr >= *start && addr <= *end {
//                 // address corresponds to the range of this device
//                 return device.read(addr);
//             }
//         }
//         0xFF // Address corresponds to no device
//     }

//     // Perform a bus write from the given device
//     fn write(&mut self, addr: u16, data: u8) {
//         for slot in self.slots.iter_mut().flatten() {
//             let (start, end, device) = slot;
//             if addr >= *start && addr <= *end {
//                 // address corresponds to range of this device
//                 device.write(addr, data);
//             }
//         }
//     }
// }
