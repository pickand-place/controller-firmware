# pickand.place Main Controller Board Rev 1.0

This is the first version of the board and contains errors. At time of writing is it still under test.

| Module | Tested | Working | Notes |
|---|:---:|:---:|---|
| 24 to 3.3v Regulator | [x] | [x] | Works perfectly, everything is within specs. |
| MCU | [x] | [x] | MCU is usable after errata 1 is fixed. |
| Reset, Debug buttons | [x] | [x] | Sometimes, MCU do not properly recover from a reset press. Signal was tested and correct, probably a software issue. |
| Debug LEDs | [x] | [x] | Working |
| Debug UART | [x] | [x] | Working |
| USB | [x] | [] | USB can be connected and properly discovered by host. Connection state are not handled properly when running. Probably a software issue. It is possible that VBUS monitoring need to be routed on board to work properly since device is not powered by VBUS (Needs investigation). |
| TMC Stepper Controllers | [] | [] |  |
| MOSFET Switches | [] | [] |  |
| Motor limit switches inputs | [] | [] |  |
| I2C Connector | [] | [] |  |
| RS485 Controller | [] | [] |  |
| CAN Controller | [] | [] |  |
| Onboard flash memory | [] | [] |  |

## Errata 

1. 2x `VCAP` pins should not be connected to `3.3v` and bypassed by capacitor. They should only be connected to capacitor to ground. Current circuit will short `3.3v` to VCore causing protection devices within chip to short the bus. Permanent damage can happen to MCU if `3.3v` is not current limited to 100-200mA max.
    
    *Workaround* - Cut trace between `VCAP1`, `VCAP2` pins and `3.3v`. Connect Low ESR `2.2uF` capacitor between each pins and ground. Capacitor used for workaround was [`TMK212B7225MGHT`](https://www.digikey.ca/en/products/detail/TMK212B7225MGHT/587-6358-1-ND/9949944)