# HeadlessZombie VCU

Project based on the ZombieVerter VCU by Damien Maguire to provide a core universal VCU (Vehicle Control Unit) module compatible with newer STM32 family such as STM32F405, that abstracts nearly all communication over CAN and SPI.

It differs from ZombieVerter in that it does not support GS450 directly (no sync serial).

It is primarily intended to run on a RetroVMS module, see https://github.com/jetpax/RetroVMS-MINI


![retroVMS MINI](images/retroVMS-MINI.png?raw=true)

using a Sparkfun Micromod STM32F405

![SparkFunSTM32F405](/images/stm32f405.png)


# Building
Builds under Platformio using VScode under Linux,MacOS and Windows

Upload with STlink or BlackMagic or dfu (change platformio.ini to suit)


![Debugger](/images/debugger.png)


Please visit the development thread on the Openinverter Forum for more information : https://openinverter.org/forum/viewtopic.php?t=5947



# Supported components

- Nissan Leaf Gen1/2/3 inverter via CAN
- Nissan Leaf Gen2 PDM (Charger and DCDC)
- Nissan LEAF Battery (all variants)
- Mitsubishi Outlander Support
- Mitsubishi Outlander drivetrain (front and rear motors/inverters) Support
- Modular BMS / SimpBMS Support
- OpenInverter CAN Support
- CCS DC fast charge via BMW i3 LIM
- Chademo DC fast charge
- ISA Shunt / BMW SBOX / VW EBOX supported via CAN
- BMW E46 CAN support
- BMW E39 CAN support
- BMW E65 CAN Support
- BMW E31 CAN Support
- Mid 2000s VAG Can support
- Subaru vehicle support
- Opel Ampera / Chevy Volt 6.5kw cabin heater
- VW LIN based 6.5kw cabin heater
- Elcon charger Support
- OBD2 Can support
- TESLA Gen 2 DCDC Converter Can support



Run the tests

`./test/test_vcu`


