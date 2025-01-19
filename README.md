# HeadlessZombie VCU

Project based on the ZombieVerter by Damien Maguire, which is itself based on the OpenInverter System by Johannes Huebner.
Provides a universal VCU (Vehicle Control Unit) for electric vehicle conversion projects.

The firmware runs on a STM32F405, specifically the [SparkFun STM32 MicroMod Processor](https://github.com/sparkfun/MicroMod_STM32_Processor), and is intended to be plugged into a [RetroVMS motherboard](https://github.com/jetpax/RetroVMS-MINI/tree/main)

![retrovmsV3](https://github.com/jetpax/RetroVMS-MINI/blob/main/images/retroVMS-MINI.png)


Please visit the [development thread on the Openinverter Forum for more information](https://openinverter.org/forum/viewtopic.php?t=5947&sid=b7d3152c818597208776f12b4deb4279)

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


# Compiling
The project is built using Platformio in VScode, so build it as any other Platformio project
