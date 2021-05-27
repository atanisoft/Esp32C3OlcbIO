# ESP32-C3 OpenLCB IO Board

The ESP32-C3 OpenLCB IO Board is a basic OpenLCB (LCC) node offering 8
configurable input/output pins with three I2C headers for future expansion.

## Powering the ESP32-C3 OpenLCB IO Board

The ESP32-C3 OpenLCB IO Board is powered by the OpenLCB (LCC) Bus or via the
on-board USB connector.

### OpenLCB (LCC) Power requirements

The ESP32-C3 OpenLCB IO Board will draw around 100mA from the OpenLCB (LCC) Bus
when the PWR_POS pin provides 15VDC. If the PWR_POS pin provides less, the node
may draw more current from the bus.

### Node Brownout detection

If the ESP32-C3 detects a brownout condition it will attempt to produce the
well-known event `01.00.00.00.00.00.FF.F1` soon after startup. This delivery
is not guaranteed.

## Pin Mapping

By default almost all pins are exposed for general IO usage by the node,
however a few pins have specialized usage as part of the screw terminals or
expansion headers.

| GPIO Pin | Usage | Notes |
| -------- | ----- | ----- |
| 0 | Activity LED | Active LOW LED |
| 1 | Bootloader Write LED | Active LOW LED |
| 2 | Bootloader LED | Active LOW LED |
| 3 | Factory Reset Button | 10k Pull-Up resistor and 100nF capacitor |
| 4 | SDA | |
| 5 | SCL | |
| 6 | CAN RX | |
| 7 | CAN TX | |
| 8 | Bootloader Request Button | 10k Pull-Up resistor and 100nF capacitor |
| 9 | TP1 | Connect to GND to access ESP bootloader |
| 10 | Not Used | |
| 11 - 17 | Not Used | Connected to Flash |
| 18 | USB D- | |
| 19 | USB D+ | |
| 20 | UART0 RX | Not Used |
| 21 | UART0 TX | Not Used |