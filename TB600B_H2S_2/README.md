# TB600B_H2S_2

Arduino library for the ECsense TB600B-H2S-2 hydrogen sulfide sensor.

## Features

- UART communication using a `HardwareSerial` port
- Sensor presence check using the lights/status query command
- Passive upload mode support
- Passive data request support
- H2S concentration, temperature, and relative humidity parsing
- Sensor information query using the one-byte `0xD7` command

## Example

See `examples/PassiveRead_Serial1/PassiveRead_Serial1.ino`.

The example uses `Serial1` for the TB600B-H2S-2 sensor and `Serial` for USB debug output.
