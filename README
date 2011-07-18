# UAV Control Link

This project is a UAV control link over a serial modem using a PlayStation 3 controller as an input.  The software produces a PPM signal output on the UAV board that can be read by typical UAV boards.

# Requirements

- [Arduino](https://www.arduino.cc/en/Main/Software)
- [Eagle](https://www.autodesk.com/products/eagle/free-download)

# Table of Contents

| Directory | Contents |
|---|----|
| js_ctrl | Takes PlayStation 3 controller input, updates its model of the airframe, relays the airframe state to joystick2ppm |
| joystick2ppm | Listens to serial traffic from js_ctrl, outputs a PPM signal based on the servo states, handles handshaking and sync |
| gcs_relay | Ground control station relay - relays traffic from js_ctrl to a serial modem |
| servotest | Decodes a PPM signal into servo outputs |

# Disclaimer

This code is old, umaintained, and probably kinda slow depending on your serial modem.  I wouldn't recommend using this to fly anything, but maybe some lessons can be learned from the code.
