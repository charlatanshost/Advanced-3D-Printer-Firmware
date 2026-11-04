; Simple test G-code for debugging
G21 ; set units to millimeters
G90 ; use absolute coordinates
G28 ; home all axes
G1 Z0.2 F7800 ; move to layer height
G1 X10 Y10 Z0.2 F1800 ; move to start
G1 X50 Y10 Z0.2 F1800 ; line to right
G1 X50 Y50 Z0.2 F1800 ; line up
G1 X10 Y50 Z0.2 F1800 ; line left
G1 X10 Y10 Z0.2 F1800 ; line down (complete square)
G1 Z5 F7800 ; lift nozzle
M30 ; end program