# Arduino-CAN-Projects

This project can be used to validate your CAN bus modules operate and as a simple CAN analyzer for these simple Arduino CAN modules. 
I wrote this as the start of an analyzer for one of my robotics projects. 

Requires libraries https://github.com/autowp/arduino-mcp2515 and https://github.com/DFRobot/LCD-KeyPad-Shield

CAN Write Test -- Target Arduino MEGA 2560 with MCP2515 module. Sends two CAN messages via MCP2515 module. See typical wiring for module, CS on pin 49.

CAN Reader -- Target Arduino MEGA 2560 with MCP2515. DFRobot keypad LCD shielf. Reads CAN messes with mask and filters via MCP2515 module. Interrupt on pin 2. CS on pin 49. Outputs a single filtered ID on DFRobot LCD. Keypad keys added for your custom menu.  
