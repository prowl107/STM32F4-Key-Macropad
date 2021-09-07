# STM32F4-Key-Macropad
A 4x4 keypad built using an STM32F4 MCU and libopencm3 framework. 

## About the Project
Ever since stepping into embedded systems, I've wanted to learn about USB devices. I decided to take on this project to learn more about USB protocol and make something cool while I'm at it. 

The idea behind the macropad was to have a 4x4 key layout that supports user defined macros, media keys, and a usable numpad. 

## Built With
- [platformio](https://platformio.org/)
- [libopencm3](https://github.com/libopencm3/libopencm3)
- STM32F4 MCU
  - Developing with [STM32F4 Discovery Kit](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)

## Status
Still in development

## Installation
- This project was developed in vscode using the platformio extension.
  
```
git clone https://github.com/prowl107/STM32F4-Key-Macropad.git
```

### Current Task List
1. Learn about Human Inteface Devices (HID)
2. Create USB descriptor hierarchy 
3. Start developing USB descriptor structures
4. Test simple key presses
5. Implement interrupts
6. Develop breakout board
7. Change modes
8. Add more functionality
9. Come up with a good project name :)
