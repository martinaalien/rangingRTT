# Ranging with Round Trip Timing for nRF52840
This project is created to do ranging measurements between two nRF52840 Development Kits. It is written as a part of a student project at Norges Teknisk-Naturvitenskapelige Universitet (NTNU) in collaboration with Nordic Semiconductor. 

The ranging algorithm is based on Round Trip Timing (RTT) and uses Time of Flight calculations to estimate distance. The measurements are displayed on a ILI9341 TFT LCD. 

## How to use
The radio_A and radio_B folders contains the code for each of the two nRF52840 DKs used. Radio A serves as an initiator for doing measurements and Radio B responds. The ILI9341 TFT LCD should be mounted on Radio A. 

### Flashing
1. Clone the repository.
2. Step in to radio_A/product/nrf52840
    - Run `make`
    - Connect the first nRF52840 DK to your computer
    - Run `make flash`
3. Step in to radio_B/product/nrf52840
    - Run `make`
    - Connect the second nRF52840 DK to your computer
    - Run `make flash`


## Current State
The project is still under development. Currently, the algorithm calculates the clock cycles the packets use between the boards. If the measurements where linear over distance, the conversion to meters would be fairly simple, but the measurements does not give linear results (yet). 

## License
MIT License Copyright (c) 2019 Martin Aalien