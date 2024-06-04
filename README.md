# Water Meter Unit

This project is a water meter unit that measures water flow, volume, temperature, and turbidity. The system also includes features like leakage detection, valve control, and balance management. The water meter unit is built using an AVR microcontroller.

## Features

- **Water Flow Measurement:** Calculates the flow rate of water.
- **Volume Calculation:** Tracks the total volume of water used.
- **Temperature Measurement:** Measures the temperature of the water.
- **Turbidity Measurement:** Measures the turbidity of the water.
- **Leakage Detection:** Detects and alerts in case of water leakage.
- **Valve Control:** Controls the water valve based on various conditions.
- **Balance Management:** Manages the user’s balance based on water usage.
- **Real-Time Clock (RTC):** Keeps track of the current time and date.

## Components

- AVR Microcontroller (ATmega series)
- Flow Sensors 
- Temperature Sensor (LM35)
- Turbidity Sensor
- Real-Time Clock (DS1307)
- LCD Display
- Buzzer
- Water Valve
- EEPROM for storing balance and volume

## Circuit Diagram
See the circuit diagram by clicking this link below
![image](https://github.com/mashai-letlatsa/Water-Meter-Unit/assets/161247807/10aa16b8-dfe6-402b-9fb2-a236273ef36e)

## Code Explanation

### Initialization Functions

- `lcdInit()`: Initializes the LCD display.
- `ADC_Init()`: Initializes the ADC for temperature and turbidity measurement.
- `i2c_init()`: Initializes I2C communication for the RTC.
- `rtc_init()`: Initializes the RTC.
- `USART_Init()`: Initializes the USART for serial communication.

### Main Loop

The main loop continuously updates the water meter unit by performing the following tasks:

- Updates the flow rate and volume.
- Displays the current temperature and turbidity on the LCD.
- Checks for leakage and controls the valve.
- Updates and displays the current time and date.
- Manages the user's balance based on water usage.

### Interrupt Service Routines (ISR)

- `ISR(INT0_vect)`: Handles external interrupts for flow measurement.
- `ISR(INT1_vect)`: Handles external interrupts for pulse counting.
- `ISR(TIMER1_OVF_vect)`: Handles Timer1 overflow interrupts for flow rate calculation.

## How to Use

1. **Setup the hardware** according to the circuit diagram.
2. **Upload the code** to the AVR microcontroller.
3. **Run the system** and monitor the LCD display for real-time updates on water flow, volume, temperature, turbidity, and leakage status.
4. **Adjust the settings** as necessary for your specific requirements.

## License
This project is licensed under the MIT License.

## Contact

For any questions or support, please contact [Kopanang Letlatsa] at [mashailetlatsa@gmail.com].


