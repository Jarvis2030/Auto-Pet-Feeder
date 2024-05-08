# Desciption
We aim to build a auto pet feeder that allow the pet master to leave for a while without starving the pet. Our project is build on the STM32F103VET6 microcontroller, 
it included two parts: Pet feeder and GPS tracker, both are connected with external sensor and achievedwireless communication using LoRa module. 

# Project preview
<img width="430" height="455" alt="GPS tracker" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/90afb002-df9f-4ed5-952e-31f3df77c7b2">
<img width="430" alt="Auto pet feeder" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/77e636ef-2d0c-41b4-9a8e-f9b693ce28a7">


# Pin assignment
<img width="460" alt="Slave board pin assignment" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/e6f6cfc8-0b83-4dd6-8037-4ea275949f02">
<img width="440" alt="Main board pin assignment" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/24631bb9-3846-4eb7-8004-ba159ca4137a">
<img width="450" alt="gps tracker pin assignment" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/d059984b-5757-46d1-ac13-08ab98a5ff6a">

# Function List
### Auto Pet feeder:
1. **Manual Mode**: Users can manually set the desired weight of food and distribute it using the control panel on the side of the feeder.
2. **Timer Mode**: Users can set a real-time alarm, and when the specified time is reached, the system will automatically distribute the pre-set weight of food to the pet.
3. **Sensor Integration**: The feeder is equipped with sensors such as an ultrasound distance sensor and MPU6050 to detect if the pet has approached the bowl and started eating. These sensors measure distance and bowl movement.
4. **Photo Capture**: Once the feeder detects that the pet has finished eating and left, it captures a photo and modify it to BW, send it through usart to computer for display. This allows users to check the last eating situation.
5. **GPS Integration**: The feeder incorporates a GPS sensor on the pet. If the feeder determines that the pet is not at home based on the location information, it will not trigger any alarms or distribute food. <br>
## Pet Tracker:
1. **Location Tracking**: The tracker is attached to the pet, and it periodically sends the pet's location using a LoRa module.
2. **Signal Monitoring**: The feeder continuously monitors the signal from the pet tracker. If the signal is lost or the location indicates that the pet is not at home, the feeder will not be activated.


# External Components
1. MPU6050
2. HX711
3. HC-SR04
4. DX-LR01
5. NEO-6M

# Communication protocol
1. USART/UART
2. I2C
3. Internal clock Timer