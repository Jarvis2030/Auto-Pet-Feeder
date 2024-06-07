# Desciption
We aim to build a auto pet feeder that allow the pet master to leave for a while without starving the pet. Our project is build on the STM32F103VET6 microcontroller, 
it included two parts: Pet feeder and GPS tracker, both are connected with external sensor and achievedwireless communication using LoRa module. 

# Project preview
<img width="430" height="455" alt="GPS tracker" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/90afb002-df9f-4ed5-952e-31f3df77c7b2">
<img width="430" alt="Auto pet feeder" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/77e636ef-2d0c-41b4-9a8e-f9b693ce28a7">


# Pin assignment
<img width="460" alt="Slave board pin assignment" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/e6f6cfc8-0b83-4dd6-8037-4ea275949f02">
<img width="440" alt="Main board pin assignment" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/24631bb9-3846-4eb7-8004-ba159ca4137a">


# Function List

## Auto Pet feeder:
1. **Manual Mode**: Users can manually set the desired weight of food and distribute it using the control panel on the side of the feeder.
2. **Timer Mode**: Users can set a real-time alarm, and when the specified time is reached, the system will automatically distribute the pre-set weight of food to the pet.
3. **Sensor Integration**: The feeder is equipped with sensors such as an ultrasound distance sensor and MPU6050 to detect if the pet has approached the bowl and started eating. These sensors measure distance and bowl movement.
4. **Photo Capture**: Once the feeder detects that the pet has finished eating and left, it captures a photo and modify it to BW, send it through usart to computer for display. This allows users to check the last eating situation.
5. **GPS Integration**: The feeder incorporates a GPS sensor on the pet. If the feeder determines that the pet is not at home based on the location information, it will not trigger any alarms or distribute food. <br>

## Pet Tracker:
1. **Location Tracking**: The tracker is attached to the pet, and it periodically sends the pet's location using a LoRa module.
2. **Signal Monitoring**: The feeder continuously monitors the signal from the pet tracker. If the signal is lost or the location indicates that the pet is not at home, the feeder will not be activated.

<img height="450" alt="Main board operation logic" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/ff3b109f-e0e2-43a8-af34-ead9d1715f52">
<img height="450" alt="GPS tracker operation logic" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/7c194044-146b-4a12-bf3a-6f3da486ffcf">


# External Components
| MPU6050   | HX711      | HC-SR04              |DX-LR01  | NEO-6M   |
| --------- |:----------:| :-------------------:|:-------:|:--------:|
| <img height="200" alt="MPU6050" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/98d105ca-1610-4d39-82cc-1c7c62c4f73d"> | <img height="200" alt="HX711" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/042aa603-a59a-4325-b613-8c6f243047a6"> | <img height="200" alt="HCSR04" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/6e5d6553-68a8-4977-b694-e432b62d145f">| <img width="200" alt="DXLR01" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/5c831647-ceab-48dc-bd56-7fa476a49f91)">|<img height="200" alt="NEO6M" src="https://github.com/Jarvis2030/Auto-Pet-Feeder/assets/77675271/aeb4cced-b46d-4270-a882-80e7aa73e55a">|
| Gyro & Accelrator      | Weight sensor |  Ultrasound distance sesonr | LoRa | GPS tracker |
