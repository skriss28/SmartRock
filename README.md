# SmartRock

Authors: Laws Smith and Stevan Kriss\
University: Washington and Lee University, Lexington, VA\
Professors: Jon Erickson and Dave Harbor\
\
Independent study collaboration between the W&L Engineering and Geology Departments.  The goal is to create a self contained measurement system that will be placed within a rock at the bottom of a nearby river to gather pressure, acceleration, and gyro data during storm/rainwater events.<br/>
  
## Electronics
 
   ![Image description](https://user-images.githubusercontent.com/63022881/78575936-01908800-77fa-11ea-98aa-a51f17e20751.jpg)
*Figure 1.  SmartRock electronics consisting of a Feather Cortex LoRa microcontroller, an IMU accelerometer, two pressure sensors, a LiPo battery, and an Adalogger for the SD card.*<br/>

**Feather M0 Cortex**<br/>
The [Feather M0 Cortex](https://www.adafruit.com/product/3178) is a microcontroller produced by Adafruit with long range radio capability.  It provides fast and effective wireless communication between two units along with sufficient processing power to quickly handle a large amount of code.  The Feather also works well with the corresponding sensors through its I2C protocol and is able to easily connect to an SD card with the Adalogger add-on.<br/>

**Sensors**<br/>
The [MPU-9250 IMU](https://www.sparkfun.com/products/13762) and the [MS5803-14BA Pressure Sensor](https://www.sparkfun.com/products/12909) are both produced by Sparkfun.  For this project, the IMU records accleration and gyros measurements for all 3 axes while the pressure sensors record the pressure at the top and bottom of the rock; these devices communicate with the Feather microcontroller through I2C protocol.<br/>

**Adalogger**<br/>
The [Adalogger Featherwing](https://www.adafruit.com/product/2922) is an additional piece produced by Adafruit that gives their Feather boards the ability to work with an SD card.  It can be soldered directly below the main microcontroller, and it allows the SmartRock to record, save, and dump large amounts of data.<br/>

## Mechanical Aspects


*Figure 2.  SmartRock 3D printed waterproofed casing to hold all necessary electronics.*<br/>

**Casing**<br/>
The casing's primary purpose was to hold all the necessary electronics inside a waterproof container.  It was designed in three pieces (bottom cap, middle shaft, top cap) with the intention of glueing them all together to form a solid cylinder; the pieces were 3D printed using a resin printer.<br/>

**External Port**<br/>
The external port was implemented with the goal of eliminating the need to continually unseal/reseal the entire capsule during its usage.  It utilizes a reed switch and magnetic connections to allow for direct wire communication to the electronics inside via the serial port of the Feather; this gives the user the ability to dump data over this pathway without having to phsically access the SD card inside.  Additionally, connecting to the port also charges the Lipo battery, so ideally once the container is sealed, it can stay shut for a while.
