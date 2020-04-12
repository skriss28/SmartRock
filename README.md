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


  
