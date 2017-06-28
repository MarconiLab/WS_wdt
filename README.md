## Weather Station Code
Sensors
TPH, rain, wind speed and direction and light. 
Anemometer, rain gauge and Wind Vane datasheet
https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly..pdf

Boards
Using Mbili and GPRSbee board. 

Updating to ThingSpeak

### On the Setup
- Setup sensors
- Setup GPRS
- Setup RTC
- Setup Sleep
- Setup wdt
- If batt is enough sync RTC online

### On the loop

If RTC interrupt fired
 claer flag
 wdt reset

If RTC interrupt fired
  clear flag
  RTC interrupt update
  take readings every 30 minutes

Put system to sleep

