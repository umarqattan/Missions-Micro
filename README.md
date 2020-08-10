# Missions-Micro
[Pressure Sensors](#pressure-sensors)  
[Motion Sensors](#motion-sensor)  
[Haptics](#haptics)  

## Pressure Sensors

### Service  
```c
#define SERVICE_UUID_1 "4fafc201-1fb5-459e-8fcc-c5c9c331914b" 
```

## Generate UUIDs
[_Online UUID Generator_](https://uuidgenerator.net)

### Characteristics
_Write to this one to update the Pressure Sensors_  
```c
uint8_t* arr = {off/on, pressureSensorsSampleRate}
/* 
send either 1 or 2 bytes

1 byte: {off/on}
	0x01 for off
	0x02 for on
2 bytes: {off/on, pressureSensorsSampleRate}
	off/on: 0x01 for off OR 0x02 for on
	pressureSensorsSampleRate:
		0x01 for very slow (512 ms)
		0x02 for slow      (256 ms)
		0x03 for medium    (128 ms)
		0x04 for fast      (64 ms)
		0x05 for very fast (32 ms)
*/

#define PRESSURE_SENSOR_CONFIGURATION_CHARACTERISTIC_UUID "9229de5d-1d33-4b66-8ac7-0e4993f743f8"
#define TOP_SENSORS_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BOTTOM_SENSORS_CHARACTERISTIC_UUID "0031a9bf-b569-4f8c-9d18-2256c9561d83"
```


## Motion Sensor

## Service  
```c
#define SERVICE_UUID_2 "7a658cba-0dcd-4d02-bb97-80296cf72dfd"
```

### Characteristics
_Write to this one to update the IMU_  
```c
uint8_t* arr = {off/on, accelGyroSampleRate, magSampleRate}

/*
send either 1 or 3 bytes

1 byte: {off/on}
	0x01 for off
	0x02 for on
3 bytes: {off/on, accelGyroSampleRate, magSampleRate}
	off/on: 0x01 for off OR 0x02 for on
	accelGyroSampleRate:
		0x01 for very slow (32 Hz)
		0x02 for slow      (64 Hz)
		0x03 for medium    (128 Hz)
		0x04 for fast      (256 Hz)
		0x05 for very fast (512 Hz)
	magSampleRate:
		0x01 for very slow (10 Hz)
		0x02 for slow      (20 Hz)
		0x03 for medium    (40 Hz)
		0x04 for fast      (60 Hz)
		0x05 for very fast (80 Hz)
*/
		
#define IMU_CONFIGURATION_CHARACTERISTIC_UUID "42ac498a-94da-4c68-b242-bcf8d92c17aa"
// {float yaw, float pitch, float roll} [in degrees]
#define YPR_CHARACTERISTIC_UUID "c7e6d447-a0a7-4e22-bf14-c0be89905302"
// {float accelX, float accelY, float accelZ} [in m/s^2]
#define ACCELEROMETER_CHARACTERISTIC_UUID "142bc492-b0cf-45ea-a496-338f21f1a815"

// {float gyroX, float gyroY, float gyroZ} [in deg/s]
#define GYROSCOPE_CHARACTERISTIC_UUID "46382de9-48e8-40b2-94fd-4612af2caa96"
// {float magX, float magY, float magZ} [microT/s]
#define MAGNETOMETER_CHARACTERISTIC_UUID "be7db1c9-cddf-41cd-8146-cadebae3b308"
```


## Haptics

## Service  
```c
#define SERVICE_UUID_3 "31408399-730b-4d53-911e-993cd531e96f"
```

### Characteristic
```c
/*
 * Haptics Configuration [range from 1 to 8 bytes]
 *  Slots 0...7 (Total of 8 slots) 
 *      slot 0: hex:0x00 (decimal:0) - hex:0x7B (decimal:123)
 *      slot 1: hex:0x00 (decimal:0) - hex:0x7B (decimal:123)
 *      ...
 *      Slot 8: hex:0x00 (decimal:0) - hex:0x7B (decimal:123)
 *  ** Note: there may be fewer than 8 slots in the haptics configuration
 */
 
#define HAPTICS_CHARACTERISTIC_UUID "ff227c5f-bfef-4db4-8ce0-e4ef8e520973"
```
