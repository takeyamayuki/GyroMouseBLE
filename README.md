# bluetooth_gyro_mouse
An arduino program using MPU6050, ESP32 with bluetooth.    
You can see how it works on [youtube](https://youtu.be/DzT40SCh3nI)

# requirements
- [ESP32 BLE Mouse library](https://github.com/T-vK/ESP32-BLE-Mouse)  

Follow the official instructions of [arduino library installation](https://www.arduino.cc/en/guide/libraries).

# program
The main program is in the [src](src) directory.  
# circuit
you can also see this picture in [circuit](circuit) directory.  

<img src="https://user-images.githubusercontent.com/22733958/118673461-fc42a100-b833-11eb-9869-2fa7800ffb94.png" width="350px">

# Usage
In the circuit

|switch|behavior|
|:-:|:-:|
|Rightmost switch|click|
|Middle switch|right click|
|Leftmost switch|scroll|
|Touch switch|stop the mouse cursor moving|

