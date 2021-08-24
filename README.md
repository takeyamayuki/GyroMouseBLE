# bluetooth_gyro_mouse
An arduino program using MPU6050, ESP32 with bluetooth.    
You can see how it works on [youtube](https://youtu.be/DzT40SCh3nI)

# requirements
Follow the official instructions of [arduino library installation](https://www.arduino.cc/en/guide/libraries).  

- [ESP32 BLE Mouse library](https://github.com/T-vK/ESP32-BLE-Mouse)  

# program
The main program is in the [src](src) directory.  
# circuit
you can also see this picture in [circuit](circuit) directory.  

# Usage
In this following circuit,  
<img src="https://user-images.githubusercontent.com/22733958/118673461-fc42a100-b833-11eb-9869-2fa7800ffb94.png" width="350px">

|switch|behavior|
|:-:|:-:|
|Rightmost switch|click|
|Middle switch|right click|
|Leftmost switch|scroll|
|Touch switch|stop the mouse cursor moving|

![github_usage1-アニメーションイメージ（大）](https://user-images.githubusercontent.com/22733958/130589777-bb9e9679-aeb6-43dd-bc9e-a26c03030525.gif)



