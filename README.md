# bluetooth_gyro_mouse
An arduino program using MPU6050, ESP32 with bluetooth.    
You can see how it works on [youtube](https://youtu.be/DzT40SCh3nI)  
- The main program is in the [src directory](src).  
- The circuit is in [circuit directory](circuit).  

# requirements
Follow the [official arduino library installation](https://www.arduino.cc/en/guide/libraries).  
- [ESP32 BLE Mouse library](https://github.com/T-vK/ESP32-BLE-Mouse)  

# Usage
In this following circuit,  
<img src="https://user-images.githubusercontent.com/22733958/118673461-fc42a100-b833-11eb-9869-2fa7800ffb94.png" width="350px">

|switch|behavior|
|:-:|:-:|
|Rightmost switch|click|
|Middle switch|right click|
|Leftmost switch|scroll|
|Touch switch|stop the mouse cursor moving|

If you face the problem of the y-axis drifting, try some of the variables "offsety" in line 23.  
<img width="216" alt="スクリーンショット 2021-08-28 午後5 08 24" src="https://user-images.githubusercontent.com/22733958/131211237-7e0db5ac-af1b-4560-a71b-4da47c72b954.png">

![github_usage1-アニメーションイメージ（大）](https://user-images.githubusercontent.com/22733958/130589777-bb9e9679-aeb6-43dd-bc9e-a26c03030525.gif)  
sorry, it's hard to see😅

