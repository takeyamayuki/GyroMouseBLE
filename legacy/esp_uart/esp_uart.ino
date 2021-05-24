#include <BleMouse.h>
BleMouse bleMouse;
float x,y,z;  
float x_kand=3;
float y_kand=3;

void setup() {
  pinMode(16,INPUT);
  pinMode(17,INPUT);
  pinMode(18,INPUT);
  
  Serial.begin(115200);
  
  bleMouse.begin();
}

void loop() {
  
  x=Serial.read();
  y=Serial.read();
  z=Serial.read();
  
  
  if(bleMouse.isConnected()) {     //接続状況確認
    if(digitalRead(18)==LOW){
       bleMouse.move(-z*x_kand,y*y_kand,0);
    }
    if(digitalRead(16)==LOW){
       bleMouse.click() ;
       delay(100);
    }
    if(digitalRead(17)==LOW){
       bleMouse.click(MOUSE_RIGHT) ;
       delay(100);
    }  
  }
}
