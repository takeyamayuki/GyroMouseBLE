void setup() {
  // put your setup code here, to run once:
  pinMode(17,INPUT);
  pinMode(15,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(17)==HIGH){
        digitalWrite(15,HIGH) ;
  }
  else digitalWrite(15,LOW) ;
}
