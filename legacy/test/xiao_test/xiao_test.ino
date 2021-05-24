void setup()
{
     // USBでCDCクラスの初期化を行う
     SerialUSB.begin(9600);  // シリアル通信の速度は9600bps
}
void loop()
{
     char inChar;

     if (SerialUSB.available() > 0) {
          // データが受信されたら１バイト読み込む
          while( -1 == (inChar = SerialUSB.read()) );
          // 読み込んだ"データ+1"で返す
          SerialUSB.println((char)(inChar+1));
     }
     delay(10);
}
