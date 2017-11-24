
int pinI1=8;//define I1 interface
int pinI2=11;//define I2 interface 
int lastTime=0;
int currentTime = 0;
int period = 1000/60;
void setup() {
  // put your setup code here, to run once:
 pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
    currentTime = millis();
  lastTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(pinI2,LOW);
 digitalWrite(pinI1,HIGH);
 delay(period/2);
  digitalWrite(pinI2,HIGH);
 digitalWrite(pinI1,LOW);
  delay(period/2);
}
