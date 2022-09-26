void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
}

void loop() {
  if (Serial.available()){
    char order = Serial.read();
    if (order != '\n'){
      if (order == '1')
        Serial.print(digitalRead(2));
      else if (order == '2')
        Serial.print(digitalRead(3));
      else if (order == '3')
        Serial.print(digitalRead(4));
    }
  }
}
