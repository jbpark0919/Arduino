#define PIN_LED 7
unsigned int toggle, count, second;
void setup() {
pinMode(PIN_LED, OUTPUT);
count = 0;
toggle = 1;
second = 1000;
}

void loop() {
toggle = toggle_state(toggle);
digitalWrite(PIN_LED, toggle);
delay(second);
second = 100;
count++;
if (count == 12) {
while(1){

}
}
}

int toggle_state(int toggle) {
return 1 - toggle;
}

