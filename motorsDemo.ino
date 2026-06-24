const int RA1 = 9;
const int RA2 = 10;
const int dec1 = 11;
const int dec2 = 12;

const int speed_fast = 100;
const int speed_slow = 50;

const int runtime = 5000;

void setup() {
  pinmode(RA1, OUTPUT);
  pinmode(RA2, OUTPUT);
  pinmode(dec1, OUTPUT);
  pinmode(dec2, OUTPUT);

}

void loop() {
  //Pattern 1: RA left then right FULL SPEED, stop, DEC Up then down FULL SPEED, stop
  

  //Patern 2: Tracking Speed repeat of pattern 1

  //Pattern 3: both motors at full speed; RA Left Dec Up, stop, RA Right Dec Down, stop

}
