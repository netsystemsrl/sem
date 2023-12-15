
/*  SENSORI TANICA -------------------------------------------------------*/
//#define LEVEL1 12  // Livello tanica 1, 3, 4, 5

/*
#define LEVEL1PIN        32        // Pin ADC per sensore livello L1
#define MaxLevCycle   5000      // Numero di campioni da sensore di livello
#define ConvFactor    10.0      // valori decimali corrispondenti a 1mm di livello

// Variabili per Sensore di Livello
float livello1zero = 0;
int letture1[MaxLevCycle] = {0};
int livello1contatore = 0;

void LevelTank(JsonDocument& sensoriQB  ){

  float livello1 = 0;
  for (int i = 0; i < MaxLevCycle; i++) livello1 = livello1 + letture1[i];
  livello1 = livello1/MaxLevCycle/ConvFactor;

  sensoriQB["TNK1"] = (livello1zero - livello1);

} */

#define LEVEL1PIN        32        // Pin ADC per sensore livello L1
#define LEVEL1PIN2       33        // Pin ADC per sensore livello L1
#define LevelTimeout     10        // timeout attesa dati da calibro [s]


// Variabili per Sensore di Livello
int i;
int sign;
long value;
float result;
//int clockpin = 33;  //LEVEL1PIN2
//int datapin = 32;   //LEVEL1PIN
unsigned long tempmicros;
unsigned long PREV_MILLIS_LEVEL;


void LevelTank(JsonDocument& sensoriQB  ){

  pinMode(LEVEL1PIN, INPUT);
  pinMode(LEVEL1PIN2, INPUT);
  PREV_MILLIS_LEVEL = millis();

  while(millis() - PREV_MILLIS_LEVEL < (LevelTimeout * 1000)) {
    while (digitalRead(LEVEL1PIN2)==HIGH && (millis() - PREV_MILLIS_LEVEL < (LevelTimeout * 1000))) {} //if clock is LOW wait until it turns to HIGH
    tempmicros=micros();
    while (digitalRead(LEVEL1PIN2)==LOW && (millis() - PREV_MILLIS_LEVEL < (LevelTimeout * 1000))) {} //wait for the end of the HIGH pulse
    if ((micros()-tempmicros)>500) { //if the HIGH pulse was longer than 500 micros we are at the start of a new bit sequence

      sign=1;
      value=0;
      for (i=0;i<23;i++) {
        while (digitalRead(LEVEL1PIN2)==HIGH && (millis() - PREV_MILLIS_LEVEL < (LevelTimeout * 1000))) { } //wait until clock returns to HIGH- the first bit is not needed
        while (digitalRead(LEVEL1PIN2)==LOW && (millis() - PREV_MILLIS_LEVEL < (LevelTimeout * 1000))) {} //wait until clock returns to LOW
        if (digitalRead(LEVEL1PIN)==LOW) {
          if (i<20) {
            value|= 1<<i;
          }
          if (i==20) {
            sign=-1;
          }
        }
      }

      result=(value*sign)/100.00;    
      sensoriQB["TNK1"] = result;
      break;
    }
    break;
  }

}
