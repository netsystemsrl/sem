/*  SENSORI AIR ----------------------------------------------------------*/
#define ANEPIN        34        // Anemometro
#define ANE_TON       15000     // TimeOut di attesa per ricezione dati da sensore (in caso non sia collegato)
#define MaxANECycle   2         // Numero di cicli di lettura da sensore AQB (sempre >=2 perche il primo ciclo è da scartare)

/******* ANEMOMETRO ********/
void Anemometer(JsonDocument& sensoriQB ){
  byte  hdldata_id = 0; // Numero del dato da ricevere dai dispositivi con codifica 4+4 bit
  bool hdldata_1st = 0; // Ricezione primo/secondo gruppo di 4-bit dai dispositivi con codifica 4+4 bit
  byte  hdldata_L = 0;  // Gruppo di 4-bit meno significativo dai dispositivi con codifica 4+4 bit
  byte  hdldata_M = 0;  // Gruppo di 4-bit più significativo dai dispositivi con codifica 4+4 bit
  int  hdldata_val = 0; // Dato dai dispositivi con codifica 4+4 bit
  bool oneshot_0 = 1;
  bool oneshot_1 = 1;
  bool oneshot_2 = 1;
  byte aneCount = 0;
  bool edge = 0; 
  int CNT_016 = 0;
  int CNT_017 = 0;
  unsigned int PREV_MILLIS_ANE = 0;
  unsigned int END_MILLIS_ANE = 0;

  PREV_MILLIS_ANE = millis();
  END_MILLIS_ANE = millis();
  AirSpeed = 0.0;

  while ((millis() - END_MILLIS_ANE) < ANE_TON) {
    // Creazione fronti Air Quality Box
    if (digitalRead(ANEPIN)) {
      if (!edge) {
        CNT_016++;
        CNT_017 = 65535; // Contatore caricato
        edge = 1;
      }
    }
    else if (edge) edge = 0;

    // Ogni 1 millisecondo decodifica AirQualityBox
    if (millis() - PREV_MILLIS_ANE >= 1) {
      PREV_MILLIS_ANE = millis();

      if (CNT_017 > 0) CNT_017--;

      if (CNT_017 >= (65535 - HANDLEINTERV)) {
        // Se il tempo tra un impulso e l'altro è più breve di HANDLEINTERV, vengono contati
        oneshot_0 = 0;
        oneshot_1 = 0;
        oneshot_2 = 0;
      }
      if (CNT_017 < (65535 - HANDLEINTERV)) { 
        // Altrimenti significa che è in arrivo un altro gruppo di impulsi (sono 2 gruppi da 4-bit)
        if (!oneshot_0) {
          if (!hdldata_1st) {     // La ricezione inizia dal secondo gruppo di 4-bit
            hdldata_L = CNT_016 - 1;  // Ricezione del secondo gruppo di 4-bit
            hdldata_1st = 1; // Ora è in attesa il primo gruppo di 4-bit
            CNT_017 = 65535; // Il contatore generale è resettato per entrare di nuovo
            // in questa routine e fare il check del primo gruppo di 4-bit...
          }
          else {      // ...che avviene qui
            hdldata_M = CNT_016 - 1;  // Ricezione del primo gruppo di 4-bit ed elaborazione
            hdldata_val = hdldata_L + (hdldata_M * 16);
            switch (hdldata_id) {
              case 0: sensoriQB["AneT"] = hdldata_val; break;
              case 1: sensoriQB["AneH"] = hdldata_val; break;
              case 2: AirSpeed = float(hdldata_val) / 10.0; sensoriQB["AneSPEED"] = (AirSpeed); break;
            }
          }
          CNT_016 = 0;
          oneshot_0 = 1;
        }
      }
      if (CNT_017 < (65535 - RXDATAINTERV)) {
        // Passa alla ricezione del dato successivo
        if (!oneshot_1) {
          hdldata_1st = 0;
          CNT_016 = 0;
          hdldata_id++;
          oneshot_1 = 1;
        }
      }
      if (CNT_017 < (65535 - HANDLERESET)) {
        // Ricomincia il giro dei dati
        if (!oneshot_2) {
          oneshot_2 = 1;
          END_MILLIS_ANE = millis();
          aneCount++;
          if (aneCount >= MaxANECycle) break;
        }
        hdldata_1st = 0;
        CNT_016 = 0;
        hdldata_id = 0;
      }
    }
  }
}