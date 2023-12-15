/*  SENSORI QB -----------------------------------------------------------*/

#define HANDLEINTERV  25        // Pausa dopo la quale il device attende il successivo gruppo di 4-bit dai dispositivi con codifica 4+4 bit [0-255ms]
#define RXDATAINTERV  200       // Pausa massima per il passaggio al dato successivo dai dispositivi con codifica 4+4 bit [0-65535ms]
#define HANDLERESET   400       // Pausa massima per una nuova attesa di dati dai dispositivi con codifica 4+4 bit [0-65535ms]
#define AQBPIN        2         // Sensore Qualità Aria
#define MaxAQBCycle   2         // Numero di cicli di lettura da sensore AQB (sempre >=2 perche il primo ciclo è da scartare)
#define AQB_TON       15000     // TimeOut di attesa per ricezione dati da sensore (in caso non sia collegato)

void AirQuality(JsonDocument& sensoriQB ){
  
  byte  hdldata_id = 0; // Numero del dato da ricevere dai dispositivi con codifica 4+4 bit
  bool hdldata_1st = 0; // Ricezione primo/secondo gruppo di 4-bit dai dispositivi con codifica 4+4 bit
  byte  hdldata_L = 0;  // Gruppo di 4-bit meno significativo dai dispositivi con codifica 4+4 bit
  byte  hdldata_M = 0;  // Gruppo di 4-bit più significativo dai dispositivi con codifica 4+4 bit
  int  hdldata_val = 0; // Dato dai dispositivi con codifica 4+4 bit
  bool oneshot_0 = 1;
  bool oneshot_1 = 1;
  bool oneshot_2 = 1;
  byte aqbCount = 0;
  bool edge = 0; 
  int CNT_016 = 0;
  int CNT_017 = 0;
  unsigned int PREV_MILLIS_AQB = 0;
  unsigned int END_MILLIS_AQB = 0;

  enum DEV_DATA {
    ID_INDEX = 0,
    RSSI_INDEX,
    T_INDEX,
    H_INDEX,
    CO2_INDEX,
    VOC_INDEX,
    CH2O_INDEX,
    PM1_INDEX,
    PM2_INDEX,
    PM4_INDEX,
    PM10_INDEX,
    O3_INDEX,
    NO2_INDEX,
    SO2_INDEX,
    CO_INDEX,
    DATA_15,
    DATA_16,
    DATA_17,
    DATA_18,
    DATA_19,
    DATA_20,
    RADON_INDEX,
    DATA_22,
    DATA_23,
    DATA_24,
    DATA_25,
    DATA_26,
    DATA_27,
    DATA_28,
    DATA_29,
    DATA_30,
    DATA_31,
    DATA_BUFFER
  };

  PREV_MILLIS_AQB = millis();
  END_MILLIS_AQB = millis();

  while ((millis() - END_MILLIS_AQB) < AQB_TON) {
    // Creazione fronti Air Quality Box
    if (digitalRead(AQBPIN)) {
      if (!edge) {
        CNT_016++;
        CNT_017 = 65535; // Contatore caricato
        edge = 1;
      }
    }
    else if (edge) edge = 0;

    // Ogni 1 millisecondo decodifica AirQualityBox
    if (millis() - PREV_MILLIS_AQB >= 1) {
      PREV_MILLIS_AQB = millis();

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
              case ID_INDEX: sensoriQB["BoxID"] = hdldata_val; break;
              case T_INDEX: sensoriQB["BoxT"] = hdldata_val-128; break;
              case H_INDEX: sensoriQB["BoxH"] = hdldata_val; break;
              case CO2_INDEX: sensoriQB["BoxCO2"] = map(hdldata_val,0,255,400,2000); break;
              case VOC_INDEX: sensoriQB["BoxVOC"] = map(hdldata_val,0,255,0,1000); break;
              case CH2O_INDEX: sensoriQB["BoxCH2O"] = hdldata_val; break;
              case PM1_INDEX: sensoriQB["BoxPM1"] = hdldata_val; break;
              case PM2_INDEX: sensoriQB["BoxPM2"] = hdldata_val; break;
              case PM4_INDEX: sensoriQB["BoxPM4"] = hdldata_val; break;
              case PM10_INDEX: sensoriQB["BoxPM10"] = hdldata_val; break;
              case O3_INDEX: sensoriQB["BoxO3"] = hdldata_val; break;
              case NO2_INDEX: sensoriQB["BoxNO2"] = hdldata_val; break;
              case SO2_INDEX: sensoriQB["BoxSO2"] = hdldata_val; break;
              case CO_INDEX: sensoriQB["BoxCO"] = hdldata_val; break;
              case RADON_INDEX: sensoriQB["BoxRDN"] = map(hdldata_val,0,255,0,1000); break;
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
          END_MILLIS_AQB = millis();
          aqbCount++;
          if (aqbCount >= MaxAQBCycle) break;
        }
        hdldata_1st = 0;
        CNT_016 = 0;
        hdldata_id = 0;
      }
    }
  }
}



