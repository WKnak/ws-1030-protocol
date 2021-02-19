/**
 * Leitura dos dados do Weather Station ws1030
 * Autor: William Knak
 * 
 * 11110101 01101100 10110001 00000000 11000000 10110101 01011100 00000000 00000001 00000000 00000001 00111001 000111100 101010
 *  245
 */
 
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp180;

#define RF_IN 5
#define LED_PACKET A2
#define LED_ACTIVITY  13
#define BUZ_PIN 9

volatile byte imprime = 0;

volatile unsigned long ultimo = 0, total = 0; 
volatile byte idx = 0;
volatile byte start_byte = 0; 

char buff[250];
byte data[13];

long pressureSeaLevel = 0;
float tempInterna = 0;
long lastMillis = -99999;
long ledMillis = 0;

String s_wind_dir;
int temp = 0;
int avg = 0;
int gust = 0;
long rain = 0;
String idEstacao = "--";
int umidade = 0;

ISR(TIMER1_COMPA_vect)
{
  static byte is_hi = 0;  
  
  static byte bit_cnt = 0;
  int BIT = -1;
  byte tmp;

  if (digitalRead(RF_IN) == HIGH) {

    if(!is_hi) {

      // borda de subida      
      
      is_hi = 1;

      const int m1 = 5; const int m2 = 38;

      // A INFORMAÇÃO DE TOTAL É REFERENTE AO TEMPO DE LOW
      if(( total>=m1 ) && (total <= m2)) {
          //buff[cnt] = '_';
          //cnt++;
      } else {
        start_byte = 0;
        //imprime = 1;        
        //cnt = 0;
      }
          
      total = 1;      
      
    } else {
      total++;
      
    }
    
  } else {
    
    if(is_hi) {

      // borda de descida

      is_hi = 0;

      // 16mHz / 1600 = 10kHz    = 0,0001 seg    = 0,1 milisegundos    = 100 microsegundos

      // A INFORMAÇÃO DE TOTAL É REFERENTE AO TEMPO DE HIGH
      if(( total>=4 ) && (total <= 14)) {
          BIT = 0x0;
          //buff[cnt] = '0';
          //cnt++;          
      } else if(( total>=16 ) && (total <= 26)) {
          BIT = 0x1;
          //buff[cnt] = '1';          
          ///cnt++;
      } else {
        BIT = 0xF;  // qualquer outro valor
        //start_byte = 0;
      }

      if(( BIT==0 ) || (BIT == 1 ) ) {
        
          if(start_byte != 245) { // ainda não detectou inicio do pacote
            start_byte <<= 1;
            start_byte |= BIT;
            
            if(start_byte == 245) {              
              //Serial.println("START BYTE 245 !!");              // detectou inicio de pacote
              digitalWrite(LED_ACTIVITY, HIGH);
              idx = 0;
              bit_cnt = 1;
              data[idx] = start_byte;
              idx++;              
              data[idx] = 0x00;
            }
            
          } else {

            // já detectou inicio do pacote;

            if(bit_cnt<=8) {
              tmp = (byte) data[idx];
              tmp <<= 1;
              tmp |= BIT;
              
              data[idx] = (byte)tmp;
              bit_cnt++;
              if(bit_cnt==9) {
                idx++;
                data[idx] = 0x00;
                bit_cnt = 1;
                if(idx == 13) {
                  imprime = 1;
                  idx = 0;
                }
              }
            }            
          }
      }

      total = 1;
      
    } else {
      total++;      
    }
  }  
}

void setup() {
  
  pinMode(LED_PACKET, OUTPUT);
  pinMode(LED_ACTIVITY, OUTPUT);
  pinMode(RF_IN, INPUT);
  
  Serial.begin(115200);
  Serial.println("WEATHER STATION WS-1030 DATA LOGGER");
  Serial.println("TiNX Tecnologia");
  Serial.println("");
  Serial.println("[TEMPERATURA] [HUMIDADE] [VENTO DIRECAO] [VENTO AVG] [VENTO RAJADA] [CHUVA ACUMULADA]");

  while (1) {

    delay(500);
    digitalWrite(LED_ACTIVITY, HIGH);
    if (bmp180.begin()) {
        Serial.println("Sensor BM180 inicializado !!");
        break;
    }

    // NUNCA CHEGA AQUI, O BEGIN TRANCA 
    Serial.println("## Sensor BM180  nao encontrado ##");
    delay(500);
    digitalWrite(LED_ACTIVITY, LOW);      
  }
    
  TCCR1A = 0x00;
  TCCR1B = 0x09;
  TCCR1C = 0x00;
  OCR1A = (800-1);
  TIMSK1 = 0x02;
  
  // enable interrupts
  sei();
}

void beep() {
  analogWrite(BUZ_PIN, 200);
  delay(10);
  analogWrite(BUZ_PIN, 0);
  
}

void outputExt() {
    String data = "[data]|";
    data  += idEstacao;
    data  += "|";
    data  += temp;
    data  += "|";
    data  += umidade;
    data  += "|";
    data  += s_wind_dir;
    data  += "|";
    data  += avg;
    data  += "|";
    data  += gust;
    data  += "|";
    data  += rain;
    data  += "|";
    data += tempInterna;
    data += "|";
    data += pressureSeaLevel;    
    data  += "|[/data]";

    digitalWrite(LED_ACTIVITY, HIGH);
    ledMillis = millis();
    
    Serial.println(data);  
}

void loop() {

  //Serial.println(bmp180.readTemperature());
  //delay(2500);

  if(imprime == 1 ) {
    start_byte = 0;
    imprime = 0;
    String res;
    byte chksum = 0x00;
    for(int x = 0; x<13; x++) {
      //Serial.print(data[x], DEC);
      //Serial.print(" ");            
      if(x<12) chksum+=data[x];
      else {
        // evita enviar duas leituras iguais
        if( (millis() < lastMillis) || ( (millis() - lastMillis) >= 1000) ) {
          lastMillis = millis();
        
          if(data[12] == chksum) {          
            // Serial.print(" ! CHKSUM OK !\r\n Direcao Vento: ");
  
            byte wind_direction = data[4] >> 5; // 3 bits + significativos
  
            /*  Direção do Vento
                000 - NORTH
                001 - NORTHEAST
                010 - EAST
                011 - SOUTHEAST
                100 - SOUTH
                101 - SOUTHWEST
                110 - WEST
                111 - NORTHWEST
             */
            
            switch(wind_direction) {
                case 0b000: s_wind_dir = "N"; break;
                case 0b001: s_wind_dir = "NE"; break;
                case 0b010: s_wind_dir = "E"; break;
                case 0b011: s_wind_dir = "SE"; break;
                case 0b100: s_wind_dir = "S"; break;
                case 0b101: s_wind_dir = "SW"; break;
                case 0b110: s_wind_dir = "W"; break;
                case 0b111: s_wind_dir = "NW"; break;              
            }
  
            // temperatura
            // falta determinar o bit do negativo
            temp = ((data[4] & 0x00000001) * 256) + data[5];
  
            // umidade
            umidade = data[6];
  
            // vento - média
            avg = data[7];
  
            // vento - rajada          
            gust = data[8];
  
            // chuva (acumulada)
            rain = ((data[10]-1) * 256) + data[11];
  
            // identificacao da estacao (até trocar as pilhas)
            idEstacao = "";
            idEstacao += data[1];
            idEstacao += "-";
            idEstacao += data[2];          
  
            pressureSeaLevel = bmp180.readSealevelPressure(37.0);
            tempInterna = bmp180.readTemperature();         
            
            outputExt();
            
            //beep();
          } else {
            //Serial.print(" # CHKSUM INVALIDO #");          
          }
        }
      }
     digitalWrite(LED_ACTIVITY, LOW);
      
    }    
    //Serial.println();  
  }  
  
  if( ledMillis && ((millis() < ledMillis) || ( (millis() - ledMillis) >= 1000) )) {
    ledMillis = 0;
    digitalWrite(LED_ACTIVITY, LOW);
  }

  /*
  if( (millis() < lastMillis) || ( (millis() - lastMillis) >= 29900) ) {
    lastMillis = millis();
    pressureSeaLevel = bmp180.readSealevelPressure(37.0);
    tempInterna = bmp180.readTemperature();
    outputInt();    
  }*/  
}
