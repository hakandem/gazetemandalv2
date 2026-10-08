#include <SPI.h>
#include <Ethernet.h>
#include <Modbus.h>
#include <ModbusIP.h>
#include <EEPROM.h> 
#include <Wire.h>               // I2C iletişimi için zorunlu paket
#include <Adafruit_ADS1X15.h>   // Adafruit ADS1115 Kütüphanesi

#ifndef W5500_ETHERNET_SHIELD
  #define W5500_ETHERNET_SHIELD
#endif

const int eepromBaslangicAdresi = 0; 

ModbusIP mb;
Adafruit_ADS1115 ads; 

//10.65.10.101
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 };
//10.65.10.100
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x02 };
IPAddress ip(10, 65, 10, 100);
IPAddress myDNS(10, 65, 10, 1);     
IPAddress gateway(10, 65, 10, 1);   
IPAddress subnet(255, 255, 255, 0); 

#define NUM_MOTORS 20
const int aktifMotorSayisi = 20; 

const int potPin[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

const int motorSagPin[NUM_MOTORS] = {2, 4, 6, 11, 46, 12, 14, 16, 18, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42};
const int motorSolPin[NUM_MOTORS] = {3, 5, 7,  9, 47, 13, 15, 17, 19, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43};

// Klon kartın motorları titretmesini engellemek için ana tolerans 10 yapıldı
const int tolerans = 10;
static int durmaSayaci[NUM_MOTORS] = {0};

static int16_t adsSonVeri[4] = {0, 0, 0, 0};
static int adsKanalIndis = 0;
static unsigned long adsSonOkumaZamani = 0;

const uint16_t ADS_KANAL_MUX[4] = {
  ADS1X15_REG_CONFIG_MUX_SINGLE_0,
  ADS1X15_REG_CONFIG_MUX_SINGLE_1,
  ADS1X15_REG_CONFIG_MUX_SINGLE_2,
  ADS1X15_REG_CONFIG_MUX_SINGLE_3
};

// Filtre Tamponları
#define FILTRE_BOYUTU 5
static int potGecmis[NUM_MOTORS][FILTRE_BOYUTU] = {0};
static int filtreIndis[NUM_MOTORS] = {0};
static bool filtreDolu[NUM_MOTORS] = {false};

// 🎯 KLON KART AYARI: Klon kartın yan pinlere sızdırdığı kılcal voltajları 
// tamamen bloke etmek için değişim eşiğini 15 birime çıkardık.
const int degisimEsigi = 15; 
static int sonKararliPotDegeri[NUM_MOTORS] = {0};

int filtreleVeEsikle(int motorIndex, int yeniDeger) {
  potGecmis[motorIndex][filtreIndis[motorIndex]] = yeniDeger;
  filtreIndis[motorIndex]++;
  
  if (filtreIndis[motorIndex] >= FILTRE_BOYUTU) {
    filtreIndis[motorIndex] = 0;
    filtreDolu[motorIndex] = true;
  }
  
  long toplam = 0;
  int adet = filtreDolu[motorIndex] ? FILTRE_BOYUTU : filtreIndis[motorIndex];
  
  if (adet == 0) return yeniDeger;
  
  for (int i = 0; i < adet; i++) {
    toplam += potGecmis[motorIndex][i];
  }
  
  int ortalamaDeger = (int)(toplam / adet);

  // Sızıntı oynaması eşikten küçükse kararsızlığı yut, eski yerinde sabit tut.
  if (abs(ortalamaDeger - sonKararliPotDegeri[motorIndex]) < degisimEsigi) {
    return sonKararliPotDegeri[motorIndex];
  }

  sonKararliPotDegeri[motorIndex] = ortalamaDeger;
  return ortalamaDeger;
}

int guvenliSinir(int deger, int altSinir, int ustSinir) {
  if (deger < altSinir) return altSinir;
  if (deger > ustSinir) return ustSinir;
  return deger;
}

void ADS1115_Resetle() {
  Wire.end(); 
  delay(50);
  Wire.begin(); 
  Wire.setWireTimeout(3000, true); 
  ads.begin(0x48);
  ads.setGain(GAIN_TWOTHIRDS);
  ads.startADCReading(ADS_KANAL_MUX[adsKanalIndis], /*continuous=*/false);
}

void W5500_YenidenBaslat() {
  Serial.println("🔌 [WATCHDOG] Ag Baglantisi Koptu! W5500 Yeniden Baslatiliyor...");
  digitalWrite(44, LOW);
  delay(100);
  digitalWrite(44, HIGH);
  delay(100);

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); 
  mb.config(mac, ip, myDNS, gateway, subnet); 
  delay(500);
}

bool digitalWriteRead(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  if (port == NOT_A_PIN) return LOW;
  return (*portOutputRegister(port) & bit) ? HIGH : LOW;
}

void setup() {
  Serial.begin(9600);
  delay(500); 

  Wire.begin();
  Wire.setWireTimeout(3000, true); 

  ads.begin(0x48);
  ads.setGain(GAIN_TWOTHIRDS); 

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motorSagPin[i], OUTPUT);
    pinMode(motorSolPin[i], OUTPUT);
    digitalWrite(motorSagPin[i], LOW);
    digitalWrite(motorSolPin[i], LOW);
  }
  
  Ethernet.init(44); 
  pinMode(44, OUTPUT);
  digitalWrite(44, HIGH);
  delay(50);

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); 

  mb.config(mac, ip, myDNS, gateway, subnet); 
  delay(2000);

  Serial.print("✅ BASARILI: Kararli IP Oturuldu: ");
  Serial.println(Ethernet.localIP());
  
  for (int i = 0; i < 40; i++) {
    if(i < 20) {
      mb.addHreg(i, 512); 
    } else {
      mb.addHreg(i, 0);   
    }
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    int eepromAdres = eepromBaslangicAdresi + (i * 2);
    int kayitliHedef;
    EEPROM.get(eepromAdres, kayitliHedef);
    if (kayitliHedef < 0 || kayitliHedef > 1023) {
      kayitliHedef = 512;
      EEPROM.put(eepromAdres, kayitliHedef); 
    }
    mb.Hreg(i, kayitliHedef); 
    sonKararliPotDegeri[i] = kayitliHedef; 
  }

  ads.startADCReading(ADS_KANAL_MUX[adsKanalIndis], /*continuous=*/false);
  adsSonOkumaZamani = millis();
}

void loop() {
  Ethernet.maintain();  
  
  static unsigned long lastLinkCheck = 0;
  if (millis() - lastLinkCheck >= 2000) {
    lastLinkCheck = millis();
    if (Ethernet.linkStatus() == LinkOFF) {
      W5500_YenidenBaslat();
    }
  }

  static unsigned long lastModbusTask = 0;
  if (millis() - lastModbusTask >= 2) {
    lastModbusTask = millis();
    mb.task(); 
  }

  if (millis() - adsSonOkumaZamani >= 15) {
    adsSonOkumaZamani = millis();
    
    int16_t hamVeri = ads.getLastConversionResults();
    if (hamVeri < 0) hamVeri = 0;
    adsSonVeri[adsKanalIndis] = hamVeri;

    adsKanalIndis = (adsKanalIndis + 1) % 4;
    ads.startADCReading(ADS_KANAL_MUX[adsKanalIndis], /*continuous=*/false);
  }

  static unsigned long lastUpdate = 0;
  static int eskiHariciDeger = -1;
  static int kilitlenmeBekcisi = 0;

  if (millis() - lastUpdate >= 50) { 
    lastUpdate = millis();

    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
      ADS1115_Resetle();
    }

    for (int i = 0; i < aktifMotorSayisi; i++) {
      int hamOkuma = 0;
      int anlikPozisyon = 0;
      
      if (i < 16) {
        // Çift Örnekleme Koruması
        analogRead(potPin[i]); 
        delayMicroseconds(120); 
        hamOkuma = analogRead(potPin[i]); 
      } else {
        int hariciKanal = i - 16; 
        int16_t hamADC = adsSonVeri[hariciKanal];
        hamOkuma = map(hamADC, 0, 18800, 0, 1023);
        hamOkuma = guvenliSinir(hamOkuma, 0, 1023);

        if (i == 16) {
          if (hamOkuma == eskiHariciDeger) {
            kilitlenmeBekcisi++;
            if (kilitlenmeBekcisi > 30) { 
              ADS1115_Resetle(); 
              kilitlenmeBekcisi = 0;
              break; 
            }
          } else {
            kilitlenmeBekcisi = 0;
          }
          eskiHariciDeger = hamOkuma;
        }
      }
      
      anlikPozisyon = filtreleVeEsikle(i, hamOkuma);
      
      mb.Hreg(i + 20, anlikPozisyon); 
      int hedefeGidilecekYer = mb.Hreg(i); 

      int aktifTolerans = (durmaSayaci[i] >= 5) ? 16 : tolerans; 

      if (abs(anlikPozisyon - hedefeGidilecekYer) <= aktifTolerans) {
        digitalWrite(motorSagPin[i], LOW);
        digitalWrite(motorSolPin[i], LOW);
        
        durmaSayaci[i]++; 
        if (durmaSayaci[i] >= 5) { 
          durmaSayaci[i] = 5; 
        }
      } else {
        durmaSayaci[i] = 0; 
        
        if (anlikPozisyon < hedefeGidilecekYer) {
          digitalWrite(motorSolPin[i], LOW); 
          delayMicroseconds(50); 
          digitalWrite(motorSagPin[i], HIGH);
        } else {
          digitalWrite(motorSagPin[i], LOW); 
          delayMicroseconds(50);
          digitalWrite(motorSolPin[i], HIGH);
        }
      }
    }
  }

  static unsigned long lastEEPROMUpdate = 0;
  if (millis() - lastEEPROMUpdate >= 1000) { 
    lastEEPROMUpdate = millis();
    for (int i = 0; i < aktifMotorSayisi; i++) {
      int anlikModbusHedefi = mb.Hreg(i); 
      int eepromAdres = eepromBaslangicAdresi + (i * 2);
      int hafizadakiDeger; 
      EEPROM.get(eepromAdres, hafizadakiDeger); 
      
      if (anlikModbusHedefi != hafizadakiDeger && anlikModbusHedefi >= 0 && anlikModbusHedefi <= 1023) { 
        EEPROM.put(eepromAdres, anlikModbusHedefi); 
      }
    }
  }
}
