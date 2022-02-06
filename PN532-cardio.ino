/* === USER CONFIGURABLE OPTIONS === */

/* General usage on boards without USB MCU requires WITH_SPICEAPI instead */
#define WITH_USBHID 1

/* Keypad on boards without USB MCU requires WITH_SPICEAPI */
#define WITH_KEYPAD 1

/* Launch game with "-api 1337 -apipass changeme -apiserial COM1 -apiserialbaud 57600" or similar */
#define WITH_SPICEAPI 0
/* Adjust your serial port here(Serial, Serial1, Serial2, etc.) - WiFi/Network support is possible, but out of scope for this project */
#define SPICEAPI_INTERFACE Serial
#define SPICEAPI_BAUD 57600
#define SPICEAPI_PASS "changeme"
/* For games with multiple readers */
#define SPICEAPI_PLAYERNUM 0

/* === END OF USER CONFIGURABLE OPTIONS === */

/* DO NOT MESS WITH THE LINES BELOW UNLESS YOU KNOW WHAT YOU'RE DOING */
#include <Wire.h>
#include "src/PN532/PN532_I2C.h"
#include "src/PN532/PN532.h"

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);

#if WITH_USBHID == 1
  #if !defined(USBCON)
    #if WITH_SPICEAPI == 0
      #error WITH_SPICEAPI option is mandatory for non-USB MCU 
    #endif
    #warning The USBHID mode can only be used with a USB MCU (e.g. Arduino Leonardo, Arduino Micro, etc.).
    #define USBHID 0
  #else
    #define USBHID 1
    #include "src/Cardio.h"
  #endif
#endif

#if WITH_KEYPAD == 1
  #if !defined(USBCON)
    #if WITH_SPICEAPI == 0
      #error WITH_SPICEAPI option is mandatory to use keypads on non-USB MCU 
    #else
      #define USBKEYPAD 0
      #define SPICEKEYPAD 1
    #endif
  #else
    #define USBKEYPAD 1
    #define SPICEKEYPAD 0
    #include <Keyboard.h>
    #include <Adafruit_MPR121.h>
    Adafruit_MPR121 cap = Adafruit_MPR121();
  #endif
 // #include <Keypad.h>
#endif

#if WITH_SPICEAPI == 1
  /* Wrapper Buffer Sizes - Should be tuned up or down depending on available memory. These are tuned for an Arduino UNO. */
  /* If your code hangs when sending a request, try adjusting it down or up */
  #define SPICEAPI_WRAPPER_BUFFER_SIZE 128
  #define SPICEAPI_WRAPPER_BUFFER_SIZE_STR 128

  #include "src/spiceapi/connection.h"
  #include "src/spiceapi/wrappers.h"

  char uidBuf[18];
  spiceapi::Connection spiceCon(256, SPICEAPI_PASS);
#endif

#if USBHID == 1
  Cardio_ Cardio;
#endif

#if WITH_KEYPAD == 1
  /* Keypad declarations */
  unsigned int buttonPin[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  unsigned long keyTimer[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  bool buttonState[12];
  bool switchType[12] = {true, true, true, true, true, true, true, true, true, true, true, true};
  char asciiKey[12] = {0x39, 0x36, 0x33, 0x63, 0x38, 0x35, 0x32, 0x64, 0x37, 0x34, 0x31, 0x30};  // 7894561230cd  en clavier US
  #define DEBOUNCE 10
#endif
 
void setup() {

#if USBKEYPAD == 1
  /* Keypad */
 //   kpd.setDebounceTime(10);
    Keyboard.begin();
    Serial.println("Adafruit MPR121 Capacitive Touch sensor test"); 
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");
#endif

/* NFC */
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata)
  {
Serial.begin(115200);
Serial.print("Didn't find PN53x board");
    while (1) {delay(10);};      // halt
  }

  // Got ok data, print it out!
/*  Serial.print("Found chip PN5"); Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);
*/
  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);
  nfc.SAMConfig();

//  memset(_prevIDm, 0, 8);

#if USBHID == 1
  Cardio.begin(false);
#endif

#if WITH_SPICEAPI == 1
  SPICEAPI_INTERFACE.begin(SPICEAPI_BAUD);
  while (!SPICEAPI_INTERFACE);
#endif
}

unsigned long lastReport = 0;
uint16_t cardBusy = 0;

// read cards loop
void loop() {
#if WITH_KEYPAD == 1
  /* KEYPAD */
  keypadCheck();
#endif
  
  /* NFC */
  if (millis()-lastReport < cardBusy) return;
  
  cardBusy = 0;
  uint8_t uid[8] = {0,0,0,0,0,0,0,0};
  uint8_t hid_data[8] = {0,0,0,0,0,0,0,0};
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // check for FeliCa card
  uint8_t ret;
  uint16_t systemCode = 0xFFFF;
  uint8_t requestCode = 0x01;       // System Code request
  uint8_t idm[8];
  uint8_t pmm[8];
  uint16_t systemCodeResponse;

  // Wait for an FeliCa type cards.
  // When one is found, some basic information such as IDm, PMm, and System Code are retrieved.
  ret = nfc.felica_Polling(systemCode, requestCode, idm, pmm, &systemCodeResponse, 40); // à changer pour détecter +/- vite ce type de carte (ori = 500)

#if WITH_KEYPAD == 1
  keypadCheck();
#endif
    if (ret == 1) {
#if USBHID == 1
      Cardio.setUID(2, idm);
      Cardio.sendState();
#endif

#if WITH_SPICEAPI == 1
      formatUid(idm, uidBuf);
      spiceapi::card_insert(spiceCon, SPICEAPI_PLAYERNUM, uidBuf);
#endif
      
      lastReport = millis();
      cardBusy = 3000;
      uidLength = 0;
      return;
    }

   // check for ISO14443 card

#if WITH_KEYPAD == 1
  keypadCheck();
#endif

    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength)) {
            Serial.println("Found a card!");
    Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("UID Value: ");  
    for (int i=0; i<8; i++) {
      hid_data[i] = uid[i%uidLength];
                Serial.print(" 0x");Serial.print(uid[i], HEX); 

    }
                    Serial.println("");

#if USBHID == 1
    Cardio.setUID(1, hid_data);
    Cardio.sendState();
#endif

#if WITH_SPICEAPI == 1
    formatUid(hid_data, uidBuf);
    spiceapi::card_insert(spiceCon, SPICEAPI_PLAYERNUM, uidBuf);
#endif

    lastReport = millis();
    cardBusy = 5000;
    return;
  }
  // no card detected
  lastReport = millis();
  cardBusy = 200;
}

#if WITH_KEYPAD == 1
void keypadCheck(){
  static uint16_t prev;
  uint16_t curr = cap.touched();
  
  if (curr == prev) return;
  
  for (int i = 0; i < 12; i++) {
    if ((curr >> i)&1)
    {
        Keyboard.press(asciiKey[i]);
        keyTimer[i] = millis();
    //    Serial.println(keyTimer[i]);         Serial.println(DEBOUNCE);
    }
    else if(millis() - keyTimer[i] > DEBOUNCE)
    {
        Keyboard.release(asciiKey[i]);
    }
  }
  prev = curr;
}
#endif

#if WITH_SPICEAPI == 1
void formatUid(uint8_t* ary, char* buf) {
  sprintf(buf, "%02X%02X%02X%02X%02X%02X%02X%02X", ary[0], ary[1], ary[2], ary[3], ary[4], ary[5], ary[6], ary[7]);
}
#endif
