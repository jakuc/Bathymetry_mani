/*
 * jrt_bridge.ino - Arduino Nano jako PRZEZROCZYSTY MOSTEK do dalmierza JRT.
 *
 * ROLA PŁYTKI: żadna. Nano nie zna protokołu JRT, nie liczy odległości i nie
 * interpretuje ramek - przepisuje bajty między USB a modułem w obie strony.
 * Powód jest ten sam co przy laser_nano.ino: logika po stronie hosta da się
 * poprawić w sekundę, a każda zmiana w firmware to przegrywanie płytki. Przy
 * nieznanym jeszcze protokole i nieustalonym modelu czujnika to różnica między
 * iteracją liczoną w sekundach a w minutach.
 *
 * PODŁĄCZENIE (ustalone pomiarowo 2026-08-29):
 *   D3  <- TXD modułu (pin 2)      Nano odbiera
 *   D2  -> RXD modułu (pin 3)      Nano nadaje
 *   D5  -> nRST modułu (pin 4)     aktywny stanem NISKIM
 *   D4  -> PWREN modułu (pin 7)    aktywny stanem WYSOKIM   (od 2026-08-30)
 *   VCC (pin 8) - z HW-131, poza Nano.
 *
 * WSZYSTKIE linie cyfrowe idą przez konwerter poziomów 5 V -> 3,3 V. To nie jest
 * kosmetyka: PWREN ma absolutne maksimum 4,0 V (tab. 5-1 instrukcji B87A), więc
 * pin Nano podpięty wprost zabiłby moduł. Konwerter zdejmuje ten problem i
 * dlatego PWREN wolno tu sterować zwykłym digitalWrite().
 *
 * PWREN JEST RÓWNIE ISTOTNY CO nRST: moduł startuje w power-down i czeka, aż
 * master podciągnie PWREN. Pin zostawiony jako wejście (tak było do 2026-08-29,
 * gdy PWREN wisiał na zewnętrznym 3,3 V) daje objaw nie do odróżnienia od
 * zepsutej transmisji - ciszę. Kolejność z instrukcji: PWREN w górę, nRST w
 * górę, ~100 ms na self-boot, dopiero potem auto-baud bajtem 0x55.
 *
 * nRST JEST TU ISTOTNY: aktywny stan niski oznacza, że pin zostawiony jako
 * wejście albo trzymany nisko przez poprzedni szkic TRZYMA MODUŁ W RESECIE.
 * Objaw jest wtedy nie do odróżnienia od zepsutej transmisji - cisza. Dlatego
 * ustawiamy go jawnie w stan wysoki i robimy jeden czysty impuls resetu przy
 * starcie, żeby moduł wystartował ze znanego stanu.
 *
 * PROTOKÓŁ MOSTKA: domyślnie czysty przelot. Trzy sekwencje sterujące pozwalają
 * zmieniać parametry BEZ przegrywania płytki - kluczowe, bo baud modułu nie jest
 * jeszcze potwierdzony i trzeba go przeskanować:
 *
 *   1B 1B 4B <idx>   ustaw baud modułu wg tabeli BAUDS, odpowiedź "#BAUD <v>"
 *   1B 1B 52         impuls nRST (20 ms w dół), odpowiedź "#RST"
 *   1B 1B 50 <0|1>   PWREN: 0 = power-down, 1 = zasilony, odpowiedź "#PWREN <v>"
 *   1B 1B 3F         status, odpowiedź "#JRTBRIDGE baud=<v> rx=D3 tx=D2 rst=D5 pwren=D4:<v>"
 *
 * Prefiks 1B 1B (dwa ESC) nie występuje w ramkach JRT - te zaczynają się od 0xAA
 * i mają najwyżej 13 bajtów - więc przelot pozostaje przezroczysty dla danych.
 *
 * OGRANICZENIE SoftwareSerial: przy 16 MHz działa pewnie do ~57600. Wyżej
 * (115200) gubi bajty i nie należy temu ufać. Dla JRT nominalne 19200 jest
 * dobrze w zakresie.
 */

#include <SoftwareSerial.h>

const uint8_t PIN_RX   = 3;   // <- TXD modułu
const uint8_t PIN_TX   = 2;   // -> RXD modułu
const uint8_t PIN_NRST = 5;   // -> nRST modułu, aktywny LOW
const uint8_t PIN_PWREN = 4;  // -> PWREN modułu (przez konwerter), aktywny HIGH

const uint32_t USB_BAUD = 115200;

// Kolejność od najbardziej prawdopodobnej: 19200 to nominal linii M703A/M88/U81.
const uint32_t BAUDS[] = {19200, 9600, 38400, 57600, 4800, 2400, 14400, 115200};
const uint8_t  N_BAUDS = sizeof(BAUDS) / sizeof(BAUDS[0]);

SoftwareSerial mod(PIN_RX, PIN_TX);
uint32_t modBaud = BAUDS[0];
uint8_t pwrEn = 1;

// Stan rozpoznawania prefiksu 1B 1B <cmd>. Trzymamy go między iteracjami loop(),
// bo bajty potrafią przyjść w osobnych przebiegach.
uint8_t escState = 0;

// Bufor ramki dla trybu 1B 1B 57. 32 B z zapasem - najdłuższa ramka JRT ma 13.
uint8_t  txBuf[32];
uint8_t  txWant = 0;      // ile bajtów jeszcze zbieramy
uint8_t  txHave = 0;

static void pulseReset()
{
  digitalWrite(PIN_NRST, LOW);
  delay(20);
  digitalWrite(PIN_NRST, HIGH);
  delay(300);            // moduł potrzebuje chwili na rozruch po zwolnieniu resetu
}

// Zimny start wg instrukcji: reset trzymany w dół NA CZAS podnoszenia zasilania,
// żeby moduł nie zaczął bootować w połowie narastania napięcia.
static void powerUp()
{
  digitalWrite(PIN_NRST, LOW);
  digitalWrite(PIN_PWREN, HIGH);
  pwrEn = 1;
  delay(50);
  digitalWrite(PIN_NRST, HIGH);
  delay(300);
}

static void powerDown()
{
  digitalWrite(PIN_NRST, LOW);
  digitalWrite(PIN_PWREN, LOW);
  pwrEn = 0;
  delay(50);
}

static void printStatus()
{
  Serial.print(F("#JRTBRIDGE baud="));
  Serial.print(modBaud);
  Serial.print(F(" rx=D3 tx=D2 rst=D5 pwren=D4:"));
  Serial.println(pwrEn ? F("1") : F("0"));
}

void setup()
{
  Serial.begin(USB_BAUD);

  // Zwolnienie resetu MUSI poprzedzić start transmisji - inaczej pierwsze bajty
  // lecą do układu, który jeszcze się nie obudził.
  pinMode(PIN_NRST, OUTPUT);
  digitalWrite(PIN_NRST, LOW);
  pinMode(PIN_PWREN, OUTPUT);
  digitalWrite(PIN_PWREN, LOW);
  delay(50);               // krótki power-down gwarantuje start ze znanego stanu

  mod.begin(modBaud);
  powerUp();

  printStatus();
}

void loop()
{
  // --- host -> moduł, z wyłuskaniem sekwencji sterujących ---
  while (Serial.available())
  {
    const uint8_t b = Serial.read();

    if (escState == 0 && b == 0x1B) { escState = 1; continue; }
    if (escState == 1)
    {
      // Drugi ESC domyka prefiks; cokolwiek innego oznacza, że pierwszy ESC był
      // zwykłą daną - trzeba go oddać do przelotu, żeby nie zniknął po cichu.
      if (b == 0x1B) { escState = 2; continue; }
      mod.write((uint8_t)0x1B);
      escState = 0;
      mod.write(b);
      continue;
    }
    if (escState == 2)
    {
      if (b == 0x4B)      { escState = 3; }   // 'K' - czeka na indeks baudu
      else if (b == 0x57) { escState = 4; }   // 'W' - czeka na dlugosc ramki
      else if (b == 0x50) { escState = 6; }   // 'P' - czeka na stan PWREN
      else
      {
        escState = 0;
        if (b == 0x52) { pulseReset(); Serial.println(F("#RST")); }
        else if (b == 0x3F) { printStatus(); }
      }
      continue;
    }
    if (escState == 4)
    {
      txWant = (b > sizeof(txBuf)) ? sizeof(txBuf) : b;
      txHave = 0;
      escState = (txWant > 0) ? 5 : 0;
      continue;
    }
    if (escState == 5)
    {
      txBuf[txHave++] = b;
      if (txHave >= txWant)
      {
        escState = 0;
        // Cala ramka juz w RAM - dopiero teraz nadajemy, bez przerw miedzy bajtami.
        for (uint8_t i = 0; i < txWant; ++i) { mod.write(txBuf[i]); }
      }
      continue;
    }
    if (escState == 6)
    {
      escState = 0;
      if (b) { powerUp(); } else { powerDown(); }
      Serial.print(F("#PWREN "));
      Serial.println(pwrEn);
      continue;
    }
    if (escState == 3)
    {
      escState = 0;
      if (b < N_BAUDS)
      {
        modBaud = BAUDS[b];
        mod.end();
        mod.begin(modBaud);
        Serial.print(F("#BAUD "));
        Serial.println(modBaud);
      }
      else
      {
        Serial.println(F("#ERR zly indeks baudu"));
      }
      continue;
    }

    mod.write(b);
  }

  // --- moduł -> host ---
  while (mod.available())
  {
    Serial.write(mod.read());
  }
}
