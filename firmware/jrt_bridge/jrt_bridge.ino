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
 * PODŁĄCZENIE (tor pod M703A, ustalone pomiarowo 2026-09-14):
 *   D2  <- TXD modułu (P8)         Nano odbiera
 *   D3  -> RXD modułu (P9)         Nano nadaje
 *   D4  -> PWR_ON (P7)             aktywny stanem WYSOKIM
 *   D5  -> nCTRL (P13)             NISKI = pomiar ciągły
 *
 * UWAGA: stary tor pod JRT LDB1/B87A miał RX i TX ODWROTNIE (D3 odbiera,
 * D2 nadaje). Zamiana nie daje żadnego błędu - moduł po prostu milczy na
 * każdym baudzie. Rozstrzygnął to dopiero skan z zamienionymi pinami.
 * Opis pinów poniżej dotyczy starego modułu:
 *
 * PODŁĄCZENIE LDB1/B87A (ustalone pomiarowo 2026-08-29):
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
 *   1B 1B 4E <0|1>   D5 na STAŁE w stanie 0/1, odpowiedź "#D5 <v>" (od 2026-09-14)
 *   1B 1B 54 <0|1>   STEMPLE CZASU linii modułu wył./wł., odpowiedź "#STAMP <v>"
 *   1B 1B 3F         status, odpowiedź "#JRTBRIDGE baud=<v> rx=D3 tx=D2 rst=D5 pwren=D4:<v>"
 *
 * M703A (od 2026-09-14) siedzi na TYCH SAMYCH pinach, ale D5 to u niego nCTRL:
 * stan niski + komenda D/M/F = pomiar CIĄGŁY, stan wysoki = stop. Impuls resetu
 * (1B 1B 52) nic by tu nie dał - tryb ciągły wymaga TRZYMANIA pinu, stąd 'N'.
 *
 * Prefiks 1B 1B (dwa ESC) nie występuje w ramkach JRT - te zaczynają się od 0xAA
 * i mają najwyżej 13 bajtów - więc przelot pozostaje przezroczysty dla danych.
 *
 * STEMPLE CZASU (od 2026-09-14) - PO CO SĄ:
 *
 * M703A w trybie ciągłym sam decyduje, kiedy odda pomiar (co 121 ms, czasem
 * 141, na słabym celu 242). Każdy punkt chmury potrzebuje więc WŁASNEGO
 * czasu, pod który podstawia się kąt z enkoderów. Czas przyjścia linii na
 * hoście jest do tego zły: po drodze jest CH340, stos USB i pętla
 * ros2_control, każde z własnym jitterem rzędu milisekund. Nano widzi bajty
 * prosto z linii modułu, więc stempluje je bez tych opóźnień.
 *
 * W trybie stempli bajty modułu NIE idą przelotem, tylko są zbierane do końca
 * linii (\n) i wysyłane jako jedna linia:
 *
 *   $<t_first>,<t_last>,<linia modułu bez \r\n>\r\n
 *
 * t_first = micros() przy odebraniu PIERWSZEGO bajtu linii (kotwica: stała
 *           odległość od końca pomiaru w module),
 * t_last  = micros() przy odebraniu \n (kontrola: t_last - t_first to czas
 *           nadawania linii, ~0,52 ms na bajt przy 19200 - inna wartość
 *           znaczy, że linia była poszarpana).
 *
 * Oba czasy to uint32 w mikrosekundach - przewijają się co ~71,6 min, host musi
 * je rozwinąć (wie z własnego zegara, która to epoka). Zegar Nano ma własny
 * dryf, więc host przelicza go na swój z dolnej obwiedni (czas_hosta - t_first).
 *
 * Bajt jest "odebrany" dopiero po całej ramce UART (SoftwareSerial czyta ją w
 * przerwaniu), więc stempel spóźnia się o ~9,5 bitu = ~0,5 ms względem zbocza
 * startu. To opóźnienie jest STAŁE i wchodzi w kotwicę mierzoną testem
 * rewersyjnym, nie w jitter.
 *
 * Tryb domyślnie WYŁĄCZONY - po resecie Nano mostek jest znów przezroczysty,
 * co zachowuje zgodność z binarnym protokołem B87A. Host włącza go przy
 * starcie i PO KAŻDYM banerze "#JRTBRIDGE" (baner = Nano się zresetowało).
 *
 * OGRANICZENIE SoftwareSerial: przy 16 MHz działa pewnie do ~57600. Wyżej
 * (115200) gubi bajty i nie należy temu ufać. Dla JRT nominalne 19200 jest
 * dobrze w zakresie.
 */

#include <SoftwareSerial.h>

const uint8_t PIN_RX   = 2;   // <- TXD modułu (M703A P8); w torze LDB1 było 3
const uint8_t PIN_TX   = 3;   // -> RXD modułu (M703A P9); w torze LDB1 było 2
const uint8_t PIN_NRST = 5;   // -> nRST (LDB1, aktywny LOW) / nCTRL (M703A, LOW = ciągły)
const uint8_t PIN_PWREN = 4;  // -> PWREN / PWR_ON modułu (przez konwerter), aktywny HIGH

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

// Tryb stempli czasu - patrz nagłówek. Bufor linii 48 B: najdłuższa linia
// M703A ("V:10112100338,43467") ma ~20 znaków.
uint8_t  stampMode = 0;
char     lineBuf[48];
uint8_t  lineLen = 0;
uint32_t lineFirstUs = 0;

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
  Serial.print(F(" rx=D"));
  Serial.print(PIN_RX);
  Serial.print(F(" tx=D"));
  Serial.print(PIN_TX);
  Serial.print(F(" rst=D5 pwren=D4:"));
  Serial.print(pwrEn ? F("1") : F("0"));
  Serial.print(F(" stamp="));
  Serial.println(stampMode);
}

static void flushLine(uint32_t lastUs)
{
  Serial.print('$');
  Serial.print(lineFirstUs);
  Serial.print(',');
  Serial.print(lastUs);
  Serial.print(',');
  Serial.write((const uint8_t *)lineBuf, lineLen);
  Serial.print(F("\r\n"));
  lineLen = 0;
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
      else if (b == 0x4E) { escState = 7; }   // 'N' - czeka na stan D5
      else if (b == 0x54) { escState = 8; }   // 'T' - czeka na tryb stempli
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
    if (escState == 8)
    {
      escState = 0;
      stampMode = b ? 1 : 0;
      lineLen = 0;             // niedokończona linia sprzed przełączenia jest bez wartości
      Serial.print(F("#STAMP "));
      Serial.println(stampMode);
      continue;
    }
    if (escState == 7)
    {
      escState = 0;
      digitalWrite(PIN_NRST, b ? HIGH : LOW);
      Serial.print(F("#D5 "));
      Serial.println(b ? 1 : 0);
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
    const uint8_t b = mod.read();
    if (!stampMode) { Serial.write(b); continue; }

    // Stempel PRZED jakąkolwiek obróbką - to jest cała wartość tego trybu.
    const uint32_t now = micros();
    if (b == '\r') { continue; }
    if (b == '\n')
    {
      if (lineLen > 0) { flushLine(now); }
      continue;
    }
    if (lineLen == 0) { lineFirstUs = now; }
    lineBuf[lineLen++] = (char)b;
    // Przepełnienie = to nie jest linia ASCII (np. binarna ramka). Oddajemy ją
    // ze stemplem zamiast gubić - host i tak ją odrzuci.
    if (lineLen >= sizeof(lineBuf)) { flushLine(now); }
  }
}
