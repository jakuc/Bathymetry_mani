/*
 * laser_nano.ino - Arduino Nano jako przetwornik ADC dla dalmierza Sharp
 * GP2Y0A710K0F (wyjście analogowe -> A0).
 *
 * ROLA PŁYTKI: Nano NIE liczy odległości. Wysyła surowe zliczenia ADC, a
 * przeliczenie na metry robi host (scripts/laser_test.py, docelowo wtyczka
 * ros2_control). Powód jest praktyczny: krzywa czujnika jest nieliniowa i
 * egzemplarzowa, więc kalibracja będzie się zmieniać - a każda zmiana stałych
 * w firmware oznaczałaby przegrywanie płytki wlutowanej w zestaw. Surowy ADC
 * jest niezmienny, kalibracja siedzi w parametrach.
 *
 * PROTOKÓŁ (linie ASCII, 115200 8N1):
 *   #LSR ready ...                      - banner po resecie (Nano resetuje się
 *                                         przy otwarciu portu - DTR)
 *   LSR,<mediana>,<min>,<max>,<n>       - ramka pomiarowa, ~20 Hz
 * mediana = odporna na pojedyncze przekłamania; min/max z tego samego okna
 * dają od ręki obraz szumu, bez zgadywania po stronie hosta.
 *
 * OKNO PRÓBKOWANIA: czujnik ma własny cykl pomiarowy 16,5 ms (±4 ms) i szybsze
 * odpytywanie zwraca tę samą próbkę, tylko przepisaną. Dlatego okno to 25
 * próbek co 2 ms (~50 ms = ~3 cykle czujnika) - dopiero taka mediana filtruje
 * cokolwiek realnego.
 *
 * ZASILANIE: GP2Y0A710K0F ciągnie impulsowo ~200 mA przy średnich ~30 mA.
 * Referencją ADC Nano jest jego własne Vcc, więc zapad na 5 V przekłada się
 * wprost na błąd odczytu. Kondensator 10 uF między Vcc a GND czujnika, blisko
 * czujnika, nie jest opcjonalny.
 */

const uint8_t  PIN_SENSOR   = A0;
const uint16_t SAMPLES      = 25;   // próbek w oknie
const uint16_t SAMPLE_GAP_MS = 2;   // odstęp między próbkami -> okno ~50 ms

uint16_t buf[SAMPLES];

static int cmp_u16(const void * a, const void * b)
{
  const uint16_t va = *(const uint16_t *)a;
  const uint16_t vb = *(const uint16_t *)b;
  return (va > vb) - (va < vb);
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_SENSOR, INPUT);
  // Pierwszy odczyt po zmianie kanału multipleksera ADC bywa zabrudzony
  // poprzednim kanałem - wyrzucamy go, zamiast wliczać do pierwszej ramki.
  analogRead(PIN_SENSOR);
  Serial.println(F("#LSR ready pin=A0 samples=25 gap_ms=2 adc_bits=10 vref=vcc"));
}

void loop()
{
  for (uint16_t i = 0; i < SAMPLES; ++i)
  {
    buf[i] = analogRead(PIN_SENSOR);
    delay(SAMPLE_GAP_MS);
  }

  qsort(buf, SAMPLES, sizeof(uint16_t), cmp_u16);

  const uint16_t median = buf[SAMPLES / 2];
  const uint16_t lo     = buf[0];
  const uint16_t hi     = buf[SAMPLES - 1];

  Serial.print(F("LSR,"));
  Serial.print(median);
  Serial.print(',');
  Serial.print(lo);
  Serial.print(',');
  Serial.print(hi);
  Serial.print(',');
  Serial.println(SAMPLES);
}
