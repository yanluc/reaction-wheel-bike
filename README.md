# Rower stabilizowany kołem reakcyjnym

## Struktura

```
inzynierka/
├── common/           # C++ współdzielony przez STM32 i RP2040 (bez zależności od HAL)
│   ├── include/hal/Hal.hpp          # interfejsy: ISpi, II2c, IOutputPin, IInputPin, IClock
│   ├── nrf24/                       # sterownik nRF24L01+ (polling, dynamic payload, ACK payload)
│   └── proto/                       # Messages.hpp (pakiety), Frame (ramkowanie USB/UART, CRC-8)
├── stm/              # firmware roweru, NUCLEO-L4A6ZG (CubeMX + CMake, C++17)
│   ├── stm.ioc                      # cała konfiguracja peryferiów -> generacja CubeMX
│   ├── Core/                        # kod generowany; zmiany tylko w USER CODE: app_main(), IRQ IMU
│   └── App/                         # aplikacja (opis niżej)
├── base/             # stacja bazowa: Waveshare RP2040-One + nRF24 <-> USB (Pico SDK, C++17)
├── tools/            # narzędzia laptopa (Python): bikelink.py (protokół), link_console.py
├── gazebo/           # wcześniejszy test STM32 <-> Gazebo (diff_drive)
└── test_stm_gazebo/  # firmware do tego testu
```

## Połączenia

```
                          ┌──────────────────────── NUCLEO-L4A6ZG (rower) ────────────────────────┐
 laptop ══USB══ ST-LINK ══╡ SWD (flash/debug) + VCP = LPUART1 PG7/PG8, 2 Mbaud (logi, SIM, komendy) │
                          │                                                                         │
     nRF24L01+ ───────────┤ SPI1 PA5/PA6/PA7, CE PD15, CSN PD14, IRQ PF12                           │
     ACS37041 (prąd) ─────┤ ADC1 A0 / PA3                                                           │
     ISM330DHCX (IMU) ────┤ SPI3 PC10/PC11/PC12, CS PD2, INT1 PF3 (EXTI)                            │
     2× VL53L1X (ToF) ────┤ I2C1 PB8/PB9, XSHUT PF15/PD8, GPIO1 PF14/PD9                            │
     silnik PWM + DIR ─ ─ ┤ do ustalenia                                                            │
                          └─────────────────────────────────────────────────────────────────────────┘
                                   ((( 2.4 GHz, kanał 20 )))
                          ┌──────────── Waveshare RP2040-One (baza) ────────────┐
     nRF24L01+ ───────────┤ SPI0 GP0/GP2/GP3, CSN GP1, CE GP4, IRQ GP5          │
 laptop ══USB-C═══════════╡ USB CDC (ramki binarne), BOOTSEL przez 1200 baud    │
                          └─────────────────────────────────────────────────────┘
```

### Rower: NUCLEO-L4A6ZG

| Urządzenie | Pin urządzenia | Złącze Nucleo | Pin MCU | Konfiguracja (`stm.ioc`) |
|---|---|---|---|---|
| nRF24L01+ | 1 GND | GND | – | – |
| | 2 VCC | 3V3 (**nie 5 V**) | – | kondensator 10–100 µF przy module |
| | 3 CE | D9 | PD15 | GPIO out `NRF_CE`, start 0 |
| | 4 CSN | D10 | PD14 | GPIO out `NRF_CSN`, start 1 |
| | 5 SCK | D13 | PA5 | SPI1_SCK, 10 MHz, mode 0 |
| | 6 MOSI | D11 | PA7 | SPI1_MOSI |
| | 7 MISO | D12 | PA6 | SPI1_MISO |
| | 8 IRQ | D8 | PF12 | GPIO in `NRF_IRQ`, pull-up |
| ACS37041 (Pololu #5441) | VDD | 3V3 | – | – |
| | GND | GND | – | – |
| | VOUT | A0 | PA3 | ADC1_IN8 (`CUR_A0`), oversampling ×256, DMA |
| | IP+ / IP− | szeregowo w zasilaniu silnika | – | 1.6 mΩ, ±30 A |
| ISM330DHCX (Adafruit 4502) | VIN | 3V3 | – | – |
| | GND | GND | – | – |
| | SCL (SCK) | D45 | PC10 | SPI3_SCK, 2.5 MHz, mode 3 |
| | DO (SDO/MISO) | D46 | PC11 | SPI3_MISO |
| | SDA (SDI/MOSI) | D47 | PC12 | SPI3_MOSI |
| | CS | D48 | PD2 | GPIO out `IMU_CS`, start 1 |
| | I1 (INT1) | D49 | PF3 | EXTI3, zbocze narastające (`IMU_INT1`) |
| VL53L1X lewy | VIN | 3V3 | – | – |
| | GND | GND | – | – |
| | SDA | D14 | PB9 | I2C1_SDA, 400 kHz, DMA |
| | SCL | D15 | PB8 | I2C1_SCL |
| | XSHUT | D2 | PF15 | GPIO open-drain `TOF_L_XSHUT`, start 0 |
| | GPIO1 | D4 | PF14 | GPIO in `TOF_L_INT` |
| VL53L1X prawy | VIN, GND, SDA, SCL | jak lewy (ta sama magistrala) | – | – |
| | XSHUT | D1 | PD8 | GPIO open-drain `TOF_R_XSHUT`, start 0 |
| | GPIO1 | D0 | PD9 | GPIO in `TOF_R_INT` |
| ST-LINK (CN1, USB) | – | USB do laptopa | PG7/PG8 | LPUART1 2 Mbaud (VCP), SWD |

- **Dodatkowo mierzone wejścia ADC:** A1 (PC0) i A2 (PC3) są skonfigurowane w ADC1 i pokazywane w logu diagnostycznym. Służą do sprawdzania okablowania i są wolne do innego użytku.
- **Stan czujnika prądu:** podłączenie VOUT → A0 to plan. W ostatnim pomiarze A0–A2 pokazywały ~0.4–0.47 V, czyli czujnik nie był wpięty, bo bez prądu powinno być ~1.65 V.
- **Numery Dxx** to oznaczenia złączy Zio (CN7–CN10), zweryfikowane z mapą wariantu STM32duino dla NUCLEO-L496ZG/L4A6ZG.
- **IMU na osobnej magistrali (SPI3):** transfery radia na SPI1 nie opóźniają odczytu IMU i odwrotnie. Płytka Adafruit domyślnie startuje w trybie I2C. Pierwszy zapis po SPI ustawia `I2C_disable`, a sam tryb SPI wybiera stan niski na CS.
- **Dwa VL53L1X mają ten sam adres 0x29.** Przy starcie oba są trzymane w resecie (XSHUT=0). Potem po kolei: zwolnienie XSHUT, zmiana adresu na 0x30 (lewy) i 0x31 (prawy). Adres nie jest pamiętany po wyłączeniu, więc sekwencja idzie przy każdym starcie.
- **XSHUT i GPIO1 są bez podciągania w MCU.** XSHUT jest open-drain, bo moduły zwykle podciągają go do zasilania czujnika (2.8 V), a open-drain jest bezpieczny w obu przypadkach. GPIO1 też ma zwykle podciąganie na module. Jaka to płytka (Adafruit 3967 czy Pololu 3415), jeszcze nie sprawdzone, bo strona kamami blokuje pobranie.
- **Zarezerwowane dla silnika:** D6 (PE9, TIM1_CH1) na PWM, D5 (PE11) lub D7 (PF13) na DIR. Na enkoder koła reakcyjnego zostaje wolna para timera, np. TIM3 lub TIM4.
- **Zajęty pin PG6:** steruje włącznikiem zasilania USB OTG (STMPS2151) i jest wyjściem. Nie podłączać do niego żadnego wyjścia, np. IRQ.

### Baza: Waveshare RP2040-One

| nRF24L01+ | RP2040-One | Funkcja |
|---|---|---|
| 1 GND | GND | – |
| 2 VCC | 3V3 (**nie 5 V**) | kondensator 10–100 µF przy module |
| 3 CE | GP4 | GPIO out |
| 4 CSN | GP1 | GPIO out (nie sprzętowy CSn SPI0) |
| 5 SCK | GP2 | SPI0 SCK, ~10 MHz, DMA |
| 6 MOSI | GP3 | SPI0 TX |
| 7 MISO | GP0 | SPI0 RX |
| 8 IRQ | GP5 | GPIO in, pull-up |

USB-C do laptopa: ramki binarne przez CDC. GP16 to wbudowana dioda WS2812, nieużywana.

## Architektura firmware roweru (`stm/App`)

```
                    ┌──────────── App (pętla, harmonogram) ────────────┐
 IPlatform ──────── │ IEstimator -> IController -> Supervisor -> IMotor │
  HwPlatform        └──────┬──────────────────────────────┬────────────┘
  SimPlatform              │ Telemetry / Command          │ logi, Telemetry, Command, Sim*
                     RadioStream (nRF24 PTX)          SerialLink (LPUART1, DMA)
```

- **`interfaces/`**: kontrakty, od których zależy aplikacja.
  - `IImu`, `IDistanceSensor`, `ICurrentSensor`, `IMotor`, `IPwmOutput`;
  - `IPlatform`, czyli komplet urządzeń;
  - `IEstimator`, `IController`.
- **Platforma** (HW albo SIM) jest wybierana w jednym miejscu, `app_main.cpp`, opcją CMake `BIKE_PLATFORM`.
  - `HwPlatform`: prawdziwy sprzęt.
    - IMU: `Ism330dhcx` (opis niżej).
    - Czujniki odległości: 2× `Vl53l1x`.
    - Czujnik prądu: `Acs37041` na ADC1 (DMA, oversampling, przeliczenie przez VREFINT).
    - Silnik: `PwmDirMotor` (PWM + DIR), na razie z atrapą wyjścia PWM.
  - `SimPlatform`: urządzenia zasilane przez symulator na PC po LPUART1. Symulator wysyła `SimSensors`, firmware odpowiada `SimActuators` (lock-step).
- **Sterowanie:**
  - `KalmanRollEstimator` liczy przechylenie (opis niżej), a `ComplementaryEstimator` zostaje jako prostsza alternatywa;
  - `NullController` i `PdController` można przełączać w locie komendą `SetController`, a parametry stroić komendą `SetParam`;
  - nowe prawo sterowania to nowa klasa `IController` dopisana do tablicy w `app_main.cpp`.
- **Bezpieczeństwo:** `Supervisor` pozwala kręcić silnikiem tylko po komendzie `Arm`. Rozbraja przy:
  - braku IMU;
  - trwającej kalibracji estymatora;
  - przechyle > 0.6 rad;
  - braku ACK z radia przez 300 ms.
- **Zasady:**
  - zero sterty;
  - interfejsy z chronionym, niewirtualnym destruktorem (inaczej linker dociąga `operator delete` i malloc);
  - konstruktory obiektów statycznych nie dotykają sprzętu, robi to dopiero `init()`.
- **DMA:**
  - LPUART1 RX: circular, odczyt licznika DMA w pętli;
  - LPUART1 TX: kolejka i porcje DMA, `send()` nie blokuje;
  - SPI1 (radio): RX i TX na DMA;
  - SPI3 (IMU): RX i TX na DMA, start z przerwania INT1;
  - I2C1 (ToF): odczyt wyników i kasowanie przerwania przez HAL DMA, bez blokowania pętli;
  - na RP2040 SPI0 też na DMA.
- **Harmonogram pętli:**
  - estymator: każda próbka IMU (6.67 kHz), cała kolejka w każdym obiegu pętli;
  - korekta z ToF: przy każdej nowej parze pomiarów (50 Hz);
  - sterowanie 500 Hz;
  - prąd 50 Hz;
  - telemetria radiowa 100 Hz;
  - telemetria po kablu 20 Hz;
  - statystyki co 1 s w logu.

## IMU (`Ism330dhcx`)

- **Konfiguracja:** akcelerometr ±4 g, żyroskop ±1000 dps, oba ODR 6.66 kHz, BDU. INT1 daje impuls 75 µs przy każdej próbce żyroskopu. Tryb impulsowy jest konieczny, bo przy poziomie zatrzaśniętym jeden zgubiony odczyt zatrzymałby przerwania od zbocza.
- **Odczyt:** jedno przerwanie na próbkę (EXTI3), bez przerwań DMA. Przerwanie:
  1. kończy poprzedni odczyt: CS w górę, 12 B do kolejki ze znacznikiem czasu (licznik cykli DWT);
  2. startuje nowy odczyt DMA 13 B od rejestru 0x22 (żyroskop, potem akcelerometr).
- **Opóźnienie i kolejka:** próbka trafia do pętli z opóźnieniem jednego okresu (150 µs). Kolejka ma 32 próbki (4.8 ms), więc zatrzymanie pętli, np. formatowanie linii logu (130–250 µs, zmierzone), nie gubi danych. Przeliczenie na jednostki SI i osie robi dopiero `read()` w pętli.
- **Zmierzony koszt CPU:** test bez czujnika z programowym wyzwalaniem EXTI3 co 150 µs (2026-10-18):
  - ~290–350 cykli na próbkę, czyli ~2.5% CPU przy 6.67 kHz;
  - 0 przepełnień kolejki i 0 nakładających się odczytów, także w sekundzie z logami.
- **Wariant tańszy, do sprawdzenia na sprzęcie:** FIFO czujnika z progiem kilku próbek dzieli koszt przez liczbę próbek w paczce, kosztem opóźnienia.
- **Prędkość SPI:** 2.5 MHz (prescaler /32 w `stm.ioc`). Burst 13 B trwa ~42 µs, czyli mieści się w okresie 150 µs. Na STEMMA linie SCK/MOSI mogą mieć podciąganie 10 kΩ i konwersję poziomów, co spowalnia zbocza; biblioteka Adafruit domyślnie używa 1 MHz. Wyższą prędkość (/16, /8 = 10 MHz) sprawdzić na sprzęcie: w spoczynku |a| ≈ 9.81 m/s² i żyroskop ≈ 0.
- **Start:** przerwania działają od `init()`. Pierwszy `read()` w pętli wyrzuca zaległości z czasu inicjalizacji ToF i radia (~0.3 s) i zeruje liczniki.
- **Diagnostyka (log co 1 s):** `IMU ok who samples/s gaps overruns overflows`, gdzie `who` musi być `0x6B`.
- **Osie:** `Ism330dhcx::Config::axis` mapuje osie czujnika na osie ramy (x do przodu, y w lewo, z w górę), także ze znakiem. Do ustawienia po zamocowaniu płytki.

### Uruchomienie z czujnikami (checklista)

Log z `tools/link_console.py --port <ST-LINK> --baud 2000000`:

| Linia | Oczekiwane | Jeśli nie |
|---|---|---|
| `IMU ok=1 who=0x6B` | WHO_AM_I się zgadza | brak `ok`: okablowanie SPI3/CS, zasilanie |
| `samples/s≈6667 overflows=0` | każda próbka dociera | `samples/s≈0` po starcie: tryb impulsowy INT1 (COUNTER_BDR_REG1 0x0B bit 7, z pamięci karty katalogowej, do potwierdzenia) albo przewód INT1 |
| `loops/s ... roll=` | w spoczynku stały, ±0.6 rad blokuje uzbrojenie | znak/osie: `Ism330dhcx::Config::axis` |
| `ToF L ok=1 ... st=9`, sensowne mm | oba czujniki mierzą | `ok=0`: XSHUT/I2C; `bad` rośnie: poza zasięgiem lub zła powierzchnia; `n` rośnie bardzo szybko: odwrotna polaryzacja GPIO1 |

## Czujniki odległości (`Vl53l1x`)

- **Sterownik:** własny, z sekwencją rejestrów z ST ULD. Tablica konfiguracji domyślnej (91 B) jest na licencji BSD-3 ST.
- **Tryb pracy:** short distance mode (do ~1.3 m, mniej czuły na światło otoczenia), budżet czasu 20 ms, pomiar co 20 ms, czyli 50 Hz na czujnik.
- **Odczyt:** stan GPIO1 (aktywny wysoki) oznacza gotowy wynik. Wtedy idzie odczyt 17 B od 0x0089 przez DMA, a potem kasowanie przerwania (0x0086), też przez DMA. Obie operacje są asynchroniczne. Magistrala jest zajęta do odebrania wyniku przez właściciela, więc dwa czujniki się nie mieszają. Bez podłączonego GPIO1 (`nullptr`) sterownik co 2 ms sprawdza rejestr statusu.
- **Poprawny pomiar:** tylko range status 9, czyli „valid”; inne są liczone jako `bad`.
- **Diagnostyka:** `ToF L ok mm st n bad | R ... | i2c err`.

## Estymacja przechylenia (`KalmanRollEstimator`)

- **Konwencja:** przechył jest prawoskrętny wokół osi x (do przodu), więc dodatni oznacza przechył w prawo, przy osiach y w lewo i z w górę.
- **Stan filtru:** [przechył, bias żyroskopu].
- **Predykcja:** żyroskop x przy każdej próbce IMU (6.67 kHz).
- **Korekty:**
  - **Akcelerometr:** `atan2(ay, az)` co 8 próbek, z dużą wariancją, bo moment koła reakcyjnego daje boczne przyspieszenie (h·α) w miejscu IMU. Pomijany, gdy |a| odbiega od g o więcej niż 1 m/s².
  - **ToF:** dwa czujniki patrzą w dół w odległości b od siebie (poprzecznie). Nad płaskim podłożem dL − dR = b·tan(przechył) dokładnie, niezależnie od wysokości. To jest główna bezwzględna referencja i z niej filtr ustala bias żyroskopu.
- **Parametry:** wariancje, b (`tofBaseline`, domyślnie 0.2 m), bramka i parametry kalibracji są w `KalmanRollEstimator::Config`. Do zestrojenia po zbudowaniu roweru.

### Kalibracja przy starcie

Po starcie (i po komendzie `calibrate`) rower musi stać nieruchomo przez 1.5 s. Pion nie jest potrzebny: może stać na nóżce albo oparty o ścianę, pod dowolnym kątem.

- **Bias żyroskopu:** średnia z żyroskopu. W bezruchu prędkość kątowa jest zerowa niezależnie od kąta.
- **Kąt początkowy:** kierunek średniego wektora przyspieszenia, `atan2(Σay, Σaz)`.
- **Przesunięcie ToF:** średni kąt z ToF minus kąt z akcelerometru. Pochłania montaż czujników, różnicę ich wysokości i nachylenie podłogi, więc ToF i akcelerometr nie „ciągną” w różne strony. Wymaga ≥ 20 par pomiarów, inaczej korekta z ToF jest wyłączona do następnej kalibracji.
- **Wykrywanie ruchu:** jeśli w oknie odchylenie standardowe żyroskopu na którejkolwiek osi przekracza 0.02 rad/s albo |a| odbiega od g o więcej niż 0.5 m/s², kalibracja zaczyna się od nowa (licznik `restarts`).
- **Blokada:** do końca kalibracji `estimatorReady = false`. `Supervisor` traktuje to jak failsafe i nie pozwala uzbroić. Telemetria ma flagę `READY`, a log co 1 s linię `estimator: ...` z kątem startowym, biasem i przesunięciem ToF.
- **Kąt startowy jest względem grawitacji,** z dokładnością do krzywizny montażu IMU. Punkt równowagi roweru i tak zależy od położenia środka masy, więc docelowo ma go znajdować sterownik: średni prąd silnika ≠ 0 oznacza złe zero.
- **Ograniczenie:** przesunięcie ToF z różnicy wysokości czujników dodaje się w dziedzinie tan(kąta), nie kąta, więc lekko zależy od kąta. Przy 8 mm różnicy i zmianie z 12° na −3° daje to ~0.1°.
- **Test na PC** (`stm/tests/kalman_test.cpp`) sprawdza:
  - kalibrację przy 5°, −8° i 12°;
  - restart przy ruchu;
  - przesunięcie montażu ToF;
  - wierność kąta przy zaburzonym akcelerometrze.

## Radio

Rower jest nadajnikiem (PTX): nadaje telemetrię i strumień, nigdy nie blokując pętli. Baza jest odbiornikiem (PRX) i w odpowiedzi (ACK payload) odsyła komendy. Piny są w sekcji [Połączenia](#połączenia).

Parametry łącza (wspólne, `nrf24::Config`): kanał 20 (2420 MHz, wybrany skanem), 2 Mbps, CRC 16 bit, adres `BIKE1`, ARD 500 µs, ARC 3, PA Low (−12 dBm).

Stacja bazowa działa na dwóch rdzeniach: core0 obsługuje tylko radio, core1 tylko USB, a łączy je kolejka. Bez tego zapis do USB blokował odbiór i przy dużym ruchu ginęło ~18% pakietów, bo FIFO RX nRF24 mieści tylko 3 pakiety.

Komendy z laptopa jadą w ACK payloadzie, więc docierają z opóźnieniem jednej wymiany. Zmierzony RTT laptop → baza → rower → baza → laptop wynosi ~21 ms.

Pin IRQ modułu (aktywny niski) zgłasza tylko zdarzenia wymagające reakcji: odebrany pakiet lub ACK payload (RX_DR) oraz pakiet niedostarczony mimo ponowień (MAX_RT). Przerwanie od każdego wysłanego pakietu (TX_DS) jest zamaskowane. Rower sprawdza pin raz na tick (odczyt GPIO, bez SPI), a baza czyta pakiety dopiero na sygnał IRQ.

Zasilanie nRF24: daj 10–100 µF między VCC a GND tuż przy module, na obu płytkach. Typowy objaw spadków napięcia: rejestry czytają się poprawnie, a nadawanie kończy się na MAX_RT.

## Przepustowość radia

Warunki pomiaru:
- kanał 20, 2 Mbps, SPI 10 MHz, payload 32 B;
- odległość ok. 1 m;
- mierzone end-to-end: rower → baza → USB → laptop;
- firmware z presetem `Release-Bench` i narzędzie `tools/radio_bench.py` (2026-10-18).

| Tryb | Pakiety/s | Payload | Straty |
|---|---|---|---|
| strumień bez ACK, kolejka modułu pełna | ~6 060 | ~194 kB/s | 0–0.03%, bez ponowień |
| **ACK, kolejka modułu pełna** | **~2 094** | **~67 kB/s** | **0%, 0 wyczerpanych ponowień** |
| ACK, pakiet po pakiecie | ~1 790 | ~57 kB/s | 0% |

- **Bez ACK:** ograniczeniem jest czas pakietu na antenie (165 µs na 32 B).
- **Z ACK:** po każdym pakiecie oba moduły przełączają się nadawanie↔odbiór (2 × 130 µs) i leci ramka ACK. To kosztuje czas w eterze, nie CPU, bo ACK i ponowienia obsługuje sam moduł.
- **Krótsze ACK:** baza dokłada puls (pustą komendę) najwyżej co 5 ms, więc większość ACK jest pusta i krótsza w eterze.

### Dwie pułapki modułu nRF24 (znalezione pomiarem)

1. **Zapis do pełnej kolejki nadawczej uszkadza dane.** Według datasheetu taki zapis jest ignorowany, ale na tych modułach pakiet w kolejce przesuwa się o jeden bajt. Przy stale pełnej kolejce uszkodzonych było ~83% pakietów. `startQueueInPlace()` zawsze sprawdza więc bit TX_FULL przed zapisem (jeden bajt SPI). Kosztuje to ~1% CPU, ale bez tego dane są uszkadzane.
2. **Pełna kolejka odbiorcza po stronie nadawcy blokuje wszystkie ACK z payloadem.** Jeśli jakikolwiek ACK payload zostanie nieodczytany, po 3 takich kolejka jest pełna i każdy kolejny pakiet kończy się wyczerpaniem ponowień (MAX_RT). Flaga RX_DR jest wtedy już skasowana, więc nic nie zgłasza problemu. Dlatego zarówno `pollTransmit()`, jak i obsługa flag w `RadioStream` opróżniają całą kolejkę odbiorczą przy każdym zdarzeniu (`drainRx()`), także przy niepowodzeniu.

## Strumień próbek (`RadioStream`)

Rower jest nadajnikiem nRF24. Tryb wybiera `LinkMode`:

| | `NoAckFec` (domyślny) | `Ack` |
|---|---|---|
| dane | bez ACK + parzystość XOR 8×8 (`stream/XorFec.hpp`) | każdy pakiet z ACK, bez parzystości |
| zgubiony pakiet | laptop odtwarza go z parzystości (serie do 8 pakietów); dłuższe serie zostają jako luki | moduł sam ponawia; po MAX_RT wystarczy skasować flagę, a pakiet idzie jeszcze raz. Nic nie ginie, dopóki łącze w ogóle działa |
| komendy | ACK payload pakietu telemetrii (co 10 ms) | ACK payload z pulsem (co ~5 ms) |
| maks. próbkowanie przy 4 × int16 (3 próbki w pakiecie) | ~6.7 kHz zmierzone (sufit ~16 kHz) | ~6 kHz sufit; **5 kHz zmierzone bez strat**; 6.7 kHz się nie mieści |
| CPU STM32 (cykle DWT, łącznie z pakowaniem) | **4.7% przy 6.7 kHz** | **3.6% przy 5 kHz** |

Jak niski koszt CPU jest osiągnięty:
- zapisy do radia przez DMA bez czekania, prosto ze slotu kolejki (zero-copy);
- krótkie transakcje (odczyt stanu, kasowanie flag) przez bezpośredni dostęp do rejestru SPI;
- flagi ACK sygnalizowane pinem IRQ (TX_DS zamaskowane);
- gorące funkcje w RAM (`HAL_RAM_FUNC`);
- w kanałach DMA przeładowywane tylko adres, licznik i enable.

Test praktyczny: firmware `Release-StreamTest` (bez ACK) lub `Release-StreamTestAck` (z ACK). Częstotliwość próbkowania ustawia `-DBIKE_STREAM_RATE_HZ=…` (wielokrotność 10). Firmware generuje 4 przebiegi: sinus 1 Hz, trójkąt 2 Hz, prostokąt 0.5 Hz i piłę 5 Hz.

```bash
tools/stream_plot.py                       # wykres na żywo; klawisz d: celowe gubienie pakietów (tylko bez ACK)
tools/stream_plot.py --headless 20 --rate 5000
```

Skrypt sprawdza każdą próbkę z tymi samymi wzorami co firmware. Wynik: 0 złych próbek we wszystkich testach.

## Skan kanałów

```bash
tools/channel_scan.py --sweeps 200
```

Stacja bazowa przemiata kanały 0–125 i dla każdego podaje, w ilu przebiegach odebrała sygnał powyżej −64 dBm (rejestr RPD). Na czas skanu łącze z rowerem jest przerwane (rower przechodzi w failsafe i wraca sam). Skan warto powtórzyć w miejscu docelowych testów, bo zajętość pasma zależy od otoczenia.

## Protokół (`common/include/proto`)

Pakiet to struktura packed, little-endian, z pierwszym bajtem `MsgType`:

| Typ | Kierunek | Rozmiar |
|---|---|---|
| `Telemetry` (0x01) | rower → baza → laptop | 29 B |
| `Command` (0x02) | laptop → baza → rower | 13 B |
| `Log` (0x03) | tekst | – |
| `BaseStatus` (0x10) | baza → laptop | – |
| `SimSensors` / `SimActuators` (0x20/0x21) | tylko UART | – |

- **Radio:** jeden pakiet = jeden payload nRF24 (≤ 32 B).
- **USB/UART:** ramka `AA 55 | len | payload | crc8`.
- **`tools/bikelink.py`** odwzorowuje te struktury. Zmiana po jednej stronie wymaga zmiany po drugiej.

## Build i flash

**Rower (STM32)**, z katalogu `stm/`:

```bash
export PATH=/opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin:$PATH
cmake --preset Release && cmake --build --preset Release            # HW
cmake --preset Release-Sim && cmake --build --preset Release-Sim    # SIM
cmake --preset Release-Bench && cmake --build --preset Release-Bench # test przepustowości radia
cmake --preset Release-StreamTestAck -DBIKE_STREAM_RATE_HZ=5000 && cmake --build --preset Release-StreamTestAck
/opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI \
  -c port=SWD -w build/Release/stm.elf -v -rst
```

Zmiana peryferiów odbywa się tylko w `stm.ioc`. Potem regeneracja:

```bash
printf 'config load %s\nproject generate\nexit\n' "$PWD/stm.ioc" > /tmp/gen.txt
~/STM32CubeMX/STM32CubeMX -q /tmp/gen.txt
```

Regeneracja nie rusza `App/`, `CMakeLists.txt`, `CMakePresets.json` ani sekcji USER CODE (sprawdzone).

Test filtru Kalmana na PC (bez sprzętu):

```bash
cd tests && g++ -std=c++17 -O2 -I../App -I../../common/include kalman_test.cpp -o /tmp/kalman_test && /tmp/kalman_test
```

**Baza (RP2040-One)**, z katalogu `base/`:

```bash
export PICO_SDK_PATH=$HOME/.pico-sdk/sdk/2.3.1 PICO_TOOLCHAIN_PATH=$HOME/.pico-sdk/toolchain/14_2_Rel1
export PATH=$HOME/.pico-sdk/toolchain/14_2_Rel1/bin:$HOME/.pico-sdk/ninja/v1.12.1:$PATH
cmake -S . -B build -G Ninja -Dpicotool_DIR=$HOME/.pico-sdk/picotool/2.3.1/picotool -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Flash RP2040: przytrzymaj BOOT, podłącz USB i skopiuj `build/bike_base.uf2` na dysk `RPI-RP2`. Gdy firmware już działa, wystarczy otworzyć port z baudrate 1200 (`python3 -c "import serial, glob; serial.Serial(glob.glob('/dev/serial/by-id/usb-Raspberry_Pi_Pico_*-if00')[0], 1200).close()"`), a płytka sama przejdzie w BOOTSEL. Bez przycisku: `picotool load -f -x build/bike_base.elf`. To wymaga reguł udev dla picotool, których na tej maszynie jeszcze nie ma.

## Konsola laptopa

```bash
cd tools
./link_console.py                                  # przez radio (auto-wykrycie bazy)
./link_console.py --cmd controller --arg 1         # przełącz sterownik
./link_console.py --cmd param --param 0 --arg 2.5  # kp = 2.5
./link_console.py --cmd arm                        # uzbrój (odmowa, dopóki failsafe)
./link_console.py --cmd calibrate                  # rozbrój i skalibruj od nowa (rower nieruchomo)
./link_console.py --port /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_*-if02 --baud 2000000   # po kablu
```

Co sekundę konsola wypisuje:
- liczbę telemetrii na sekundę;
- zgubione pakiety;
- RTT pingu;
- flagi roweru;
- retransmisje radia;
- stan bazy.

Te same komendy działają po kablu.

## Do zrobienia (potrzebne decyzje sprzętowe)

| Urządzenie | Potrzebna decyzja | Plan |
|---|---|---|
| IMU, czujniki odległości | geometria montażu: osie IMU, rozstaw b czujników ToF, znak | wpisać w `Ism330dhcx::Config::axis` i `KalmanRollEstimator::Config` |
| Silnik | timer/kanał PWM, pin DIR, częstotliwość PWM | `IPwmOutput` na TIM |

Każde urządzenie to nowa klasa implementująca istniejący interfejs i podmiana `Null*` w `HwPlatform`. Reszta kodu się nie zmienia.
