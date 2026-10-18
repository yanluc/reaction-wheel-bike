# Rower stabilizowany kołem reakcyjnym

## Struktura

```
inzynierka/
├── common/           # C++ współdzielony przez STM32 i RP2040 (bez zależności od HAL)
│   ├── include/hal/Hal.hpp          # interfejsy: ISpi, IOutputPin, IClock
│   ├── nrf24/                       # sterownik nRF24L01+ (polling, dynamic payload, ACK payload)
│   └── proto/                       # Messages.hpp (pakiety), Frame (ramkowanie USB/UART, CRC-8)
├── stm/              # firmware roweru, NUCLEO-L4A6ZG (CubeMX + CMake, C++17)
│   ├── stm.ioc                      # cała konfiguracja peryferiów -> generacja CubeMX
│   ├── Core/                        # kod generowany; jedyna zmiana: app_main() w USER CODE 2
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
     IMU (SPI)  ─ ─ ─ ─ ─ ┤ do ustalenia                                                            │
     2× odległość (I2C) ─ ┤ do ustalenia                                                            │
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
| ST-LINK (CN1, USB) | – | USB do laptopa | PG7/PG8 | LPUART1 2 Mbaud (VCP), SWD |

- **Dodatkowo mierzone wejścia ADC:** A1 (PC0) i A2 (PC3) są skonfigurowane w ADC1 i pokazywane w logu diagnostycznym. Służą do sprawdzania okablowania i są wolne do innego użytku.
- **Stan czujnika prądu:** podłączenie VOUT → A0 to plan. W ostatnim pomiarze A0–A2 pokazywały ~0.4–0.47 V, czyli czujnik nie był wpięty, bo bez prądu powinno być ~1.65 V.
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
  - `HwPlatform`: prawdziwy sprzęt. Czujnik prądu to `Acs37041` na ADC1 (DMA, oversampling, przeliczenie przez VREFINT). IMU i czujniki odległości to na razie `Null*`, zgłaszające brak danych. Silnik to `PwmDirMotor` (PWM + DIR) z atrapą wyjścia PWM.
  - `SimPlatform`: urządzenia zasilane przez symulator na PC po LPUART1. Symulator wysyła `SimSensors`, firmware odpowiada `SimActuators` (lock-step).
- **Sterowanie:**
  - `ComplementaryEstimator` liczy przechylenie;
  - `NullController` i `PdController` można przełączać w locie komendą `SetController`, a parametry stroić komendą `SetParam`;
  - nowe prawo sterowania to nowa klasa `IController` dopisana do tablicy w `app_main.cpp`.
- **Bezpieczeństwo:** `Supervisor` pozwala kręcić silnikiem tylko po komendzie `Arm`. Rozbraja przy:
  - braku IMU;
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
  - na RP2040 SPI0 też na DMA.
- **Harmonogram pętli:**
  - sterowanie 500 Hz;
  - czujniki pomocnicze 50 Hz;
  - telemetria radiowa 100 Hz;
  - telemetria po kablu 20 Hz;
  - statystyki co 1 s w logu.

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
| IMU | model i piny | SPI2 lub SPI3 z DMA, osobna magistrala od radia |
| Czujniki odległości | model i adresy I2C (2 sztuki o tym samym adresie wymagają XSHUT albo dwóch magistral) | I2C z DMA |
| Silnik | timer/kanał PWM, pin DIR, częstotliwość PWM | `IPwmOutput` na TIM |

Każde urządzenie to nowa klasa implementująca istniejący interfejs i podmiana `Null*` w `HwPlatform`. Reszta kodu się nie zmienia.
