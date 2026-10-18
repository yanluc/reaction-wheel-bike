# STM32 (NUCLEO-L4A6ZG) → Gazebo diff_drive

STM32 wysyła komendy prędkości po UART, a robot `vehicle_blue` w Gazebo nimi jedzie. Odometria wraca z symulacji do STM32 (diody i tryb powrotu do bazy).

```
STM32 LPUART1 (PG7/PG8) ──ST-LINK VCP /dev/ttyACM0 @ 2 Mbaud──> stm32_gz.py ──gz-transport──> gz sim diff_drive_stm32.sdf
     ^                                                            │  gz.msgs.Twist    /model/vehicle_blue/cmd_vel
     └──────────────────── MSG_ODOM (50 Hz) <─────────────────────┘  gz.msgs.Odometry /model/vehicle_blue/odometry
```
Bez ROS-a: bridge rozmawia z Gazebo przez gz-transport (Python bindings z Gazebo Harmonic).

## Firmware (`../test_stm_gazebo`)

Konfiguracja w `.ioc`, kod wygenerowany przez CubeMX. Ręczny kod jest tylko w sekcjach `USER CODE` w `Core/Src/main.c`.

- SYSCLK 80 MHz (PLL z MSI), APB1 /1 → LPUART1 taktowany 80 MHz.
- LPUART1: 2 000 000 baud, 8N1, overrun detection wyłączone.
- RX: DMA2_Channel7 (request 4), tryb circular. Pętla główna czyta licznik DMA, więc nie potrzeba przerwania UART.
- Pętla sterowania 50 Hz.

| Element | Działanie |
|---|---|
| B1 (niebieski przycisk) | zmienia tryb: STOP → FORWARD → CIRCLE → RETURN_HOME |
| LD2 (niebieska) | świeci, gdy robot w symulacji faktycznie jedzie (odometria) |
| LD3 (czerwona) | świeci, gdy brak odometrii > 500 ms (brak łącza z hostem) |

RETURN_HOME to regulator P, który na podstawie odometrii z Gazebo wraca do punktu startu.

Generacja, build i flash:

```bash
# regeneracja po zmianie .ioc (headless)
printf 'config load %s\nproject generate\nexit\n' "$PWD/test_stm_gazebo.ioc" > /tmp/gen.txt
~/STM32CubeMX/STM32CubeMX -q /tmp/gen.txt

export PATH=/opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin:$PATH
cmake --preset Release && cmake --build --preset Release
/opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI \
  -c port=SWD -w build/Release/test_stm_gazebo.elf -v -rst
```

## Host

```bash
./run.sh /dev/ttyACM0 2000000
```

Uruchamia `gz sim` z GUI i `stm32_gz.py`. `source /opt/ros/jazzy/setup.bash` w skrypcie jest potrzebny tylko dlatego, że Gazebo Harmonic i jego bindingi Pythona są zainstalowane w prefiksie ROS. Żaden proces ROS nie startuje.

`diff_drive_stm32.sdf` to kopia przykładu `diff_drive.sdf` z gz-sim. Jedyna zmiana: `odom_publish_frequency` dla `vehicle_blue` z 1 na 50 Hz (przy 1 Hz sprzężenie zwrotne nie działa).

Bridge co 5 s loguje `rx frames/s, lost, crc errors`. To podstawowa metryka jakości łącza przy danym baudrate. Jeśli STM32 milknie na 300 ms, bridge zatrzymuje robota.

## Protokół

Ramka w obu kierunkach: `0xAA 0x55 | type | len | payload | crc8`. CRC-8 (poly 0x07, init 0) liczone po `type..payload`, payload little-endian.

| type | kierunek | payload |
|---|---|---|
| 0x01 CMD | MCU → host | int16 vx [mm/s], int16 wz [mrad/s], uint8 mode, uint8 seq |
| 0x02 ODOM | host → MCU | int32 x [mm], int32 y [mm], int16 yaw [mrad], int16 v [mm/s] |

## Baudrate

2 Mbaud to teoretyczny sufit dla VCP w ST-LINK/V2-1 na Nucleo-144 (USART w F103: 36 MHz / 16 = 2.25 Mbit/s, a 2 Mbit/s daje dzielnik bez błędu). LPUART1 przy 80 MHz pozwala na więcej (do ~26 Mbit/s), więc to ST-LINK jest wąskim gardłem. Uwaga: **nie zweryfikowane na sprzęcie**. Jeśli `crc errors` / `lost` rosną, zmień `LPUART1.BaudRate` w `.ioc` (np. na 1000000 lub 921600), zregeneruj kod i uruchom `run.sh` z tym samym baudem.

Po podłączeniu płytki sprawdź wersję ST-LINK: `lsusb | grep 0483`.
- `374b` oznacza ST-LINK/V2-1 i wtedy 2 Mbaud to sufit.
- `374e`, `374f` lub `3753` oznacza STLINK-V3, który pozwala na więcej.

Ruch to ok. 550 B/s w każdą stronę, więc liczniki `crc errors`/`lost` sprawdzają poprawność timingu bitów, a nie przepustowość.

## Diagnostyka po pierwszym flashu

Zaraz po resecie, bez uruchomionego hosta, LD3 (czerwona) musi świecić, bo nie ma jeszcze odometrii. Jeśli obie diody są ciemne, inicjalizacja wpadła w `Error_Handler`. Najbardziej podejrzany jest nieużywany `MX_USB_OTG_FS_PCD_Init`: wyłącz USB_OTG_FS w `.ioc` i zregeneruj kod.
