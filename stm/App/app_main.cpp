// Platform is chosen at build time: CMake BIKE_PLATFORM (HW / SIM) -> BIKE_PLATFORM_SIM.

#include "app_main.h"

#include "App.hpp"
#include "bench/RadioBench.hpp"
#include "bench/StreamTest.hpp"
#include "control/Controllers.hpp"
#include "control/Estimators.hpp"
#include "main.h"
#include "nrf24/Nrf24.hpp"
#include "platform/hw/HwPlatform.hpp"
#include "platform/sim/SimPlatform.hpp"

extern "C" {
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef hlpuart1;
extern ADC_HandleTypeDef hadc1;
}

namespace {
using namespace bike;

// Constructors only store references; hardware is touched in App::init(), after HAL init.
stm32::DwtClock clock;
stm32::DmaSpi radioSpi(&hspi1);
stm32::Pin radioCe(NRF_CE_GPIO_Port, NRF_CE_Pin);
stm32::Pin radioCsn(NRF_CSN_GPIO_Port, NRF_CSN_Pin);
stm32::InputPin radioIrq(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin);
stm32::Pin ledBlue(LD2_GPIO_Port, LD2_Pin);
stm32::Pin ledRed(LD3_GPIO_Port, LD3_Pin);
stm32::SerialLink serial(&hlpuart1);

#if BIKE_PLATFORM_SIM
SimPlatform platform(serial);
#else
stm32::AdcDma adc(&hadc1);
stm32::DmaSpi imuSpi(&hspi3);
stm32::Pin imuCs(IMU_CS_GPIO_Port, IMU_CS_Pin);
stm32::DmaI2c i2c(&hi2c1);
stm32::Pin tofLeftXshut(TOF_L_XSHUT_GPIO_Port, TOF_L_XSHUT_Pin);
stm32::InputPin tofLeftGpio1(TOF_L_INT_GPIO_Port, TOF_L_INT_Pin);
stm32::Pin tofRightXshut(TOF_R_XSHUT_GPIO_Port, TOF_R_XSHUT_Pin);
stm32::InputPin tofRightGpio1(TOF_R_INT_GPIO_Port, TOF_R_INT_Pin);
HwPlatform platform(HwPlatform::Deps{clock, adc, imuSpi, imuCs, i2c, tofLeftXshut, &tofLeftGpio1,
                                     tofRightXshut, &tofRightGpio1},
                    Ism330dhcx::Config{});
#endif

nrf24::Nrf24 radio(radioSpi, radioCe, radioCsn, clock, &radioIrq);

KalmanRollEstimator estimator(KalmanRollEstimator::Config{});
NullController nullController;
PdController pdController(2.0f, 0.2f);
IController* const controllers[] = {&nullController, &pdController};

Supervisor supervisor(Supervisor::Limits{});

}  // namespace

void app_imu_data_ready_irq(uint32_t stamp) {
#if BIKE_PLATFORM_SIM
    (void)stamp;
#else
    platform.imuDriver().onDataReady(stamp);
#endif
}

void app_main(void) {
    clock.init();

#if BIKE_RADIO_BENCH
    static RadioBench bench(radio, clock, serial);
    bench.run(nrf24::Config{});
#elif BIKE_STREAM_TEST
#if BIKE_STREAM_ACK
    static StreamTest streamTest(radio, clock, serial, ledBlue, LinkMode::Ack);
#else
    static StreamTest streamTest(radio, clock, serial, ledBlue, LinkMode::NoAckFec);
#endif
    streamTest.run(nrf24::Config{});
#endif

    // Forwarding sink breaks the RadioStream <-> App construction cycle.
    static class Sink final : public ICommandSink {
    public:
        void onCommand(const proto::Command& cmd) override {
            if (target != nullptr) target->onCommand(cmd);
        }
        App* target = nullptr;
    } sink;

    static RadioStream radioLink(radio, clock, sink);
    static App application(App::Deps{
        clock,
        serial,
        platform,
        radioLink,
        estimator,
        controllers,
        static_cast<uint8_t>(sizeof(controllers) / sizeof(controllers[0])),
        supervisor,
        ledBlue,
        ledRed,
    });
    sink.target = &application;

    application.init(nrf24::Config{});
    application.run();
}
