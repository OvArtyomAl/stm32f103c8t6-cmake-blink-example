#include <algorithm>
#include <cstdint>

#include "system_stm32f1xx.h"

#define DEFINE_DEFAULT_ISR(name) \
  extern "C" \
  __attribute__((interrupt)) \
  __attribute__((weak)) \
  __attribute__((noreturn)) \
  void name() { \
      while(true); \
  }

/* dummy Exception Handlers */
DEFINE_DEFAULT_ISR(NMI_Handler)
DEFINE_DEFAULT_ISR(HardFault_Handler)
DEFINE_DEFAULT_ISR(MemManage_Handler)
DEFINE_DEFAULT_ISR(BusFault_Handler)
DEFINE_DEFAULT_ISR(UsageFault_Handler)
DEFINE_DEFAULT_ISR(SVC_Handler)
DEFINE_DEFAULT_ISR(DebugMon_Handler)
DEFINE_DEFAULT_ISR(PendSV_Handler)
DEFINE_DEFAULT_ISR(SysTick_Handler)

/* external interrupts handler */
DEFINE_DEFAULT_ISR(WWDG_IRQHandler)
DEFINE_DEFAULT_ISR(PVD_IRQHandler)
DEFINE_DEFAULT_ISR(TAMPER_IRQHandler)
DEFINE_DEFAULT_ISR(RTC_IRQHandler)
DEFINE_DEFAULT_ISR(FLASH_IRQHandler)
DEFINE_DEFAULT_ISR(RCC_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI0_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI1_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI2_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI3_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI4_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel1_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel2_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel3_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel4_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel5_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel6_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel7_IRQHandler)
DEFINE_DEFAULT_ISR(ADC1_2_IRQHandler)
DEFINE_DEFAULT_ISR(USB_HP_CAN1_TX_IRQHandler)
DEFINE_DEFAULT_ISR(USB_LP_CAN1_RX0_IRQHandler)
DEFINE_DEFAULT_ISR(CAN1_RX1_IRQHandler)
DEFINE_DEFAULT_ISR(CAN1_SCE_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI9_5_IRQHandler)
DEFINE_DEFAULT_ISR(TIM1_BRK_IRQHandler)
DEFINE_DEFAULT_ISR(TIM1_UP_IRQHandler)
DEFINE_DEFAULT_ISR(TIM1_TRG_COM_IRQHandler)
DEFINE_DEFAULT_ISR(TIM1_CC_IRQHandler)
DEFINE_DEFAULT_ISR(TIM2_IRQHandler)
DEFINE_DEFAULT_ISR(TIM3_IRQHandler)
DEFINE_DEFAULT_ISR(TIM4_IRQHandler)
DEFINE_DEFAULT_ISR(I2C1_EV_IRQHandler)
DEFINE_DEFAULT_ISR(I2C1_ER_IRQHandler)
DEFINE_DEFAULT_ISR(I2C2_EV_IRQHandler)
DEFINE_DEFAULT_ISR(I2C2_ER_IRQHandler)
DEFINE_DEFAULT_ISR(SPI1_IRQHandler)
DEFINE_DEFAULT_ISR(SPI2_IRQHandler)
DEFINE_DEFAULT_ISR(USART1_IRQHandler)
DEFINE_DEFAULT_ISR(USART2_IRQHandler)
DEFINE_DEFAULT_ISR(USART3_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI15_10_IRQHandler)
DEFINE_DEFAULT_ISR(RTC_Alarm_IRQHandler)
DEFINE_DEFAULT_ISR(USBWakeUp_IRQHandler)

extern std::uint32_t _estack;//__StackTop;
extern "C" void Reset_Handler();

const volatile std::uintptr_t g_pfnVectors[]
__attribute__((section(".isr_vector"))) {
  // Stack Ptr initialization
  reinterpret_cast<std::uintptr_t>(&_estack),
  // Entry point
  reinterpret_cast<std::uintptr_t>(Reset_Handler),
  // Exceptions
  reinterpret_cast<std::uintptr_t>(NMI_Handler),              /* Non maskable interrupt. */
  reinterpret_cast<std::uintptr_t>(HardFault_Handler),        /* HardFault_Handler */
  reinterpret_cast<std::uintptr_t>(MemManage_Handler),        /* Memory management */
  reinterpret_cast<std::uintptr_t>(BusFault_Handler),         /* Pre-fetch fault, memory access fault */
  reinterpret_cast<std::uintptr_t>(UsageFault_Handler),       /* Undefined instruction or illegal state */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(SVC_Handler),              /* System service call via SWI instruction */
  reinterpret_cast<std::uintptr_t>(DebugMon_Handler),         /* Debug Monitor */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(PendSV_Handler),           /* Pendable request for system service */
  reinterpret_cast<std::uintptr_t>(SysTick_Handler),          /* System tick timer */
  // External Interrupts
  reinterpret_cast<std::uintptr_t>(WWDG_IRQHandler),
  reinterpret_cast<std::uintptr_t>(PVD_IRQHandler),
  reinterpret_cast<std::uintptr_t>(TAMPER_IRQHandler),
  reinterpret_cast<std::uintptr_t>(RTC_IRQHandler),
  reinterpret_cast<std::uintptr_t>(FLASH_IRQHandler),
  reinterpret_cast<std::uintptr_t>(RCC_IRQHandler),
  reinterpret_cast<std::uintptr_t>(EXTI0_IRQHandler),
  reinterpret_cast<std::uintptr_t>(EXTI1_IRQHandler),
  reinterpret_cast<std::uintptr_t>(EXTI2_IRQHandler),
  reinterpret_cast<std::uintptr_t>(EXTI3_IRQHandler),
  reinterpret_cast<std::uintptr_t>(EXTI4_IRQHandler),
  reinterpret_cast<std::uintptr_t>(DMA1_Channel1_IRQHandler),
  reinterpret_cast<std::uintptr_t>(DMA1_Channel2_IRQHandler),
  reinterpret_cast<std::uintptr_t>(DMA1_Channel3_IRQHandler),
  reinterpret_cast<std::uintptr_t>(DMA1_Channel4_IRQHandler),
  reinterpret_cast<std::uintptr_t>(DMA1_Channel5_IRQHandler),
  reinterpret_cast<std::uintptr_t>(DMA1_Channel6_IRQHandler),
  reinterpret_cast<std::uintptr_t>(DMA1_Channel7_IRQHandler),
  reinterpret_cast<std::uintptr_t>(ADC1_2_IRQHandler),
  reinterpret_cast<std::uintptr_t>(USB_HP_CAN1_TX_IRQHandler),
  reinterpret_cast<std::uintptr_t>(USB_LP_CAN1_RX0_IRQHandler),
  reinterpret_cast<std::uintptr_t>(CAN1_RX1_IRQHandler),
  reinterpret_cast<std::uintptr_t>(CAN1_SCE_IRQHandler),
  reinterpret_cast<std::uintptr_t>(EXTI9_5_IRQHandler),
  reinterpret_cast<std::uintptr_t>(TIM1_BRK_IRQHandler),
  reinterpret_cast<std::uintptr_t>(TIM1_UP_IRQHandler),
  reinterpret_cast<std::uintptr_t>(TIM1_TRG_COM_IRQHandler),
  reinterpret_cast<std::uintptr_t>(TIM1_CC_IRQHandler),
  reinterpret_cast<std::uintptr_t>(TIM2_IRQHandler),
  reinterpret_cast<std::uintptr_t>(TIM3_IRQHandler),
  reinterpret_cast<std::uintptr_t>(TIM4_IRQHandler),
  reinterpret_cast<std::uintptr_t>(I2C1_EV_IRQHandler),
  reinterpret_cast<std::uintptr_t>(I2C1_ER_IRQHandler),
  reinterpret_cast<std::uintptr_t>(I2C2_EV_IRQHandler),
  reinterpret_cast<std::uintptr_t>(I2C2_ER_IRQHandler),
  reinterpret_cast<std::uintptr_t>(SPI1_IRQHandler),
  reinterpret_cast<std::uintptr_t>(SPI2_IRQHandler),
  reinterpret_cast<std::uintptr_t>(USART1_IRQHandler),
  reinterpret_cast<std::uintptr_t>(USART2_IRQHandler),
  reinterpret_cast<std::uintptr_t>(USART3_IRQHandler),
  reinterpret_cast<std::uintptr_t>(EXTI15_10_IRQHandler),
  reinterpret_cast<std::uintptr_t>(RTC_Alarm_IRQHandler),
  reinterpret_cast<std::uintptr_t>(USBWakeUp_IRQHandler),
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                  /* 0 */
};

// This function for STM32 is located in "./cmsis_device_f<x>/Source/Templates/system_stm32f1xx.c"
extern void SystemInit(void);
// This is the firmware entry point
extern int main();

extern "C"
void Reset_Handler() {
  // Initialize data section
  extern std::uint8_t _sdata;
  extern std::uint8_t _edata;
  extern std::uint8_t _sidata;
  std::size_t size = static_cast<std::size_t>(&_edata - &_sdata);
  std::copy(&_sidata, &_sidata + size, &_sdata);

  // Initialize bss section
  extern std::uint8_t __bss_start__;
  extern std::uint8_t __bss_end__;
  std::fill(&__bss_start__, &__bss_end__, UINT8_C(0x00));

  // Initialize static objects by calling their constructors
  typedef void (*function_t)();
  extern function_t __init_array_start;
  extern function_t __init_array_end;
  std::for_each(&__init_array_start, &__init_array_end, [](const function_t pfn) {
      pfn();
  });

  // Do not forget to initialize the clock circuits!!!! ! ! ! !  !   !    !     !
  SystemInit();

  main();
  while(1) {
  }
}
