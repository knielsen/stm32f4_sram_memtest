/*
 * This program turns on the 4 leds of the stm32f4 discovery board
 * one after another.
 * It defines shortcut definitions for the led pins and
 * stores the order of the leds in an array which is being
 * iterated in a loop.
 *
 * This program is free human culture like poetry, mathematics
 * and science. You may use it as such.
 */

#include <math.h>

#include <stm32f4_discovery.h>


#define MCU_HZ 168000000
#define SRAM_SIZE ((uint32_t)(2*1024*1024))
#define SRAM ((uint16_t *)(uint32_t)0x60000000)

/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;


static void delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}


static void setup_serial(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* enable peripheral clock for USART2 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration:  USART2 TX on PA2 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect USART2 pins to AF2 */
  // TX = PA2
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  USART_Cmd(USART2, ENABLE); // enable USART2
}


static void
setup_led(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}


static void
setup_dummy_gpio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


__attribute__ ((unused))
static void
led_on(void)
{
  GPIO_SetBits(GPIOG, GPIO_Pin_15);
}


__attribute__ ((unused))
static void
led_off(void)
{
  GPIO_ResetBits(GPIOG, GPIO_Pin_15);
}


static void
serial_putchar(USART_TypeDef* USARTx, uint32_t c)
{
  while(!(USARTx->SR & USART_FLAG_TC));
  USART_SendData(USARTx, c);
}


static void
serial_puts(USART_TypeDef *usart, const char *s)
{
  while (*s)
    serial_putchar(usart, (uint8_t)*s++);
}


static void
serial_output_hexdig(USART_TypeDef* USARTx, uint32_t dig)
{
  serial_putchar(USARTx, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


__attribute__ ((unused))
static void
serial_output_hexbyte(USART_TypeDef* USARTx, uint8_t byte)
{
  serial_output_hexdig(USARTx, byte >> 4);
  serial_output_hexdig(USARTx, byte & 0xf);
}


__attribute__ ((unused))
static void
println_uint32(USART_TypeDef* usart, uint32_t val)
{
  char buf[13];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(usart, buf);
}


__attribute__ ((unused))
static void
println_int32(USART_TypeDef* usart, int32_t val)
{
  if (val < 0)
  {
    serial_putchar(usart, '-');
    println_uint32(usart, (uint32_t)0 - (uint32_t)val);
  }
  else
    println_uint32(usart, val);
}


static void
float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after)
{
  float a;
  uint32_t d;
  uint8_t leading_zero;

  if (f == 0.0f)
  {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  if (f < 0)
  {
    *buf++ = '-';
    f = -f;
  }
  a =  powf(10.0f, (float)dig_before);
  if (f >= a)
  {
    buf[0] = '#';
    buf[1] = '\0';
    return;
  }
  leading_zero = 1;
  while (dig_before)
  {
    a /= 10.0f;
    d = (uint32_t)(f / a);
    if (leading_zero && d == 0 && a >= 10.0f)
      *buf++ = ' ';
    else
    {
      leading_zero = 0;
      *buf++ = '0' + d;
      f -= d*a;
    }
    --dig_before;
  }
  if (!dig_after)
  {
    *buf++ = '\0';
    return;
  }
  *buf++ = '.';
  do
  {
    f *= 10.0f;
    d = (uint32_t)f;
    *buf++ = '0' + d;
    f -= (float)d;
    --dig_after;
  } while (dig_after);
  *buf++ = '\0';
}


__attribute__ ((unused))
static void
println_float(USART_TypeDef* usart, float f,
              uint32_t dig_before, uint32_t dig_after)
{
  char buf[21];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(usart, buf);
}


static void
setup_systick(void)
{
  SysTick->LOAD = 0xffffff;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


static inline uint32_t
get_time(void)
{
  return SysTick->VAL;
}
static inline uint32_t
calc_time_from_val(uint32_t start, uint32_t stop)
{
  return (start - stop) & 0xffffff;
}


static inline uint32_t
calc_time(uint32_t start)
{
  uint32_t stop = get_time();
  return calc_time_from_val(start, stop);
}


static float
calc_time_usec(uint32_t start, uint32_t stop)
{
  uint32_t dur = calc_time_from_val(start, stop);
  return (float)dur * (1000000.0f/(float)MCU_HZ);
}


static void
check_timing(void)
{
  uint32_t t_start, t_end;
  uint32_t cnt;

  /* Do a million iterations in a controlled loop to check timing precision. */
  cnt = 1000000;
  t_start = get_time();
  __asm __volatile(
        "1:\n\t"
        "subs %[cnt], #1\n\t"
        "bne 1b\n"
        : [cnt] "+r" (cnt)
  );
  t_end = get_time();
  serial_puts(USART2, "Timing check usec: ");
  println_float(USART2, calc_time_usec(t_start, t_end), 6, 1);
}


static void
check_read_speed(void)
{
  uint32_t i;
  uint32_t t_start, t_end, duration;
  float cycles, usec;
  volatile uint16_t *p = SRAM;
  static const uint32_t iterations = 1000;

  serial_puts(USART2, "Read test:\r\n");
  t_start = get_time();
  i = iterations;
  do
  {
    (void)(*p);
    (void)(*p);
    (void)(*p);
    (void)(*p);
    (void)(*p);
    (void)(*p);
    -- i;
  } while (i != 0);
  t_end = get_time();

  duration = calc_time_from_val(t_start, t_end);
  cycles = (float)duration*(1.0f/(float)(iterations*1));
  usec = (float)duration*(1000000.0f/(float)MCU_HZ/(float)(iterations*1));
  serial_puts(USART2, "Burst read cycles: ");
  println_float(USART2, cycles, 2, 4);
  serial_puts(USART2, "Burst read nsec: ");
  println_float(USART2, usec*1000.0f, 3, 3);
}


static void
check_write_speed(void)
{
  uint32_t i;
  uint32_t t_start, t_end, duration;
  float cycles, usec;
  volatile uint16_t *p = SRAM;
  static const uint32_t iterations = 1000;

  serial_puts(USART2, "Write test:\r\n");
  t_start = get_time();
  i = iterations;
  do
  {
    (void)(*p);
    (void)(*p);
    (void)(*p);
    -- i;
  } while (i != 0);
  t_end = get_time();

  duration = calc_time_from_val(t_start, t_end);
  cycles = (float)duration*(1.0f/(float)(iterations*1));
  usec = (float)(cycles-4)*(1000000000.0f/(float)MCU_HZ/(float)(iterations*3));
  serial_puts(USART2, "Burst write cycles/loop: ");
  println_float(USART2, cycles, 2, 4);
  serial_puts(USART2, "Burst nsec/write: ");
  println_float(USART2, usec*1000.0f, 3, 3);
}


static void
check_readwrite_speed(void)
{
  uint32_t i;
  uint32_t t_start, t_end, duration;
  float cycles, usec;
  volatile uint16_t *p = SRAM;
  static const uint32_t iterations = 1000;

  serial_puts(USART2, "Read/write test:\r\n");
  t_start = get_time();
  i = iterations;
  do
  {
    (void)*p;
    *p = iterations;
    -- i;
  } while (i != 0);
  t_end = get_time();

  duration = calc_time_from_val(t_start, t_end);
  cycles = (float)duration*(1.0f/(float)(iterations*1));
  usec = (float)(cycles-4)*(1000000000.0f/(float)MCU_HZ/(float)(iterations*3));
  serial_puts(USART2, "Burst readwrite cycles/loop: ");
  println_float(USART2, cycles, 2, 4);
  serial_puts(USART2, "Burst nsec/readwrite: ");
  println_float(USART2, usec*1000.0f, 3, 3);
}


static void
check_gpioreadwrite_speed(void)
{
  uint32_t i;
  uint32_t t_start, t_end, duration;
  float cycles, usec;
  volatile uint16_t *p = SRAM;
  static const uint32_t iterations = 1000;
  volatile uint32_t *gpio_set_ptr = (volatile uint32_t *)&GPIOG->BSRRL;
  uint32_t gpio_set_val = 1<<15;
  volatile uint32_t *gpio_read_ptr = &GPIOA->IDR;

  serial_puts(USART2, "GPIO/read/write test:\r\n");
  t_start = get_time();
  i = iterations;
  do
  {
    (void)*gpio_read_ptr;
    (void)*p;
    *p = iterations;
    *gpio_set_ptr = gpio_set_val;
    -- i;
  } while (i != 0);
  t_end = get_time();

  duration = calc_time_from_val(t_start, t_end);
  cycles = (float)duration*(1.0f/(float)(iterations));
  usec = (float)(cycles)*(1000000000.0f/(float)MCU_HZ/(float)(iterations));
  serial_puts(USART2, "Burst gpioreadwrite cycles/loop: ");
  println_float(USART2, cycles, 2, 4);
  serial_puts(USART2, "Burst nsec/loop: ");
  println_float(USART2, usec*1000.0f, 3, 3);
}


int main(void)
{
  setup_systick();
  delay(2000000);
  setup_serial();
  setup_led();
  setup_dummy_gpio();
  serial_puts(USART2, "Initialising...\r\n");
  delay(2000000);

  serial_puts(USART2, "Hello world, ready to blink!\r\n");
  led_off();
  check_timing();
  check_read_speed();
  check_write_speed();
  check_readwrite_speed();
  check_gpioreadwrite_speed();
  led_off();

  while (1)
  {
  }

  return 0;
}
