```cpp
/***********************************************************************
 * Project  : IoT Data Logger with Threshold Violation Detection
 * MCU      : LPC2148 (ARM7TDMI-S) @ 60 MHz
 * Compiler : Keil uVision 4/5 (ARMCC)
 * Sensor   : DHT11 (Temperature & Humidity)
 * Display  : 16x2 LCD (HD44780, 8-bit mode)
 * Memory   : AT24C256 EEPROM (I2C)
 * RTC      : DS1307 (I2C)
 * WiFi     : ESP01 (ESP8266) via UART0
 * Cloud    : ThingSpeak (HTTP GET)
 * Alert    : Buzzer + LED on threshold violation
 * Input    : 4x4 Matrix Keypad
 * Author   : Seelam Santhosh
 * Date     : 2025
 ***********************************************************************/

#include <lpc214x.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ===================================================================
 *  USER CONFIGURATION  --  Edit these before flashing
 * =================================================================== */
#define WIFI_SSID         "YourWiFiSSID"
#define WIFI_PASS         "YourWiFiPassword"
#define THINGSPEAK_KEY    "YOUR_THINGSPEAK_WRITE_API_KEY"

/* ===================================================================
 *  CLOCK CONFIGURATION
 * =================================================================== */
#define FOSC        12000000UL          /* External crystal: 12 MHz   */
#define PLL_MUL     5                   /* PLL multiplier             */
#define CCLK        (FOSC * PLL_MUL)   /* CPU clock: 60 MHz          */
#define PCLK        (CCLK / 4)         /* Peripheral clock: 15 MHz   */

/* ===================================================================
 *  DHT11  --  Single-wire on P0.10
 * =================================================================== */
#define DHT11_PIN       (1UL << 10)
#define DHT11_DIR_OUT() (IODIR0 |=  DHT11_PIN)
#define DHT11_DIR_IN()  (IODIR0 &= ~DHT11_PIN)
#define DHT11_HIGH()    (IOSET0  =  DHT11_PIN)
#define DHT11_LOW()     (IOCLR0  =  DHT11_PIN)
#define DHT11_READ      ((IOPIN0 &  DHT11_PIN) ? 1 : 0)

/* ===================================================================
 *  LCD  --  8-bit mode
 *  Data lines : P1.16 (D0) to P1.23 (D7)
 *  Control    : P1.24=RS   P1.25=RW   P1.26=EN
 * =================================================================== */
#define LCD_DATA_MASK   0x00FF0000UL
#define LCD_RS          (1UL << 24)
#define LCD_RW          (1UL << 25)
#define LCD_EN          (1UL << 26)
#define LCD_ALL_CTRL    (LCD_RS | LCD_RW | LCD_EN)

/* LCD Commands */
#define LCD_CMD_CLEAR       0x01
#define LCD_CMD_HOME        0x02
#define LCD_CMD_FUNC_8BIT   0x38    /* 8-bit, 2-line, 5x8 font   */
#define LCD_CMD_DISP_ON     0x0C    /* Display ON, cursor OFF     */
#define LCD_CMD_ENTRY_MODE  0x06    /* Increment, no shift        */
#define LCD_CMD_LINE1       0x80    /* DDRAM address line 1       */
#define LCD_CMD_LINE2       0xC0    /* DDRAM address line 2       */

/* ===================================================================
 *  BUZZER & ALERT LED
 *  Buzzer : P0.20
 *  LED    : P0.21
 * =================================================================== */
#define BUZZER      (1UL << 20)
#define ALERT_LED   (1UL << 21)

/* ===================================================================
 *  BIT-BANG I2C  --  SDA=P0.2  SCL=P0.3
 * =================================================================== */
#define SDA_PIN     (1UL << 2)
#define SCL_PIN     (1UL << 3)

#define SDA_HIGH()  { IODIR0 &= ~SDA_PIN; }                    /* release -> pulled high by resistor */
#define SDA_LOW()   { IODIR0 |=  SDA_PIN; IOCLR0 = SDA_PIN; } /* drive low                          */
#define SCL_HIGH()  { IOSET0 = SCL_PIN; delay_us(5); }
#define SCL_LOW()   { IOCLR0 = SCL_PIN; delay_us(5); }
#define SDA_READ    ((IOPIN0 & SDA_PIN) ? 1 : 0)

/* I2C Device Addresses (write mode, bit0=0) */
#define EEPROM_ADDR     0xA0    /* AT24C256  */
#define RTC_ADDR        0xD0    /* DS1307    */

/* EEPROM storage addresses for thresholds */
#define EE_ADDR_TEMP    0x0000
#define EE_ADDR_HUM     0x0001

/* ===================================================================
 *  UART0  --  ESP01 WiFi Module
 * =================================================================== */
#define BAUD_RATE       9600

/* ===================================================================
 *  4x4 KEYPAD
 *  Rows : P0.22, P0.23, P0.24, P0.25  (OUTPUT)
 *  Cols : P0.26, P0.27, P0.28, P0.29  (INPUT)
 * =================================================================== */
#define ROW_MASK    0x03C00000UL    /* P0.22-P0.25 */
#define COL_MASK    0x3C000000UL    /* P0.26-P0.29 */

/* ===================================================================
 *  GLOBAL VARIABLES
 * =================================================================== */
volatile uint8_t g_temp        = 0;
volatile uint8_t g_hum         = 0;
volatile uint8_t g_thresh_temp = 35;    /* default threshold: 35 C  */
volatile uint8_t g_thresh_hum  = 80;    /* default threshold: 80 %  */
volatile uint8_t g_alert_active = 0;

/* =================================================================== *
 *                                                                     *
 *   SECTION 1: DELAY FUNCTIONS                                        *
 *                                                                     *
 * =================================================================== */

void delay_us(uint32_t us)
{
    uint32_t i;
    for (i = 0; i < (us * (CCLK / 1000000UL) / 5); i++)
    {
        __nop();
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 2: PLL INIT  (CPU = 60 MHz)                              *
 *                                                                     *
 * =================================================================== */

void pll_init(void)
{
    PLLCON  = 0x01;             /* Step 1: Enable PLL              */
    PLLCFG  = 0x24;             /* Step 2: M=5, P=2 -> 60 MHz     */
    PLLFEED = 0xAA;             /* Feed sequence                   */
    PLLFEED = 0x55;

    while (!(PLLSTAT & (1 << 10)));  /* Step 3: Wait for PLL lock  */

    PLLCON  = 0x03;             /* Step 4: Enable + Connect PLL    */
    PLLFEED = 0xAA;
    PLLFEED = 0x55;

    VPBDIV  = 0x00;             /* PCLK = CCLK/4 = 15 MHz          */
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 3: LCD DRIVER  (8-bit, P1 port)                         *
 *                                                                     *
 * =================================================================== */

void lcd_pulse_en(void)
{
    IOSET1 = LCD_EN;
    delay_us(2);
    IOCLR1 = LCD_EN;
    delay_us(2);
}

void lcd_write(uint8_t data, uint8_t rs)
{
    /* Set all LCD pins as output */
    IODIR1 |= (LCD_DATA_MASK | LCD_ALL_CTRL);

    /* Clear data lines and control lines */
    IOCLR1 = (LCD_DATA_MASK | LCD_ALL_CTRL);

    /* Set RS: 1 = data, 0 = command */
    if (rs)
    {
        IOSET1 = LCD_RS;
    }

    /* Place data on D0-D7 (P1.16-P1.23) */
    IOSET1 = ((uint32_t)data << 16) & LCD_DATA_MASK;

    /* Pulse Enable */
    lcd_pulse_en();
    delay_us(100);
}

void lcd_cmd(uint8_t cmd)
{
    lcd_write(cmd, 0);
    delay_ms(2);
}

void lcd_data(uint8_t d)
{
    lcd_write(d, 1);
}

void lcd_init(void)
{
    delay_ms(20);               /* Power-on delay              */
    lcd_cmd(LCD_CMD_FUNC_8BIT); /* Function set: 8-bit, 2-line */
    lcd_cmd(LCD_CMD_DISP_ON);   /* Display ON, cursor OFF      */
    lcd_cmd(LCD_CMD_ENTRY_MODE);/* Entry mode: auto-increment  */
    lcd_cmd(LCD_CMD_CLEAR);     /* Clear display               */
    delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    if (row == 0)
        lcd_cmd(LCD_CMD_LINE1 + col);
    else
        lcd_cmd(LCD_CMD_LINE2 + col);
}

void lcd_print(const char *str)
{
    while (*str)
    {
        lcd_data((uint8_t)(*str));
        str++;
    }
}

void lcd_print_num(uint8_t num)
{
    lcd_data('0' + (num / 10));
    lcd_data('0' + (num % 10));
}

void lcd_clear(void)
{
    lcd_cmd(LCD_CMD_CLEAR);
    delay_ms(2);
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 4: DHT11 SENSOR DRIVER                                   *
 *                                                                     *
 * =================================================================== */

/*
 * dht11_read()
 * Returns 1 on success, 0 on error (checksum fail or no response).
 * Stores temperature in *temp (Celsius) and humidity in *hum (%).
 */
uint8_t dht11_read(uint8_t *temp, uint8_t *hum)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t i;
    uint8_t j;

    /* --- Step 1: MCU sends START signal --- */
    DHT11_DIR_OUT();    /* Set pin as output    */
    DHT11_LOW();        /* Pull low for 18 ms   */
    delay_ms(18);
    DHT11_HIGH();       /* Pull high            */
    delay_us(30);       /* Wait 20-40 us        */

    /* --- Step 2: Switch to input, check DHT11 response --- */
    DHT11_DIR_IN();     /* Set pin as input     */
    delay_us(10);

    if (DHT11_READ)
    {
        /* DHT11 did not pull line low -> sensor not responding */
        return 0;
    }

    delay_us(80);       /* DHT11 holds low 80 us */

    if (!DHT11_READ)
    {
        /* DHT11 did not pull line high -> error */
        return 0;
    }

    delay_us(80);       /* DHT11 holds high 80 us */

    /* --- Step 3: Read 40 bits (5 bytes) --- */
    for (j = 0; j < 5; j++)
    {
        for (i = 0; i < 8; i++)
        {
            /* Wait for rising edge (start of each bit) */
            while (!DHT11_READ);

            /* Delay 30 us: if line still high -> bit=1, else bit=0 */
            delay_us(30);

            data[j] <<= 1;

            if (DHT11_READ)
            {
                data[j] |= 0x01;
            }

            /* Wait for falling edge (end of bit) */
            while (DHT11_READ);
        }
    }

    /* --- Step 4: Verify checksum --- */
    if (data[4] != (uint8_t)(data[0] + data[1] + data[2] + data[3]))
    {
        return 0;   /* Checksum mismatch */
    }

    *hum  = data[0];    /* Humidity integer part    */
    *temp = data[2];    /* Temperature integer part */

    return 1;   /* Success */
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 5: BIT-BANG I2C DRIVER                                   *
 *                                                                     *
 * =================================================================== */

void i2c_init(void)
{
    /* Set SCL as output, SDA released (input with pull-up) */
    IODIR0 |=  SCL_PIN;
    IODIR0 &= ~SDA_PIN;
    IOSET0  =  SCL_PIN;
}

void i2c_start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    SDA_LOW();  /* SDA falls while SCL is high = START */
    SCL_LOW();
}

void i2c_stop(void)
{
    SDA_LOW();
    SCL_HIGH();
    SDA_HIGH(); /* SDA rises while SCL is high = STOP  */
}

/*
 * i2c_write_byte()
 * Sends one byte MSB first.
 * Returns 1 if ACK received, 0 if NACK.
 */
uint8_t i2c_write_byte(uint8_t byte)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80)
            SDA_HIGH();
        else
            SDA_LOW();

        SCL_HIGH();
        SCL_LOW();
        byte <<= 1;
    }

    /* Read ACK bit */
    SDA_HIGH();
    SCL_HIGH();
    ack = !SDA_READ;    /* ACK = SDA pulled low by slave */
    SCL_LOW();

    return ack;
}

/*
 * i2c_read_byte()
 * Reads one byte MSB first.
 * Send ack=1 to ACK (more bytes to read), ack=0 to NACK (last byte).
 */
uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t byte = 0;

    SDA_HIGH();     /* Release SDA for slave to drive */

    for (i = 0; i < 8; i++)
    {
        SCL_HIGH();
        byte = (byte << 1) | SDA_READ;
        SCL_LOW();
    }

    /* Send ACK or NACK */
    if (ack)
        SDA_LOW();
    else
        SDA_HIGH();

    SCL_HIGH();
    SCL_LOW();
    SDA_HIGH();

    return byte;
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 6: EEPROM DRIVER  (AT24C256 via I2C)                    *
 *                                                                     *
 * =================================================================== */

void eeprom_write_byte(uint16_t addr, uint8_t data)
{
    i2c_start();
    i2c_write_byte(EEPROM_ADDR);            /* Device address + Write   */
    i2c_write_byte((uint8_t)(addr >> 8));   /* High byte of address     */
    i2c_write_byte((uint8_t)(addr & 0xFF)); /* Low byte of address      */
    i2c_write_byte(data);                   /* Data byte                */
    i2c_stop();
    delay_ms(10);   /* AT24C256 internal write cycle: max 10 ms */
}

uint8_t eeprom_read_byte(uint16_t addr)
{
    uint8_t value;

    /* Set address (dummy write) */
    i2c_start();
    i2c_write_byte(EEPROM_ADDR);
    i2c_write_byte((uint8_t)(addr >> 8));
    i2c_write_byte((uint8_t)(addr & 0xFF));

    /* Repeated START + read */
    i2c_start();
    i2c_write_byte(EEPROM_ADDR | 0x01);    /* Device address + Read    */
    value = i2c_read_byte(0);              /* Read byte, send NACK     */
    i2c_stop();

    return value;
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 7: DS1307 RTC DRIVER  (I2C)                             *
 *                                                                     *
 * =================================================================== */

/* DS1307 register map */
#define RTC_REG_SECONDS     0x00
#define RTC_REG_MINUTES     0x01
#define RTC_REG_HOURS       0x02
#define RTC_REG_DAY         0x03
#define RTC_REG_DATE        0x04
#define RTC_REG_MONTH       0x05
#define RTC_REG_YEAR        0x06

/* BCD to decimal conversion */
uint8_t bcd_to_dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

/* Decimal to BCD conversion */
uint8_t dec_to_bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} RTC_Time;

void rtc_set_time(RTC_Time *t)
{
    i2c_start();
    i2c_write_byte(RTC_ADDR);
    i2c_write_byte(RTC_REG_SECONDS);
    i2c_write_byte(dec_to_bcd(t->seconds));
    i2c_write_byte(dec_to_bcd(t->minutes));
    i2c_write_byte(dec_to_bcd(t->hours));
    i2c_write_byte(0x01);                   /* Day: not used            */
    i2c_write_byte(dec_to_bcd(t->date));
    i2c_write_byte(dec_to_bcd(t->month));
    i2c_write_byte(dec_to_bcd(t->year));
    i2c_stop();
}

void rtc_get_time(RTC_Time *t)
{
    /* Set read pointer to register 0 */
    i2c_start();
    i2c_write_byte(RTC_ADDR);
    i2c_write_byte(0x00);

    /* Read all 7 time registers */
    i2c_start();
    i2c_write_byte(RTC_ADDR | 0x01);
    t->seconds = bcd_to_dec(i2c_read_byte(1) & 0x7F);  /* mask CH bit */
    t->minutes = bcd_to_dec(i2c_read_byte(1));
    t->hours   = bcd_to_dec(i2c_read_byte(1) & 0x3F);  /* 24-hr mode  */
    i2c_read_byte(1);                                   /* skip day    */
    t->date    = bcd_to_dec(i2c_read_byte(1));
    t->month   = bcd_to_dec(i2c_read_byte(1));
    t->year    = bcd_to_dec(i2c_read_byte(0));
    i2c_stop();
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 8: UART0 DRIVER  (ESP01 @ 9600 baud)                    *
 *                                                                     *
 * =================================================================== */

void uart0_init(void)
{
    /* Select UART0 pins: P0.0=TXD0, P0.1=RXD0 */
    PINSEL0 |= 0x00000005;

    /* DLAB=1 to access divisor latches */
    U0LCR = 0x83;               /* 8-bit, No parity, 1 stop bit   */

    /* Set baud rate divisor */
    uint32_t divisor = PCLK / (16 * BAUD_RATE);
    U0DLL = (uint8_t)(divisor & 0xFF);
    U0DLM = (uint8_t)(divisor >> 8);

    /* DLAB=0, normal operation */
    U0LCR = 0x03;

    /* Enable and reset TX/RX FIFOs */
    U0FCR = 0x07;
}

void uart0_putc(char c)
{
    /* Wait until Transmit Holding Register is empty */
    while (!(U0LSR & (1 << 5)));
    U0THR = (uint8_t)c;
}

void uart0_puts(const char *str)
{
    while (*str)
    {
        uart0_putc(*str);
        str++;
    }
}

char uart0_getc(void)
{
    /* Wait until data is available in RX FIFO */
    while (!(U0LSR & (1 << 0)));
    return (char)U0RBR;
}

/* Wait for a keyword in UART response (simple polling) */
uint8_t uart0_wait_for(const char *keyword, uint32_t timeout_ms)
{
    char    buf[64];
    uint8_t idx = 0;
    uint32_t elapsed = 0;

    memset(buf, 0, sizeof(buf));

    while (elapsed < timeout_ms)
    {
        if (U0LSR & (1 << 0))
        {
            buf[idx] = (char)U0RBR;
            idx = (idx + 1) % 63;

            if (strstr(buf, keyword))
                return 1;
        }
        delay_ms(1);
        elapsed++;
    }
    return 0;   /* Timeout */
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 9: ESP01 WiFi DRIVER                                     *
 *                                                                     *
 * =================================================================== */

void esp_send_cmd(const char *cmd)
{
    uart0_puts(cmd);
    uart0_puts("\r\n");
}

/*
 * esp01_init()
 * Configures ESP01 in station mode and connects to WiFi.
 * Returns 1 on success, 0 on failure.
 */
uint8_t esp01_init(void)
{
    uint8_t status = 0;

    /* Test AT */
    esp_send_cmd("AT");
    status = uart0_wait_for("OK", 2000);
    if (!status) return 0;

    /* Reset module */
    esp_send_cmd("AT+RST");
    delay_ms(2000);

    /* Set station mode */
    esp_send_cmd("AT+CWMODE=1");
    uart0_wait_for("OK", 2000);

    /* Connect to WiFi */
    uart0_puts("AT+CWJAP=\"");
    uart0_puts(WIFI_SSID);
    uart0_puts("\",\"");
    uart0_puts(WIFI_PASS);
    uart0_puts("\"\r\n");
    status = uart0_wait_for("OK", 10000);
    if (!status) return 0;

    /* Single connection mode */
    esp_send_cmd("AT+CIPMUX=0");
    uart0_wait_for("OK", 2000);

    return 1;
}

/*
 * thingspeak_upload()
 * Sends temperature and humidity to ThingSpeak via HTTP GET.
 * field1 = temperature, field2 = humidity
 */
uint8_t thingspeak_upload(uint8_t temp, uint8_t hum)
{
    char http_req[200];
    char len_cmd[32];
    uint8_t status;

    /* Build HTTP GET request string */
    sprintf(http_req,
        "GET /update?api_key=%s&field1=%d&field2=%d HTTP/1.1\r\n"
        "Host: api.thingspeak.com\r\n"
        "Connection: close\r\n\r\n",
        THINGSPEAK_KEY, temp, hum);

    /* Open TCP connection to ThingSpeak */
    esp_send_cmd("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80");
    status = uart0_wait_for("OK", 5000);
    if (!status) return 0;

    delay_ms(500);

    /* Tell ESP01 how many bytes to send */
    sprintf(len_cmd, "AT+CIPSEND=%d", (int)strlen(http_req));
    esp_send_cmd(len_cmd);
    status = uart0_wait_for(">", 3000);
    if (!status)
    {
        esp_send_cmd("AT+CIPCLOSE");
        return 0;
    }

    /* Send HTTP request */
    uart0_puts(http_req);
    status = uart0_wait_for("SEND OK", 5000);

    delay_ms(500);
    esp_send_cmd("AT+CIPCLOSE");
    delay_ms(500);

    return status;
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 10: BUZZER & LED ALERT                                   *
 *                                                                     *
 * =================================================================== */

void alert_on(void)
{
    IOSET0 = BUZZER | ALERT_LED;
    g_alert_active = 1;
}

void alert_off(void)
{
    IOCLR0 = BUZZER | ALERT_LED;
    g_alert_active = 0;
}

void alert_beep(uint8_t times)
{
    uint8_t i;
    for (i = 0; i < times; i++)
    {
        IOSET0 = BUZZER | ALERT_LED;
        delay_ms(100);
        IOCLR0 = BUZZER | ALERT_LED;
        delay_ms(100);
    }
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 11: 4x4 MATRIX KEYPAD DRIVER                            *
 *                                                                     *
 * =================================================================== */

const char keymap[4][4] =
{
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

/*
 * keypad_scan()
 * Scans all rows and columns.
 * Returns the character of the pressed key, or 0 if no key pressed.
 */
char keypad_scan(void)
{
    uint8_t  row;
    uint8_t  col;
    uint32_t col_state;

    /* Rows = output, Cols = input */
    IODIR0 |=  ROW_MASK;
    IODIR0 &= ~COL_MASK;

    for (row = 0; row < 4; row++)
    {
        /* Drive one row low at a time */
        IOCLR0 = ROW_MASK;
        IOSET0 = (1UL << (22 + row));
        delay_us(50);

        col_state = (IOPIN0 & COL_MASK) >> 26;

        for (col = 0; col < 4; col++)
        {
            if (col_state & (1 << col))
            {
                /* Wait for key release */
                while ((IOPIN0 & COL_MASK) >> 26);
                delay_ms(20);   /* Debounce */
                return keymap[row][col];
            }
        }
    }

    return 0;   /* No key pressed */
}

/*
 * keypad_get_number()
 * Prompts user on LCD line 2 to enter a 2-digit number.
 * '#' key confirms the entry.
 * Returns the entered value as uint8_t.
 */
uint8_t keypad_get_number(void)
{
    char    key;
    uint8_t value    = 0;
    uint8_t digit_count = 0;

    lcd_set_cursor(1, 0);
    lcd_print("Val:    # = OK  ");

    lcd_set_cursor(1, 4);

    while (1)
    {
        key = keypad_scan();

        if (key >= '0' && key <= '9')
        {
            if (digit_count < 3)
            {
                value = (value * 10) + (key - '0');
                lcd_data(key);
                digit_count++;
            }
        }
        else if (key == '#')
        {
            break;          /* Confirm entry */
        }
        else if (key == '*')
        {
            /* Clear entry */
            value       = 0;
            digit_count = 0;
            lcd_set_cursor(1, 4);
            lcd_print("     ");
            lcd_set_cursor(1, 4);
        }
    }

    return value;
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 12: DISPLAY HELPERS                                      *
 *                                                                     *
 * =================================================================== */

/*
 * display_sensor_data()
 * LCD Line 1: T:XXC  H:XX%
 * LCD Line 2: Th:XX/XX  OK / ALERT
 */
void display_sensor_data(uint8_t temp, uint8_t hum, uint8_t sensor_ok)
{
    char line[17];

    /* Line 1: Live readings */
    lcd_set_cursor(0, 0);
    if (sensor_ok)
    {
        sprintf(line, "T:%2dC  H:%2d%%   ", temp, hum);
    }
    else
    {
        sprintf(line, "DHT11 ERROR!    ");
    }
    lcd_print(line);

    /* Line 2: Thresholds and alert status */
    lcd_set_cursor(1, 0);
    if (g_alert_active)
    {
        sprintf(line, "Th:%2d/%2d ALERT! ", g_thresh_temp, g_thresh_hum);
    }
    else
    {
        sprintf(line, "Th:%2d/%2d  OK   ", g_thresh_temp, g_thresh_hum);
    }
    lcd_print(line);
}

/*
 * display_rtc()
 * Shows RTC timestamp on LCD line 2 for 2 seconds.
 */
void display_rtc(RTC_Time *t)
{
    char line[17];
    sprintf(line, "%02d:%02d:%02d %02d/%02d  ",
            t->hours, t->minutes, t->seconds,
            t->date,  t->month);
    lcd_set_cursor(1, 0);
    lcd_print(line);
    delay_ms(2000);
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 13: THRESHOLD MENU (via Keypad)                          *
 *                                                                     *
 * =================================================================== */

void menu_set_temp_threshold(void)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Set Temp Thresh:");
    g_thresh_temp = keypad_get_number();
    eeprom_write_byte(EE_ADDR_TEMP, g_thresh_temp);

    /* Confirm on LCD */
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Temp Threshold  ");
    lcd_set_cursor(1, 0);
    char msg[17];
    sprintf(msg, "Set to: %d C    ", g_thresh_temp);
    lcd_print(msg);
    delay_ms(1500);
    lcd_clear();
}

void menu_set_hum_threshold(void)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Set Hum Thresh: ");
    g_thresh_hum = keypad_get_number();
    eeprom_write_byte(EE_ADDR_HUM, g_thresh_hum);

    /* Confirm on LCD */
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Hum Threshold   ");
    lcd_set_cursor(1, 0);
    char msg[17];
    sprintf(msg, "Set to: %d %%   ", g_thresh_hum);
    lcd_print(msg);
    delay_ms(1500);
    lcd_clear();
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 14: SYSTEM INIT                                          *
 *                                                                     *
 * =================================================================== */

void system_init(void)
{
    /* --- GPIO directions --- */
    /* Port 1: LCD data + control lines as output */
    IODIR1 |= LCD_DATA_MASK | LCD_ALL_CTRL;

    /* Port 0: Buzzer, LED, SCL as output; SDA and DHT11 as input initially */
    IODIR0 |= (BUZZER | ALERT_LED | SCL_PIN);
    IODIR0 &= ~(DHT11_PIN | SDA_PIN);

    /* Ensure alert is OFF at startup */
    IOCLR0 = BUZZER | ALERT_LED;

    /* --- Init peripherals --- */
    pll_init();
    i2c_init();
    lcd_init();

    /* --- Startup screen --- */
    lcd_set_cursor(0, 0);
    lcd_print("IoT DataLogger  ");
    lcd_set_cursor(1, 0);
    lcd_print("Initializing... ");
    delay_ms(1500);

    /* --- Load thresholds from EEPROM --- */
    uint8_t ee_temp = eeprom_read_byte(EE_ADDR_TEMP);
    uint8_t ee_hum  = eeprom_read_byte(EE_ADDR_HUM);

    /* Use EEPROM values only if they are in valid range */
    if (ee_temp > 0 && ee_temp < 100)
        g_thresh_temp = ee_temp;
    if (ee_hum  > 0 && ee_hum  < 100)
        g_thresh_hum  = ee_hum;

    lcd_set_cursor(0, 0);
    lcd_print("Thresh Loaded:  ");
    char info[17];
    sprintf(info, "T:%dC  H:%d%%    ", g_thresh_temp, g_thresh_hum);
    lcd_set_cursor(1, 0);
    lcd_print(info);
    delay_ms(1500);

    /* --- UART + ESP01 --- */
    lcd_set_cursor(0, 0);
    lcd_print("Connecting WiFi ");
    lcd_set_cursor(1, 0);
    lcd_print("Please wait...  ");

    uart0_init();
    uint8_t wifi_ok = esp01_init();

    lcd_set_cursor(0, 0);
    if (wifi_ok)
    {
        lcd_print("WiFi Connected! ");
        alert_beep(2);
    }
    else
    {
        lcd_print("WiFi FAILED!    ");
        lcd_set_cursor(1, 0);
        lcd_print("Check SSID/Pass ");
        alert_beep(5);
    }
    delay_ms(2000);

    /* --- EINT1 config (push switch on P0.14) --- */
    EXTMODE  |= (1 << 1);   /* Edge sensitive           */
    EXTPOLAR |= (1 << 1);   /* Rising edge              */
    EXTINT   |= (1 << 1);   /* Clear any pending flag   */

    lcd_clear();
}

/* =================================================================== *
 *                                                                     *
 *   SECTION 15: MAIN                                                 *
 *                                                                     *
 * =================================================================== */

int main(void)
{
    uint8_t  temp          = 0;
    uint8_t  hum           = 0;
    uint8_t  sensor_ok     = 0;
    uint32_t upload_timer  = 0;     /* Counts 100 ms ticks          */
    uint32_t rtc_timer     = 0;     /* Counts ticks for RTC display */
    uint8_t  upload_status = 0;
    RTC_Time now;
    char     key;

    /* --- System startup --- */
    system_init();

    /* ---------------------------------------------------------------
     *  MAIN LOOP
     *  Each iteration ~100 ms.
     *  Cloud upload every ~15 s (150 iterations).
     *  RTC display every ~10 s (100 iterations).
     * --------------------------------------------------------------- */
    while (1)
    {
        /* ─────────────────────────────────────────────────────────
         *  1. Read DHT11 sensor
         * ───────────────────────────────────────────────────────── */
        sensor_ok = dht11_read(&temp, &hum);

        if (sensor_ok)
        {
            g_temp = temp;
            g_hum  = hum;
        }

        /* ─────────────────────────────────────────────────────────
         *  2. Threshold violation check
         * ───────────────────────────────────────────────────────── */
        if (sensor_ok)
        {
            if (g_temp > g_thresh_temp || g_hum > g_thresh_hum)
            {
                alert_on();
            }
            else
            {
                alert_off();
            }
        }

        /* ─────────────────────────────────────────────────────────
         *  3. Update LCD display
         * ───────────────────────────────────────────────────────── */
        display_sensor_data(g_temp, g_hum, sensor_ok);

        /* ─────────────────────────────────────────────────────────
         *  4. Show RTC timestamp every 10 seconds
         * ───────────────────────────────────────────────────────── */
        rtc_timer++;
        if (rtc_timer >= 100)
        {
            rtc_get_time(&now);
            display_rtc(&now);
            rtc_timer = 0;
        }

        /* ─────────────────────────────────────────────────────────
         *  5. Keypad input
         *     A = Set temperature threshold
         *     B = Set humidity threshold
         *     D = Manual upload to ThingSpeak
         *     C = Show RTC timestamp now
         * ───────────────────────────────────────────────────────── */
        key = keypad_scan();

        if (key == 'A')
        {
            menu_set_temp_threshold();
        }
        else if (key == 'B')
        {
            menu_set_hum_threshold();
        }
        else if (key == 'D')
        {
            lcd_set_cursor(1, 0);
            lcd_print("Uploading...    ");
            upload_status = thingspeak_upload(g_temp, g_hum);
            lcd_set_cursor(1, 0);
            if (upload_status)
                lcd_print("Upload OK!      ");
            else
                lcd_print("Upload FAILED!  ");
            delay_ms(1500);
        }
        else if (key == 'C')
        {
            rtc_get_time(&now);
            display_rtc(&now);
        }

        /* ─────────────────────────────────────────────────────────
         *  6. Auto cloud upload every 15 seconds
         * ───────────────────────────────────────────────────────── */
        upload_timer++;
        if (upload_timer >= 150)
        {
            upload_status = thingspeak_upload(g_temp, g_hum);
            upload_timer  = 0;

            /* Brief status on LCD line 2 */
            lcd_set_cursor(1, 0);
            if (upload_status)
                lcd_print("Cloud: OK       ");
            else
                lcd_print("Cloud: FAIL     ");
            delay_ms(800);
        }

        /* ─────────────────────────────────────────────────────────
         *  7. Loop delay
         * ───────────────────────────────────────────────────────── */
        delay_ms(100);
    }

    return 0;
}
```
/* ========================== END OF main.c ========================== */

