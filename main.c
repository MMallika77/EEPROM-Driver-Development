#include "main.h"

#include "stm32f4xx.h"

#define EEPROM_ADDR  0x50 // 7-bit address (A0,A1,A2 = GND)

/* ---------------- GPIO INIT ---------------- */
void I2C1_GPIO_Init(void)
{
    /* Enable GPIOB clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* Set PB8 & PB9 to Alternate Function mode */
    GPIOB->MODER &= ~((3 << (8*2)) | (3 << (9*2)));
    GPIOB->MODER |=  ((2 << (8*2)) | (2 << (9*2)));

    /* Open-drain */
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);

    /* Very High Speed */
    GPIOB->OSPEEDR &= ~((3 << (8*2)) | (3 << (9*2)));
    GPIOB->OSPEEDR |=  ((3 << (8*2)) | (3 << (9*2)));

    /* No internal pull-ups (use external 4.7kΩ) */
    GPIOB->PUPDR &= ~((3 << (8*2)) | (3 << (9*2)));

    /* Select AF4 (I2C1) */
    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));   // PB8, PB9 are in AFR[0]
    GPIOB->AFR[1] |=  ((4 << 0) | (4 << 4));       // AF4 = I2C
}

/* ---------------- I2C INIT ---------------- */
void I2C1_Init(void)
{
    /* Enable I2C1 clock */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* Reset I2C1 (optional but recommended) */
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /* Disable I2C before configuration */
    I2C1->CR1 &= ~I2C_CR1_PE;

    /* APB1 frequency in MHz (16 MHz) */
    I2C1->CR2 = 42;

    /* Standard Mode (100 kHz)
       CCR = Fpclk1 / (2 * Fscl)
       = 42MHz / (2 * 100kHz) = 80
    */
    I2C1->CCR = 210;

    /* TRISE
       = (Fpclk1 / 1MHz) + 1
       = 42 + 1 = 17
    */
    I2C1->TRISE = 43;

    /* Enable ACK */
    I2C1->CR1 |= I2C_CR1_ACK;

    /* Enable I2C */
    I2C1->CR1 |= I2C_CR1_PE;
}

/* ---------------- I2C START ---------------- */
int I2C1_Start(uint8_t address, uint8_t direction)
{
    while(I2C1->SR2 & I2C_SR2_BUSY);

    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));

    (void)I2C1->SR1;

    I2C1->DR = (address << 1) | direction;

    while(!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)));

    if(I2C1->SR1 & I2C_SR1_AF)
    {
        I2C1->SR1 &= ~I2C_SR1_AF;
        I2C1->CR1 |= I2C_CR1_STOP;
        return 0; // error
    }

    (void)I2C1->SR1;
    (void)I2C1->SR2;

    return 1; // success
}

/* ---------------- I2C STOP ---------------- */
void I2C1_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

/* ---------------- I2C WRITE BYTE ---------------- */
int I2C1_WriteByte(uint8_t data)
{
    while(!(I2C1->SR1 & (I2C_SR1_TXE | I2C_SR1_AF)));

    if(I2C1->SR1 & I2C_SR1_AF)
    {
        I2C1->SR1 &= ~I2C_SR1_AF;
        return 0;
    }

    I2C1->DR = data;

    while(!(I2C1->SR1 & (I2C_SR1_BTF | I2C_SR1_AF)));

    if(I2C1->SR1 & I2C_SR1_AF)
    {
        I2C1->SR1 &= ~I2C_SR1_AF;
        return 0;
    }

    return 1;
}


uint8_t I2C1_ReadByte(uint8_t ack)
{
    if(ack)
        I2C1->CR1 |= I2C_CR1_ACK;
    else
        I2C1->CR1 &= ~I2C_CR1_ACK;

    while(!(I2C1->SR1 & I2C_SR1_RXNE));

    return I2C1->DR;
}

void EEPROM_WriteByte(uint16_t mem_addr, uint8_t data)
{
    I2C1_Start(EEPROM_ADDR, 0);

    I2C1_WriteByte(mem_addr >> 8);
    I2C1_WriteByte(mem_addr & 0xFF);
    I2C1_WriteByte(data);

    I2C1_Stop();

    // ACK Polling
    while(!I2C1_Start(EEPROM_ADDR, 0));
    I2C1_Stop();
}

/* ---------------- EEPROM READ (Single Byte) ---------------- */
uint8_t EEPROM_ReadByte(uint16_t mem_addr)
{
    uint8_t data;

    /* Send memory address */
    I2C1_Start(EEPROM_ADDR, 0);
    I2C1_WriteByte(mem_addr >> 8);
    I2C1_WriteByte(mem_addr & 0xFF);

    /* Repeated START in read mode */
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (EEPROM_ADDR << 1) | 1;

    while(!(I2C1->SR1 & I2C_SR1_ADDR));

    /* Single byte reception */
    I2C1->CR1 &= ~I2C_CR1_ACK;   // Disable ACK
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP;   // Generate STOP

    while(!(I2C1->SR1 & I2C_SR1_RXNE));
    data = I2C1->DR;

    return data;
}
/* ---------------- MAIN ---------------- */
volatile uint32_t pclk1 = 0;
int main(void)
{
    volatile uint8_t value[6] = {0};
    pclk1 = HAL_RCC_GetPCLK1Freq();

    I2C1_GPIO_Init();
    I2C1_Init();

    EEPROM_WriteByte(0x0000, 10);    // Write '10'
    EEPROM_WriteByte(0x0001, 1);    // Write '1'
    EEPROM_WriteByte(0x0002, 2);    // Write '2'
    EEPROM_WriteByte(0x0003, 3);    // Write '3'
    EEPROM_WriteByte(0x0004, 4);    // Write '4'
    EEPROM_WriteByte(0x0005, 5);    // Write '5'

    value[0] = EEPROM_ReadByte(0x0000);  // Read back
    value[1] = EEPROM_ReadByte(0x0001);  // Read back
    value[2] = EEPROM_ReadByte(0x0002);  // Read back
    value[3] = EEPROM_ReadByte(0x0003);  // Read back
    value[4] = EEPROM_ReadByte(0x0020);  // Read back




    while(1)
    {
        // Check "value" in debugger
        value[5] = EEPROM_ReadByte(0x0007);  // Read back

    }
}
