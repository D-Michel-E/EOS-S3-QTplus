/* Copyright 2021 Stefan Holst

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <sys/param.h>

#include "regs/aip.h"
#include "regs/cru.h"
#include "regs/iomux.h"
#include "regs/misc.h"
#include "regs/fpga.h"
#include "regs/pmu.h"
#include "regs/pkfb.h"
#include "regs/timer.h"

#include "uart.h"
#include "i2c.h"
#include "spi.h"
#include "io.h"
#include "fpga.h"

#include "libraries/ads1220/ads1220.h"

#include "hw/template/build/top_bit.h"

extern uint32_t uptime_ms; // global from startup.c

extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;

typedef struct
{
    uint16_t dwords;    // download size in double(8-byte)-words minus 1.
    uint8_t baudr_mult; // baudrate multiplier SPI->BAUDR is set to 2*baudr_mult.
    uint8_t id;         // id for EOS S3, must be 0x20.
} tFlashBootHeader;

typedef struct
{
    volatile uint32_t increments;
    volatile uint32_t brightness;
    volatile uint32_t lfsr;
} tWBregs;

void CS_UP(void)
{
    // GPIO_CS_GROUP and GPIO_CS_PIN depend on your schematic.
    // HAL_GPIO_WritePin(GPIO_CS_GROUP, GPIO_CS_PIN, GPIO_PIN_SET);
}
void CS_DOWN(void)
{
    // GPIO_CS_GROUP and GPIO_CS_PIN depend on your schematic.
    // HAL_GPIO_WritePin(GPIO_CS_GROUP, GPIO_CS_PIN, GPIO_PIN_RESET);
}
void TRANSMIT(uint8_t data)
{
    // SPI_DRIVER depends on your configuration.
    spi_tx (0x2, &data, sizeof(uint8_t));
}
uint32_t flipUint16Horizontally(uint32_t bitmask)
{
    uint32_t flippedMask = 0;
    for(unsigned int bit = 0; bit < 32; ++bit)
    {
        uint32_t currentBit = (bitmask & (1 << bit)) >> bit;
        flippedMask |= currentBit << (31 - bit);
    }
    return flippedMask;
}

uint32_t RECEIVE(void)
{
    uint32_t dataR = 0;
    // SPI_DRIVER depends on your configuration.
    spi_rx (0x2, &dataR, 3);
    // printf("\n%x\n",flipUint16Horizontally(dataR)/256);
    return dataR;
}
void DELAY(uint32_t us)
{
    uint32_t current_tvalue = TIMER->VALUE; // 2 MHz down-counter
    uint32_t last_systick_tvalue = current_tvalue;
    // DELAY_US depends on your code.
    while(-current_tvalue < -last_systick_tvalue + us*5){
    // printf("%i\n",current_tvalue);
     current_tvalue = TIMER->VALUE; // 2 MHz down-counter
    }
    // DELAY_US(us);
}
ADS1220_Handler_t Handler = {0};



int main(void)
{
    tWBregs *wb_regs = (tWBregs *)FPGA_WB_BASE;
    uint32_t rom_bytes = (uint32_t)&__etext + ((uint32_t)&__data_end__ - (uint32_t)&__data_start__);
    uint8_t btn_oldstate = 0;
    uint32_t read_addr = 0;
    uint32_t erase_addr = 0;
    uint8_t data[256];
    int32_t x, y, z;
    uint16_t aIN = 0;
    uint16_t bIN = 0;

    Handler.ADC_CS_HIGH = CS_UP;
    Handler.ADC_CS_LOW = CS_DOWN;
    Handler.ADC_Transmit = TRANSMIT;
    Handler.ADC_Receive = RECEIVE;
    Handler.ADC_Delay_US = DELAY;
    // Initialize 115200 8N1 UART on pads 44/45 for printf and output banner:
    uart_init();
    printf("\n\n\n\n");
    printf("SparkFun QuickLogic Thing Plus - EOS S3 MCU + eFPGA Bare-Bones Demo\n");
    printf("https://github.com/D-Michel-E/EOS-S3-QTplus\n");
    printf("ROM image size: %d (0x%x) bytes\n", rom_bytes, rom_bytes);

    // Set up and verify flash chip communication:
    spi_init();
    tFlashBootHeader header;
    printf("Flash ID: %x\n", spi_flash_read_id());
    spi_flash_read(0x120, (uint8_t *)&header, sizeof(header));
    printf("CfgSM boot header at 0x120: id=%x ", header.id);
    if (header.id == 0x20)
        printf("speed=%d size=%dkiBytes\n", header.baudr_mult, (1 + header.dwords) * 8 / 1024);
    else
        printf("<invalid>\n");

    i2c_accel_init();
    io_init();
    io_adc_read(&aIN, 0x2);

    // ensure Packet FIFO power up
    PMU->FFE_FB_PF_SW_WU |= PMU_FFE_FB_PF_SW_WU_PF_WU_Msk;
    while (!(PMU->PF_STATUS & 1))
        ;
    // C01=9MHz already configured by spi_init()
    CRU->C01_CLK_GATE |= CRU_C01_CLK_GATE_PATH_2_PF;
    PKFB->CTRL |= PKFB_CTRL_PF0_ENABLE | PKFB_CTRL_PF0_PUSH_MUX_FFE; // Enable push from fabric.

    printf("\nPress <space> for help.\n");



    int32_t ADC_Data;

// Passing Parameters as NULL to use default configurations.
    ADS1220_Parameters_t paramsADS;
    paramsADS.InputMuxConfig = P0NAVSS;
    paramsADS.VoltageRef = AnalogSupply;
    paramsADS.DataRate = _1000_SPS_;
    paramsADS.OperatingMode = NormalMode;
    paramsADS.PGAdisable = 1;
    ADS1220_Init(&Handler, &paramsADS);
    // ADS1220_ActivateContinuousMode(&Handler);
    // Default conversion mode is Single-shot

    ADS1220_StartSync(&Handler);
    while (1)
    {
            // printf("\ndrdy: %i\n", ((~MISC->IO_INPUT)  & 0b00001000    ));

        // Report USR button state changes:
        uint8_t btn_state = io_get_usrbtn();
        if (btn_state != btn_oldstate)
        {
            if (btn_state)
            {
                printf("USR button pressed\n");
                printf("PKFB 0 cnt %x\n", PKFB->PF0_CNT);
                while (!(PKFB->PF0_CNT & PKFB_PFx_CNT_EMPTY))
                    printf("  %08x\n", PKFB->PF0_DATA);
            }
            else
            {
                printf("USR button released\n");
                printf("PKFB 0 cnt %x\n", PKFB->PF0_CNT);
                while (!(PKFB->PF0_CNT & PKFB_PFx_CNT_EMPTY))
                    printf("  %08x\n", PKFB->PF0_DATA);
            }
            btn_oldstate = btn_state;
        }

    while(((~MISC->IO_INPUT)  & 0b00001000)!=8){}
    ADS1220_ReadData(&Handler, &ADC_Data);
    ADS1220_StartSync(&Handler);
    // ADC_Data = flipUint16Horizontally(ADC_Data)/256;
    printf("ADCRaw: %u\n", ADC_Data);
    // printf("\nADC Raw: %x\n", ((~MISC->IO_INPUT)  & 0b00001000));
        // Process commands from UART:
        if (uart_rx_available())
        {


    // 2.048 is internal voltage reference and is used as default config.
    // 1 is default adc gain value and it must be equivalent to the gain config in ADS1220_Parameters_t.
    // printf("ADC Raw: 0x%X | ADC Voltage : %f\r\n", ADC_Data, ADCValueToVoltage(ADC_Data, 2.048, 1));
        // Flash green LED every 1.024s:
        // io_set_green(!(uptime_ms & 0x3ff));
                if (MISC->FB_DEVICE_ID == 0xf01d)
                {
                io_adc_read(&bIN, 0x2);
                wb_regs->increments += (bIN - aIN)/40;
                }
            switch (uart_rx() & 0xff)
            {
            case 'f':
                printf("Giving control of USR button and blue red LED to FPGA...\n");
                IOMUX->PAD[18] = IOMUX_PAD_18_FSEL_FBIO18 | IOMUX_PAD_E_4MA;
                IOMUX->PAD[22] = IOMUX_PAD_22_FSEL_FBIO22 | IOMUX_PAD_E_4MA;  // Route GPIO6 -> Red LED on Pad 22
                IOMUX->PAD[21] = IOMUX_PAD_21_FSEL_FBIO21 | IOMUX_PAD_E_4MA;  // Route GPIO5 -> Green LED on Pad 21
                IOMUX->PAD[6] = IOMUX_PAD_6_FSEL_FBIO6 | IOMUX_PAD_OEN_DISABLE | IOMUX_PAD_REN_ENABLE;
                IOMUX->PAD[7] = IOMUX_PAD_7_FSEL_FBIO7 | IOMUX_PAD_OEN_DISABLE | IOMUX_PAD_REN_ENABLE | IOMUX_PAD_P_PULLDOWN; // keep reset input low.
                IOMUX->FBIO_SEL_1 = (1 << 6) | (1 << 7);

                printf("Configuring FPGA...\n");
                fpga_configure((uint32_t *)top_bit);
                printf("Done. Fabric Device ID: %x\n", MISC->FB_DEVICE_ID);
                printf("Influence blue LED by pressing USR button.\n");
                break;
            case 'a':
                i2c_accel_read(&x, &y, &z);
                io_adc_read(&bIN, 0x2); //0x4
                printf("X %6d Y %6d Z %6d BAT %d uptime %d\n", x, y, z,bIN, uptime_ms);
                break;
            case 'r':
                spi_flash_read(read_addr, data, 256);
                for (int i = 0; i < 256; i += 16)
                {
                    printf("%06x", i + read_addr);
                    for (int j = 0; j < 16; j++)
                        printf(" %02x", data[i + j]);
                    printf("\n");
                }
                read_addr += 256;
                break;
            case 'e':
                printf("Erase a 4096-byte sector starting at 0x%x ?\n", erase_addr);
                printf("<y> - Yes\n<any other key> - Abort.\n");
                io_set_green(1);
                io_set_red(1);
                if ((uart_rx() & 0xff) == 'y')
                {
                    io_set_green(0);
                    printf("Erasing...\n");
                    spi_flash_erase_sector(erase_addr);
                    printf("Done.\n");
                    erase_addr += 4096;
                }
                else
                    printf("Aborted.\n");
                io_set_red(0);
                break;
            case 'w':
                printf("Write currently loaded image (%d bytes) to flash?\n", rom_bytes);
                printf("<y> - Yes\n<any other key> - Abort.\n");
                io_set_green(1);
                io_set_red(1);
                if ((uart_rx() & 0xff) == 'y')
                {
                    printf("Writing...  ('*'=sector erase, '.'=page prog. success, 'E'=page prog. error)\n");
                    io_set_green(0);
                    for (int sector_offset = 0; sector_offset < rom_bytes; sector_offset += 4096)
                    {
                        uint32_t sector_bytes = MIN(4096, rom_bytes - sector_offset);
                        printf("*");
                        spi_flash_erase_sector(sector_offset);
                        for (int page_offset = 0; page_offset < sector_bytes; page_offset += 256)
                        {
                            uint32_t addr = sector_offset + page_offset;
                            uint8_t *data = (uint8_t *)(addr);
                            if (spi_flash_program_and_verify_page(addr, data, MIN(256, rom_bytes - addr)))
                                printf("E");
                            else
                                printf(".");
                        }
                        printf("\n");
                    }
                    printf("Done.\n");
                }
                else
                    printf("Aborted.\n");
                io_set_red(0);
                break;
            case 'R':
                printf("Fabric Device ID: %x\n", MISC->FB_DEVICE_ID);
                if (MISC->FB_DEVICE_ID == 0xf01d)
                {
                    printf("Increments: 0x%08x\n", wb_regs->increments);
                    printf("Brightness: 0x%08x\n", wb_regs->brightness);
                    printf("LFSR value: 0x%08x\n", wb_regs->lfsr);
                }
                break;
            case 'U':
                if (MISC->FB_DEVICE_ID == 0xf01d)
                {
                    wb_regs->increments += 0x200;
                    printf("Increments: 0x%08x\n", wb_regs->increments);
                }
                break;
            case 'D':
                if (MISC->FB_DEVICE_ID == 0xf01d)
                {
                    wb_regs->increments -= 0x200;
                    printf("Increments: 0x%08x\n", wb_regs->increments);
                }
                break;
            default:
                printf("<a> - read accelerometer and battery ADC.\n");
                printf("<r> - read one page of flash memory.\n");
                printf("<e> - erase a sector of flash memory.\n");
                printf("<w> - write currently loaded image to flash.\n");
                printf("<f> - configure FPGA with blink example.\n");
                if (MISC->FB_DEVICE_ID == 0xf01d)
                {
                    printf("  <R> - read WB registers of blink example design.\n");
                    printf("  <U> - Speed up LED blinking.\n");
                    printf("  <D> - Slow down LED blinking.\n");
                }
                printf("<any other key> - print this message.\n\n");
                break;
            }
        }
    }

    return 0;
}
