

/**
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>

#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

int main(void)
{
    uint32_t ADC_value[4];
    volatile uint32_t temp_avg;
    volatile float temp_value_C;
    volatile float temp_value_F;

    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Configure four steps in the ADC sequencer
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);

    // Set the interrupt flag when the sample is done
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 1);

    while (1)
    {
        ADCIntClear(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC0_BASE, 1);
        while(!ADCIntStatus(ADC0_BASE, 1, false));
        ADCSequenceDataGet(ADC0_BASE, 1, ADC_value);

        temp_avg = (ADC_value[0] + ADC_value[1] + ADC_value[2] + ADC_value[3]) / 4;

        // TEMP = 147.5 – ((75 * (VREFP – VREFN) * ADCVALUE) / 4096)
        temp_value_C = (float)(1475 - ((2475 * temp_avg)) / 4096) / 10;

        // F = ((C * 9) + 160) / 5
        temp_value_F = (float)((temp_value_C * 9) + 160) / 5;
    }
}
