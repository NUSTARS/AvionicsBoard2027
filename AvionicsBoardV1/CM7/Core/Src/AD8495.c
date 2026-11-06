#include "AD8495.h"

extern ADC_HandleTypeDef hadc1;

#define TC_ADC_VREF_V 3.3f
#define TC_ADC_MAX 65535.0f
#define TC_V_PER_DEGC 0.005f
#define TC_AMP_REF_V 0.0f

static uint8_t adc_ready;

HAL_StatusTypeDef AD8495_Init(void)
{
    adc_ready = 0U;
    ADC_InjectionConfTypeDef config = {0};
    config.InjectedNbrOfConversion = AD8495_CHANNEL_COUNT;
    config.InjectedSamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    config.InjectedSingleDiff = ADC_SINGLE_ENDED;
    config.InjectedOffsetNumber = ADC_OFFSET_NONE;
    config.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    config.InjectedChannel = ADC_CHANNEL_16;
    config.InjectedRank = ADC_INJECTED_RANK_1;
    HAL_StatusTypeDef status = HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);
    if (status != HAL_OK) return status;

    config.InjectedChannel = ADC_CHANNEL_17;
    config.InjectedRank = ADC_INJECTED_RANK_2;
    status = HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);
    if (status != HAL_OK) return status;

    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    if (status != HAL_OK) return status;

    status = HAL_ADCEx_InjectedStart(&hadc1);
    adc_ready = (status == HAL_OK);
    return status;
}

HAL_StatusTypeDef AD8495_Read(AD8495_Reading readings[AD8495_CHANNEL_COUNT])
{
    if (!adc_ready || !__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS) ||
        READ_BIT(hadc1.Instance->CR, ADC_CR_JADSTART) != 0U)
        return HAL_BUSY;

    /* Separate result registers retain both channels until the next trigger. */
    readings[0].raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    readings[1].raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
    SET_BIT(hadc1.Instance->CR, ADC_CR_JADSTART);

    for (unsigned channel = 0U; channel < AD8495_CHANNEL_COUNT; channel++)
    {
        uint16_t raw = readings[channel].raw;
        readings[channel].temp_c =
            (raw * (TC_ADC_VREF_V / TC_ADC_MAX) - TC_AMP_REF_V) / TC_V_PER_DEGC;
    }
    return HAL_OK;
}
