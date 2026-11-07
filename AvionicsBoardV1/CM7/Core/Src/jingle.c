#include "jingle.h"

extern TIM_HandleTypeDef htim2;

#define SPEAKER_CHANNEL TIM_CHANNEL_3 /* PA2 / PWM3 */
#define BEEP_MS 1000U
#define FAILURE_BEEP_MS 3000U
#define GPS_REMINDER_MS 15000U

typedef enum
{
    SOUND_SD_FAIL,
    SOUND_SD_OK,
    SOUND_GPS_LOCK,
    SOUND_GPS_MISSING,
    SOUND_COUNT
} SoundEvent;

/* Failure tones last three seconds. Success uses two one-second tones
 * separated by one second of silence. */
static const uint16_t frequencies[SOUND_COUNT] = {200U, 1000U, 1800U, 400U};
static volatile uint32_t pending_events;
static volatile uint8_t gps_lock_announced;
static uint32_t next_gps_reminder_ms;
static uint32_t timer_clock_hz;
static uint32_t stage_started_ms;
static SoundEvent active_event;
static uint8_t active;
static uint8_t stage;

static uint32_t TIM2_InputClockHz(void)
{
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t apb_divider = hclk / pclk;
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0U)
        return apb_divider <= 4U ? hclk : 4U * pclk;
    return apb_divider == 1U ? pclk : 2U * pclk;
}

static void SetTone(uint16_t frequency)
{
    __HAL_TIM_SET_COMPARE(&htim2, SPEAKER_CHANNEL, 0U);
    if (frequency == 0U) return;

    uint32_t period = (timer_clock_hz + frequency / 2U) / frequency;
    __HAL_TIM_SET_PRESCALER(&htim2, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1U);
    __HAL_TIM_SET_COMPARE(&htim2, SPEAKER_CHANNEL, period / 2U);
    htim2.Instance->EGR = TIM_EGR_UG;
}

static void QueueEvent(SoundEvent event)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (event != SOUND_GPS_MISSING || !gps_lock_announced)
        pending_events |= 1UL << event;
    __set_PRIMASK(mask);
}

void Jingle_Init(void)
{
    timer_clock_hz = TIM2_InputClockHz();
    pending_events = 0U;
    gps_lock_announced = 0U;
    active = 0U;
    /* HAL ticks start at boot. The first no-lock reminder is due at 15 s. */
    next_gps_reminder_ms = GPS_REMINDER_MS;
    SetTone(0U);
}

void Jingle_NotifySDResult(uint8_t success)
{
    if (success)
        QueueEvent(SOUND_SD_OK);
    else
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        pending_events &= ~(1UL << SOUND_SD_OK);
        pending_events |= 1UL << SOUND_SD_FAIL;
        __set_PRIMASK(mask);
    }
}

void Jingle_UpdateGPSLock(uint8_t locked)
{
    if (locked)
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        pending_events &= ~(1UL << SOUND_GPS_MISSING);
        if (!gps_lock_announced)
        {
            pending_events |= 1UL << SOUND_GPS_LOCK;
            gps_lock_announced = 1U;
        }
        __set_PRIMASK(mask);
    }
}

static void StartNextSound(uint32_t now)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t pending = pending_events;
    for (unsigned event = 0U; event < SOUND_COUNT; event++)
    {
        if ((pending & (1UL << event)) != 0U)
        {
            active_event = (SoundEvent)event;
            pending_events &= ~(1UL << event);
            __set_PRIMASK(mask);
            stage = 0U;
            stage_started_ms = now;
            SetTone(frequencies[event]);
            active = HAL_TIM_PWM_Start(&htim2, SPEAKER_CHANNEL) == HAL_OK;
            return;
        }
    }
    __set_PRIMASK(mask);
}

void Jingle_Service(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_gps_reminder_ms) >= 0)
    {
        uint32_t periods = (now - next_gps_reminder_ms) / GPS_REMINDER_MS + 1U;
        next_gps_reminder_ms += periods * GPS_REMINDER_MS;
        if (!gps_lock_announced) QueueEvent(SOUND_GPS_MISSING);
    }
    if (!active)
    {
        StartNextSound(now);
        return;
    }
    uint32_t duration_ms = BEEP_MS;
    if (stage == 0U &&
        (active_event == SOUND_SD_FAIL || active_event == SOUND_GPS_MISSING))
        duration_ms = FAILURE_BEEP_MS;
    if ((uint32_t)(now - stage_started_ms) < duration_ms) return;
    stage_started_ms = now;

    if (stage == 0U)
    {
        SetTone(0U);
        stage = (active_event == SOUND_SD_OK || active_event == SOUND_GPS_LOCK) ? 1U : 3U;
    }
    else if (stage == 1U)
    {
        SetTone(frequencies[active_event]);
        stage = 2U;
    }
    else if (stage == 2U)
    {
        SetTone(0U);
        stage = 3U;
    }
    else
    {
        (void)HAL_TIM_PWM_Stop(&htim2, SPEAKER_CHANNEL);
        active = 0U;
        StartNextSound(now);
    }
}
