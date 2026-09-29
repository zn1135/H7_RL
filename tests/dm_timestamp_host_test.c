#include "dm.h"
#include "machine_config.h"
#include "mono_ns.h"
#include <assert.h>

#define __get_PRIMASK() 0u
#define __disable_irq() ((void)0)
#define __set_PRIMASK(value) ((void)(value))
#include "../imcalib/user-lib/dm.c"

static uint64_t mock_ns;
static uint32_t mock_tick;
static const machine_cfg_t test_machine = {
    .dm_pos_max = 12.5f,
    .dm_vel_max = 45.0f,
    .dm_trq_max = 54.0f,
};
const machine_cfg_t *const machine = &test_machine;

uint64_t Mono_Ns_Get(void) { return mock_ns; }
uint32_t HAL_GetTick(void) { return mock_tick; }

int main(void)
{
    uint8_t first[8] = {0u, 0x40u, 0x00u, 0u, 0u, 0u, 0u, 0u};
    uint8_t second[8] = {0u, 0x80u, 0x00u, 0u, 0u, 0u, 0u, 0u};
    dm_motor_feedback_t *feedback = &dm_motor_feedback[0];
    float first_angle;

    mock_ns = 1000000u;
    mock_tick = 1u;
    Dm_Read(feedback, 0u, first, sizeof(first));
    Dm_Parse();
    first_angle = feedback->pos_rad;
    assert(feedback->parsed_rx_ns == 1000000u);

    mock_ns = 2000000u;
    mock_tick = 2u;
    Dm_Read(feedback, 0u, second, sizeof(second));
    assert(feedback->last_rx_ns == 2000000u);
    assert(feedback->parsed_rx_ns == 1000000u);
    assert(feedback->pos_rad == first_angle);

    Dm_Parse();
    assert(feedback->parsed_rx_ns == 2000000u);
    assert(feedback->pos_rad != first_angle);
    return 0;
}
