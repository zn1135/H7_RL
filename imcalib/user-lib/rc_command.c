#include "rc_command.h"

/* 死区 + 限幅 → [-1,1] */
static float Rc_Axis(int16_t raw, uint16_t deadband)
{
    int16_t value;

    value = DR16_Deadline(raw, deadband);
    if (value > DR16_CH_LIMIT)
    {
        value = DR16_CH_LIMIT;
    }
    else if (value < -DR16_CH_LIMIT)
    {
        value = -DR16_CH_LIMIT;
    }
    return (float)value / (float)DR16_CH_LIMIT;
}

/* DR16 快照 → 指令; 离线全零 */
void Rc_Command_Update(rc_command_t *cmd, const dr16_t *rc)
{
    if (!rc->online)
    {
        cmd->vel = 0.0f;
        cmd->yaw = 0.0f;
        cmd->len = 0.0f;
        cmd->ang = 0.0f;
        cmd->s1 = 0u;
        cmd->s2 = 0u;
        cmd->online = 0u;
        return;
    }
    cmd->vel = Rc_Axis(rc->ch1, RC_DEADBAND_VEL);
    cmd->yaw = -Rc_Axis(rc->ch0, RC_DEADBAND_YAW);
    cmd->len = Rc_Axis(rc->wheel, RC_DEADBAND_LEN);
    cmd->ang = Rc_Axis(rc->ch3, RC_DEADBAND_ANG);
    cmd->s1 = rc->s1;
    cmd->s2 = rc->s2;
    cmd->online = 1u;
}
