#ifndef HOST_POLICY_USB_H
#define HOST_POLICY_USB_H

#include <stdint.h>

/* HPI1 上位机策略链路。USB 回调只入队，解析和发送在 commTask。 */
void HostPolicy_RxIsr(const uint8_t *data, uint32_t len);
void HostPolicy_RxOverflowIsr(void);
void HostPolicy_Process(void);
void HostPolicy_Pump(void);
void HostPolicy_SubmitInput(const float obs[25], const float history[125],
                            const float command[3], uint64_t sample_us);
void HostPolicy_LastTrainingAction(float action[6]);
uint8_t HostPolicy_ModeLock(void);
uint8_t HostPolicy_Armed(void);
uint8_t HostPolicy_EnableAllowed(void);
uint32_t HostPolicy_AppliedSeq(void);

#endif
