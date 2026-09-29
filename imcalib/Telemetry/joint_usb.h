#ifndef JOINT_USB_H
#define JOINT_USB_H

#include <stdint.h>
#include "torque_output.h"

/* USB 回调仅入队；commTask 解析和发送，actuationTask 唯一下发。 */
void JointUsb_RxIsr(const uint8_t *data, uint32_t len);
void JointUsb_RxOverflowIsr(void);
void JointUsb_Process(void);
void JointUsb_Pump(void);
void JointUsb_ActuationTick(const torque_output_t *torque, uint64_t queue_ns,
                            uint8_t dm_ok);
uint8_t JointUsb_PhysicalPermit(void);
uint8_t JointUsb_ModeLock(void);
uint8_t JointUsb_StreamRequested(void);
uint8_t JointUsb_EnableAllowed(void);
void JointUsb_Compute(torque_output_t *torque);

#endif
