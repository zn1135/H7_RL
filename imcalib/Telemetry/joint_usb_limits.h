#ifndef JOINT_USB_LIMITS_H
#define JOINT_USB_LIMITS_H

/* 台架实测并由作者确认后才可配置；0 表示禁止非零输出。 */
#define JOINT_USB_LIMITS_APPROVED 0u
#define JOINT_USB_TORQUE_MAX_NM {0.0f, 0.0f, 0.0f, 0.0f}
#define JOINT_USB_SPEED_MAX_RAD_S {0.0f, 0.0f, 0.0f, 0.0f}
#define JOINT_USB_POS_MIN_RAD {0.0f, 0.0f, 0.0f, 0.0f}
#define JOINT_USB_POS_MAX_RAD {0.0f, 0.0f, 0.0f, 0.0f}

#endif
