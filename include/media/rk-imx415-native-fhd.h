/* SPDX-License-Identifier: GPL-2.0 */
/* RK3588 IMX415 native FullFOV FHD: private kernel-to-kernel operations.
 * NO UAPI ioctls, no exported userspace ABI. v4l2_subdev core.command
 * is intentionally not forwarded by subdev_do_ioctl() default case.
 * Only the owning CIF capture path can issue the ordered sensor stages.
 */
#ifndef _MEDIA_RK_IMX415_NATIVE_FHD_H
#define _MEDIA_RK_IMX415_NATIVE_FHD_H

#define RK_IMX415_FHD_KERNEL_STAGE_CMD 0x524B8501u
#define RK_IMX415_FHD_KERNEL_QUIET_CMD 0x524B8502u
#define RK_IMX415_FHD_KERNEL_QUERY_CMD 0x524B8503u

enum rk_imx415_native_fhd_stage {
    RKMODULE_NATIVE_FHD_PROBE = 0,
    RKMODULE_NATIVE_FHD_HOLD = 1,
    RKMODULE_NATIVE_FHD_WAKE = 2,
    RKMODULE_NATIVE_FHD_RELEASE = 3,
};

/* Values match the local sensor's one-shot enum. This is private ABI
 * between the in-tree IMX415 and RKCIF modules, never userspace.
 */
enum rk_imx415_native_fhd_status {
    RK_IMX415_FHD_IDLE = 0,
    RK_IMX415_FHD_HOLDING = 1,
    RK_IMX415_FHD_WAKING = 2,
    RK_IMX415_FHD_RELEASED = 3,
    RK_IMX415_FHD_UNCERTAIN = 4,
};

#endif /* _MEDIA_RK_IMX415_NATIVE_FHD_H */
