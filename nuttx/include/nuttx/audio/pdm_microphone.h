/****************************************************************************
 * include/nuttx/audio/pdm_microphone.h
 *
 * drivers for MEMS microphones which using the PDM bus as data interface
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
 *
 * References:
 *
 * -  The framework for this driver is based on Ken Pettit's VS1053 driver.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_AUDIO_PDM_MICROPHONE_H
#define __DRIVERS_AUDIO_PDM_MICROPHONE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#ifdef CONFIG_AUDIO_PDM_MIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 *
 * CONFIG_AUDIO_PDM_MIC - Enables pdm microphone support
 * CONFIG_PDM_MIC_INFLIGHT - Maximum number of buffers that the PDM_MIC driver
 *   will send to the PDM driver before any have completed.
 * CONFIG_PDM_MIC_MSG_PRIO - Priority of messages sent to the PDM_MIC worker
 *   thread.
 * CONFIG_PDM_MIC_BUFFER_SIZE - Preferred buffer size
 * CONFIG_PDM_MIC_NUM_BUFFERS - Preferred number of buffers
 * CONFIG_PDM_MIC_WORKER_STACKSIZE - Stack size to use when creating the the
 *   PCM510X worker thread.
 */

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

#ifndef CONFIG_PDM
#  error CONFIG_PDM is required by the pdm microphone driver
#endif

/* Default configuration values */

#ifndef CONFIG_PDM_MIC_INFLIGHT
#  define CONFIG_PDM_MIC_INFLIGHT          4
#endif

#if CONFIG_PDM_MIC_INFLIGHT > 255
#  error CONFIG_PDM_MIC_INFLIGHT must fit in a uint8_t
#endif

#ifndef CONFIG_PDM_MIC_MSG_PRIO
#  define CONFIG_PDM_MIC_MSG_PRIO          1
#endif

#ifndef CONFIG_PDM_MIC_BUFFER_SIZE
#  define CONFIG_PDM_MIC_BUFFER_SIZE       1024
#endif

#ifndef CONFIG_PDM_MIC_NUM_BUFFERS
#  define CONFIG_PDM_MIC_NUM_BUFFERS       4
#endif

#ifndef CONFIG_PDM_MIC_WORKER_STACKSIZE
#  define CONFIG_PDM_MIC_WORKER_STACKSIZE  768
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pdm_mic_initialize
 *
 * Description:
 *   Initialize the pdm_mic device.
 *
 * Input Parameters:
 *   pdm     - An pdm driver instance
 *
 * Returned Value:
 *   A new lower half audio interface for the PDM_MIC device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

struct pdm_dev_s;         /* Forward reference. Defined in include/nuttx/audio/pdm.h */
struct audio_lowerhalf_s; /* Forward reference. Defined in nuttx/audio/audio.h */

FAR struct audio_lowerhalf_s *pdm_mic_initialize(FAR struct pdm_dev_s *pdm);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_PDM_MIC */
#endif /* __INCLUDE_NUTTX_AUDIO_PDM_MIC_H */

