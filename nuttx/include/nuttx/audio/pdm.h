/****************************************************************************
 * include/nuttx/audio/pdm.h
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_AUDIO_PDM_H
#define __INCLUDE_NUTTX_AUDIO_PDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/audio/audio.h>

#ifdef CONFIG_PDM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Access macros ************************************************************/

/****************************************************************************
 * Name: PDM_RXSAMPLERATE
 *
 * Description:
 *   Set the PDM RX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an PDM receiver or if (2) the sample rate is
 *   driven by the PDM frame clock.  This may also have unexpected side-
 *   effects of the RX sample is coupled with the TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The PDM sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_RXSAMPLERATE(d,f) ((d)->ops->pdm_rxsamplerate(d,f))

/****************************************************************************
 * Name: PDM_RXDATAWIDTH
 *
 * Description:
 *   Set the PDM RX data width.  The RX bitrate is determined by
 *   channel * sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The PDM data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_RXDATAWIDTH(d,b) ((d)->ops->pdm_rxdatawidth(d,b))

/****************************************************************************
 * Name: PDM_RXCHANNEL
 *
 * Description:
 *   Set the PDM rx data buffer channel.  This is not setting the PDM hardware
 *   channels, because the PDM hardware is always stereo. However, In some
 *   situation, we might only want one channel's data, or the data in left channel
 *   is same as that in the right channel.In these situations, we can use this API
 *   to set PDM to mono, then it will save either left or right channel's data
 *   to buffer
 *
 *   When we call this function to set the RX buffer as mono, and set the
 *   sample width as 8 bits.The audio data will be arranged as the following:
 *   |---left0---||---left1---||---left2---||---left3---|
 *   |---left4---||---left5---||---left6---||---left7---|
 *   |---8bits---||---8bits---||---8bits---||---8bits---|
 *   |--------------------32bits------------------------|
 *   when we set the RX buffer as mono and set the sample width as 16 bits.
 *   |-----------left0--------||-----------left1--------|
 *   |-----------left1--------||-----------left3--------|
 *   |-----------16bits-------||-----------16bits-------|
 *   |--------------------32bits------------------------|
 *   when we set the RX buffer as mono and set the sample width as 24 bits.
 *   |--------||--------------left0---------------------|
 *   |--------||--------------left1---------------------|
 *   |--------||-------------24bits---------------------|
 *   |--------------------32bits------------------------|
 *   when we set the RX buffer as mono and set the sample width as 32 bits.
 *   |------------------------left0---------------------|
 *   |------------------------left1---------------------|
 *   |--------------------32bits------------------------|
 *   |--------------------32bits------------------------|
 *   when we set the RX buffer as stereo and set the sample width as 8 bits.
 *   |---left0---||--right0---||---left1---||--right1---|
 *   |---left2---||--right2---||---left3---||--right3---|
 *   |---8bits---||---8bits---||---8bits---||---8bits---|
 *   |--------------------32bits------------------------|
 *   when we set the RX buffer as stereo and set the sample width as 16 bits.
 *   |-----------left0--------||----------right0--------|
 *   |-----------left1--------||----------right2--------|
 *   |-----------16bits-------||-----------16bits-------|
 *   |--------------------32bits------------------------|
 *   when we set the RX buffer as stereo and set the sample width as 24 bits.
 *   |--------||--------------left0---------------------|
 *   |--------||-------------right0---------------------|
 *   |--------||-------------24bits---------------------|
 *   |--------------------32bits------------------------|
 *   when we set the RX buffer as stereo and set the sample width as 32 bits.
 *   |------------------------left0---------------------|
 *   |-----------------------right0---------------------|
 *   |--------------------32bits------------------------|
 *   |--------------------32bits------------------------|
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   channel - The PDM data buffer channel. 1 stands for mono, 2 stands for stereo.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_RXCHANNEL(d,b) ((d)->ops->pdm_txchannel(d,b))


/****************************************************************************
 * Name: PDM_RECEIVE
 *
 * Description:
 *   Receive a block of data from PDM.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to recieve data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete.
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

#define PDM_RECEIVE(d,b,c,a,t) ((d)->ops->pdm_receive(d,b,c,a,t))

/****************************************************************************
 * Name: PDM_TXSAMPLERATE
 *
 * Description:
 *   Set the PDM TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an PDM transmitter or if (2) the sample rate is
 *   driven by the PDM frame clock.  This may also have unexpected side-
 *   effects of the TX sample is coupled with the RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The PDM sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_TXSAMPLERATE(d,f) ((d)->ops->pdm_txsamplerate(d,f))

/****************************************************************************
 * Name: PDM_TXDATAWIDTH
 *
 * Description:
 *   Set the PDM TX data width.  The TX bitrate is determined by
 *   2 * sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The PDM data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_TXDATAWIDTH(d,b) ((d)->ops->pdm_txdatawidth(d,b))

/****************************************************************************
 * Name: PDM_TXCHANNEL
 *
 * Description:
 *   Set the PDM tx data buffer channel.  This is not setting the PDM hardware
 *   channels, because the PDM hardware is always stereo. Howerer, the data beffer
 *   to be sent might be mono.To handle this situation, we should send the same
 *   audio sample at both left and right channels.
 *   When we call this function to set the TX buffer as mono, and set the
 *   sample width as 8 bits.we assume the data beffer is arranged as the following:
 *   |---left0---||---left1---||---left2---||---left3---|
 *   |---left4---||---left5---||---left6---||---left7---|
 *   |---8bits---||---8bits---||---8bits---||---8bits---|
 *   |--------------------32bits------------------------|
 *   when we set the TX buffer as mono and set the sample width as 16 bits.
 *   |-----------left0--------||-----------left1--------|
 *   |-----------left1--------||-----------left3--------|
 *   |-----------16bits-------||-----------16bits-------|
 *   |--------------------32bits------------------------|
 *   when we set the TX buffer as mono and set the sample width as 24 bits.
 *   |--------||--------------left0---------------------|
 *   |--------||--------------left1---------------------|
 *   |--------||-------------24bits---------------------|
 *   |--------------------32bits------------------------|
 *   when we set the TX buffer as mono and set the sample width as 32 bits.
 *   |------------------------left0---------------------|
 *   |------------------------left1---------------------|
 *   |--------------------32bits------------------------|
 *   |--------------------32bits------------------------|
 *   when we set the TX buffer as stereo and set the sample width as 8 bits.
 *   |---left0---||--right0---||---left1---||--right1---|
 *   |---left2---||--right2---||---left3---||--right3---|
 *   |---8bits---||---8bits---||---8bits---||---8bits---|
 *   |--------------------32bits------------------------|
 *   when we set the TX buffer as stereo and set the sample width as 16 bits.
 *   |-----------left0--------||----------right0--------|
 *   |-----------left1--------||----------right2--------|
 *   |-----------16bits-------||-----------16bits-------|
 *   |--------------------32bits------------------------|
 *   when we set the TX buffer as stereo and set the sample width as 24 bits.
 *   |--------||--------------left0---------------------|
 *   |--------||-------------right0---------------------|
 *   |--------||-------------24bits---------------------|
 *   |--------------------32bits------------------------|
 *   when we set the TX buffer as stereo and set the sample width as 32 bits.
 *   |------------------------left0---------------------|
 *   |-----------------------right0---------------------|
 *   |--------------------32bits------------------------|
 *   |--------------------32bits------------------------|
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   channel - The PDM data buffer channel. 1 stands for mono, 2 stands for stereo.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define PDM_TXCHANNEL(d,b) ((d)->ops->pdm_txchannel(d,b))


/****************************************************************************
 * Name: PDM_SEND
 *
 * Description:
 *   Send a block of data on PDM.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer completes.
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

#define PDM_SEND(d,b,c,a,t) ((d)->ops->pdm_send(d,b,c,a,t))

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* Transfer complete callbacks */

struct pdm_dev_s;
typedef CODE void (*pdm_callback_t)(FAR struct pdm_dev_s *dev,
                                    FAR struct ap_buffer_s *apb, FAR void *arg, int result);

/* The PDM vtable */

struct pdm_ops_s
{
  /* Receiver methods */

  CODE uint32_t (*pdm_rxsamplerate)(FAR struct pdm_dev_s *dev, uint32_t rate);
  CODE uint32_t (*pdm_rxdatawidth)(FAR struct pdm_dev_s *dev, int bits);
  CODE uint32_t (*pdm_rxchannel)(FAR struct pdm_dev_s *dev, int channel);
  CODE int      (*pdm_receive)(FAR struct pdm_dev_s *dev,
                               FAR struct ap_buffer_s *apb, pdm_callback_t callback,
                               FAR void *arg, uint32_t timeout);

  /* Transmitter methods */

  CODE uint32_t (*pdm_txsamplerate)(FAR struct pdm_dev_s *dev, uint32_t rate);
  CODE uint32_t (*pdm_txdatawidth)(FAR struct pdm_dev_s *dev, int bits);
  CODE uint32_t (*pdm_txchannel)(FAR struct pdm_dev_s *dev, int channel);
  CODE int      (*pdm_send)(FAR struct pdm_dev_s *dev,
                            FAR struct ap_buffer_s *apb, pdm_callback_t callback,
                            FAR void *arg, uint32_t timeout);
};

/* PDM private data.  This structure only defines the initial fields of the
 * structure visible to the PDM client.  The specific implementation may
 * add additional, device specific fields
 */

struct pdm_dev_s
{
  FAR const struct pdm_ops_s *ops;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PDM */
#endif /* __INCLUDE_NUTTX_AUDIO_PDM_H */

