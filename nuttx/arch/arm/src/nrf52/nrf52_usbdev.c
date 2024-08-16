/****************************************************************************
 * arch/arm/src/nrf52/nrf52_usbdev.c
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
 *   Copyright (C) 2018 Zglue Inc. All rights reserved.
 *   Author: Levin Li <zhiqiang@zglue.com>
 *
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>

#include "nvic.h"
#include "cache.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "nrf.h"
#include "nrf_usbd.h"
#include "nrfx_usbd_errata.h"
#include "nrf52_usbdev.h"
#include "nrf52_power.h"
#include "nrf52_clock.h"

#if defined(CONFIG_USBDEV) && defined(CONFIG_NRF52_USBD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
//#define nrf52_trace    uwarn
#define nrf52_trace    uinfo

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_USBDEV_SETUP_MAXDATASIZE
#define CONFIG_USBDEV_SETUP_MAXDATASIZE CONFIG_USBDEV_EP0_MAXSIZE
#endif

#ifndef CONFIG_USB_PRI
#define CONFIG_USB_PRI NVIC_SYSH_MAXNORMAL_PRIORITY
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_USB_INFO
#undef CONFIG_NRF52_USBDEV_REGDEBUG
#endif


/* Endpoint identifiers. The NRF52 supports up to 18 mono-directional or 9
 * bidirectional endpoints. The fact that EP0 is bidirectional, then there is
 * 2 control (1 IN , 1 OUT), 14 bulk/interrupt (7 IN , 7 OUT) and
 * 2 ISO (1 IN, 1 OUT) endpoints
 * a functional limitation of EP0 + 7 bi-directional endpoints + ISO = 9.
 * We'll define NRF52_NENDPOINTS to be 18 for generic EP.
 * 0 is for special ctrl, 8 is for special isoc
 * however, because that is how many
 * endpoint register sets there are.
 */

/* they are spitted into 9 end point group
 * 0,1 | 1,2 | 3,4 | 5,6 | 7,8 |......
 * EP0 | EP1 | EP2 | EP3 | EP4 |......
 */
#define NRF52_NENDPOINTS      (NRF_USBD_EPOUT_CNT + NRF_USBD_EPIN_CNT)

#define EP0                   0
#define EP0_OUT               0
#define EP0_IN                1
#define EPISOC                8

#define NRF52_ENDP_BIT(ep)    (1 << (ep))
#define NRF52_ENDP_ALLSET     (0x3FFFF)

/* Packet sizes.  We us a fixed 64 max packet size for all bulk endpoint types */

#define NRF52_MAXPACKET_SHIFT (6)
#define NRF52_MAXPACKET_SIZE  (1 << (NRF52_MAXPACKET_SHIFT))
#define NRF52_MAXPACKET_MASK  (NRF52_MAXPACKET_SIZE-1)

#define NRF52_EP0MAXPACKET    NRF52_MAXPACKET_SIZE

#define NRF52_EP8MAXPACKET    512

/* USB-related masks */

#define REQRECIPIENT_MASK     (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

#define NRF52_USBD_INT_ALL_MASK  (0xFFFFFFFFUL)


/* Request queue operations *************************************************/

#define nrf52_reqempty(ep)     ((ep)->head == NULL)
#define nrf52_reqpeek(ep)      ((ep)->head)

/* USB trace ****************************************************************/
/* Trace error codes */

#define NRF52_TRACEERR_ALLOCFAIL            0x0001
#define NRF52_TRACEERR_BADCLEARFEATURE      0x0002
#define NRF52_TRACEERR_BADDEVGETSTATUS      0x0003
#define NRF52_TRACEERR_BADEPGETSTATUS       0x0004
#define NRF52_TRACEERR_BADEPNO              0x0005
#define NRF52_TRACEERR_BADEPTYPE            0x0006
#define NRF52_TRACEERR_BADGETCONFIG         0x0007
#define NRF52_TRACEERR_BADGETSETDESC        0x0008
#define NRF52_TRACEERR_BADGETSTATUS         0x0009
#define NRF52_TRACEERR_BADSETADDRESS        0x000a
#define NRF52_TRACEERR_BADSETCONFIG         0x000b
#define NRF52_TRACEERR_BADSETFEATURE        0x000c
#define NRF52_TRACEERR_BINDFAILED           0x000d
#define NRF52_TRACEERR_DISPATCHSTALL        0x000e
#define NRF52_TRACEERR_DRIVER               0x000f
#define NRF52_TRACEERR_DRIVERREGISTERED     0x0010
#define NRF52_TRACEERR_EP0BADCTR            0x0011
#define NRF52_TRACEERR_EP0SETUPSTALLED      0x0012
#define NRF52_TRACEERR_EPBUFFER             0x0013
#define NRF52_TRACEERR_EPDISABLED           0x0014
#define NRF52_TRACEERR_EPOUTNULLPACKET      0x0015
#define NRF52_TRACEERR_EPRESERVE            0x0016
#define NRF52_TRACEERR_INVALIDCTRLREQ       0x0017
#define NRF52_TRACEERR_INVALIDPARMS         0x0018
#define NRF52_TRACEERR_IRQREGISTRATION      0x0019
#define NRF52_TRACEERR_NOTCONFIGURED        0x001a
#define NRF52_TRACEERR_REQABORTED           0x001b

/* Trace interrupt codes */

#define NRF52_TRACEINTID_CLEARFEATURE       0x0001
#define NRF52_TRACEINTID_DEVGETSTATUS       0x0002
#define NRF52_TRACEINTID_DISPATCH           0x0003
#define NRF52_TRACEINTID_EP0IN              0x0004
#define NRF52_TRACEINTID_EP0INDONE          0x0005
#define NRF52_TRACEINTID_EP0OUTDONE         0x0006
#define NRF52_TRACEINTID_EP0SETUPDONE       0x0007
#define NRF52_TRACEINTID_EP0SETUPSETADDRESS 0x0008
#define NRF52_TRACEINTID_EPGETSTATUS        0x0009
#define NRF52_TRACEINTID_EPINDONE           0x000a
#define NRF52_TRACEINTID_EPINQEMPTY         0x000b
#define NRF52_TRACEINTID_EPOUTDONE          0x000c
#define NRF52_TRACEINTID_EPOUTPENDING       0x000d
#define NRF52_TRACEINTID_EPOUTQEMPTY        0x000e
#define NRF52_TRACEINTID_ESOF               0x000f
#define NRF52_TRACEINTID_GETCONFIG          0x0010
#define NRF52_TRACEINTID_GETSETDESC         0x0011
#define NRF52_TRACEINTID_GETSETIF           0x0012
#define NRF52_TRACEINTID_GETSTATUS          0x0013
#define NRF52_TRACEINTID_EPINTERRUPT        0x0014
#define NRF52_TRACEINTID_IFGETSTATUS        0x0015
#define NRF52_TRACEINTID_LPCTR              0x0016
#define NRF52_TRACEINTID_RSTINTERRUPT       0x0017
#define NRF52_TRACEINTID_NOSTDREQ           0x0018
#define NRF52_TRACEINTID_RESET              0x0019
#define NRF52_TRACEINTID_SETCONFIG          0x001a
#define NRF52_TRACEINTID_SETFEATURE         0x001b
#define NRF52_TRACEINTID_SUSP               0x001c
#define NRF52_TRACEINTID_SYNCHFRAME         0x001d
#define NRF52_TRACEINTID_WKUP               0x001e
#define NRF52_TRACEINTID_EP0SETUPOUT        0x001f
#define NRF52_TRACEINTID_EP0SETUPOUTDATA    0x0020
#define NRF52_TRACEINTID_SETADDR            0x0021

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* The various states of a control pipe */

enum nrf52_ep0state_e
{
  EP0STATE_IDLE = 0,        /* No request in progress */
  EP0STATE_RESPONSE,        /* response setup data */
  EP0STATE_SETUP_OUT,       /* Set up recived with data for device OUT in progress */
  EP0STATE_SETUP_READY,     /* Set up was recived prior and is in ctrl,
                             * now the data has arrived */
  EP0STATE_WRREQUEST,       /* Write request in progress */
  EP0STATE_RDREQUEST,       /* Read request in progress */
  EP0STATE_STALLED          /* We are stalled */
};

/* Resume states */

enum nrf52_rsmstate_e
{
  RSMSTATE_IDLE = 0,        /* Device is either fully suspended or running */
  RSMSTATE_STARTED,         /* Resume sequence has been started */
  RSMSTATE_WAITING          /* Waiting (on ESOFs) for end of sequence */
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

/* A container for a request so that the request make be retained in a list */

struct nrf52_req_s
{
  struct usbdev_req_s    req;           /* Standard USB request */
  struct nrf52_req_s    *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct nrf52_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct nrf52_ep_s.
   */

  struct usbdev_ep_s     ep;            /* Standard endpoint structure */

  /* NRF52-specific fields */

  struct nrf52_usbdev_s *dev;           /* Reference to private driver data */
  struct nrf52_req_s    *head;          /* Request list for this endpoint */
  struct nrf52_req_s    *tail;
  uint8_t                eppos;         /* EP position in eplist */
  uint8_t                eptype;        /* USB_EP_ATTR_XFER_INT | BULK | ISOC */
  uint8_t                stalled: 1;    /* true: Endpoint is stalled */
  uint8_t                busy: 1;       /* true: endpoint is transfering */
  uint8_t                txnullpkt: 1;  /* Null packet needed at end of transfer */
  uint8_t                xfer: 1;       /* true: need to update xfer point */
  uint16_t               req_len;       /* the out transaction length from host */
};

/*
 * only PWR_READY , Suspend , resume can access usbd register
 */
enum phy_status
{
  NRF52_USBPHY_STATUS_UNINIT = 0,
  NRF52_USBPHY_STATUS_INITIALIZED,
  NRF52_USBPHY_STATUS_DETECT,  /* VBUS is ready to enable USBD Phy */
  NRF52_USBPHY_STATUS_REMOVE,  /* VBUS is lost, USBD should be disabled under this event */
  NRF52_USBPHY_STATUS_PWR_READY, /* PWR is ready to enable usbd on usb bus (pullup) */
  NRF52_USBPHY_STATUS_SUSPEND,
  NRF52_USBPHY_STATUS_RESUME,
};

enum ep_status
{
  NRF52_EP_STATUS_IDLE = 0,
  NRF52_EP_STATUS_INPROGRESS,
};

enum phy_connect
{
  NRF52_DEV_DISCONNECT = 0,     /* pull up  is not set */
  NRF52_DEV_CONNECTED,       /* pullup is set */
  NRF52_DEV_CONNECTING,     /* pullup is waiting to setup */
};

struct nrf52_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structnrf52_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* NRF52-specific fields */

  /* indicate the phy status , only one EP transfer for one time */
  enum ep_status         epstatus;

  uint8_t                ep0dir;        /*true : in ; false : out*/
  uint8_t                ep0state;      /* State of EP0 (see enum nrf52_ep0state_e) */
  uint8_t                rsmstate;      /* Resume state (see enum nrf52_rsmstate_e) */
  uint8_t                nesofs;        /* ESOF counter (for resume support) */
  uint8_t                selfpowered: 1; /* 1: Device is self powered */
  uint8_t                response: 1;   /* 1: indicate there is response data for setup */
  enum phy_status        phystatus;     /* usb phy power status */
  enum phy_connect       connect;       /* usb phy power status */
  uint32_t               epavail;       /* Bitset of available endpoints */
  uint32_t               imask;         /* Current interrupt mask */

  /* E0 SETUP data buffering.
   *
   * ctrl
   *   The 8-byte SETUP request is received on the EP0 OUT endpoint and is
   *   saved.
   *
   * ep0data
   *   For OUT SETUP requests, the SETUP data phase must also complete before
   *   the SETUP command can be processed.  The ep0 packet receipt logic
   *   nrf52_ep0_rdrequest will save the accompanying EP0 OUT data in
   *   ep0data[] before the SETUP command is re-processed.
   *
   * ep0datlen
   *   Lenght of OUT DATA received in ep0data[]
   */

  struct usb_ctrlreq_s   ctrl;          /* Last EP0 request */

  uint8_t                ep0data[CONFIG_USBDEV_SETUP_MAXDATASIZE];
  uint16_t               ep0datalen;

  /* The endpoint list */
  struct nrf52_ep_s      eplist[NRF52_NENDPOINTS];
  pthread_t              monitor;
  sem_t                  sem;
  uint32_t               out_pack;  /* debug : OUT package number except ctrl */
  uint32_t               out_cnt;
  uint32_t               in_pack;   /* debug : IN package number except ctrl */
  uint32_t               in_cnt;
};

/* epdatastatus : bit 23...17  out;  bit 7...2 in */
#define NRF52_EPOUT_DONE_MASK   0xFE0000UL
#define NRF52_EPIN_DONE_MASK    0xFE

#define NRF52_DEFAULT_INT      ( NRF_USBD_INT_USBRESET_MASK     | \
                                 0      | \
                                 NRF_USBD_INT_EP0DATADONE_MASK  | \
                                 NRF_USBD_INT_ENDEPIN0_MASK     | \
                                 NRF_USBD_INT_ENDEPOUT0_MASK    | \
                                 NRF_USBD_INT_USBEVENT_MASK     | \
                                 NRF_USBD_INT_EP0SETUP_MASK     | \
                                 NRF_USBD_INT_DATAEP_MASK       | \
                                 NRF_USBD_INT_ACCESSFAULT_MASK)
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-Level Helpers ********************************************************/


static uint8_t nrf52_index2eplog(uint8_t index);
static uint8_t nrf52_eplog2index(uint8_t eplog);

/* Suspend/Resume Helpers ***************************************************/

static void   nrf52_suspend(struct nrf52_usbdev_s *priv);
static void   nrf52_initresume(struct nrf52_usbdev_s *priv, bool self_wakeup);
static void   nrf52_esofpoll(struct nrf52_usbdev_s *priv) ;

/* Request Helpers **********************************************************/

static struct nrf52_req_s *nrf52_rqdequeue(struct nrf52_ep_s *privep);
static void   nrf52_rqenqueue(struct nrf52_ep_s *privep,
                              struct nrf52_req_s *req);
static inline void
nrf52_abortrequest(struct nrf52_ep_s *privep,
                   struct nrf52_req_s *privreq, int16_t result);
static void   nrf52_reqcomplete(struct nrf52_ep_s *privep, int16_t result);
static void   nrf52_ep_rw_setup(struct nrf52_usbdev_s *priv,
                                struct nrf52_ep_s *privep, const uint8_t *data, uint32_t nbytes);
static int    nrf52_rw_request(struct nrf52_usbdev_s *priv,
                               struct nrf52_ep_s *privep);

static void   nrf52_cancelrequests(struct nrf52_ep_s *privep);

/* Interrupt level processing ***********************************************/
static void nrf52_ep0setup(struct nrf52_usbdev_s *priv);

static void   nrf52_epdone(struct nrf52_usbdev_s *priv, uint8_t epno);
static int    nrf52_usbd_interrupt(int irq, void *context, FAR void *arg);

/* Endpoint helpers *********************************************************/

static inline bool
nrf52_epreserved(struct nrf52_usbdev_s *priv, int epno);

static bool nrf52_rw_update_xfer(struct nrf52_ep_s *privep);

/* Endpoint operations ******************************************************/

static int  nrf52_epconfigure(struct usbdev_ep_s *ep,
                              const struct usb_epdesc_s *desc, bool last);
static int  nrf52_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *nrf52_epallocreq(struct usbdev_ep_s *ep);
static void nrf52_epfreereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req);
#ifdef CONFIG_USBDEV_DMA
static void *nrf52_epallocbuffer(FAR struct usbdev_ep_s *ep, uint16_t bytes);
static void nrf52_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif
static int  nrf52_epsubmit(struct usbdev_ep_s *ep, struct usbdev_req_s *req);
static int  nrf52_epcancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req);
static int  nrf52_epstall(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations *****************************************/

static struct usbdev_ep_s *
nrf52_allocep(struct usbdev_s *dev, uint8_t epno, bool in, uint8_t eptype);
static void nrf52_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int  nrf52_pullup(FAR struct usbdev_s *dev,  bool enable);
static int  nrf52_getframe(struct usbdev_s *dev);
static int  nrf52_wakeup(struct usbdev_s *dev);
static int  nrf52_selfpowered(struct usbdev_s *dev, bool selfpowered);

/* Initialization/Reset *****************************************************/

static void nrf52_reset(struct nrf52_usbdev_s *priv, bool hwreset);
static void nrf52_hwreset(struct nrf52_usbdev_s *priv);
static void nrf52_hwsetup(struct nrf52_usbdev_s *priv);
static void nrf52_hwshutdown(struct nrf52_usbdev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct nrf52_usbdev_s g_usbddev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = nrf52_epconfigure,
  .disable     = nrf52_epdisable,
  .allocreq    = nrf52_epallocreq,
  .freereq     = nrf52_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = nrf52_epallocbuffer,
  .freebuffer  = nrf52_epfreebuffer,
#endif
  .submit      = nrf52_epsubmit,
  .cancel      = nrf52_epcancel,
  .stall       = nrf52_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = nrf52_allocep,
  .freeep      = nrf52_freeep,
  .getframe    = nrf52_getframe,
  .wakeup      = nrf52_wakeup,
  .selfpowered = nrf52_selfpowered,
  .pullup      = nrf52_pullup,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(NRF52_TRACEINTID_CLEARFEATURE       ),
  TRACE_STR(NRF52_TRACEINTID_DEVGETSTATUS       ),
  TRACE_STR(NRF52_TRACEINTID_DISPATCH           ),
  TRACE_STR(NRF52_TRACEINTID_EP0IN              ),
  TRACE_STR(NRF52_TRACEINTID_EP0INDONE          ),
  TRACE_STR(NRF52_TRACEINTID_EP0OUTDONE         ),
  TRACE_STR(NRF52_TRACEINTID_EP0SETUPDONE       ),
  TRACE_STR(NRF52_TRACEINTID_EP0SETUPSETADDRESS ),
  TRACE_STR(NRF52_TRACEINTID_EPGETSTATUS        ),
  TRACE_STR(NRF52_TRACEINTID_EPINDONE           ),
  TRACE_STR(NRF52_TRACEINTID_EPINQEMPTY         ),
  TRACE_STR(NRF52_TRACEINTID_EPOUTDONE          ),
  TRACE_STR(NRF52_TRACEINTID_EPOUTPENDING       ),
  TRACE_STR(NRF52_TRACEINTID_EPOUTQEMPTY        ),
  TRACE_STR(NRF52_TRACEINTID_ESOF               ),
  TRACE_STR(NRF52_TRACEINTID_GETCONFIG          ),
  TRACE_STR(NRF52_TRACEINTID_GETSETDESC         ),
  TRACE_STR(NRF52_TRACEINTID_GETSETIF           ),
  TRACE_STR(NRF52_TRACEINTID_GETSTATUS          ),
  TRACE_STR(NRF52_TRACEINTID_EPINTERRUPT        ),
  TRACE_STR(NRF52_TRACEINTID_IFGETSTATUS        ),
  TRACE_STR(NRF52_TRACEINTID_LPCTR              ),
  TRACE_STR(NRF52_TRACEINTID_RSTINTERRUPT        ),
  TRACE_STR(NRF52_TRACEINTID_NOSTDREQ           ),
  TRACE_STR(NRF52_TRACEINTID_RESET              ),
  TRACE_STR(NRF52_TRACEINTID_SETCONFIG          ),
  TRACE_STR(NRF52_TRACEINTID_SETFEATURE         ),
  TRACE_STR(NRF52_TRACEINTID_SUSP               ),
  TRACE_STR(NRF52_TRACEINTID_SYNCHFRAME         ),
  TRACE_STR(NRF52_TRACEINTID_WKUP               ),
  TRACE_STR(NRF52_TRACEINTID_EP0SETUPOUT        ),
  TRACE_STR(NRF52_TRACEINTID_EP0SETUPOUTDATA    ),
  TRACE_STR(NRF52_TRACEINTID_SETADDR            ),
  TRACE_STR_END
};
#endif

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(NRF52_TRACEERR_ALLOCFAIL            ),
  TRACE_STR(NRF52_TRACEERR_BADCLEARFEATURE      ),
  TRACE_STR(NRF52_TRACEERR_BADDEVGETSTATUS      ),
  TRACE_STR(NRF52_TRACEERR_BADEPGETSTATUS       ),
  TRACE_STR(NRF52_TRACEERR_BADEPNO              ),
  TRACE_STR(NRF52_TRACEERR_BADEPTYPE            ),
  TRACE_STR(NRF52_TRACEERR_BADGETCONFIG         ),
  TRACE_STR(NRF52_TRACEERR_BADGETSETDESC        ),
  TRACE_STR(NRF52_TRACEERR_BADGETSTATUS         ),
  TRACE_STR(NRF52_TRACEERR_BADSETADDRESS        ),
  TRACE_STR(NRF52_TRACEERR_BADSETCONFIG         ),
  TRACE_STR(NRF52_TRACEERR_BADSETFEATURE        ),
  TRACE_STR(NRF52_TRACEERR_BINDFAILED           ),
  TRACE_STR(NRF52_TRACEERR_DISPATCHSTALL        ),
  TRACE_STR(NRF52_TRACEERR_DRIVER               ),
  TRACE_STR(NRF52_TRACEERR_DRIVERREGISTERED     ),
  TRACE_STR(NRF52_TRACEERR_EP0BADCTR            ),
  TRACE_STR(NRF52_TRACEERR_EP0SETUPSTALLED      ),
  TRACE_STR(NRF52_TRACEERR_EPBUFFER             ),
  TRACE_STR(NRF52_TRACEERR_EPDISABLED           ),
  TRACE_STR(NRF52_TRACEERR_EPOUTNULLPACKET      ),
  TRACE_STR(NRF52_TRACEERR_EPRESERVE            ),
  TRACE_STR(NRF52_TRACEERR_INVALIDCTRLREQ       ),
  TRACE_STR(NRF52_TRACEERR_INVALIDPARMS         ),
  TRACE_STR(NRF52_TRACEERR_IRQREGISTRATION      ),
  TRACE_STR(NRF52_TRACEERR_NOTCONFIGURED        ),
  TRACE_STR(NRF52_TRACEERR_REQABORTED           ),
  TRACE_STR_END
};
#endif

/****************************************************************************
 * Private Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_dispatchrequest
 ****************************************************************************/

static void nrf52_dispatchrequest(struct nrf52_usbdev_s *priv)
{
  int ret;

  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        priv->ep0data, priv->ep0datalen);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_DISPATCHSTALL), 0);
          priv->ep0state = EP0STATE_STALLED;
        }
    }
}

static uint32_t nrf52_dma_flag ;
/****************************************************************************
 * Low-Level Helpers
 ****************************************************************************/
void nrf52_usbd_package_clear(void)
{
  g_usbddev.out_pack = 0;
  g_usbddev.out_cnt  = 0;
  g_usbddev.in_pack  = 0;
  g_usbddev.in_cnt   = 0;
}

uint32_t nrf52_usbd_get_out_pack(void)
{
  return g_usbddev.out_pack;
}

uint32_t nrf52_usbd_get_out_data_len(void)
{
  return g_usbddev.out_cnt;
}

uint32_t nrf52_usbd_get_in_pack(void)
{
  return g_usbddev.in_pack;
}

uint32_t nrf52_usbd_get_in_data_len(void)
{
  return g_usbddev.in_cnt;
}

/**
 * @brief Mark that EasyDMA is working.
 *
 * Internal function to set the flag informing about EasyDMA transfer pending.
 * This function is called always just after the EasyDMA transfer is started.
 */
static inline void nrf52_usbd_dma_pending_set(void)
{
  if (nrfx_usbd_errata_199())
    {
      *((volatile uint32_t *)0x40027C1C) = 0x00000082;
    }
  nrf52_dma_flag = true;
}

/**
 * @brief Mark that EasyDMA is free.
 *
 * Internal function to clear the flag informing about EasyDMA transfer pending.
 * This function is called always just after the finished EasyDMA transfer is detected.
 */
static inline void nrf52_usbd_dma_pending_clear(void)
{
  if (nrfx_usbd_errata_199())
    {
      *((volatile uint32_t *)0x40027C1C) = 0x00000000;
    }
  nrf52_dma_flag = false;
}

static void nrf52_usbd_transfer_out_drop(uint8_t ep)
{
  if (nrfx_usbd_errata_200())
    {
      *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7C5 + (2u * NRF_USBD_EP_NR_GET(ep));
      *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = 0;
      (void)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
    }
  else
    {
      if (!NRF_USBD_EPISO_CHECK(ep))
        {
          nrf_usbd_epout_clear(ep);
        }
    }
}

static void nrf52_usbd_setup_data_clear(void)
{
  if (nrfx_usbd_errata_104())
    {
      /* For this fix to work properly, it must be ensured that the task is
       * executed twice one after another - blocking ISR. This is however a temporary
       * solution to be used only before production version of the chip. */
      nrf_usbd_task_trigger(NRF_USBD_TASK_EP0RCVOUT);
      nrf_usbd_task_trigger(NRF_USBD_TASK_EP0RCVOUT);
    }
  else
    {
      nrf_usbd_task_trigger(NRF_USBD_TASK_EP0RCVOUT);
    }
}

static inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );

  return (result);
}

/****************************************************************************
 * Name: nrf52_setimask
 ****************************************************************************/

static void
nrf52_setimask(struct nrf52_usbdev_s *priv, uint32_t setbits, uint32_t clrbits)
{

  /* Adjust the interrupt mask bits in the shadow copy first */

  priv->imask &= ~clrbits;
  priv->imask |= setbits;

  /* Then make the interrupt mask bits in the INTENSET / INTENCLR register
   */

  nrf_usbd_int_enable(setbits);
  nrf_usbd_int_disable(clrbits);

}

/****************************************************************************
 * Name: nrf52_ep_is_stalled
 ****************************************************************************/

static inline bool nrf52_ep_is_stalled(uint8_t eplog)
{
  return nrf_usbd_ep_is_stall(eplog);
}

/****************************************************************************
 * Name: nrf52_rqdequeue
 ****************************************************************************/

static struct nrf52_req_s *nrf52_rqdequeue(struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *ret = privep->head;

  if (ret)
    {
      privep->head = ret->flink;
      if (!privep->head)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_rqenqueue
 ****************************************************************************/

static void nrf52_rqenqueue(struct nrf52_ep_s *privep, struct nrf52_req_s *req)
{
  req->flink = NULL;
  if (!privep->head)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }
}

/****************************************************************************
 * Name: nrf52_abortrequest
 ****************************************************************************/

static inline void
nrf52_abortrequest(struct nrf52_ep_s *privep, struct nrf52_req_s *privreq, int16_t result)
{
  usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_REQABORTED), (uint16_t)USB_EPNO(privep->ep.eplog));

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/****************************************************************************
 * Name: nrf52_reqcomplete
 ****************************************************************************/

static void nrf52_reqcomplete(struct nrf52_ep_s *privep, int16_t result)
{
  struct nrf52_req_s *privreq;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = enter_critical_section();
  privreq = nrf52_rqdequeue(privep);
  leave_critical_section(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      bool stalled = privep->stalled;
      if (USB_EPNO(privep->ep.eplog) == EP0)
        {
          privep->stalled = (privep->dev->ep0state == EP0STATE_STALLED);
        }

      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->req.callback(&privep->ep, &privreq->req);

      /* Restore the stalled indication */

      privep->stalled = stalled;
    }
}

/****************************************************************************
 * Name: nrf52_epwrite
 ****************************************************************************/

static void nrf52_ep_rw_setup(struct nrf52_usbdev_s *priv,
                              struct nrf52_ep_s *privep,
                              const uint8_t *buf, uint32_t nbytes)
{
  uint8_t epno = USB_EPNO(privep->ep.eplog);
  /* setup the easyDMA buffer and size and start transfer */
  nrf_usbd_task_t task;
  nrf_usbd_int_mask_t int_mask;

  usbtrace(TRACE_WRITE(epno), nbytes);

  if (nbytes > 0 || (nbytes == 0 && privep->txnullpkt))
    {

      /* set started , end interrupt */
      if (USB_ISEPOUT(privep->ep.eplog))
        {
          task = NRF_USBD_TASK_STARTEPOUT0 + epno * 4;
          int_mask = 1 << (USBD_INTEN_ENDEPOUT0_Pos + epno);
          epno |= USB_DIR_OUT;
        }
      else
        {
          task = NRF_USBD_TASK_STARTEPIN0 + epno * 4;
          int_mask = 1 << (USBD_INTEN_ENDEPIN0_Pos + epno);
          epno |= USB_DIR_IN;
        }

      /* NOT use dual buffer mode now . will be support in further
       *     int_mask |= NRF_USBD_INT_STARTED_MASK;
       */
      nrf52_usbd_dma_pending_set();
      privep->txnullpkt = 0;
      nrf_usbd_ep_easydma_set(epno, (uint32_t)buf, nbytes);

      privep->busy = true;

      nrf52_setimask(priv, int_mask, 0);
      nrf_usbd_task_trigger(task);
    }

}

/****************************************************************************
 * Interrupt Level Processing
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_wrrequest_ep0
 *
 * Description:
 *   Handle the ep0 state on writes.
 *
 ****************************************************************************/

inline static int nrf52_wrrequest_ep0(struct nrf52_usbdev_s *priv,
                                      struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *privreq = NULL;
  uint8_t *buf = NULL;
  int nbytes = 0;
  int bytesleft;
  bool one_shot = false;
  uint8_t epno = 0;
  nrf_usbd_task_t task;
  uint32_t int_mask = 0;

  /* Check the request from the head of the endpoint request queue */

  privreq = nrf52_reqpeek(privep);
  if (!privreq)
    {
      /* There is no TX transfer in progress and no new pending TX
       * requests to send.
       */

      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPINQEMPTY), 0);
      return -ENOENT;
    }

  uinfo("epno=%d req=%p: len=%d xfrd=%d nullpkt=%d\n", privep->ep.eplog, privreq,
        privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* Get the number of bytes left to be sent in the packet */

  bytesleft         = privreq->req.len - privreq->req.xfrd;
  nbytes            = bytesleft;

  /* If we are not sending a NULL packet, then clip the size to maxpacket
   * and check if we need to send a following NULL packet.
   */

  if (nbytes > privep->ep.maxpacket)
    {
      nbytes =  privep->ep.maxpacket;
    }
  else
    {
      one_shot = true;
    }

  /* Send the packet (might be a null packet nbytes == 0) */

  buf = privreq->req.buf + privreq->req.xfrd;
  /* set started , end interrupt */

  task = NRF_USBD_TASK_STARTEPIN0;
  int_mask = 1 << (USBD_INTEN_ENDEPIN0_Pos);
  epno |= USB_DIR_IN;

  nrf52_usbd_dma_pending_set();
  nrf_usbd_ep_easydma_set(epno, (uint32_t)buf, nbytes);

  if (one_shot)
    {
      nrf52_trace("one shot: len %d\n", nbytes);
      nrf_usbd_shorts_enable(NRF_USBD_SHORT_EP0DATADONE_EP0STATUS_MASK);
      nrf52_setimask(priv, int_mask, 0);
      nrf_usbd_task_trigger(task);
      privreq->req.xfrd = nbytes;
      nrf52_reqcomplete(privep, OK);
      priv->ep0state = EP0STATE_IDLE;

    }
  else
    {
      nrf52_trace("NOT one shot\n");
      nrf52_setimask(priv, int_mask, 0);
      nrf_usbd_task_trigger(task);
      priv->ep0state = EP0STATE_WRREQUEST;
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_ep0setup_status
 * epno: the logic no which includes direction attribute
 ****************************************************************************/
static void nrf52_ep0setup_status(void)
{
  nrf_usbd_task_trigger(NRF_USBD_TASK_EP0STATUS);
}

/****************************************************************************
 * Name: nrf52_ep0out_end
 ****************************************************************************/
static inline void nrf52_ep0out_end(struct nrf52_usbdev_s *priv)
{
  uint32_t size;

  size = nrf_usbd_epout_size_get(USB_DIR_OUT);
  nrf52_trace("ep0 out size: %d., status %d\n", size, priv->ep0state);

  priv->ep0state = EP0STATE_SETUP_READY;
  nrf52_ep0setup(priv);

  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0OUTDONE), 0xF);
}

/****************************************************************************
 * Name: nrf52_ep0in_datadone
 ****************************************************************************/
static inline void nrf52_ep0in_datadone(struct nrf52_usbdev_s *priv)
{

  if (priv->ep0dir != true)
    {
      nrf52_trace("only accept IN epdatadone event\n");
      return;
    }

  /* EP0 IN: device-to-host (DIR=0) */

  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0INDONE), 0);

  priv->eplist[EP0_IN].busy = false;

  if (priv->ep0state == EP0STATE_WRREQUEST)
    {
      /* Are we processing the completion of one packet of an outgoing request
       * from the class driver?
       */

      if (NULL == nrf52_reqpeek(&priv->eplist[EP0_IN]))
        {
          nrf52_ep0setup_status();

          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0INDONE), 2);
          priv->ep0state = EP0STATE_IDLE;
        }
      else
        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0INDONE), 3);
          /* update write data point */
          nrf52_rw_update_xfer(&priv->eplist[EP0_IN]);
          nrf52_wrrequest_ep0(priv, &priv->eplist[EP0_IN]);
        }
    }

}

/****************************************************************************
 * Name: nrf52_rdrequest_ep0
 *
 * Description:
 *   This function is called from the nrf52_ep0_setup handler when the ep0state
 *   is EP0STATE_SETUP_OUT and it will setup EP0 OUT data stage.
 *
 ****************************************************************************/

static void nrf52_rdrequest_ep0(struct nrf52_usbdev_s *priv, uint8_t *buf, uint32_t len)
{
  uint8_t epno = 0 | USB_DIR_OUT;

  ASSERT(priv->ep0state == EP0STATE_SETUP_OUT);

  /* setup easydma & start data stage on OUT traction : 52840 Page: 489
   * use shortcut to start OUT0 transfer, and ENDOUT0 will be generated
   */
  nrf52_usbd_dma_pending_set();
  nrf52_usbd_transfer_out_drop(USB_DIR_OUT);

  nrf_usbd_ep_easydma_set(epno, (uint32_t)buf, len);

  nrf52_setimask(priv, USBD_INTEN_ENDEPOUT0_Pos, 0);

  nrf_usbd_shorts_enable(NRF_USBD_SHORT_EP0DATADONE_STARTEPOUT0_MASK);

  nrf52_usbd_setup_data_clear();
}

static void nrf52_readsetup(struct usb_ctrlreq_s *ctrl)
{
  uint16_t data;

  ctrl->type     = nrf_usbd_setup_bmrequesttype_get();
  ctrl->req      = nrf_usbd_setup_brequest_get();

  data           = nrf_usbd_setup_wvalue_get();
  ctrl->value[0] = data & 0xFF;
  ctrl->value[1] = (data >> 8) & 0xFF;

  data           = nrf_usbd_setup_windex_get();
  ctrl->index[0] = data & 0xFF;
  ctrl->index[1] = (data >> 8) & 0xFF;

  data           = nrf_usbd_setup_wlength_get();
  ctrl->len[0]   = data & 0xFF;
  ctrl->len[1]   = (data >> 8) & 0xFF;

}

/****************************************************************************
 * Name: nrf52_ep0setup
 ****************************************************************************/
static void nrf52_setup_response_in(struct nrf52_usbdev_s *priv, uint8_t *buf, uint32_t len)
{

  nrf52_usbd_dma_pending_set();
  nrf_usbd_ep_easydma_set(USB_DIR_IN, (uint32_t)buf, len);

  nrf_usbd_shorts_enable(NRF_USBD_SHORT_EP0DATADONE_EP0STATUS_MASK);

  /* on the end interrupt to clear dma_pending */
  nrf52_setimask(priv, USBD_INTEN_ENDEPIN0_Pos, 0);
  nrf_usbd_task_trigger(NRF_USBD_TASK_STARTEPIN0);

}

static void nrf52_ep0setup(struct nrf52_usbdev_s *priv)
{

  struct nrf52_ep_s   *ep0 = NULL;
  struct nrf52_req_s  *privreq = NULL;
  struct nrf52_ep_s   *privep = NULL;
  uint16_t            value;
  uint16_t            index;
  uint16_t            len;
  uint8_t             response[2];
  bool                handled = false;
  uint8_t             epno;
  int                 nbytes = 0; /* Assume zero-length packet */

  /* disable short cut firstly */
  nrf_usbd_shorts_disable(0xFFFFUL);

  /* Check to see if called from the DATA phase of a SETUP Transfer */

  if (priv->ep0state != EP0STATE_SETUP_READY)
    {
      /* Now, extract setup command data */

      nrf52_readsetup(&priv->ctrl);

      value = GETUINT16(priv->ctrl.value);
      index = GETUINT16(priv->ctrl.index);
      len   = GETUINT16(priv->ctrl.len);

      nrf52_trace("SETUP: type=%02x req=%02x value=%04x index=%04x len=%#04x\n",
                  priv->ctrl.type, priv->ctrl.req, value, index, len);

      /* Is this an setup with OUT */

      if (USB_REQ_ISOUT(priv->ctrl.type))
        {

          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0SETUPOUT), len);

          /* At this point priv->ctrl is the setup packet. */
          ep0     = &priv->eplist[EP0_OUT];

          priv->ep0dir = false;
          /*  we need to setup ep0 data transfer here */
          priv->ep0datalen = len;

          if (len > 0)
            {
              priv->ep0state = EP0STATE_SETUP_OUT;

              nrf52_rdrequest_ep0(priv, priv->ep0data, len);
              return;
            }
          else
            {
              priv->ep0state = EP0STATE_SETUP_READY;
            }
        }
      else
        {
          ep0     = &priv->eplist[EP0_IN];
          priv->ep0dir = true;
          priv->ep0state = EP0STATE_SETUP_READY;
        }
    }
  else
    {
      nrf52_trace("ep0state is SETUP_READY, ep0dir %s.\n", priv->ep0dir ? "IN" : "OUT");
      nrf52_trace("ep0_OUT : %#x, ep0_IN : %#x.\n", priv->eplist[EP0_OUT].ep.eplog,
                  priv->eplist[EP0_IN].ep.eplog);

      if (false == priv->ep0dir)
        {
          ep0 = &priv->eplist[EP0_OUT];
        }
      else
        {
          ep0 = &priv->eplist[EP0_IN];
        }
    }

  /* Terminate any pending requests (doesn't work if the pending request
   * was a zero-length transfer!)
   */
  privreq = nrf52_reqpeek(ep0);

  while (!nrf52_reqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->ep.eplog), privreq->req.xfrd);
      nrf52_reqcomplete(ep0, result);
    }

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled  = 0;
  ep0->busy   = 0;

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_NOSTDREQ), priv->ctrl.type);

      /* Let the class implementation handle all non-standar requests */
      nrf52_dispatchrequest(priv);
      return;
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */
  response[0] = 0;
  response[1] = 0;

  switch (priv->ctrl.req)
    {
      case USB_REQ_GETSTATUS:
        {
          /* type:  device-to-host; recipient = device, interface, endpoint
           * value: 0
           * index: zero interface endpoint
           * len:   2; data = status
           */

          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_GETSTATUS), priv->ctrl.type);
          if (len != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
              index != 0 || value != 0)
            {
              usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADEPGETSTATUS), 0);
              priv->ep0state = EP0STATE_STALLED;
            }
          else
            {
              switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
                {
                  case USB_REQ_RECIPIENT_ENDPOINT:
                    {
                      epno = USB_EPNO(index);
                      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPGETSTATUS), epno);
                      if (epno >= NRF52_NENDPOINTS)
                        {
                          usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADEPGETSTATUS), epno);
                          priv->ep0state = EP0STATE_STALLED;
                        }
                      else
                        {
                          nbytes     = 2; /* Response size: 2 bytes */

                          if (nrf52_ep_is_stalled(0xFF & index))
                            {
                              /* IN Endpoint stalled */

                              response[0] = 1; /* Stalled */
                            }
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_DEVICE:
                    {
                      if (index == 0)
                        {
                          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_DEVGETSTATUS), 0);

                          /* Features:  Remote Wakeup=YES; selfpowered=? */

                          response[0] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                                        (1 << USB_FEATURE_REMOTEWAKEUP);
                          nbytes      = 2; /* Response size: 2 bytes */
                        }
                      else
                        {
                          usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADDEVGETSTATUS), 0);
                          priv->ep0state = EP0STATE_STALLED;
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_INTERFACE:
                    {
                      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_IFGETSTATUS), 0);
                      nbytes  = 2; /* Response size: 2 bytes */
                    }
                    break;

                  default:
                    {
                      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADGETSTATUS), 0);
                      priv->ep0state = EP0STATE_STALLED;
                    }
                    break;
                }
            }
        }
        break;

      case USB_REQ_CLEARFEATURE:
        {
          /* type:  host-to-device; recipient = device, interface or endpoint
           * value: feature selector
           * index: zero interface endpoint;
           * len:   zero, data = none
           */

          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_CLEARFEATURE), priv->ctrl.type);
          if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
            {
              /* Let the class implementation handle all recipients (except for the
               * endpoint recipient)
               */

              nrf52_dispatchrequest(priv);
              handled = true;
            }
          else
            {
              /* Endpoint recipient */

              epno = USB_EPNO(index);
              if (epno < NRF52_NENDPOINTS &&
                  value == USB_FEATURE_ENDPOINTHALT && len == 0)
                {
                  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_CLEARFEATURE), index);

                  if (nrf_usbd_dtoggle_get(index) != NRF_USBD_DTOGGLE_DATA0)
                    {
                      nrf_usbd_dtoggle_set(index, NRF_USBD_DTOGGLE_DATA0);
                    }

                  privep = &priv->eplist[nrf52_eplog2index(index)];
                  (void)nrf52_epstall(&privep->ep, true);
                  priv->ep0state = EP0STATE_IDLE;
                }
              else
                {
                  usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADCLEARFEATURE), 0);
                  priv->ep0state = EP0STATE_STALLED;
                }
            }
        }
        break;

      case USB_REQ_SETFEATURE:
        {
          /* type:  host-to-device; recipient = device, interface, endpoint
           * value: feature selector
           * index: zero interface endpoint;
           * len:   0; data = none
           */

          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SETFEATURE), priv->ctrl.type);
          if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) &&
              value == USB_FEATURE_TESTMODE)
            {
              /* Special case recipient=device test mode */

              uinfo("test mode: %d\n", index);
            }
          else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
            {
              /* The class driver handles all recipients except recipient=endpoint */

              nrf52_dispatchrequest(priv);
              handled = true;
            }
          else
            {
              /* Handler recipient=endpoint */

              usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SETFEATURE), index);

              epno = USB_EPNO(index);
              if (epno < NRF52_NENDPOINTS &&
                  value == USB_FEATURE_ENDPOINTHALT && len == 0)
                {
                  privep = &priv->eplist[nrf52_eplog2index(index)];
                  (void)nrf52_epstall(&privep->ep, false);
                }
              else
                {
                  usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADSETFEATURE), 0);
                  priv->ep0state = EP0STATE_STALLED;
                }
            }
        }
        break;

      case USB_REQ_SETADDRESS:
        {
          /* type:  host-to-device; recipient = device
           * value: device address
           * index: 0
           * len:   0; data = none
           */
          if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
              index  == 0 && len == 0 && value < 128)
            {
              /* usb controller had handled set-address cmd , didn't need to
               * do status stage init action
               */
              usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0SETUPSETADDRESS), value);
              priv->ep0state = EP0STATE_IDLE;
              handled = true;
            }
          else
            {
              priv->ep0state = EP0STATE_STALLED;
            }
          /* Note that setting of the device address will be deferred.  A zero-length
           * packet will be sent and the device address will be set when the zero-
           * length packet transfer completes.
           */
        }
        break;

      case USB_REQ_GETDESCRIPTOR:
      /* type:  device-to-host; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */
      case USB_REQ_SETDESCRIPTOR:
        /* type:  host-to-device; recipient = device
         * value: descriptor type and index
         * index: 0 or language ID;
         * len:   descriptor len; data = descriptor
         */

        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_GETSETDESC), priv->ctrl.type);
          if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
            {
              /* The request seems valid... let the class implementation handle it */

              nrf52_dispatchrequest(priv);
              handled = true;
            }
          else
            {
              usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADGETSETDESC), 0);
              priv->ep0state = EP0STATE_STALLED;
            }
        }
        break;

      case USB_REQ_GETCONFIGURATION:
        /* type:  device-to-host; recipient = device
         * value: 0;
         * index: 0;
         * len:   1; data = configuration value
         */

        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_GETCONFIG), priv->ctrl.type);
          if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
              value == 0 && index == 0 && len == 1)
            {
              /* The request seems valid... let the class implementation handle it */

              nrf52_dispatchrequest(priv);
              handled = true;
            }
          else
            {
              usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADGETCONFIG), 0);
              priv->ep0state = EP0STATE_STALLED;
            }
        }
        break;

      case USB_REQ_SETCONFIGURATION:
        /* type:  host-to-device; recipient = device
         * value: configuration value
         * index: 0;
         * len:   0; data = none
         */

        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SETCONFIG), priv->ctrl.type);
          if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
              index == 0 && len == 0)
            {
              /* The request seems valid... let the class implementation handle it */

              nrf52_dispatchrequest(priv);
              handled = true;
            }
          else
            {
              usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADSETCONFIG), 0);
              priv->ep0state = EP0STATE_STALLED;
            }
        }
        break;

      case USB_REQ_GETINTERFACE:
      /* type:  device-to-host; recipient = interface
       * value: 0
       * index: interface;
       * len:   1; data = alt interface
       */
      case USB_REQ_SETINTERFACE:
        /* type:  host-to-device; recipient = interface
         * value: alternate setting
         * index: interface;
         * len:   0; data = none
         */

        {
          /* Let the class implementation handle the request */

          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_GETSETIF), priv->ctrl.type);
          nrf52_dispatchrequest(priv);
          handled = true;
        }
        break;

      case USB_REQ_SYNCHFRAME:
        /* type:  device-to-host; recipient = endpoint
         * value: 0
         * index: endpoint;
         * len:   2; data = frame number
         */

        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SYNCHFRAME), 0);
        }
        break;

      default:
        {
          usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDCTRLREQ), priv->ctrl.req);
          priv->ep0state = EP0STATE_STALLED;
        }
        break;
    }

  /* At this point, the request has been handled and there are three possible
   * outcomes:
   *
   * 1. The setup request was successfully handled above and a response packet
   *    must be sent (may be a zero length packet).
   * 2. The request was successfully handled by the class implementation.  In
   *    case, the EP0 IN response has already been queued and the local variable
   *    'handled' will be set to true and ep0state != EP0STATE_STALLED;
   * 3. An error was detected in either the above logic or by the class implementation
   *    logic.  In either case, priv->state will be set EP0STATE_STALLED
   *    to indicate this case.
   *
   * NOTE: Non-standard requests are a special case.  They are handled by the
   * class implementation and this function returned early above, skipping this
   * logic altogether.
   */

  if (EP0STATE_STALLED == priv->ep0state)
    {
      nrf52_trace("Setup stalled.\n");
      nrf_usbd_task_trigger(NRF_USBD_TASK_EP0STALL);
    }

  if (priv->ep0state != EP0STATE_STALLED && !handled)
    {
      /* We will response.  First, restrict the data length to the length
       * requested in the setup packet
       */

      if (nbytes > len)
        {
          nbytes = len;
        }

      /* Send the response (might be a zero-length packet) */

      if (0 == nbytes)
        {
          /* just finish setup stage , starting status stage */
          nrf52_ep0setup_status();
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0SETUPOUTDATA), 1);
          priv->ep0state = EP0STATE_IDLE;
        }
      else
        {
          priv->ep0data[0] = response[0];
          priv->ep0data[1] = response[1];
          if (false == priv->ep0dir)
            {
              nrf52_trace("NOT in direction.\n");
            }
          nrf52_setup_response_in(priv, priv->ep0data, nbytes);

          priv->ep0state = EP0STATE_IDLE;
        }
    }
}

/****************************************************************************
 * Name: nrf52_wrrequest | nrf52_wrrequest_update_xfer
 * nrf52_rw_update_xfer : return true for continue TX , false , TX done
 ****************************************************************************/
static bool nrf52_rw_update_xfer(struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *privreq;
  uint16_t  bytesleft;
  uint16_t xfer_len;

  privreq = nrf52_reqpeek(privep);

  if (NULL == privreq)
    {
      /* NO request on queue */
      privep->busy = false;
      return false;
    }

  if (USB_ISEPOUT(privep->ep.eplog))
    {
      xfer_len = nrf_usbd_ep_amount_get(privep->ep.eplog);

      privep->busy = false;
      g_usbddev.out_cnt += xfer_len;
      privreq->req.xfrd += xfer_len;

      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);

      nrf52_reqcomplete(privep, OK);

      return false;

    }
  else
    {
      xfer_len = nrf_usbd_ep_amount_get(privep->ep.eplog);
      g_usbddev.in_cnt += xfer_len;
    }

  /* Update for the next data IN interrupt */

  privreq->req.xfrd += xfer_len;
  bytesleft          = privreq->req.len - privreq->req.xfrd;

  /* If all of the bytes were sent (including any final null packet)
   * then we are finished with the request buffer).
   */

  if (bytesleft == 0)
    {
      /* Return the write / read request to the class driver */

      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);
      privep->txnullpkt = 0;
      privep->busy = false;
      nrf52_reqcomplete(privep, OK);

      return false;
    }

  return true;
}

static int nrf52_rw_request(struct nrf52_usbdev_s *priv, struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *privreq;
  uint8_t *buf;
  int nbytes;
  int bytesleft;

  /* Check the request from the head of the endpoint request queue */

  privreq = nrf52_reqpeek(privep);
  if (!privreq)
    {
      /* There is no TX transfer in progress and no new pending TX
       * requests to send.
       */

      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPINQEMPTY), 0);
      return -ENOENT;
    }

  uinfo("epno=%d req=%p: len=%d xfrd=%d nullpkt=%d\n", privep->ep.eplog, privreq,
        privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* Get the number of bytes left to be sent in the packet */

  bytesleft         = privreq->req.len - privreq->req.xfrd;
  nbytes            = bytesleft;

  /* If we are not sending a NULL packet, then clip the size to maxpacket
   * and check if we need to send a following NULL packet.
   */

  if (USB_ISEPIN(privep->ep.eplog))
    {
      /* Device to Host */
      if ( nbytes >= privep->ep.maxpacket)
        {
          nbytes =  privep->ep.maxpacket;
        }
    }
  else
    {
      /* Host to Device */
      if (privep->req_len <= nbytes)
        {
          nbytes = privep->req_len;
        }
      else
        {
          uerr("OUT: request buffer is NOT enough big.\n");
        }
    }
  /* Send the packet (might be a null packet nbytes == 0) */

  buf = privreq->req.buf + privreq->req.xfrd;
  nrf52_ep_rw_setup(priv, privep, buf, nbytes);

  return OK;
}


/****************************************************************************
 * Name: nrf52_cancelrequests
 ****************************************************************************/

static void nrf52_cancelrequests(struct nrf52_ep_s *privep)
{
  while (!nrf52_reqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (nrf52_reqpeek(privep))->req.xfrd);
      nrf52_reqcomplete(privep, -ESHUTDOWN);
    }
}

static void nrf52_epdone(struct nrf52_usbdev_s *priv, uint8_t epno)
{
  struct nrf52_ep_s *privep;
  uint8_t logno = nrf52_eplog2index(epno);
  /* Decode and service non control endpoints interrupt */

  privep = &priv->eplist[logno];

  /* We get here when an END Event of  endpoint interrupt occurs.
   * 1. started event (indicate  the second buffer can be passed)
   * 2. endpoint event (indicate the easyDMA done)
   */

  nrf52_trace(" EP End Event: epno %#x.\n", epno);

  nrf52_usbd_dma_pending_clear();

  nrf52_rw_update_xfer(privep);

}

/****************************************************************************
 * Name: nrf52_usbd_isr_handle
 * this api only handle reset / resume / suspend
 ****************************************************************************/

static int nrf52_usbd_isr_handle(void)
{

  struct nrf52_usbdev_s *priv = &g_usbddev;

  /* Handle Reset interrupts.  When this event occurs, the peripheral is left
   * in the same conditions it is left by the system reset (but with the
   * USB controller enabled).
   */

  if (nrf_usbd_event_get_and_clear(NRF_USBD_EVENT_USBRESET))
    {
      /* Reset interrupt received. Clear the RESET interrupt status. */

      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_RESET), 0);
      nrf52_trace("Reset Event.\n");
      /* Restore our power-up state and exit now because istr is no longer
       * valid.
       */

//      nrf52_reset(priv, false);
    }

  /* Handle resume interrupts.  This interrupt is only enable while the USB is
   * suspended.
   */

  if (nrf_usbd_int_enable_check(NRF_USBD_INT_USBEVENT_MASK) &&
      nrf_usbd_event_get_and_clear(NRF_USBD_EVENT_USBEVENT))
    {

      uint32_t cause = nrf_usbd_eventcause_get();

      nrf52_trace("USB Event: %#x.\n", cause);

      /* Wakeup interrupt received. Clear the WKUP interrupt status.  The
       * cause of the resume is indicated in the FNR register
       */
      usbtrace(TRACE_INTENTRY(NRF52_TRACEINTID_RSTINTERRUPT), (uint16_t)cause);

      if (cause & NRF_USBD_EVENTCAUSE_SUSPEND_MASK)
        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SUSP), 0);
          nrf52_trace("Suspend Event.\n");

          /* Clear of the suspend bit must be done after */

          nrf_usbd_eventcause_clear(NRF_USBD_EVENTCAUSE_SUSPEND_MASK);

          nrf52_suspend(priv);
        }

      /* reset usb event data */
      cause = nrf_usbd_eventcause_get();

      if (cause & NRF_USBD_EVENTCAUSE_RESUME_MASK)
        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_WKUP), 0);
          nrf52_trace("Resume Event.\n");

          nrf_usbd_eventcause_clear(NRF_USBD_EVENTCAUSE_RESUME_MASK);

          if (priv->phystatus <= NRF52_USBPHY_STATUS_REMOVE)
            {
              nrf52_initresume(priv, false);
            }
          priv->rsmstate = RSMSTATE_IDLE;
        }

    }

  /* handle SOF for reusme condition */
  if (nrf_usbd_int_enable_check(NRF_USBD_INT_SOF_MASK) &&
      nrf_usbd_event_get_and_clear(NRF_USBD_EVENT_SOF))
    {

      /* Resume handling timing is made with ESOFs */

      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_ESOF), 0);
      nrf52_esofpoll(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_usbd_interrupt
 ****************************************************************************/
static int nrf52_usbd_interrupt(int irq, void *context, FAR void *arg)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct nrf52_usbdev_s *priv = &g_usbddev;

  usbtrace(TRACE_INTENTRY(NRF52_TRACEINTID_EPINTERRUPT), 0);

  uint8_t  epno = 0;
  uint32_t event_epin = NRF_USBD_EVENT_ENDEPIN0;
  uint32_t event_epout = NRF_USBD_EVENT_ENDEPOUT0;

  uint32_t in_mask    = NRF_USBD_INT_ENDEPIN0_MASK;
  uint32_t out_mask    = NRF_USBD_INT_ENDEPOUT0_MASK;


  /* always handle ctrl EP firstly , because only one EP transfer for one time */
  /* handle setup stage */
  if (nrf_usbd_event_get_and_clear(NRF_USBD_EVENT_EP0SETUP))
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0SETUPOUTDATA), 0);

      nrf52_ep0setup(priv);
    }

  /* handle ctrl point IN data transaction end */
  if (nrf_usbd_int_enable_check(in_mask) && nrf_usbd_event_get_and_clear(event_epin))
    {
      nrf52_trace("EP0 IN End\n");
      nrf52_usbd_dma_pending_clear();
    }

  /* handle ctrl point OUT data transaction end */
  if (nrf_usbd_int_enable_check(out_mask) && nrf_usbd_event_get_and_clear(event_epout))
    {
      nrf52_trace("EP0 OUT End\n");

      nrf52_usbd_dma_pending_clear();
      nrf_usbd_shorts_disable(NRF_USBD_SHORT_EP0DATADONE_STARTEPOUT0_MASK);
      nrf52_ep0out_end(priv);
    }

  /* handle ctrl point IN data transaction Done */
  if (nrf_usbd_event_get_and_clear(NRF_USBD_EVENT_EP0DATADONE))
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0SETUPDONE), 0);

      nrf52_ep0in_datadone(priv);
    }

  /* handle normal endpoint end ISR */

  epno = 1;
  event_epin = NRF_USBD_EVENT_ENDEPIN1;
  event_epout = NRF_USBD_EVENT_ENDEPOUT1;

  in_mask    = NRF_USBD_INT_ENDEPIN1_MASK;
  out_mask    = NRF_USBD_INT_ENDEPOUT1_MASK;

  do
    {
      if (nrf_usbd_int_enable_check(in_mask) && nrf_usbd_event_get_and_clear(event_epin))
        {
          uint8_t tick_delay = 100;
          do
            {
              int32_t ack_flag = nrf_usbd_epdatastatus_get() && (1 << epno);
              if (ack_flag)
                {
                  break;
                }
              else
                {
                  up_udelay(1);
                }
            }
          while (tick_delay--);

          nrf52_epdone(priv, epno | NRF_USBD_EP_DIR_IN);
        }

      if (nrf_usbd_int_enable_check(out_mask) && nrf_usbd_event_get_and_clear(event_epout))
        {
          nrf52_epdone(priv, epno | NRF_USBD_EP_DIR_OUT);
        }

      epno++;
      event_epin += 4;
      event_epout += 4;
      in_mask <<= 1;
      out_mask <<= 1;
    }
  while (event_epin <= NRF_USBD_EVENT_ENDEPIN7);

  /* handle ISO endpoint ISR */
  if (nrf_usbd_int_enable_check(NRF_USBD_INT_ENDISOIN0_MASK))
    {
      if (nrf_usbd_event_get_and_clear(NRF_USBD_EVENT_ENDISOIN0))
        {
          nrf52_epdone(priv, 8 | NRF_USBD_EP_DIR_IN);
        }
    }

  if (nrf_usbd_int_enable_check(NRF_USBD_INT_ENDISOOUT0_MASK))
    {
      if (nrf_usbd_event_get_and_clear(NRF_USBD_EVENT_ENDISOOUT0))
        {
          nrf52_epdone(priv, 8 | NRF_USBD_EP_DIR_OUT);
        }
    }

  /* handle bulk | interrupt  OUT / IN ACK transaction */
  if (nrf_usbd_event_get_and_clear(NRF_USBD_EVENT_DATAEP))
    {
      uint32_t epdatastatus = nrf_usbd_epdatastatus_get_and_clear();

      uint32_t epin_mask = NRF_USBD_EPDATASTATUS_EPIN1_MASK;

      uint32_t epout_mask = NRF_USBD_EPDATASTATUS_EPOUT1_MASK;
      uint8_t epno_index ;

      for ( epno = 1; epno < 8; epno++)
        {
          /* handle IN transaction */
          if (epin_mask & epdatastatus)
            {
              priv->in_pack++;
              epno_index = nrf52_eplog2index(epno | USB_DIR_IN);
              struct nrf52_ep_s *privep = &priv->eplist[epno_index];

              if (!nrf52_reqempty(privep) && false == nrf52_dma_flag)
                {
                  /* Continuing current un-finished Write request */
                  nrf52_rw_request(priv, privep);
                }
            }
          epin_mask <<= 1;

          /* Handle OUT transaction */
          if (epout_mask & epdatastatus)
            {
              epno_index = nrf52_eplog2index(epno | USB_DIR_OUT);

              uint32_t out_size = nrf_usbd_epout_size_get(epno);
              uinfo("\t EP_Status : epno %d, size %d.\n", (epno | USB_DIR_OUT), out_size);

              priv->out_pack++;

              uinfo("OUT transaction : pack %d, out_data_len %d.\n", priv->out_pack, priv->out_cnt);
              priv->eplist[epno_index].req_len = out_size;;
              ASSERT(priv->eplist[epno_index].busy == false);
              if (false == nrf52_dma_flag )
                {
                  if (OK != nrf52_rw_request(priv, &priv->eplist[epno_index]))
                    {
                      uwarn("No Buffer to receiver more data.\n");
                    }

                  /* clear epout to make sure the next epdata event */
                  nrf52_usbd_transfer_out_drop(epno);
                }
              else
                {
                  uwarn("\t\t There is DMA transferring in progress.\n");
                }
            }
          epout_mask <<= 1;
        }
    }

  /* handle reset | resume | suspend */
  nrf52_usbd_isr_handle();

  usbtrace(TRACE_INTEXIT(NRF52_TRACEINTID_EPINTERRUPT), 0);

  return OK;
}


/****************************************************************************
 * Suspend/Resume Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: nrf52_suspend
 ****************************************************************************/

static void nrf52_suspend(struct nrf52_usbdev_s *priv)
{

  /* Notify the class driver of the suspend event */

  if (priv->driver)
    {
      CLASS_SUSPEND(priv->driver, &priv->usbdev);
    }

  /* If we are not a self-powered device, the got to low-power mode */
  if (false == priv->selfpowered)
    {
      priv->phystatus = NRF52_USBPHY_STATUS_SUSPEND;

      nrf_usbd_lowpower_enable();

      nrf52_setimask(priv, NRF_USBD_EVENT_SOF, 0);

      if (nrfx_usbd_errata_171())
        {
          if (*((volatile uint32_t *)(0x4006EC00)) == 0x00000000)
            {
              *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
              *((volatile uint32_t *)(0x4006EC14)) = 0x00000000;
              *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
            }
          else
            {
              *((volatile uint32_t *)(0x4006EC14)) = 0x00000000;
            }
        }
    }

}

/****************************************************************************
 * Name: nrf52_initresume
 * self_wakeup : if this wakeup is generated by perph or host
 ****************************************************************************/

static void nrf52_initresume(struct nrf52_usbdev_s *priv, bool self_wakeup)
{
  /* This function is called when either (1) a WKUP interrupt is received from
   * the host PC, or (2) the class device implementation calls the wakeup()
   * method.
   */
  /* Clear the USB low power mode (lower power mode was not set if this is
   * a self-powered device. ).
   */
  if (false == priv->selfpowered)
    {
      priv->phystatus = NRF52_USBPHY_STATUS_RESUME;

      nrf_usbd_lowpower_disable();
      if (nrfx_usbd_errata_171())
        {
          if (*((volatile uint32_t *)(0x4006EC00)) == 0x00000000)
            {
              *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
              *((volatile uint32_t *)(0x4006EC14)) = 0x000000C0;
              *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
            }
          else
            {
              *((volatile uint32_t *)(0x4006EC14)) = 0x000000C0;
            }
        }
    }

  /* Restore full power -- whatever that means for this particular board */
  if (self_wakeup)
    {
      /*
       * The remote wakeup device must hold the resume signaling for at
       * least 1 ms but for no more than 15 ms (TDRSMUP).
       * At the end of this period, the device stops driving the bus
       * (puts its drivers into the high-impedance state and
       * does not drive the bus to the J state).
       */
      nrf_usbd_dpdmvalue_set(NRF_USBD_DPDMVALUE_RESUME);
      nrf_usbd_task_trigger(NRF_USBD_TASK_DRIVEDPDM);
      up_udelay(1000);
      nrf_usbd_task_trigger(NRF_USBD_TASK_NODRIVEDPDM);
    }

  /* Notify the class driver of the resume event */

  if (priv->driver)
    {
      CLASS_RESUME(priv->driver, &priv->usbdev);
    }
}

/****************************************************************************
 * Name: nrf52_esofpoll
 ****************************************************************************/

static void nrf52_esofpoll(struct nrf52_usbdev_s *priv)
{

  /* Called periodically from ESOF interrupt after RSMSTATE_STARTED */

  switch (priv->rsmstate)
    {
      /* One ESOF after internal resume requested */

      case RSMSTATE_STARTED:
        priv->rsmstate = RSMSTATE_WAITING;
        priv->nesofs   = 2;
        break;

      /* Countdown before completing the operation */

      case RSMSTATE_WAITING:
        priv->nesofs--;
        if (priv->nesofs == 0)
          {
            /* Okay.. we are ready to resume normal operation */

            priv->rsmstate = RSMSTATE_IDLE;

            /* Disable ESOF polling, disable the SUSP interrupt, and enable
             * the WKUP interrupt.  Clear any pending WKUP interrupt.
             */

            nrf52_setimask(priv, 0, NRF_USBD_INT_SOF_MASK);
          }
        break;

      case RSMSTATE_IDLE:
      default:
        priv->rsmstate = RSMSTATE_IDLE;
        break;
    }
}

/****************************************************************************
 * Endpoint Helpers
 ****************************************************************************/


/****************************************************************************
 * Name: nrf52_epreserved
 ****************************************************************************/

static inline bool nrf52_epreserved(struct nrf52_usbdev_s *priv, int epno)
{
  return ((priv->epavail & NRF52_ENDP_BIT(epno)) == 0);
}


/****************************************************************************
 * Endpoint operations
 ****************************************************************************/
/****************************************************************************
 * Name: nrf52_epconfigure
 ****************************************************************************/

static int nrf52_epconfigure(struct usbdev_ep_s *ep,
                             const struct usb_epdesc_s *desc,
                             bool last)
{
  struct nrf52_ep_s *privep = (struct nrf52_ep_s *)ep;
  uint16_t maxpacket;
  uint8_t  epno;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !desc)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: ep=%p desc=%p\n");
      return -EINVAL;
    }
#endif

  /* Get the unadorned endpoint address */

  epno = USB_EPNO(desc->addr);
  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));

  flags = enter_critical_section();

  /* Set the requested type */

  privep->eptype = desc->attr;

  /* Get the maxpacket size of the endpoint. */

  maxpacket = GETUINT16(desc->mxpacketsize);
  DEBUGASSERT(maxpacket <= NRF52_MAXPACKET_SIZE);
  ep->maxpacket = maxpacket;

  /* Get the subset matching the requested direction */

  ep->eplog = desc->addr;

  nrf_usbd_ep_enable(ep->eplog);

  if (USB_ISEPOUT(ep->eplog))
    {
      nrf52_usbd_transfer_out_drop(ep->eplog);
    }

  /* check toggle status and force to DATA0 and un-stall */
  if (nrf_usbd_dtoggle_get(desc->addr) != NRF_USBD_DTOGGLE_DATA0)
    {
      nrf_usbd_dtoggle_set(desc->addr, NRF_USBD_DTOGGLE_DATA0);
    }

  if (NRF_USBD_EPISO_CHECK(desc->addr) == false)
    {
      nrf_usbd_ep_unstall(desc->addr);
    }

  privep->stalled = false;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: nrf52_epdisable
 ****************************************************************************/

static int nrf52_epdisable(struct usbdev_ep_s *ep)
{
  struct nrf52_ep_s *privep = (struct nrf52_ep_s *)ep;
  nrf_usbd_int_mask_t int_mask = 0;
  irqstate_t flags;
  uint8_t epno;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  epno = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Cancel any ongoing activity */

  flags = enter_critical_section();

#if 0
  /* Disable TX or RX */

  nrf_usbd_ep_disable(epno | USB_DIR_OUT);
  nrf_usbd_ep_disable(epno | USB_DIR_IN);

  int_mask = 1 << (USBD_INTEN_ENDEPOUT0_Pos + epno);

#else
  nrf_usbd_ep_disable(ep->eplog);
#endif

  nrf52_cancelrequests(privep);

  if (USB_ISEPIN(ep->eplog))
    {
      if (epno == EPISOC )
        {
          int_mask |= NRF_USBD_INT_ENDISOIN0_MASK;
        }
      else
        {
          int_mask |= 1 << (USBD_INTEN_ENDEPIN0_Pos + epno);
        }
    }
  else
    {
      int_mask |= 1 << (USBD_INTEN_ENDEPOUT0_Pos + epno);
    }

  nrf52_setimask(privep->dev, 0, int_mask);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_epallocreq
 ****************************************************************************/

static struct usbdev_req_s *nrf52_epallocreq(struct usbdev_ep_s *ep)
{
  struct nrf52_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

  privreq = (struct nrf52_req_s *)kmm_malloc(sizeof(struct nrf52_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct nrf52_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: nrf52_epfreereq
 ****************************************************************************/

static void nrf52_epfreereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct nrf52_req_s *privreq = (struct nrf52_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  kmm_free(privreq);
}

/****************************************************************************
 * Name: nrf52_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *nrf52_epallocbuffer(FAR struct usbdev_ep_s *ep, uint16_t bytes)
{
  uint32_t buffer;
  usbtrace(TRACE_EPALLOCBUFFER, ep->eplog);

  /* Bulk / Interrupt / ISO transfer request the memory should be 4 byte aligned */

  buffer = (uint32_t)kmm_malloc(bytes);

  if (buffer & 0x3)
    {
      usbtrace(TRACE_EPALLOCBUFFER, 0xFF);
      assert(0);
    }

  return (void *)buffer;
}

/****************************************************************************
 * Name: nrf52_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

static void nrf52_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
  usbtrace(TRACE_EPFREEBUFFER, ep->eplog);

  kmm_free(buf);
}
#endif

/****************************************************************************
 * Name: nrf52_epsubmit
 ****************************************************************************/

static int nrf52_epsubmit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct nrf52_req_s *privreq = (struct nrf52_req_s *)req;
  struct nrf52_ep_s *privep = (struct nrf52_ep_s *)ep;
  struct nrf52_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: req=%p callback=%p buf=%p ep=%p\n",
           req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, ep->eplog);
  priv = privep->dev;

  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
      uerr("ERROR: driver=%p\n", priv->driver);
      return -ESHUTDOWN;
    }

  /* Handle the request from the class driver */

  epno        = USB_EPNO(ep->eplog);

  flags       = enter_critical_section();

  req->result = -EINPROGRESS;
  req->xfrd   = 0;
  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      nrf52_abortrequest(privep, privreq, -EBUSY);
      uerr("ERROR: stalled\n");
      ret = -EBUSY;
    }
  else if (EP0 == epno)
    {
      if (true == priv->ep0dir)
        {
          nrf52_rqenqueue(privep, privreq);
          ret = nrf52_wrrequest_ep0(priv, privep);
        }
      else
        {
          /* NOTE: Data values or size may be tested here to decide if clear or stall.
           * If errata 154 is present the data transfer is acknowledged by the hardware. */
          /* send out Status Stage */
          if (false == nrfx_usbd_errata_154())
            {
              nrf52_trace("EP0 OUT Status traction.\n");

              nrf52_ep0setup_status();
            }
          priv->ep0state = EP0STATE_IDLE;

          nrf52_trace("EP0 OUT traction.\n");
          return ret;
        }
    }
  else if (USB_ISEPIN(ep->eplog))
    {
      /* Handle IN (device-to-host) requests. */
      /* Add the new request to the request queue for the IN endpoint */

      nrf52_rqenqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(epno), req->len);

      /* If the IN endpoint FIFO is available, then transfer the data now */

      if ((!privep->busy ) && (false == nrf52_dma_flag))
        {
          ret = nrf52_rw_request(priv, privep);
        }
    }
  else
    {
      /* Handle OUT (host-to-device) requests */
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      bool req_list_empty = sq_empty(privep);

      nrf52_rqenqueue(privep, privreq);

      if (req_list_empty)
        {
          nrf52_usbd_transfer_out_drop(ep->eplog);
        }

      usbtrace(TRACE_OUTREQQUEUED(epno), ep->eplog);
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: nrf52_epcancel
 ****************************************************************************/

static int nrf52_epcancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct nrf52_ep_s *privep = (struct nrf52_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_USB
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = enter_critical_section();
  nrf52_cancelrequests(privep);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_epstall
 ****************************************************************************/

static int nrf52_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct nrf52_ep_s *privep;
  struct nrf52_usbdev_s *priv;
  uint16_t status;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_USB
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  privep = (struct nrf52_ep_s *)ep;
  priv   = (struct nrf52_usbdev_s *)privep->dev;

  /* STALL or RESUME the endpoint */

  flags = enter_critical_section();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, USB_EPNO(ep->eplog));

  /* Get status of the endpoint; stall the request if the endpoint is
   * disabled
   */

  status = nrf_usbd_ep_enable_check(ep->eplog);

  /* Handle the resume condition */

  if (resume)
    {
      /* Resuming a stalled endpoint */

      usbtrace(TRACE_EPRESUME, ep->eplog);
      privep->stalled = false;

      if (USB_ISEPOUT(ep->eplog))
        {
          nrf52_usbd_transfer_out_drop(ep->eplog);
        }

      nrf_usbd_ep_unstall(ep->eplog);

      if (USB_ISEPIN(ep->eplog))
        {
          if (status && (false == privep->busy))
            {
              /* Restart any queued write requests */

              if (USB_EPNO(ep->eplog) == EP0)
                {
                  (void)nrf52_wrrequest_ep0(priv, privep);
                }
              else
                {
                  (void)nrf52_rw_request(priv, privep);
                }
            }
        }
    }

  /* Handle the stall condition */

  else
    {
      usbtrace(TRACE_EPSTALL, ep->eplog);
      privep->stalled = true;

      nrf_usbd_ep_stall(ep->eplog);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/
/****************************************************************************
 * Name: nrf52_allocep
 ****************************************************************************/
static uint8_t nrf52_eplog2index(uint8_t eplog)
{
  /* they are spitted into 9 end point group
   * 0,1 | 1,2 | 3,4 | 5,6 | 7,8 |......
   * EP0 | EP1 | EP2 | EP3 | EP4 |......
   * convert logic epno to internal eplist index
   */
  uint8_t index;

  index = USB_EPNO(eplog) * 2;
  if (USB_ISEPIN(eplog))
    {
      index += 1;
    }

  return index;
}

static uint8_t nrf52_index2eplog(uint8_t index)
{
  /* they are spitted into 9 end point group
   * 0,1 | 1,2 | 3,4 | 5,6 | 7,8 |......
   * EP0 | EP1 | EP2 | EP3 | EP4 |......
   * convert internal eplist index to logic epno
   */
  uint8_t eplog;

  eplog = index / 2;
  if (index % 2)
    {
      eplog |= USB_DIR_IN;
    }

  return eplog;
}

static struct usbdev_ep_s *nrf52_allocep(struct usbdev_s *dev, uint8_t epno,
                                         bool in, uint8_t eptype)
{
  struct nrf52_usbdev_s *priv = (struct nrf52_usbdev_s *)dev;
  struct nrf52_ep_s *privep = NULL;
  uint16_t epset;
  irqstate_t irq_flag;
  int  index;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  /* Ignore any direction bits in the logical address */

  index = nrf52_eplog2index(epno);
  epno = USB_EPNO(epno);

  ASSERT(epno > 0);


  /* A logical address of 0 means that any endpoint will do */

  if (index > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the requested
       * 'logical' endpoint.  All of the other checks will still be performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (index >= NRF52_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADEPNO), (uint16_t)index);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = NRF52_ENDP_BIT(index);
    }

  /* Check if the selected endpoint number is available */

  irq_flag = enter_critical_section();
  if (priv->epavail & epset)
    {
      priv->epavail &= ~epset;
      privep = &priv->eplist[index];
    }
  else
    {
      ASSERT(0);
    }
  leave_critical_section(irq_flag);

  if (!privep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_EPRESERVE), (uint16_t)epset);
      return NULL;
    }

  privep->eptype = eptype;
  privep->ep.eplog = epno;

  return &privep->ep;

}


/****************************************************************************
 * Name: nrf52_freeep
 ****************************************************************************/

static void nrf52_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct nrf52_usbdev_s *priv;
  struct nrf52_ep_s *privep;

#ifdef CONFIG_DEBUG_USB
  if (!dev || !ep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  priv   = (struct nrf52_usbdev_s *)dev;
  privep = (struct nrf52_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));

  if (priv && privep)
    {

      /* Mark the endpoint as available */

      irqstate_t irq_flag = enter_critical_section();
      priv->epavail |= NRF52_ENDP_BIT(nrf52_eplog2index(privep->ep.eplog));
      leave_critical_section(irq_flag);

    }
}

/****************************************************************************
 * Name: nrf52_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

static int nrf52_pullup(struct usbdev_s *dev, bool enable)
{
  struct nrf52_usbdev_s *priv = (struct nrf52_usbdev_s *)dev;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  if (priv->phystatus >= NRF52_USBPHY_STATUS_PWR_READY)
    {
      if (enable)
        {
          priv->connect = NRF52_DEV_CONNECTED;
          nrf_usbd_pullup_enable();
        }
      else
        {
          priv->connect = NRF52_DEV_DISCONNECT;
          nrf_usbd_pullup_disable();
        }
    }
  else
    {
      if (enable)
        {
          /* pwr is not ready , put into connecting status */
          priv->connect = NRF52_DEV_CONNECTING;
        }
      else
        {
          nrf_usbd_pullup_disable();
          priv->connect = NRF52_DEV_DISCONNECT;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_getframe
 ****************************************************************************/

static int nrf52_getframe(struct usbdev_s *dev)
{
  int framecnt;
#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware
   * there is no implement , if using , should do in further
   */

  framecnt = nrf_usbd_framecntr_get();

  usbtrace(TRACE_DEVGETFRAME, framecnt);

  return framecnt;
}

/****************************************************************************
 * Name: nrf52_wakeup
 ****************************************************************************/

static int nrf52_wakeup(struct usbdev_s *dev)
{
  struct nrf52_usbdev_s *priv = (struct nrf52_usbdev_s *)dev;
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);
#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Start the resume sequence.  The actual resume steps will be driven
   * by the ESOF interrupt and .
   */

  flags = enter_critical_section();
  nrf52_initresume(priv, true);
  priv->rsmstate = RSMSTATE_STARTED;

  nrf52_setimask(priv, NRF_USBD_INT_SOF_MASK | NRF_USBD_INT_USBEVENT_MASK, 0);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_selfpowered
 ****************************************************************************/

static int nrf52_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct nrf52_usbdev_s *priv = (struct nrf52_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Initialization/Reset
 ****************************************************************************/
static void nrf52_usbd_event_clear_all(void)
{
  for (int event = NRF_USBD_EVENT_USBRESET; event <= NRF_USBD_EVENT_ACCESSFAULT ; event += 4)
    {
      nrf_usbd_event_clear(event);
    }
}
/****************************************************************************
 * Name: nrf52_reset
 ****************************************************************************/

static void nrf52_reset(struct nrf52_usbdev_s *priv, bool hwreset)
{
  int epno;

  /* Reset endpoints */

  for (epno = 0; epno < NRF52_NENDPOINTS; epno++)
    {
      struct nrf52_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are canceled
       * with status -ESHUTDOWN, then will not be requeued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling nrf52_epdisable
       * for each of its configured endpoints.
       */

      nrf52_cancelrequests(privep);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->busy      = false;
      privep->txnullpkt = false;
    }

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */
  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }
  /* Re-configure the USB controller in its initial, unconnected state */

  if (hwreset)
    {
      nrf52_hwreset(priv);
    }

  /* Reset the device state structure */

  priv->ep0state  = EP0STATE_IDLE;
  priv->rsmstate  = RSMSTATE_IDLE;

  priv->usbdev.speed = USB_SPEED_FULL;
}

/****************************************************************************
 * Name: nrf52_hwreset
 ****************************************************************************/

static void nrf52_hwreset(struct nrf52_usbdev_s *priv)
{
  /* Put the USB controller into reset, clear all interrupt enables */

  irqstate_t flags;

  flags = enter_critical_section();

  /* Disable interrupts (and perhaps take the USB controller out of reset) */

  priv->imask = 0;
  nrf_usbd_int_disable(NRF52_USBD_INT_ALL_MASK);

  /* Clear any pending interrupts */
  nrf52_usbd_event_clear_all();

  /* Enable interrupts at the USB controller */

  nrf52_setimask(priv, NRF52_DEFAULT_INT, 0);
  leave_critical_section(flags);
  return;
}

static void nrf52_usbd_enable(void)
{
  irqstate_t flags;

  flags = enter_critical_section();
  /* Prepare for READY event receiving */
  nrf_usbd_eventcause_clear(NRF_USBD_EVENTCAUSE_READY_MASK \
                            | NRF_USBD_EVENTCAUSE_SUSPEND_MASK \
                            | NRF_USBD_EVENTCAUSE_RESUME_MASK);

  if (nrfx_usbd_errata_187())
    {
      if (*((volatile uint32_t *)(0x4006EC00)) == 0x00000000)
        {
          *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
          *((volatile uint32_t *)(0x4006ED14)) = 0x00000003;
          *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
        }
      else
        {
          *((volatile uint32_t *)(0x4006ED14)) = 0x00000003;
        }
    }

  if (nrfx_usbd_errata_171())
    {
      if (*((volatile uint32_t *)(0x4006EC00)) == 0x00000000)
        {
          *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
          *((volatile uint32_t *)(0x4006EC14)) = 0x000000C0;
          *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
        }
      else
        {
          *((volatile uint32_t *)(0x4006EC14)) = 0x000000C0;
        }
    }

  nrf_usbd_enable();
  /* Waiting for peripheral to enable, this should take a few us */
  while (0 == (NRF_USBD_EVENTCAUSE_READY_MASK & nrf_usbd_eventcause_get()))
    {
      /* Empty loop */
    }
  nrf_usbd_eventcause_clear(NRF_USBD_EVENTCAUSE_READY_MASK);

  if (nrfx_usbd_errata_171() || nrfx_usbd_errata_187())
    {
      if (*((volatile uint32_t *)(0x4006EC00)) == 0x00000000)
        {
          *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
          *((volatile uint32_t *)(0x4006EC14)) = 0x00000000;
          *((volatile uint32_t *)(0x4006EC00)) = 0x00009375;
        }
      else
        {
          *((volatile uint32_t *)(0x4006EC14)) = 0x00000000;
        }

    }

  if (nrfx_usbd_errata_166())
    {
      *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7E3;
      *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = 0x40;
      ARM_ISB();
      ARM_DSB();
    }

  nrf_usbd_isosplit_set(NRF_USBD_ISOSPLIT_Half);

  nrf52_usbd_dma_pending_clear();

  leave_critical_section(flags);
  return;
}

static int nrf52_usbd_disable(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  nrf52_pullup((struct usbdev_s *)&g_usbddev, false);

  nrf_usbd_disable();

  nrf52_usbd_dma_pending_clear();

  leave_critical_section(flags);

  return OK;

}

/****************************************************************************
 * Name: nrf52_hwsetup
 ****************************************************************************/

static void nrf52_hwsetup(struct nrf52_usbdev_s *priv)
{
  int epno;

  /* Power the USB controller, put the USB controller into reset, disable
   * all USB interrupts
   */

  nrf52_usbd_disable();

  /* Initialize the device state structure.  NOTE: many fields
   * have the initial value of zero and, hence, are not explicitly
   * initialized here.
   */

  memset(priv, 0, sizeof(struct nrf52_usbdev_s));
  priv->epstatus     = NRF52_EP_STATUS_IDLE;
  priv->usbdev.ops   = &g_devops;
  priv->usbdev.ep0   = &priv->eplist[EP0_IN].ep;
  priv->epavail      = NRF52_ENDP_ALLSET & (~ NRF52_ENDP_BIT(EP0_IN)) & (~ NRF52_ENDP_BIT(EP0_OUT));

  /* Initialize the endpoint list */

  for (epno = 0; epno < NRF52_NENDPOINTS; epno++)
    {
      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the (physical) endpoint number which is just the index to the
       * endpoint.
       */

      priv->eplist[epno].ep.ops = &g_epops;
      priv->eplist[epno].dev    = priv;
      priv->eplist[epno].eppos  = epno;
      priv->eplist[epno].ep.eplog = nrf52_index2eplog(epno);

      /* We will use a fixed maxpacket size for all endpoints (perhaps
       * ISOC endpoints could have larger maxpacket???).  A smaller
       * packet size can be selected when the endpoint is configured.
       */

      priv->eplist[epno].ep.maxpacket = NRF52_MAXPACKET_SIZE;
    }

  /* Select a smaller endpoint size for EP0 & bigger size for ISOC */

#if NRF52_EP0MAXPACKET < NRF52_MAXPACKET_SIZE
  priv->eplist[EP0_IN].ep.maxpacket  = NRF52_EP0MAXPACKET;
  priv->eplist[EP0_OUT].ep.maxpacket = NRF52_EP0MAXPACKET;
#endif
  priv->eplist[EPISOC * 2].ep.maxpacket   = NRF52_EP8MAXPACKET;
  priv->eplist[EPISOC * 2 + 1].ep.maxpacket = NRF52_EP8MAXPACKET;

  priv->phystatus = NRF52_USBPHY_STATUS_INITIALIZED;
}

/****************************************************************************
 * Name: nrf52_hwshutdown
 ****************************************************************************/

static void nrf52_hwshutdown(struct nrf52_usbdev_s *priv)
{
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts and force the USB controller into reset */

  nrf_usbd_int_disable(NRF52_USBD_INT_ALL_MASK);

  /* Clear any pending interrupts */

  nrf52_usbd_event_clear_all();

  /* Power down the USB controller */

  nrf52_usbd_disable();

  priv->phystatus = NRF52_USBPHY_STATUS_UNINIT;
}

static void nrf52_usbd_callback(uint32_t power_status)
{
  struct nrf52_usbdev_s *priv = (struct nrf52_usbdev_s *)&g_usbddev;
  /* Enable USBD after VBUS has been detected only */

  if (priv->phystatus != NRF52_USBPHY_STATUS_UNINIT)
    {
      sem_post(&priv->sem);
    }
}

/* monitor usb phy status change
 *
 */
static int nrf52_usbd_monitor(int argc, char **argv)
{
  struct nrf52_usbdev_s *priv = &g_usbddev;
  uint32_t pwr_status;
  while (1)
    {
      sem_wait(&priv->sem);
      pwr_status = nrf52_power_get_usbphy_status();

      _warn("phystatus %d, pwr_status : %d.\n", priv->phystatus, pwr_status);
      switch (priv->phystatus)
        {
          case NRF52_USBPHY_STATUS_RESUME:
          case NRF52_USBPHY_STATUS_SUSPEND:
            if (pwr_status == NRF52_POWER_USBD_REMOVE)
              {
                priv->phystatus = NRF52_USBPHY_STATUS_REMOVE;
                nrf52_usbd_disable();
              }
            break;
          case NRF52_USBPHY_STATUS_INITIALIZED:
          case NRF52_USBPHY_STATUS_DETECT:
          case NRF52_USBPHY_STATUS_PWR_READY:
            if (pwr_status == NRF52_POWER_USBD_PWRRDY)
              {
                priv->phystatus = NRF52_USBPHY_STATUS_PWR_READY;
                {
                  nrf52_pullup((struct usbdev_s *)priv, true);
                }
              }
            else if (pwr_status == NRF52_POWER_USBD_REMOVE)
              {
                nrf52_reset(priv, true);
                nrf52_usbd_disable();
                priv->phystatus = NRF52_USBPHY_STATUS_REMOVE;
              }
            else if (pwr_status == NRF52_POWER_USBD_DETECT)
              {
                nrf52_usbd_enable();

                priv->phystatus = NRF52_USBPHY_STATUS_DETECT;
              }
            break;
          case NRF52_USBPHY_STATUS_REMOVE:
            if (pwr_status == NRF52_POWER_USBD_DETECT)
              {
                nrf52_usbd_enable();
                priv->phystatus = NRF52_USBPHY_STATUS_DETECT;
              }
          case NRF52_USBPHY_STATUS_UNINIT:
            break;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: up_usbinitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void up_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct nrf52_usbdev_s *priv = &g_usbddev;

  usbtrace(TRACE_DEVINIT, 0);

  sem_init(&priv->sem, 0, 0);

  /* Power up the USB controller, but leave it in the reset state */

  nrf52_hwsetup(priv);

  nrf52_clock_hsclk_start();

  /* Attach USB controller interrupt handlers.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(USBD_IRQn, nrf52_usbd_interrupt, NULL) != 0)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_IRQREGISTRATION),
               (uint16_t)USBD_IRQn);
      goto errout;
    }

  /* Reset/Re-initialize the USB hardware */

  nrf52_reset(priv, true);

  nrf52_power_initlialize();

  nrf52_power_register_callback(nrf52_usbd_callback, POWER_CALLBACK_USBD);

  return;

errout:
  up_usbuninitialize();
}

/****************************************************************************
 * Name: up_usbuninitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_usbuninitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct nrf52_usbdev_s *priv = &g_usbddev;
  irqstate_t flags;

  flags = enter_critical_section();
  usbtrace(TRACE_DEVUNINIT, 0);

  /* Disable and detach the USB IRQs */

  up_disable_irq(USBD_IRQn);
  irq_detach(USBD_IRQn);
  nrf52_power_register_callback(NULL, POWER_CALLBACK_USBD);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Put the hardware in an inactive state */

  nrf52_hwshutdown(priv);

  sem_destroy(&priv->sem);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct nrf52_usbdev_s *priv = &g_usbddev;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_USB
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  if (0 == priv->monitor)
    {
      priv->monitor = kthread_create("NRF52 USB", 128, 2048,
                                     (main_t)nrf52_usbd_monitor, (FAR char * const *)NULL);
    }

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BINDFAILED), (uint16_t) - ret);
    }
  else
    {

#ifdef CONFIG_ARCH_IRQPRIO
      /* Set the interrupt priority */

      up_prioritize_irq(USBD_IRQn, CONFIG_USB_PRI);
#endif

      /* Enable USB controller interrupts at the NVIC */

      up_enable_irq(USBD_IRQn);

      /* Enable pull-up to connect the device.  The host should enumerate us
       * some time after this
       */

      nrf52_pullup(&priv->usbdev, true);
      priv->usbdev.speed = USB_SPEED_FULL;
    }

  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver. If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct nrf52_usbdev_s *priv = &g_usbddev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_USB
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = enter_critical_section();
  nrf52_reset(priv, true);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(USBD_IRQn);

  /* Put the hardware in an inactive state.  Then bring the hardware back up
   * in the reset state (this is probably not necessary, the nrf52_reset()
   * call above was probably sufficient).
   */

  nrf52_hwshutdown(priv);

  /* Unhook the driver */

  priv->driver = NULL;
  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_USBDEV && CONFIG_NRF52_USBD */
