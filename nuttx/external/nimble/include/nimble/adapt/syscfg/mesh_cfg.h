#ifndef H_MYNEWT_MESHCFG_
#define H_MYNEWT_MESHCFG_

#include <nuttx/config.h>
#ifdef CONFIG_BLE_NIMBLE_HOST_MESH_SUPPORT

#ifdef MYNEWT_VAL_MSYS_1_BLOCK_SIZE
#undef MYNEWT_VAL_MSYS_1_BLOCK_SIZE
#define MYNEWT_VAL_MSYS_1_BLOCK_SIZE (292)
#endif

#ifdef MYNEWT_VAL_BLE_MESH
#undef MYNEWT_VAL_BLE_MESH
#define MYNEWT_VAL_BLE_MESH (1)
#endif

/*BLE MESH Start *****************************************************/

#ifndef MYNEWT_VAL_BLE_MESH_ADV_BUF_COUNT
#define MYNEWT_VAL_BLE_MESH_ADV_BUF_COUNT (10)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_ADV_TASK_PRIO
#define MYNEWT_VAL_BLE_MESH_ADV_TASK_PRIO (9)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_APP_KEY_COUNT
#define MYNEWT_VAL_BLE_MESH_APP_KEY_COUNT (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_CRPL
#define MYNEWT_VAL_BLE_MESH_CRPL (10)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_TESTING
#define MYNEWT_VAL_BLE_MESH_TESTING (0)
#endif

/*BLE MESH debug config*/
#ifndef MYNEWT_VAL_BLE_MESH_DEBUG
#define MYNEWT_VAL_BLE_MESH_DEBUG (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_NET
#define MYNEWT_VAL_BLE_MESH_DEBUG_NET (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_TRANS
#define MYNEWT_VAL_BLE_MESH_DEBUG_TRANS (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_BEACON
#define MYNEWT_VAL_BLE_MESH_DEBUG_BEACON (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_CRYPTO
#define MYNEWT_VAL_BLE_MESH_DEBUG_CRYPTO (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_PROV
#define MYNEWT_VAL_BLE_MESH_DEBUG_PROV (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_ACCESS
#define MYNEWT_VAL_BLE_MESH_DEBUG_ACCESS (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_MODEL
#define MYNEWT_VAL_BLE_MESH_DEBUG_MODEL (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_ADV
#define MYNEWT_VAL_BLE_MESH_DEBUG_ADV (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_LOW_POWER
#define MYNEWT_VAL_BLE_MESH_DEBUG_LOW_POWER (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_FRIEND
#define MYNEWT_VAL_BLE_MESH_DEBUG_FRIEND (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_PROXY
#define MYNEWT_VAL_BLE_MESH_DEBUG_PROXY (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_CFG_CLI
#define MYNEWT_VAL_BLE_MESH_CFG_CLI (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_HEALTH_CLI
#define MYNEWT_VAL_BLE_MESH_HEALTH_CLI (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_SHELL_MODELS
#define MYNEWT_VAL_BLE_MESH_SHELL_MODELS (0)
#endif

/*
* Automatically enable LPN functionality once provisioned and start
* looking for Friend nodes. If this option is disabled LPN mode
* needs to be manually enabled by calling bt_mesh_lpn_set(true).
* node.
*/
#ifndef MYNEWT_VAL_BLE_MESH_LPN_AUTO
#define MYNEWT_VAL_BLE_MESH_LPN_AUTO (1)
#endif

/*
* Time in seconds from the last received message, that the node
* will wait before starting to look for Friend nodes.
*/
#ifndef MYNEWT_VAL_BLE_MESH_LPN_AUTO_TIMEOUT
#define MYNEWT_VAL_BLE_MESH_LPN_AUTO_TIMEOUT (15)
#endif

/*
* Perform the Friendship establishment using low power, with
* the help of a reduced scan duty cycle. The downside of this
* is that the node may miss out on messages intended for it
* until it has successfully set up Friendship with a Friend
* node.
*/
#ifndef MYNEWT_VAL_BLE_MESH_LPN_ESTABLISHMENT
#define MYNEWT_VAL_BLE_MESH_LPN_ESTABLISHMENT (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEV_UUID
#define MYNEWT_VAL_BLE_MESH_DEV_UUID ((uint8_t[16]){0x22, 0x23, 0})
#endif

#ifndef MYNEWT_VAL_BLE_MESH_FRIEND
#define MYNEWT_VAL_BLE_MESH_FRIEND (0)
#endif

/*
* Time in seconds between Friend Requests, if a previous Friend
* Request did not receive any acceptable Friend Offers.
*/
#ifndef MYNEWT_VAL_BLE_MESH_LPN_RETRY_TIMEOUT
#define MYNEWT_VAL_BLE_MESH_LPN_RETRY_TIMEOUT (8)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LPN_INIT_POLL_TIMEOUT
#define MYNEWT_VAL_BLE_MESH_LPN_INIT_POLL_TIMEOUT (300)
#endif

/*
* PollTimeout timer is used to measure time between two
* consecutive requests sent by the Low Power node. If no
* requests are received by the Friend node before the
* PollTimeout timer expires, then the friendship is considered
* terminated. The value is in units of 100 milliseconds, so e.g.
* a value of 300 means 30 seconds.
*/
#ifndef MYNEWT_VAL_BLE_MESH_LPN_POLL_TIMEOUT
#define MYNEWT_VAL_BLE_MESH_LPN_POLL_TIMEOUT (300)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_FRIEND_SUB_LIST_SIZE
#define MYNEWT_VAL_BLE_MESH_FRIEND_SUB_LIST_SIZE (5)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_FRIEND_SEG_RX
#define MYNEWT_VAL_BLE_MESH_FRIEND_SEG_RX (5)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_FRIEND_LPN_COUNT
#define MYNEWT_VAL_BLE_MESH_FRIEND_LPN_COUNT (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_FRIEND_QUEUE_SIZE
#define MYNEWT_VAL_BLE_MESH_FRIEND_QUEUE_SIZE (16)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_FRIEND_RECV_WIN
#define MYNEWT_VAL_BLE_MESH_FRIEND_RECV_WIN (255)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_FRIEND_SUB_LIST_SIZE
#define MYNEWT_VAL_BLE_MESH_FRIEND_SUB_LIST_SIZE (16)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_GATT_PROXY
#define MYNEWT_VAL_BLE_MESH_GATT_PROXY (1)
#endif

/*
* This option determines for how long the local node advertises
* using Node Identity. The given value is in seconds. The
* specification limits this to 60 seconds, and implies that to
* be the appropriate value as well, so just leaving this as the
* default is the safest option.
*/
/*In some corner case the Node Identify will take
 *long time waiting for remote provisioner to initiate
 *a GATT connection, so we enlarge the timeout to 30s.
 */
#ifndef MYNEWT_VAL_BLE_MESH_NODE_ID_TIMEOUT
#define MYNEWT_VAL_BLE_MESH_NODE_ID_TIMEOUT (30)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_IV_UPDATE_TEST
#define MYNEWT_VAL_BLE_MESH_IV_UPDATE_TEST (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LABEL_COUNT
#define MYNEWT_VAL_BLE_MESH_LABEL_COUNT (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LOW_POWER
#define MYNEWT_VAL_BLE_MESH_LOW_POWER (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LPN_GROUPS
#define MYNEWT_VAL_BLE_MESH_LPN_GROUPS (10)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LPN_MIN_QUEUE_SIZE
#define MYNEWT_VAL_BLE_MESH_LPN_MIN_QUEUE_SIZE (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LPN_POLL_TIMEOUT
#define MYNEWT_VAL_BLE_MESH_LPN_POLL_TIMEOUT (100)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LPN_RECV_DELAY
#define MYNEWT_VAL_BLE_MESH_LPN_RECV_DELAY (20)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LPN_RECV_WIN_FACTOR
#define MYNEWT_VAL_BLE_MESH_LPN_RECV_WIN_FACTOR (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LPN_RSSI_FACTOR
#define MYNEWT_VAL_BLE_MESH_LPN_RSSI_FACTOR (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_LPN_SCAN_LATENCY
#define MYNEWT_VAL_BLE_MESH_LPN_SCAN_LATENCY (10)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_MODEL_GROUP_COUNT
#define MYNEWT_VAL_BLE_MESH_MODEL_GROUP_COUNT (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_MODEL_KEY_COUNT
#define MYNEWT_VAL_BLE_MESH_MODEL_KEY_COUNT (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_MSG_CACHE_SIZE
#define MYNEWT_VAL_BLE_MESH_MSG_CACHE_SIZE (10)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_PB_ADV
#define MYNEWT_VAL_BLE_MESH_PB_ADV (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_PB_GATT
#define MYNEWT_VAL_BLE_MESH_PB_GATT (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_PROV
#define MYNEWT_VAL_BLE_MESH_PROV (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_PROXY
#define MYNEWT_VAL_BLE_MESH_PROXY (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_PROXY_FILTER_SIZE
#define MYNEWT_VAL_BLE_MESH_PROXY_FILTER_SIZE (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_RELAY
#define MYNEWT_VAL_BLE_MESH_RELAY (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_RX_SDU_MAX
#define MYNEWT_VAL_BLE_MESH_RX_SDU_MAX (384)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_RX_SEG_MSG_COUNT
#define MYNEWT_VAL_BLE_MESH_RX_SEG_MSG_COUNT (2)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_SUBNET_COUNT
#define MYNEWT_VAL_BLE_MESH_SUBNET_COUNT (1)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_TX_SEG_MSG_COUNT
#define MYNEWT_VAL_BLE_MESH_TX_SEG_MSG_COUNT (4)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_TX_SEG_MAX
#define MYNEWT_VAL_BLE_MESH_TX_SEG_MAX (4)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_SETTINGS
#define MYNEWT_VAL_BLE_MESH_SETTINGS (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_IVU_DIVIDER
#define MYNEWT_VAL_BLE_MESH_IVU_DIVIDER (4)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEVICE_NAME
#define MYNEWT_VAL_BLE_MESH_DEVICE_NAME "Nimble_zglue_node"
#endif

#ifndef MYNEWT_VAL_BLE_MESH_DEBUG_SETTINGS
#define MYNEWT_VAL_BLE_MESH_DEBUG_SETTINGS (0)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_RPL_STORE_TIMEOUT
#define MYNEWT_VAL_BLE_MESH_RPL_STORE_TIMEOUT (5)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_STORE_TIMEOUT
#define MYNEWT_VAL_BLE_MESH_STORE_TIMEOUT (2)
#endif

#ifndef MYNEWT_VAL_BLE_MESH_SEQ_STORE_RATE
#define MYNEWT_VAL_BLE_MESH_SEQ_STORE_RATE (128)
#endif

/*BLE MESH end*****************************************************/
#endif /*CONFIG_BLE_NIMBLE_HOST_MESH_SUPPORT*/

#endif /*H_MYNEWT_MESHCFG_*/
