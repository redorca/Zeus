/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "syscfg/syscfg.h"

#if MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST)

#include <inttypes.h>
#include <string.h>
#include <stdio.h>

#include "sysinit/sysinit.h"
#include "host/ble_hs.h"
#include "config/config.h"
#include "base64/base64.h"
#include "store/config/ble_store_config.h"
#include "ble_store_config_priv.h"

static int
ble_store_config_conf_set(int argc, char **argv, char *val);
static int
ble_store_config_conf_export(void (*func)(char *name, char *val),
                             enum conf_export_tgt tgt);

static struct conf_handler ble_store_config_conf_handler = {
    .ch_name = "ble_hs",
    .ch_get = NULL,
    .ch_set = ble_store_config_conf_set,
    .ch_commit = NULL,
    .ch_export = ble_store_config_conf_export
};

#define BLE_STORE_CONFIG_SEC_ENCODE_SZ      \
    BASE64_ENCODE_SIZE(sizeof (struct ble_store_value_sec))

#define BLE_STORE_CONFIG_SEC_SET_ENCODE_SZ  \
    (MYNEWT_VAL(BLE_STORE_MAX_BONDS) * BLE_STORE_CONFIG_SEC_ENCODE_SZ + 1)

#define BLE_STORE_CONFIG_CCCD_ENCODE_SZ     \
    BASE64_ENCODE_SIZE(sizeof (struct ble_store_value_cccd))

#define BLE_STORE_CONFIG_CCCD_SET_ENCODE_SZ \
    (MYNEWT_VAL(BLE_STORE_MAX_CCCDS) * BLE_STORE_CONFIG_CCCD_ENCODE_SZ + 1)

#define BLE_STORE_FLIE_NAME_LEN    (8)

#define BLE_STORE_OUR_SEC_PATH           "/internal/ble_our"
#define BLE_STORE_PEER_SEC_PATH          "/internal/ble_peer"
#define BLE_STORE_CCCD_PATH              "/internal/ble_cccd"

/*Here we use static variable instead of temp variable,
 *becasue the two variable size is defined by custom config
 *and will take too many case stack overflow.
 */
char ble_store_sec[BLE_STORE_CONFIG_SEC_SET_ENCODE_SZ] = {0};
char ble_store_cccd[BLE_STORE_CONFIG_CCCD_SET_ENCODE_SZ] = {0};

static void
ble_store_config_serialize_arr(const void *arr, int obj_sz, int num_objs,
                               char *out_buf, int buf_sz)
{
    int arr_size;

    arr_size = obj_sz * num_objs;
    assert(BASE64_ENCODE_SIZE(arr_size) <= buf_sz);

    base64_encode(arr, arr_size, out_buf, 1);
}

static int
ble_store_config_deserialize_arr(const char *enc,
                                 void *out_arr,
                                 int obj_sz,
                                 int *out_num_objs)
{
    int len;

    len = base64_decode(enc, out_arr);
    BLE_HS_LOG(DEBUG, "out_len = %d,",len);

    if (len < 0) {
        return OS_EINVAL;
    }

    *out_num_objs = len / obj_sz;
    return 0;
}

static void
ble_store_config_persist_val_read(uint8_t type)
{
    FAR FILE * ble_store_file = NULL;
    const char * file_path = NULL;
    char * out;
    uint16_t out_size;

    if(BLE_STORE_CONFIG_OUR_SEC == type) {
        file_path = BLE_STORE_OUR_SEC_PATH;
    } else if(BLE_STORE_CONFIG_PEER_SEC == type) {
        file_path = BLE_STORE_PEER_SEC_PATH;
    } else if(BLE_STORE_CONFIG_CCCD == type) {
        file_path = BLE_STORE_CCCD_PATH;
    } else {
        BLE_HS_LOG(ERROR, "Create BLE config type not defined: %d\n", type);
        return;
    }

    ble_store_file = fopen(file_path, "r");
    if (NULL == ble_store_file) {
            /*For first time, file is not exzist, need create it.*/
            ble_store_file = fopen(file_path, "w");
            if (NULL == ble_store_file) {
            BLE_HS_LOG(ERROR, "\nCreate BLE file %s failed: %d\n", file_path, get_errno());
            return ;
        } else {
            BLE_HS_LOG(DEBUG, "\nFiLe %s Created for save config value.\n", file_path);
            fclose(ble_store_file);
            return ;
        }
    } else {
        /*If file already exist, read info to ram.*/
        uint16_t out_cnt = 0;
        if(BLE_STORE_CONFIG_CCCD == type) {
            out = ble_store_cccd;
            out_size = sizeof(ble_store_cccd);
        } else {
            out = ble_store_sec;
            out_size = sizeof(ble_store_sec);
        }

        memset(out, 0, out_size);
        out_cnt = fread(out, 1, out_size, ble_store_file);
        BLE_HS_LOG(DEBUG, "\nRead from %s, %d bytes read out,",file_path, out_cnt);

        if(out_cnt > 0) {
            if(BLE_STORE_CONFIG_OUR_SEC == type) {
                ble_store_config_deserialize_arr(
                              out,
                              ble_store_config_our_secs,
                              sizeof *ble_store_config_our_secs,
                              &ble_store_config_num_our_secs);
                BLE_HS_LOG(DEBUG, "config number = %d,",ble_store_config_num_our_secs);
            } else if(BLE_STORE_CONFIG_PEER_SEC == type) {
                ble_store_config_deserialize_arr(
                              out,
                              ble_store_config_peer_secs,
                              sizeof *ble_store_config_peer_secs,
                              &ble_store_config_num_peer_secs);
                BLE_HS_LOG(DEBUG, "config number = %d,",ble_store_config_num_peer_secs);
            } else if(BLE_STORE_CONFIG_CCCD == type) {
                ble_store_config_deserialize_arr(
                              out,
                              ble_store_config_cccds,
                              sizeof *ble_store_config_cccds,
                              &ble_store_config_num_cccds);
                BLE_HS_LOG(DEBUG, "config number = %d,",ble_store_config_num_cccds);
            } else {
                BLE_HS_LOG(ERROR, "there is no config saved.");
            }
            BLE_HS_LOG(DEBUG, "\n");
        }
        fclose(ble_store_file);
    }
}

static int
ble_store_config_persist_val_write(uint8_t type, char * buf, uint16_t len)
{
    FAR FILE * ble_store_file = NULL;
    const char *file_path = NULL;
    uint32_t w_len = 0;

    if(BLE_STORE_CONFIG_OUR_SEC == type) {
        file_path = BLE_STORE_OUR_SEC_PATH;
    } else if(BLE_STORE_CONFIG_PEER_SEC == type) {
        file_path = BLE_STORE_PEER_SEC_PATH;
    } else if(BLE_STORE_CONFIG_CCCD == type) {
        file_path = BLE_STORE_CCCD_PATH;
    } else {
        BLE_HS_LOG(ERROR, "\nBLE config type not defined...\n");
        return BLE_HS_ESTORE_FAIL;
    }

    ble_store_file = fopen(file_path, "r+");
    if (NULL == ble_store_file) {
        BLE_HS_LOG(ERROR, "\nOpen config file %s failed...\n", file_path);
        return BLE_HS_ESTORE_FAIL;
    }

    w_len = fwrite(buf, 1, len, ble_store_file);
    if (w_len != len) {
        BLE_HS_LOG(ERROR, "\nWrite config file %s failed...\n", file_path);
        fclose(ble_store_file);
        return BLE_HS_ESTORE_FAIL;
    }
    fclose(ble_store_file);
    return 0;
}

static int
ble_store_config_conf_set(int argc, char **argv, char *val)
{
    int rc;

    if (argc == 1) {
        if (strcmp(argv[0], "our_sec") == 0) {
            rc = ble_store_config_deserialize_arr(
                    val,
                    ble_store_config_our_secs,
                    sizeof *ble_store_config_our_secs,
                    &ble_store_config_num_our_secs);
            return rc;
        } else if (strcmp(argv[0], "peer_sec") == 0) {
            rc = ble_store_config_deserialize_arr(
                    val,
                    ble_store_config_peer_secs,
                    sizeof *ble_store_config_peer_secs,
                    &ble_store_config_num_peer_secs);
            return rc;
        } else if (strcmp(argv[0], "cccd") == 0) {
            rc = ble_store_config_deserialize_arr(
                    val,
                    ble_store_config_cccds,
                    sizeof *ble_store_config_cccds,
                    &ble_store_config_num_cccds);
            return rc;
        }
    }
    return OS_ENOENT;
}

static int
ble_store_config_conf_export(void (*func)(char *name, char *val),
                             enum conf_export_tgt tgt)
{
    union {
        char sec[BLE_STORE_CONFIG_SEC_SET_ENCODE_SZ];
        char cccd[BLE_STORE_CONFIG_CCCD_SET_ENCODE_SZ];
    } buf;

    ble_store_config_serialize_arr(ble_store_config_our_secs,
                                   sizeof *ble_store_config_our_secs,
                                   ble_store_config_num_our_secs,
                                   buf.sec,
                                   sizeof buf.sec);
    func("ble_hs/our_sec", buf.sec);

    ble_store_config_serialize_arr(ble_store_config_peer_secs,
                                   sizeof *ble_store_config_peer_secs,
                                   ble_store_config_num_peer_secs,
                                   buf.sec,
                                   sizeof buf.sec);
    func("ble_hs/peer_sec", buf.sec);

    ble_store_config_serialize_arr(ble_store_config_cccds,
                                   sizeof *ble_store_config_cccds,
                                   ble_store_config_num_cccds,
                                   buf.cccd,
                                   sizeof buf.cccd);
    func("ble_hs/cccd", buf.cccd);

    return 0;
}

static int
ble_store_config_persist_sec_set(uint8_t type,
                                 const struct ble_store_value_sec *secs,
                                 int num_secs)
{
    int rc;

    memset(ble_store_sec, 0, sizeof(ble_store_sec));
    ble_store_config_serialize_arr(secs, sizeof *secs, num_secs,
                                   ble_store_sec, sizeof(ble_store_sec));
    rc = ble_store_config_persist_val_write(type, ble_store_sec, sizeof(ble_store_sec));
    if (rc != 0) {
        return BLE_HS_ESTORE_FAIL;
    }

    return 0;
}

int
ble_store_config_persist_our_secs(void)
{
    int rc;

    rc = ble_store_config_persist_sec_set(BLE_STORE_CONFIG_OUR_SEC,
                                          ble_store_config_our_secs,
                                          ble_store_config_num_our_secs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

int
ble_store_config_persist_peer_secs(void)
{
    int rc;

    rc = ble_store_config_persist_sec_set(BLE_STORE_CONFIG_PEER_SEC,
                                          ble_store_config_peer_secs,
                                          ble_store_config_num_peer_secs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

int
ble_store_config_persist_cccds(void)
{
    int rc;

    memset(ble_store_cccd, 0, sizeof(ble_store_cccd));

    ble_store_config_serialize_arr(ble_store_config_cccds,
                                   sizeof *ble_store_config_cccds,
                                   ble_store_config_num_cccds,
                                   ble_store_cccd,
                                   sizeof(ble_store_cccd));
    rc = ble_store_config_persist_val_write(BLE_STORE_CONFIG_CCCD, ble_store_cccd, sizeof(ble_store_cccd));
    if (rc != 0) {
        return BLE_HS_ESTORE_FAIL;
    }

    return 0;
}

void
ble_store_config_conf_init(void)
{
/* Use nuttx FAT file system, which based on MTD devices, to save config into internal/external flash.*/
/* In nimble, we need save three type key for SC, our_sec, peer_sec, cccd.*/

    /*Variable not used in nuttx way.*/
    (void) ble_store_config_conf_handler;

    /*1. Our SEC*/
    ble_store_config_persist_val_read(BLE_STORE_CONFIG_OUR_SEC);
    /*2. Peer SEC*/
    ble_store_config_persist_val_read(BLE_STORE_CONFIG_PEER_SEC);
    /*3. CCCD*/
    ble_store_config_persist_val_read(BLE_STORE_CONFIG_CCCD);
}

#endif /* MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST) */
