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

#ifndef H_SYSINIT_
#define H_SYSINIT_

#include <inttypes.h>
#include <assert.h>
#include "syscfg/syscfg.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t sysinit_active;
extern int printf(FAR const IPTR char *fmt, ...);

void sysinit_start(void);
void sysinit_end(void);

#define SYSINIT_PANIC_MSG(msg)    printf("NIMBLE_PANIC_%s_%d_%s.",__FILE__, __LINE__, msg)

#define SYSINIT_PANIC_ASSERT_MSG(rc, msg) do \
{                                            \
    if (!(rc)) {                             \
        SYSINIT_PANIC_MSG(msg);              \
    }                                        \
    ASSERT(rc);                              \
} while (0)

#define SYSINIT_PANIC_ASSERT(rc) SYSINIT_PANIC_ASSERT_MSG(rc, NULL)

/**
 * Asserts that system initialization is in progress.  This macro is used to
 * ensure packages don't get initialized a second time after system
 * initialization has completed.
 */
#if MYNEWT_VAL(SYSINIT_CONSTRAIN_INIT)
#define SYSINIT_ASSERT_ACTIVE() assert(sysinit_active)
#else
#define SYSINIT_ASSERT_ACTIVE()
#endif

void
nimble_sysinit_start(void);

void
nimble_sysinit_end(void);

#ifdef __cplusplus
}
#endif

#endif
