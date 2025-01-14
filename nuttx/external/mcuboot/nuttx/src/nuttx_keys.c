/****************************************************************************
 * nuttx_keys.c
 *
 *   Copyright (C) 2011, 2013, 2015-2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017, 2018  Zglue  Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Levin Li     <zhiqiang@zglue.com>
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
#include <bootutil/sign_key.h>

#if defined(MCUBOOT_SIGN_RSA)

static const unsigned char root_rsa_2048_public_der[] = {
  0x30, 0x82, 0x01, 0x0a, 0x02, 0x82, 0x01, 0x01,
  0x00, 0xc9, 0x33, 0x80, 0xc9, 0x59, 0x4a, 0xa1,
  0xe3, 0x62, 0x33, 0x0f, 0x1e, 0xe6, 0x78, 0x19,
  0x62, 0x69, 0x8d, 0x1b, 0x46, 0xbe, 0x7b, 0xd6,
  0xb6, 0x4a, 0x06, 0x2f, 0xef, 0xda, 0x37, 0x6e,
  0xe7, 0x49, 0x03, 0x70, 0x77, 0x77, 0x38, 0x1d,
  0x0a, 0xd1, 0xca, 0x1b, 0xf8, 0x54, 0x2c, 0x04,
  0xe3, 0x8a, 0x82, 0x87, 0x74, 0x7b, 0xce, 0xd1,
  0x13, 0x3e, 0x17, 0xe9, 0xc9, 0x23, 0x27, 0x09,
  0x3f, 0xf2, 0xb6, 0x37, 0xbf, 0x21, 0x2e, 0x36,
  0x75, 0x6a, 0x47, 0x84, 0x67, 0xcb, 0x7d, 0xe6,
  0x4d, 0x6a, 0x05, 0xf1, 0x5b, 0xbe, 0x48, 0xff,
  0x22, 0x99, 0x11, 0x18, 0xa8, 0x7e, 0x28, 0x2f,
  0x48, 0xa3, 0x13, 0x11, 0x79, 0x56, 0x9f, 0x8e,
  0x0b, 0x0f, 0x1e, 0x3e, 0xdb, 0x5a, 0x3a, 0x38,
  0x13, 0x52, 0x48, 0xe2, 0x6d, 0xdf, 0x88, 0xe4,
  0x6f, 0xee, 0xf3, 0x8f, 0xdc, 0xc9, 0x53, 0xb9,
  0x40, 0xb3, 0x4c, 0x92, 0xd3, 0xb3, 0x69, 0x67,
  0x7c, 0x98, 0xcf, 0x15, 0x43, 0xdf, 0xbc, 0xfb,
  0x6b, 0x3a, 0x7a, 0x8d, 0x08, 0xa3, 0xc7, 0x99,
  0xbf, 0x63, 0x59, 0xf5, 0x10, 0x91, 0xf0, 0x7d,
  0x98, 0xe8, 0x03, 0x1b, 0x53, 0x2f, 0x7c, 0x8a,
  0xa6, 0xad, 0x67, 0xbc, 0x65, 0x43, 0xd7, 0x57,
  0xb3, 0xe5, 0xcc, 0x47, 0x48, 0x79, 0x85, 0xd4,
  0x7b, 0x0c, 0x46, 0xa8, 0x3e, 0xa5, 0x21, 0xaa,
  0xa0, 0xa8, 0x72, 0x5f, 0x66, 0x74, 0xd3, 0x59,
  0x69, 0x62, 0xda, 0x5f, 0xd3, 0x13, 0x20, 0x16,
  0x3e, 0x53, 0xc5, 0xdb, 0x62, 0x12, 0x8b, 0x87,
  0x20, 0xec, 0xff, 0xa6, 0xf5, 0x49, 0x4a, 0x07,
  0x80, 0x96, 0x90, 0xe8, 0xdd, 0x47, 0x73, 0xc8,
  0x2b, 0x7a, 0xd3, 0x7d, 0xcb, 0xe7, 0x42, 0x9f,
  0x86, 0x3c, 0xef, 0xd2, 0x0a, 0x1e, 0x1a, 0xd7,
  0x3b, 0x14, 0x33, 0x17, 0x90, 0x80, 0x13, 0x9d,
  0xad, 0x02, 0x03, 0x01, 0x00, 0x01,
};
static const unsigned int root_rsa_2048_public_der_len = 270;

#elif defined(MCUBOOT_SIGN_EC256)

static const unsigned char root_ecdsa_256_public_der[] = {
  0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02,
  0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03,
  0x42, 0x00, 0x04, 0x74, 0x50, 0x2e, 0x28, 0x64, 0x21, 0xdc, 0x95, 0x4e,
  0xa4, 0xe4, 0x4b, 0x29, 0x95, 0xca, 0x04, 0x8a, 0x90, 0xa6, 0x66, 0x93,
  0xe2, 0x31, 0x25, 0xd2, 0xc3, 0xca, 0x61, 0x0f, 0x31, 0x60, 0x1f, 0x7e,
  0x28, 0xb6, 0xf3, 0xde, 0x53, 0xa1, 0xb7, 0xb6, 0x1d, 0x31, 0xf1, 0xb9,
  0xdc, 0xb9, 0x0b, 0x40, 0xd6, 0x45, 0x8d, 0x66, 0x42, 0x4b, 0xc1, 0x78,
  0x36, 0xf1, 0x35, 0xfc, 0xd5, 0xcf, 0xa1
};
static const unsigned int root_ecdsa_256_public_der_len = 91;

#else
#error "No public key available for given signing algorithm."
#endif


const struct bootutil_key bootutil_keys[] = {

#ifdef MCUBOOT_SIGN_RSA
    {
        .key = root_rsa_2048_public_der,
        .len = &root_rsa_2048_public_der_len,
    },
#endif

#ifdef MCUBOOT_SIGN_EC256
    {
        .key = root_ecdsa_256_public_der,
        .len = &root_ecdsa_256_public_der_len,
    },
#endif

};

const int bootutil_key_cnt = sizeof(bootutil_keys)/sizeof(struct bootutil_key);

