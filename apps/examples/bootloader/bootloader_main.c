/****************************************************************************
 * examples/mcuboot/bootloader_main.c
 *
 *   Copyright (C) 2017 Zglue. All rights reserved.
 *   Author: Levin Li <zhiqiang@zglue.com>
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
#include <stdio.h>
#include <syslog.h>
#include <nuttx/irq.h>
#ifndef __ASSEMBLY__
#include <nuttx/compiler.h>
#include <stdint.h>
#endif

#include "bootutil/image.h"
#include "bootutil/bootutil.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/
extern void os_heap_init(void);
extern int flash_device_base(uint8_t fd_id, uintptr_t *ret);
/****************************************************************************
 * bootloader_main
 ****************************************************************************/
void bootloader_app_start_impl(uint32_t start_addr)
{
  __asm__ __volatile__
  (
    "ldr   r0, [%0]\t\n"            // Get App initial MSP for bootloader.
    "msr   msp, r0\t\n"             // Set the main stack pointer to the applications MSP.
    "ldr   r0, [%0, #0x04]\t\n"     // Load Reset handler into R0.

    "movs  r4, #0xFF\t\n"           // Move ones to R4.
    "sxtb  r4, r4\t\n"              // Sign extend R4 to obtain 0xFFFFFFFF instead of 0xFF.

    "mrs   r5, IPSR\t\n"            // Load IPSR to R5 to check for handler or thread mode.
    "cmp   r5, #0x00\t\n"           // Compare, if 0 then we are in thread mode and can continue to reset handler of bootloader.
    "bne   isr_abort\t\n"           // If not zero we need to exit current ISR and jump to reset handler of bootloader.

    "mrs   r1, control\t\n"         // Get CONTROL register value
    "movs  r2, #0x02\t\n"           // load 2 to r2
    "bic   r1, r2\t\n"              // clear value of CONTROL->SPSEL - > make sure MSP will be used
    "msr   control, r1\t\n"         // set the stack pointer to MSP

    "mov   lr, r4\t\n"              // Clear the link register and set to ones to ensure no return.
    "bx    r0\t\n"                  // Branch to reset handler of bootloader.

    "isr_abort:  \t\n"

    "mov   r5, r4\t\n"              // Fill with ones before jumping to reset handling. Will be popped as LR when exiting ISR. Ensures no return to application.
    "mov   r6, r0\t\n"              // Move address of reset handler to R6. Will be popped as PC when exiting ISR. Ensures the reset handler will be executed when exist ISR.
    "movs  r7, #0x21\t\n"           // Move MSB reset value of xPSR to R7. Will be popped as xPSR when exiting ISR. xPSR is 0x21000000 thus MSB is 0x21.
    "rev   r7, r7\t\n"              // Reverse byte order to put 0x21 as MSB.
    "push  {r4-r7}\t\n"             // Push everything to new stack to allow interrupt handler to fetch it on exiting the ISR.

    "movs  r4, #0x00\t\n"           // Fill with zeros before jumping to reset handling. We be popped as R0 when exiting ISR (Cleaning up of the registers).
    "movs  r5, #0x00\t\n"           // Fill with zeros before jumping to reset handling. We be popped as R1 when exiting ISR (Cleaning up of the registers).
    "movs  r6, #0x00\t\n"           // Fill with zeros before jumping to reset handling. We be popped as R2 when exiting ISR (Cleaning up of the registers).
    "movs  r7, #0x00\t\n"           // Fill with zeros before jumping to reset handling. We be popped as R3 when exiting ISR (Cleaning up of the registers).
    "push  {r4-r7}\t\n"             // Push zeros (R4-R7) to stack to prepare for exiting the interrupt routine.

    "movs  r0, #0xF9\t\n"           // Move the execution return command into register, 0xFFFFFFF9.
    "sxtb  r0, r0\t\n"              // Sign extend R0 to obtain 0xFFFFFFF9 instead of 0xF9.
    "bx    r0\t\n"                  // No return - Handler mode will be exited. Stack will be popped and execution will continue in reset handler initializing other application.
    ".align\t\n"
    :: "r" (start_addr)             // Argument list for the gcc assembly. start_addr is %0.
    :  "r0", "r4", "r5", "r6", "r7" // List of register maintained manually.
  );
}

static void do_boot(struct boot_rsp *rsp)
{
  uintptr_t flash_base;
  int rc;

  /* The beginning of the image is the ARM vector table, containing
   * the initial stack pointer address and the reset vector
   * consecutively. Manually set the stack pointer and jump into the
   * reset vector
   */
  rc = flash_device_base(rsp->br_flash_dev_id, &flash_base);
  assert(rc == 0);

  up_irq_disable();

  bootloader_app_start_impl(flash_base +
                            rsp->br_image_off +
                            rsp->br_hdr->ih_hdr_size);

}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int bootloader_main(int argc, char *argv[])
#endif
{
  struct boot_rsp rsp;
  int rc;

  syslog(LOG_INFO, "Starting bootloader\n");

  os_heap_init();

  rc = boot_go(&rsp);
  if (rc != 0)
    {
      printf("Unable to find bootable image\n");
      while (1);
    }

  syslog(LOG_INFO, "Bootloader chainload address offset: 0x%x\n",
         rsp.br_image_off);

  printf("Jumping to the first image slot\n");

  do_boot(&rsp);

  return 0;
}
