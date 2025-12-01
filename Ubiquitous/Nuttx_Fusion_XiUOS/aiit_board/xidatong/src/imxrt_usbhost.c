/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/src/imxrt_usbhost.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <sys/mount.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/ehci.h>
#include <nuttx/wdog.h>

#include <imxrt_ehci.h>

#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_usbotg.h"
#include "imxrt_periphclks.h"
#include "xidatong.h"

#include <arch/board/board.h>  /* Must always be included last */

#if defined(CONFIG_IMXRT_USBOTG) || defined(CONFIG_USBHOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_USBHOST_DEFPRIO
#  define CONFIG_USBHOST_DEFPRIO 50
#endif

#ifndef CONFIG_USBHOST_STACKSIZE
#  ifdef CONFIG_USBHOST_HUB
#    define CONFIG_USBHOST_STACKSIZE 1536
#  else
#    define CONFIG_USBHOST_STACKSIZE 1024
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Retained device driver handle */

static struct usbhost_connection_s *g_ehciconn;

#ifdef CONFIG_XIDATONG_USB_AUTOMOUNT
/* Unmount retry timer */

static struct wdog_s g_umount_tmr[CONFIG_XIDATONG_USB_AUTOMOUNT_NUM_BLKDEV];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ehci_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to the EHCI root hub.
 *
 ****************************************************************************/

static int ehci_waiter(int argc, char *argv[])
{
  FAR struct usbhost_hubport_s *hport;

  uinfo("ehci_waiter:  Running\n");
  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_ehciconn, &hport));
      syslog(LOG_INFO, "ehci_waiter: %s\n",
             hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(g_ehciconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_XIDATONG_USB_AUTOMOUNT
/****************************************************************************
 * Name: usb_msc_connect
 *
 * Description:
 *   Mount the USB mass storage device
 *
 ****************************************************************************/

static void usb_msc_connect(FAR void *arg)
{
  int  index  = (int)arg;
  char sdchar = 'a' + index;
  int  ret;

  char blkdev[32];
  char mntpnt[32];

  DEBUGASSERT(index >= 0 &&
              index < CONFIG_XIDATONG_USB_AUTOMOUNT_NUM_BLKDEV);

  wd_cancel(&g_umount_tmr[index]);

  /* Resetup the event. */

  usbhost_msc_notifier_setup(usb_msc_connect, WORK_USB_MSC_CONNECT,
      sdchar, arg);

  snprintf(blkdev, sizeof(blkdev), "%s%c",
      CONFIG_XIDATONG_USB_AUTOMOUNT_BLKDEV, sdchar);
  snprintf(mntpnt, sizeof(mntpnt), "%s%c",
      CONFIG_XIDATONG_USB_AUTOMOUNT_MOUNTPOINT, sdchar);

  /* Mount */

  ret = nx_mount((FAR const char *)blkdev, (FAR const char *)mntpnt,
      CONFIG_XIDATONG_USB_AUTOMOUNT_FSTYPE, 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Mount failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: unmount_retry_timeout
 *
 * Description:
 *   A previous unmount failed because the volume was busy... busy meaning
 *   the volume could not be unmounted because there are open references
 *   the files or directories in the volume.  When this failure occurred,
 *   the unmount logic setup a delay and this function is called as a result
 *   of that delay timeout.
 *
 *   This function will attempt the unmount again.
 *
 * Input Parameters:
 *   Standard wdog timeout parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void unmount_retry_timeout(wdparm_t arg)
{
  int  index  = arg;
  char sdchar = 'a' + index;

  finfo("Timeout!\n");
  DEBUGASSERT(index >= 0 &&
              index < CONFIG_XIDATONG_USB_AUTOMOUNT_NUM_BLKDEV);

  /* Resend the notification. */

  usbhost_msc_notifier_signal(WORK_USB_MSC_DISCONNECT, sdchar);
}

/****************************************************************************
 * Name: usb_msc_disconnect
 *
 * Description:
 *   Unmount the USB mass storage device
 *
 ****************************************************************************/

static void usb_msc_disconnect(FAR void *arg)
{
  int  index  = (int)arg;
  char sdchar = 'a' + index;
  int  ret;

  char mntpnt[32];

  DEBUGASSERT(index >= 0 &&
              index < CONFIG_XIDATONG_USB_AUTOMOUNT_NUM_BLKDEV);

  wd_cancel(&g_umount_tmr[index]);

  /* Resetup the event. */

  usbhost_msc_notifier_setup(usb_msc_disconnect, WORK_USB_MSC_DISCONNECT,
      sdchar, arg);

  snprintf(mntpnt, sizeof(mntpnt), "%s%c",
      CONFIG_XIDATONG_USB_AUTOMOUNT_MOUNTPOINT, sdchar);

  /* Unmount */

  ret = nx_umount2((FAR const char *)mntpnt, MNT_FORCE);
  if (ret < 0)
    {
      /* We expect the error to be EBUSY meaning that the volume could
       * not be unmounted because there are currently reference via open
       * files or directories.
       */

      if (ret == -EBUSY)
        {
          finfo("WARNING: Volume is busy, try again later\n");

          /* Start a timer to retry the umount2 after a delay */

          ret = wd_start(&g_umount_tmr[index],
                          MSEC2TICK(CONFIG_XIDATONG_USB_AUTOMOUNT_UDELAY),
                          unmount_retry_timeout, index);
          if (ret < 0)
            {
              ferr("ERROR: wd_start failed: %d\n", ret);
            }
        }

      /* Other errors are fatal */

      else
        {
          ferr("ERROR: Unmount failed!\n");
        }
    }
}
#endif /* CONFIG_XIDATONG_USB_AUTOMOUNT */


/****************************************************************************
 * Name: imxrt_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

int imxrt_usbhost_initialize(void)
{
  pid_t pid;
  int ret;
#ifdef CONFIG_XIDATONG_USB_AUTOMOUNT
  int index;
#endif

  imxrt_clockall_usboh3();

  /* Make sure we don't accidentally switch on USB bus power */

  *((uint32_t *)IMXRT_USBNC_USB_OTG2_CTRL) = USBNC_PWR_POL;
  *((uint32_t *)0x400d9030)                = (1 << 21);
  *((uint32_t *)0x400d9000)                = 0;

  /* Setup pins, with power initially off */

  imxrt_config_gpio(GPIO_USBOTG_ID);

  /* First, register all of the class drivers needed to support the drivers
   * that we care about
   */

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB host Mass Storage Class */

#ifdef CONFIG_XIDATONG_USB_AUTOMOUNT
  /* Initialize the notifier listener for automount */

  for (index = 0; index < CONFIG_XIDATONG_USB_AUTOMOUNT_NUM_BLKDEV; index++)
    {
      char sdchar = 'a' + index;

      usbhost_msc_notifier_setup(usb_msc_connect,
          WORK_USB_MSC_CONNECT, sdchar, (FAR void *)(intptr_t)index);
      usbhost_msc_notifier_setup(usb_msc_disconnect,
          WORK_USB_MSC_DISCONNECT, sdchar, (FAR void *)(intptr_t)index);
    }
#endif

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the CDC/ACM serial class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Register the USB host HID keyboard class driver */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the KBD class\n");
    }
#endif

  /* Then get an instance of the USB EHCI interface. */

  g_ehciconn = imxrt_ehci_initialize(0);

  if (!g_ehciconn)
    {
      uerr("ERROR: imxrt_ehci_initialize failed\n");
      return -ENODEV;
    }

  /* Start a thread to handle device connection. */

  pid = kthread_create("EHCI Monitor", CONFIG_USBHOST_DEFPRIO,
                       CONFIG_USBHOST_STACKSIZE,
                       (main_t)ehci_waiter, (FAR char * const *)NULL);
  if (pid < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each platform that implements the OHCI or EHCI host
 *   interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.
 *            Since the IMXRT has only a downstream port, zero is
 *            the only possible value for this parameter.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define HCOR ((volatile struct ehci_hcor_s *)IMXRT_USBOTG_HCOR_BASE)

void imxrt_usbhost_vbusdrive(int rhport, bool enable)
{
  uint32_t regval;

  uinfo("RHPort%d: enable=%d\n", rhport + 1, enable);

  /* The IMXRT has only a single root hub port */

  if (rhport == 0)
    {
      /* Then enable or disable VBUS power */

      regval = HCOR->portsc[rhport];
      regval &= ~EHCI_PORTSC_PP;
      if (enable)
        {
          regval |= EHCI_PORTSC_PP;
        }

      HCOR->portsc[rhport] = regval;
    }
}

/****************************************************************************
 * Name: imxrt_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent condition
 *   is detected.
 *
 * Input Parameters:
 *   handler - New overcurrent interrupt handler
 *   arg     - The argument that will accompany the interrupt
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#if 0 /* Not ready yet */
int imxrt_setup_overcurrent(xcpt_t handler, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Configure the interrupt */

#warning Missing logic

  leave_critical_section(flags);
  return OK;
}
#endif /* 0 */

#endif /* CONFIG_IMXRT_USBOTG || CONFIG_USBHOST */
