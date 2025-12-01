/*
* Copyright (c) 2020 AIIT XUOS Lab
* XiOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/

/**
 * @file imxrt_sdhc_automount.c
 * @brief imxrt board sd card automount
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022.04.07
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_FS_AUTOMOUNTER_DEBUG) && !defined(CONFIG_DEBUG_FS)
#  define CONFIG_DEBUG_FS 1
#endif

#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/automount.h>
#include "hardware/imxrt_pinmux.h"
#include "xidatong.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL (FAR void *)0
#endif

#ifndef OK
#  define OK 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the changeable state of the automounter */

struct imxrt_automount_state_s
{
  volatile automount_handler_t handler;    /* Upper half handler */
  FAR void *arg;                           /* Handler argument */
  bool enable;                             /* Fake interrupt enable */
  bool pending;                            /* Set if there an event while disabled */
};

/* This structure represents the static configuration of an automounter */

struct imxrt_automount_config_s
{
  /* This must be first thing in structure so that we can simply cast from
   * struct automount_lower_s to struct imxrt_automount_config_s
   */

  struct automount_lower_s lower;            /* Publicly visible part */
  FAR struct imxrt_automount_state_s *state; /* Changeable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  imxrt_sdhc_attach(FAR const struct automount_lower_s *lower,
        automount_handler_t isr, FAR void *arg);
static void imxrt_sdhc_enable(FAR const struct automount_lower_s *lower,
        bool enable);
static bool imxrt_sdhc_inserted(FAR const struct automount_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imxrt_automount_state_s g_sdhc_state;
static const struct imxrt_automount_config_s g_sdhc_config =
{
  .lower        =
    {
    .fstype     = CONFIG_XIDATONG_SDIO_AUTOMOUNT_FSTYPE,
    .blockdev   = CONFIG_XIDATONG_SDIO_AUTOMOUNT_BLKDEV,
    .mountpoint = CONFIG_XIDATONG_SDIO_AUTOMOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(CONFIG_XIDATONG_SDIO_AUTOMOUNT_DDELAY),
    .udelay     = MSEC2TICK(CONFIG_XIDATONG_SDIO_AUTOMOUNT_UDELAY),
    .attach     = imxrt_sdhc_attach,
    .enable     = imxrt_sdhc_enable,
    .inserted   = imxrt_sdhc_inserted
    },
  .state        = &g_sdhc_state
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  imxrt_sdhc_attach
 *
 * Description:
 *   Attach a new SDHC event handler
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *   isr   - The new event handler to be attach
 *   arg   - Client data to be provided when the event handler is invoked.
 *
 *  Returned Value:
 *    Always returns OK
 *
 ****************************************************************************/

static int imxrt_sdhc_attach(FAR const struct automount_lower_s *lower,
        automount_handler_t isr, FAR void *arg)
{
  FAR const struct imxrt_automount_config_s *config;
  FAR struct imxrt_automount_state_s *state;

  /* Recover references to our structure */

  config = (FAR struct imxrt_automount_config_s *)lower;
  DEBUGASSERT(config != NULL && config->state != NULL);

  state = config->state;

  /* Save the new handler info (clearing the handler first to eliminate race
   * conditions).
   */

  state->handler = NULL;
  state->pending = false;
  state->arg     = arg;
  state->handler = isr;
  return OK;
}

/****************************************************************************
 * Name:  imxrt_sdhc_enable
 *
 * Description:
 *   Enable card insertion/removal event detection
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *   enable - True: enable event detection; False: disable
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

static void imxrt_sdhc_enable(FAR const struct automount_lower_s *lower,
        bool enable)
{
  FAR const struct imxrt_automount_config_s *config;
  FAR struct imxrt_automount_state_s *state;
  irqstate_t flags;

  /* Recover references to our structure */

  config = (FAR struct imxrt_automount_config_s *)lower;
  DEBUGASSERT(config != NULL && config->state != NULL);

  state = config->state;

  /* Save the fake enable setting */

  flags = enter_critical_section();
  state->enable = enable;

  /* Did an interrupt occur while interrupts were disabled? */

  if (enable && state->pending)
    {
      /* Yes.. perform the fake interrupt if the interrutp is attached */

      if (state->handler)
        {
          uint8_t inserted = imxrt_gpio_read(PIN_USDHC1_CD);
          if (0 == inserted)
            {
              state->handler(&config->lower, state->arg, true);
            }
          else
            {
              state->handler(&config->lower, state->arg, false);
            }
        }

      state->pending = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: imxrt_sdhc_inserted
 *
 * Description:
 *   Check if a card is inserted into the slot.
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *
 *  Returned Value:
 *    True if the card is inserted; False otherwise
 *
 ****************************************************************************/

static bool imxrt_sdhc_inserted(FAR const struct automount_lower_s *lower)
{
  uint8_t inserted = imxrt_gpio_read(PIN_USDHC1_CD);
  if (0 == inserted)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  imxrt_sdhc_automount_event
 *
 * Description:
 *   The SDHC card detection logic has detected an insertion or removal
 *   event.
 *   It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the SDHC0 slot: SDHC0_SLOTNO or SDHC1_SLOTNO.
 *      There is a terminology problem here:  Each SDHC supports two slots,
 *      slot A and slot B. Only slot A is used.
 *      So this is not a really a slot, but an HSCMI peripheral number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

int imxrt_sdhc_automount_event(void *arg)
{
  FAR const struct imxrt_automount_config_s *config = &g_sdhc_config;
  FAR struct imxrt_automount_state_s *state = &g_sdhc_state;

  /* Is the auto-mounter interrupt attached? */

  if (state->handler)
    {
      /* Yes.. Have we been asked to hold off interrupts? */

      if (!state->enable)
        {
          /* Yes.. just remember that there is a pending interrupt. We will
           * deliver the interrupt when interrupts are "re-enabled."
           */

          state->pending = true;
        }
      else
        {
          /* No.. forward the event to the handler */

          uint8_t inserted = imxrt_gpio_read(PIN_USDHC1_CD);
          if (0 == inserted)
            {
              state->handler(&config->lower, state->arg, true);
            }
          else
            {
              state->handler(&config->lower, state->arg, false);
            }
        }
    }
    return 0;
}

/****************************************************************************
 * Name:  imxrt_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured SDHC
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

void imxrt_automount_initialize(void)
{
  FAR void *handle;

  finfo("Initializing automounter(s)\n");

  /* Initialize the SDHC0 auto-mounter */

  handle = automount_initialize(&g_sdhc_config.lower);
  if (!handle)
    {
      ferr("ERROR: Failed to initialize auto-mounter for SDHC0\n");
    }
}
