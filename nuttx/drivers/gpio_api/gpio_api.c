

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include "gpio_api.h"

#if defined(CONFIG_ARCH_VIRTUAL_GPIO)

typedef struct
{
  uint8_t   pin_mask;
}

#endif
