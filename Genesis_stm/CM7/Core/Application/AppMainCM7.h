#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ClockHandling.h"
#include "Serializer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "SpeedEstimation.h"



void AppCM7_Main();