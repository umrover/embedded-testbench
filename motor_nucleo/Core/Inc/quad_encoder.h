#pragma once

#include <closed_loop_control.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_it.h"

#include "pin.h"
#include "hbridge.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    TIM_TypeDef *tim;

    int32_t counts;
    int16_t counts_raw_prev;
    int16_t counts_raw_now;
} QuadEncoder;

void init_quad_encoder(QuadEncoder* quad_encoder);

QuadEncoder *new_quad_encoder(TIM_HandleTypeDef *_htim, TIM_TypeDef *_tim);

void update_quad_encoder(QuadEncoder* quad_encoder);

void set_encoder_counts(QuadEncoder *quad_encoder, int32_t counts);
