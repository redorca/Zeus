/****************************************************************************
 * apps/utils/step_counting.c
 *
 *   Copyright (C) 2018 zGlue Inc. All rights reserved.
 *   Author: Zhengwei Wang <zhengwei@zglue.com>
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sensors/ioctl.h>
#include <utils/step_counting.h>

#ifdef CONFIG_STEPCNT_ALGO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define STEP_CNT_RESOLUTION 8
#define STEP_CNT_ODR 540
#define STEP_CNT_RANGE 4

#define STEP_COUNTING_THREAD_STACK_SIZE 2048
#define ACCELE_SAMPLE_COUNT 30
#define ACCELE_SLEEP_TIME_MS 400

#define PEAK_SUCCESSIVE_RISE 2
#define VALLEY_SUCCESSIVE_DOWN 2

//combine = sqrt(x*x + y*y + z*z),the range of x,y,z axis is -127 to +127,so the maximum of combine value is 127*sqrt(3)
#define MAX_COMBINE_VALUE  220
//the floating weight coefficient is {0.01509692,0.03662365,0.09209563,0.15669359,0.19949021,0.19949021,0.15669359,0.09209563,0.03662365,0.01509692}
//convert float number to fixed number, fixed = float*pow(2,24)
#define FLOATING_TO_FIXED 24
#define WEIGHTING_COEFFICIENT {253284,  614443, 1545108,  2628882,  3346890,  3346890,  2628882,  1545108,  614443, 253284}
//set this value to true if the corresponding weight coefficient is minus, otherwise is false.
#define WEIGHT_SIGN           {false,   false,  false,    false,    false,    false,    false,    false,    false,  false,}
#define LENTH_WEIGHT          10

//pattern match
#define ENTER_PATTERN_THRESHOLD 5 //if 5 continuous matched peaks detected, it enters run/walk pattern
#define LEAVE_PATTERN_THRESHOLD 5//if 5 continuous unmatched peaks detected, it leaves run/walk pattern
#define NO_PEAK_THRESHOLD 100
//walk
/*if a peak matches walk pattern,that means its amplitude is in the range of (WALK_AMPTITUDE_MIN, WALK_AMPTITUDE_MAX)
and its distance from last peak is in range of (WALK_PERIOD_MIN, WALK_PERIOD_MAX)*/
#define WALK_AMPTITUDE_MIN 30
#define WALK_AMPTITUDE_MAX 60
#define WALK_PERIOD_MIN 15
#define WALK_PERIOD_MAX 100
//run
#define RUN_AMPTITUDE_MIN 50
#define RUN_AMPTITUDE_MAX 220
#define RUN_PERIOD_MIN 10
#define RUN_PERIOD_MAX 40

//"gap" points to the distance in amplitude between peak and valley
#define GAP_RECORD_NUM 4
#define INITIAL_GAP_VALUE 5

#define GAP_STAIR_VALUE {{20,5},{30,10},{50,20},{100,30},{0xFF,40}}
#define LENTH_GAP_STAIR_ARRAY 5
#define GAP_MAX_COUNTABLE 150
#define VALLEY_MAX_COUNTABLE 80
#define NO_MATCH_PEAK_THRESHOLD 150
#define STARTING_FARE_FOR_CONTINUE_COUNT 8

#define STEP_MQ_NAME_FORMATE "STEP_%8X"
#define STEP_MQ_NAME_LENTH  15
#define STEP_MQ_MAX_MSG     4
#define STEP_MQ_MSG_SIZE    1
#define STEP_MQ_PRIO 1

#define INT16_IN_ONE_SAMPLE 3
#define PREVENT_ROLLBACK_UINT8 254

enum
{
  FLAT_NO,
  FLAT_PEAK,
  FLAT_VALLEY,
};



typedef enum
{
  STEP_CMD_STOP,
  STEP_CMD_RESET,
} STEP_cmd;

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef struct _Type_FIR_handler_
{
  const uint32_t *ptr_weight;
  const bool *ptr_weight_sign;
  uint8_t lenth_weight;
  uint8_t float_to_fixed_shift;
  uint16_t FIR_history_data[LENTH_WEIGHT + 1];
} FIR_handler_t;

typedef struct _Type_PATT_extra_handler_
{
  int16_t last_slope;
  uint16_t last_data;
  uint16_t record;
  uint8_t count_positive;
  uint8_t count_negative;
  uint8_t flat_flag;
} PATT_extra_handler_t;

typedef struct _Type_peak_
{
  uint16_t value;
  bool flag_peak;
  bool flag_valley;
} peak_t;

typedef struct _Type_pattern_match_
{
  uint8_t value;//point value
  bool peak_or_not;//whether this point a peak or not
  bool walk_match;//whether this point matches walk pattern
  bool run_match;//whether this point matches run pattern
  uint8_t last_valley;//if this point is a peak and it matches walk or run pattern, this value gives the corresponding valley value
} pattern_match_t;

typedef struct _Type_STEP_algo_result_
{
  uint32_t step_total;
  step_patt motion_pattern;
} step_algo_result_t;


typedef struct _Type_STEP_counter_private_
{
  const char *data_source_file;
  step_algo_result_t *result;
  FIR_handler_t FIR_filter;
  uint8_t gap_record[GAP_RECORD_NUM + 1];
  uint8_t gap_threshold;
  uint8_t count_last_peak;//count how many point passed since last peak
  bool get_last_valley;//True if we got a valley before a peak appear
  uint8_t record_last_valley;//record the valley we got
  uint8_t pattern_walk_count;//count continuous walk pattern matched peaks
  uint8_t pattern_run_count;//count continuous run pattern matched peaks
  uint8_t count_no_peak;//the same function with count_last_peak
  bool whether_in_run_pattern;
  bool whether_in_walk_pattern;
  bool whether_in_static_pattern;
  uint8_t count_no_match_peak;//count how many point passed since last peak that match walk or run pattern
  uint8_t init_count;
  PATT_extra_handler_t STEP_pattern;
} STEP_counter_private;



typedef struct _Type_STEP_counter_t_
{
  bool running;
  pthread_t thread_id;
  const step_algo_result_t step_result;
  mqd_t mq;
} step_counter_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
const static uint8_t threshold_table[LENTH_GAP_STAIR_ARRAY][2] = GAP_STAIR_VALUE;
const static uint32_t FIR_weight[] = WEIGHTING_COEFFICIENT;
const static bool wight_sign[] = WEIGHT_SIGN;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/*a simple square function using dichotomy*/
uint16_t int_sqrt(uint16_t num)
{
  uint32_t mid;
  uint32_t diff;
  uint32_t low = 0;
  uint32_t high = MAX_COMBINE_VALUE;

  if (num <= 1)
    {
      return num;
    }
  if (num >= MAX_COMBINE_VALUE * MAX_COMBINE_VALUE)
    {
      return MAX_COMBINE_VALUE;
    }
  while ((high - low) > 1)
    {
      mid = (low + high) >> 1;
      diff = (mid) * (mid);
      if (diff > num)
        {
          high = mid;
        }
      else if (diff < num)
        {
          low = mid;
        }
      else
        {
          low = mid;
          break;
        }
    }

  return (uint16_t)low;
}

/*combine = sqrt(x*x + y*y + z*z), x_y_z is a pointer pointing to the address storing a set of x,y,z value*/
uint16_t calculate_combine(int16_t *x_y_z)
{
  uint16_t sum = 0;
  uint16_t temp;
  uint8_t i;

  for (i = 0; i < 3; i++)
    {
      if (*x_y_z < 0)
        {
          temp = (uint16_t)(0 - *x_y_z);
        }
      else
        {
          temp = (uint16_t)(*x_y_z);
        }

      sum += temp * temp;

      x_y_z++;
    }

  return int_sqrt(sum);
}




/*do a FIR filter, O(n) = W(n)*I(n) + W(n-1)*I(n-1)+ .... + W(0)*I(0)
O(n) stand for the Nth output data, W(n) stand for Nth wight coefficient and the I(n)
stand for the Nth input data. In order to avoid using the floating number, we must convert
the floating number weight coefficient to fixed point number, we do the convertion using
this formula W' = W*pow(2,N), W stand for floating number, W' stand for fixed point.*/
uint16_t FIR_filter(uint16_t new_data, FIR_handler_t *ptr_handler)
{
  uint8_t i;
  uint32_t sum;
  uint32_t sum_minus = 0;
  uint32_t sum_plus = 0;
  uint32_t temp_result;

  ptr_handler->FIR_history_data[ptr_handler->lenth_weight - 1] = new_data;
  for (i = 0; i < ptr_handler->lenth_weight; i++)
    {
      temp_result = ptr_handler->ptr_weight[i] * ptr_handler->FIR_history_data[i];
      if (ptr_handler->ptr_weight_sign[i] == true)
        {
          sum_minus += temp_result;
        }
      else
        {
          sum_plus += temp_result;
        }

      ptr_handler->FIR_history_data[i] = ptr_handler->FIR_history_data[i + 1];
    }

  if (sum_plus < sum_minus)
    {
      sum = 0;
    }
  else
    {
      sum = sum_plus - sum_minus;
    }

  sum = sum >> ptr_handler->float_to_fixed_shift;
  return sum;
}

/*This function draw the peaks and valleys from a data flow. If P(n) is a
peak, it must satisfy this rule, P(n-N)<P(n-N + 1)<...<P(n)>...P(n+M-1)>P(n+M).
If P(n) is a valley, it satisfy this rule, P(n-M)>P(n-M + 1)>...>P(n)<...P(n+N-1)<P(n+N).
In these two formula,M,N is two constants,and defined by PEAK_SUCCESSIVE_RISE
and VALLEY_SUCCESSIVE_DOWN respectively.*/
peak_t pattern_extraction(uint16_t data, PATT_extra_handler_t *ptr_handler )
{
  int16_t slope;
  peak_t result = {.value = 0, .flag_peak = false, .flag_valley = false};

  slope = data - ptr_handler->last_data;
  if (slope > 0)
    {
      if (ptr_handler->last_slope < 0)
        {
          ptr_handler->record = ptr_handler->last_data;
          ptr_handler->count_positive = 0;
        }
      else if (ptr_handler->last_slope == 0)
        {
          if (ptr_handler->flat_flag == FLAT_VALLEY)
            {
              ptr_handler->record = ptr_handler->last_data;
              ptr_handler->count_positive = 0;
            }
        }

      ptr_handler->count_positive ++;
      if ((ptr_handler->count_positive >= PEAK_SUCCESSIVE_RISE) && (ptr_handler->count_negative >= VALLEY_SUCCESSIVE_DOWN))
        {
          result.value = ptr_handler->record;
          result.flag_valley = true;
          ptr_handler->count_negative = 0;
        }
    }
  else if (slope == 0)
    {
      if (ptr_handler->last_slope > 0)
        {
          ptr_handler->flat_flag = FLAT_PEAK;
        }
      else if (ptr_handler->last_slope < 0)
        {
          ptr_handler->flat_flag = FLAT_VALLEY;
        }
    }
  else
    {
      if (ptr_handler->last_slope > 0)
        {
          ptr_handler->record = ptr_handler->last_data;
          ptr_handler->count_negative = 0;
        }
      else if (ptr_handler->last_slope == 0)
        {
          if (ptr_handler->flat_flag == FLAT_PEAK)
            {
              ptr_handler->record = ptr_handler->last_data;
              ptr_handler->count_negative = 0;
            }
        }

      ptr_handler->count_negative ++;
      if ((ptr_handler->count_positive >= PEAK_SUCCESSIVE_RISE) && (ptr_handler->count_negative >= VALLEY_SUCCESSIVE_DOWN))
        {
          result.value = ptr_handler->record;
          result.flag_peak = true;
          ptr_handler->count_positive = 0;
        }
    }


  ptr_handler->last_data = data;
  ptr_handler->last_slope = slope;

  return result;
}

/**/
pattern_match_t STEP_pattern_match(STEP_counter_private *priv, peak_t data)
{
//  static uint16_t STEP_data.count_last_peak = 0;
  pattern_match_t result = {data.value, false, false, false, 0};

  if (data.flag_peak == true)
    {
      result.peak_or_not = true;
      //check whether this point matches walk pattern
      if ((data.value > WALK_AMPTITUDE_MIN) && (data.value <= WALK_AMPTITUDE_MAX) && \
          (priv->count_last_peak > WALK_PERIOD_MIN) && (priv->count_last_peak <= WALK_PERIOD_MAX))
        {
          result.walk_match = true;
        }
      else
        {
          result.walk_match = false;
        }
      //check whether this point matches run pattern
      if ((data.value > RUN_AMPTITUDE_MIN) && (data.value <= RUN_AMPTITUDE_MAX) && \
          (priv->count_last_peak > RUN_PERIOD_MIN) && (priv->count_last_peak <= RUN_PERIOD_MAX))
        {
          result.run_match = true;
        }
      else
        {
          result.run_match = false;
        }

      //reset the counter for no peak time
      priv->count_last_peak = 0;

      if (priv->get_last_valley == false)
        {
          //if we didnot find a valley before a peak is detected ,we ignore this peak by setting the gap to zero
          result.last_valley = data.value;
        }
      else
        {
          result.last_valley = priv->record_last_valley;
        }

      priv->get_last_valley = false;//reset this flag because the valley value is already used
    }
  else
    {
      if (data.flag_valley == true)
        {
          priv->get_last_valley = true;
          priv->record_last_valley = data.value;
        }
      priv->count_last_peak ++;
      if (priv->count_last_peak >= PREVENT_ROLLBACK_UINT8)
        {
          priv->count_last_peak = PREVENT_ROLLBACK_UINT8;  //prevent this value to rollback
        }

    }
  return result;
}

void judge_threshold(bool *current, bool *next, uint8_t *ptr_count)
{
  uint8_t threshold;

  if (*current != *next)
    {
      (*ptr_count)++;
      threshold = (*next == true) ? ENTER_PATTERN_THRESHOLD : LEAVE_PATTERN_THRESHOLD;
      if (*ptr_count > threshold)
        {
          *current = *next;
          *ptr_count = 0;
        }
    }
  else
    {
      *ptr_count = 0;
    }
}


/*this function decide which motion pattern to show to the user.
In order to prevent the motion pattern change too frequent, we apply a delay strategy.
For example ,if we want to enter run pattern, we must collect a number of continues peaks
that matches run pattern;if we want to exit run pattern,we must collect a number of
continues unmatched peaks as well.
 */
void STEP_pattern_display(STEP_counter_private *priv, pattern_match_t pattern_match_result)
{

  if (pattern_match_result.peak_or_not == true)
    {
      priv->count_no_peak = 0;
      priv->whether_in_static_pattern = false;
      judge_threshold(&priv->whether_in_walk_pattern, &pattern_match_result.walk_match, &priv->pattern_walk_count);
      judge_threshold(&priv->whether_in_run_pattern, &pattern_match_result.run_match, &priv->pattern_run_count);
    }
  else
    {
      //if no peak is detected in a period of time, that meas user is still, so we enter static pattern
      priv->count_no_peak ++;
      if (priv->count_no_peak >= NO_PEAK_THRESHOLD)
        {
          priv->whether_in_static_pattern = true;
          priv->whether_in_run_pattern = false;
          priv->whether_in_walk_pattern = false;
        }
    }

  if ((priv->whether_in_walk_pattern == false) && (priv->whether_in_run_pattern == false)
      && (priv->whether_in_static_pattern == false))
    {
      priv->result->motion_pattern = STEP_PATT_UNKNOWN;
    }
  else if (priv->whether_in_static_pattern == true)
    {
      priv->result->motion_pattern = STEP_PATT_STATIC;
    }
  else if ( priv->whether_in_walk_pattern == true)
    {
      priv->result->motion_pattern = STEP_PATT_WALK;
    }
  else
    {
      priv->result->motion_pattern = STEP_PATT_RUN;
    }
}

void find_next_gap_threshold(STEP_counter_private *priv, uint8_t new_gap)
{
  uint8_t i;
  uint32_t sum = 0;
  uint8_t average_value;

  for (i = 0; i < GAP_RECORD_NUM; i++)
    {
      sum += priv->gap_record[i];
      priv->gap_record[i] = priv->gap_record[i + 1];
    }

  priv->gap_record[GAP_RECORD_NUM - 1] = new_gap;
  average_value = (uint8_t)(sum / GAP_RECORD_NUM);

  for (i = 0; i < LENTH_GAP_STAIR_ARRAY; i++)
    {
      if (average_value < threshold_table[i][0])
        {
          priv->gap_threshold = threshold_table[i][1];
          break;
        }
    }
}

void reset_gap_record(STEP_counter_private *priv)
{
  uint8_t i;

  for (i = 0; i < GAP_RECORD_NUM; i++)
    {
      priv->gap_record[i] = INITIAL_GAP_VALUE;
    }

  priv->gap_threshold = INITIAL_GAP_VALUE;
}

void STEP_count_step(STEP_counter_private *priv, pattern_match_t pattern_match_result)
{
  uint8_t gap_now;

  if ((pattern_match_result.peak_or_not == true) && \
      ((pattern_match_result.walk_match == true) || (pattern_match_result.run_match == true)))
    {
      priv->count_no_match_peak = 0;//reset no match peak counter
      gap_now = pattern_match_result.value - pattern_match_result.last_valley;
      //this is to prevent user shake the device on purpose
      if ((gap_now <= GAP_MAX_COUNTABLE) && (pattern_match_result.last_valley < VALLEY_MAX_COUNTABLE))
        {
          if (gap_now >= priv->gap_threshold)
            {
              //We start counting steps after ten steps is made within 3 seconds.
              //This is to prevent some interfering conditions like standing up or typewriting.
              if (priv->init_count < STARTING_FARE_FOR_CONTINUE_COUNT)
                {
                  priv->init_count ++;
                }
              else if (priv->init_count == STARTING_FARE_FOR_CONTINUE_COUNT)
                {
                  priv->init_count ++;
                  priv->result->step_total += priv->init_count;
                }
              else
                {
                  priv->result->step_total ++;
                }
            }
          if (gap_now >= INITIAL_GAP_VALUE)
            {
              find_next_gap_threshold(priv, gap_now);
            }
        }

    }
  else
    {
      priv->count_no_match_peak ++;
      if (priv->count_no_match_peak == NO_MATCH_PEAK_THRESHOLD)
        {
          priv->init_count = 0;
          reset_gap_record(priv);
        }
    }
}

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/*every time several sets of accelerometer data is collected, call this function to get
the newest motion pattern and update the total step counter
acc_sensor_data points to the location of raw data,they may be stored in a two-dimensional
array, like raw_data[N][3].
num_xyz_sets is the number of how many sets of raw data is stored in this array */
void STEP_algo_run(STEP_counter_private *priv, int16_t *acc_sensor_data, int32_t num_xyz_sets)
{
  uint16_t xyz_combine;
  uint8_t after_FIR;
  peak_t find_peak;
  pattern_match_t find_pattern;


  while (num_xyz_sets > 0)
    {
      xyz_combine = calculate_combine(acc_sensor_data);
      after_FIR = FIR_filter(xyz_combine, &priv->FIR_filter);;
      find_peak = pattern_extraction(after_FIR, &priv->STEP_pattern);;
      find_pattern = STEP_pattern_match(priv, find_peak);
      STEP_pattern_display(priv, find_pattern);
      STEP_count_step(priv, find_pattern);

      acc_sensor_data += INT16_IN_ONE_SAMPLE;
      num_xyz_sets --;
    }

}

/*call this function to reset the algorithm, this will set the counter of total step to zero*/
void STEP_reset_algo(STEP_counter_private *priv)
{
  const char *temp_file = priv->data_source_file;
  step_algo_result_t *temp_result = priv->result;

  memset(priv, 0, sizeof(STEP_counter_private));

  priv->data_source_file = temp_file;
  priv->result = temp_result;
  priv->result->step_total = 0;
  priv->result->motion_pattern = STEP_PATT_UNKNOWN;

  priv->FIR_filter.float_to_fixed_shift = FLOATING_TO_FIXED;
  priv->FIR_filter.lenth_weight = LENTH_WEIGHT;
  priv->FIR_filter.ptr_weight = FIR_weight;
  priv->FIR_filter.ptr_weight_sign = wight_sign;

  reset_gap_record(priv);

}


static void *STEP_thread_func(pthread_addr_t pvarg)
{
  int ret;
  int buffer_len;
  STEP_counter_private *STEP_private;
  sn_ga_raw_s *ptr_raw;
  int read_len;
  int error_code;
  int sample_total;
  int filefd;
  sn_ga_param_s param;
  char mq_name[STEP_MQ_NAME_LENTH];
  mqd_t mq;
  STEP_cmd cmd;
  bool running = true;

  STEP_private = (STEP_counter_private *)pvarg;

  filefd = open(STEP_private->data_source_file, O_RDONLY);
  if (filefd < 0)
    {
      syslog(LOG_ERR, "Can't Open Device for Read. Reason:%d\n", filefd);
      return NULL;
    }

  /*set accelemeter configration*/
  param.resolution = STEP_CNT_RESOLUTION;
  param.odr = STEP_CNT_ODR;
  param.power_mode = GA_MODE_NORMAL;
  param.range = STEP_CNT_RANGE;
  ret = ioctl(filefd, SNIOC_A_SPARAM, (unsigned long)&param);
  if (OK != ret)
    {
      error_code = errno;
      syslog(LOG_ERR, "SNIOC_A_SPARAM IOCtrl failed %d\r\n", error_code);
      ret = error_code;
      goto errout_with_fd;
    }

  /* Starting device */
  ret = ioctl(filefd, SNIOC_START, (uint32_t)NULL);
  if (OK != ret)
    {
      error_code = errno;
      syslog(LOG_ERR, "Error: SNIOC_START IOCtrl failed %d\r\n", error_code);
      ret = error_code;
      goto errout_with_fd;
    }

  snprintf(mq_name, STEP_MQ_NAME_LENTH, STEP_MQ_NAME_FORMATE, (uint32_t)STEP_private);

  /*open the message queue*/
  mq = mq_open(mq_name, O_RDONLY | O_NONBLOCK);
  if (mq == (mqd_t) - 1)
    {
      error_code = errno;
      syslog(LOG_ERR,  "mq_open failed, errno=%d\n", error_code);
      goto errout_with_fd;
    }


  /*alloc buffer for raw data*/
  buffer_len = ACCELE_SAMPLE_COUNT * sizeof(sn_ga_raw_s);
  ptr_raw = (sn_ga_raw_s *)malloc(buffer_len);

  if (ptr_raw == NULL)
    {
      syslog(LOG_ERR, "Error: memory malloc failed!\r\n");
      goto errout_with_mq;
    }

  memset(ptr_raw, 0x00, buffer_len);

  while (running)
    {

      read_len = read(filefd, ptr_raw, buffer_len);

      if (read_len < 0)
        {
          error_code = errno;
          syslog(LOG_ERR, "Error: read sensor failed %d\n", error_code);
          break;
        }

#ifndef CONFIG_MC3672_USE_INT
      usleep(ACCELE_SLEEP_TIME_MS * USEC_PER_MSEC);
#endif

      sample_total = read_len / sizeof(sn_ga_raw_s);

      STEP_algo_run(STEP_private, (int16_t *)ptr_raw, sample_total);

      ret = mq_receive(mq, (char *)&cmd, (sizeof(STEP_cmd)), NULL);

      if (ret > 0)
        {
          switch (cmd)
            {
              case STEP_CMD_STOP:
                running = false;
                break;
              case STEP_CMD_RESET:
                STEP_reset_algo(STEP_private);
                break;
              default:
                break;
            }
        }
    }

  /* stopping device */
  ret = ioctl(filefd, SNIOC_STOP, (unsigned long)&param);
  if (OK != ret)
    {
      error_code = errno;
      syslog(LOG_ERR, "Error: SNIOC_STOP IOCtrl failed %d\n", error_code);
      goto errout_with_mem;
    }

errout_with_mem:
  free(ptr_raw);

errout_with_mq:
  mq_close(mq);

errout_with_fd:
  close(filefd);

  return NULL;

}



/****************************************************************************
 * Name: stepCNT_start
 ****************************************************************************/
int stepCNT_start(stepCNT_t *ptr_handle, const char *file_path)
{
  STEP_counter_private *STEP_data;
  step_counter_t *counter;
  struct sched_param  sparam;
  pthread_attr_t      tattr;
  struct mq_attr attr;
  int ret;
  pthread_t thread_id;
  mqd_t mq;
  char mq_name[STEP_MQ_NAME_LENTH];

  if ((ptr_handle == NULL) || (file_path == NULL))
    {
      ret = EINVAL;
      goto errout;
    }

  /*Create data structure for algorithm*/

  STEP_data = malloc(sizeof(STEP_counter_private));

  if (STEP_data == NULL)
    {
      syslog(LOG_ERR, "memory allocation failed!\n");
      ret = ENOMEM;
      goto errout;
    }

  memset(STEP_data, 0, sizeof(STEP_counter_private));

  STEP_data->data_source_file = file_path;
  STEP_data->FIR_filter.float_to_fixed_shift = FLOATING_TO_FIXED;
  STEP_data->FIR_filter.lenth_weight = LENTH_WEIGHT;
  STEP_data->FIR_filter.ptr_weight = FIR_weight;
  STEP_data->FIR_filter.ptr_weight_sign = wight_sign;

  reset_gap_record(STEP_data);

  /*Create handler structure for users*/

  counter = malloc(sizeof(step_counter_t));

  if (counter == NULL)
    {
      syslog(LOG_ERR, "memory allocation failed!\n");
      ret = ENOMEM;
      goto errout_priv;
    }

  counter->running = false;

  STEP_data->result = (step_algo_result_t *)&counter->step_result;
  STEP_data->result->motion_pattern = STEP_PATT_UNKNOWN;
  STEP_data->result->step_total = 0;


  /*Creat a message queue to communicate with the main thread*/

  /* Fill in attributes for message queue */

  snprintf(mq_name, STEP_MQ_NAME_LENTH, STEP_MQ_NAME_FORMATE, (uint32_t)STEP_data);

  attr.mq_maxmsg  = STEP_MQ_MAX_MSG;
  attr.mq_msgsize = STEP_MQ_MSG_SIZE;
  attr.mq_flags   = 0;

  /*open the message queue*/
  mq = mq_open(mq_name, O_WRONLY | O_CREAT | O_NONBLOCK, 0666, &attr);
  if (mq == (mqd_t) - 1)
    {
      ret = errno;
      syslog(LOG_ERR,  "mq_open failed, errno=%d\n", errno);
      goto errout_counter;
    }

  counter->mq = mq;


  /*create step counting thread*/
  pthread_attr_init(&tattr);
  sparam.sched_priority = SCHED_PRIORITY_DEFAULT;
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, STEP_COUNTING_THREAD_STACK_SIZE);

  ret = pthread_create(&thread_id, &tattr, STEP_thread_func,
                       (pthread_addr_t) STEP_data);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to create step counting thread: %d\n", ret);
      goto errout_mq;
    }

  /* Name the thread */

  pthread_setname_np(thread_id, "stepcounting");

  /*detach items_num*/
  pthread_detach(thread_id);

  counter->thread_id = thread_id;
  *ptr_handle = (stepCNT_t)counter;

  return OK;

errout_mq:
  mq_close(mq);
errout_counter:
  free(counter);
errout_priv:
  free(STEP_data);
errout:
  return -ret;

}

/****************************************************************************
 * Name: stepCNT_reset
 ****************************************************************************/
int stepCNT_reset(stepCNT_t handle)
{
  int ret;
  step_counter_t *counter = (step_counter_t *)handle;
  STEP_cmd msg;

  if (handle == NULL)
    {
      return -EINVAL;
    }

  if (counter->thread_id)
    {
      msg = STEP_CMD_RESET;
      ret = mq_send(counter->mq, (const char *)&msg, (sizeof(STEP_cmd)), STEP_MQ_PRIO);

      if (ret != OK)
        {
          ret = errno;
          syslog(LOG_ERR, "mq_send failed: %d\n", ret);
          return -ret;
        }
    }

  return OK;
}


/****************************************************************************
 * Name: StepCNT_stop
 ****************************************************************************/
int stepCNT_stop(stepCNT_t handle)
{
  pthread_addr_t return_value;
  int ret;
  step_counter_t *counter = (step_counter_t *)handle;
  STEP_cmd msg;

  if (handle == NULL)
    {
      return -EINVAL;
    }

  if (counter->thread_id)
    {
      msg = STEP_CMD_STOP;
      ret = mq_send(counter->mq, (const char *)&msg, (sizeof(STEP_cmd)), STEP_MQ_PRIO);

      if (ret != OK)
        {
          ret = errno;
          syslog(LOG_ERR, "mq_send failed: %d\n", ret);
          return -ret;
        }

      pthread_join(counter->thread_id, &return_value);
      counter->thread_id = 0;
      mq_close(counter->mq);
      free(handle);
    }

  return OK;
}

/****************************************************************************
 * Name: stepCNT_total
 ****************************************************************************/

int stepCNT_total(stepCNT_t handle, uint32_t *total)
{
  step_counter_t *counter = (step_counter_t *)handle;

  if ((handle == NULL) || (total == NULL))
    {
      return -EINVAL;
    }

  *total = counter->step_result.step_total;

  return OK;
}

/****************************************************************************
 * Name: stepCNT_motion_patt
 ****************************************************************************/

int stepCNT_motion_patt(stepCNT_t handle, step_patt *patt)
{
  step_counter_t *counter = (step_counter_t *)handle;

  if ((handle == NULL) || (patt == NULL))
    {
      return -EINVAL;
    }

  *patt = counter->step_result.motion_pattern;

  return OK;
}


#endif /*CONFIG_STEPCNT_ALGO*/


