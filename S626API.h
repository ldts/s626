/***************************************************************************
 *   Copyright (C) 2014 by Wojciech Domski                                 *
 *   Wojciech.Domski@gmail.com                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef S626API_HEADER

#define S626API_HEADER

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtdm/rtdm.h>
#include <xeno_config.h>
#include <analogy/analogy.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
  char * DeviceName;
  char * BoardName;
  a4l_desc_t dsc;
  a4l_lnkdesc_t lnkdsc;
  a4l_insn_t config;
  unsigned int data[4];
  int fd;
  int device;

  //each bit corresponds to a channel
  //'0' -> +/- 5  V
  //'1' -> +/- 10 V
  int adc_range;
} ts626;

ts626 * s626_init(const char * nBoardName, const char * nDeviceName);

int s626_deinit(ts626 * s626);

int s626_set_options( ts626 * s626);

int s626_set_bus( ts626 * s626, unsigned long bus);

int s626_set_slot( ts626 * s626, unsigned long slot);

int s626_open(ts626 * s626);

int s626_close(ts626 * s626);

int s626_gpct_conf_enc(ts626 * s626, unsigned int subd, unsigned int chan);

int s626_gpct_read_enc(ts626 * s626, unsigned int subd, unsigned int chan,
    int * value);

int s626_get_subd_count(ts626 * s626);

a4l_sbinfo_t * s626_get_subd_info(ts626 * s626, unsigned int subd);

int s626_get_subd_type(ts626 * s626, unsigned int subd);

int s626_dio_read(ts626 * s626, unsigned int subd, unsigned int mask, int * value);

int s626_dio_write(ts626 * s626, unsigned int subd, unsigned int mask, unsigned int value);

int s626_adc_read(ts626 * s626, unsigned int subd, unsigned int channel, char * buffer, unsigned int count);

int s626_adc_set_range(ts626 * s626, unsigned int mask, unsigned int ranges);

int s626_adc_get_range(ts626 * s626);

int s626_dac_write(ts626 * s626, unsigned int subd, unsigned int channel, char * buffer);

#ifdef __cplusplus
}
#endif

#endif

