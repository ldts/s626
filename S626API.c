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

#include "S626API.h"

ts626 * s626_init(const char * nBoardName, const char * nDeviceName) {
  ts626 * s626;

  s626 = malloc(sizeof(ts626));

  s626->BoardName = malloc(strlen(nBoardName) + 1);
  strcpy(s626->BoardName, nBoardName);

  s626->DeviceName = malloc(strlen(nDeviceName) + 1);
  strcpy(s626->DeviceName, nDeviceName);

  memset(&s626->lnkdsc, 0, sizeof(a4l_lnkdesc_t));
  //s626->lnkdsc.bname = s626->BoardName;
  s626->lnkdsc.bname = malloc(strlen(nBoardName) + 1);
  strcpy(s626->lnkdsc.bname, nBoardName);
  s626->lnkdsc.bname_size = strlen(s626->BoardName);

  s626->dsc.sbdata = NULL;
  strcpy(&s626->dsc.board_name[0], s626->BoardName);

  //set all to +/- 5 V range
  s626->adc_range = 0x0000;

  return s626;
}

int s626_deinit(ts626 * s626) {

  free(s626->DeviceName);
  free(s626->BoardName);

  free(s626->lnkdsc.bname);

  free(s626);

  return 0;
}

int s626_set_options( ts626 * s626)
{
	if( s626)
	{
		s626->lnkdsc.opts_size = 2 * sizeof(unsigned long);
		s626->lnkdsc.opts = malloc(s626->lnkdsc.opts_size);

		return 0;
	}
	else
		return -1;
}

int s626_set_bus( ts626 * s626, unsigned long bus)
{
	if(s626)
	{
		if(s626->lnkdsc.opts)
		{
			unsigned long int * opt;
			opt = (unsigned long int *)s626->lnkdsc.opts;
			opt[0] = bus;
		}
		else
			return -2;
	}
	else
		return -1;

	return 0;
}

int s626_set_slot( ts626 * s626, unsigned long slot)
{
	if(s626)
	{
		if(s626->lnkdsc.opts)
		{
			unsigned long int * opt;
			opt = (unsigned long int *)s626->lnkdsc.opts;
			opt[1] = slot;
		}
		else
			return -2;
	}
	else
		return -1;

	return 0;
}

int s626_open(ts626 * s626) {
  int err;

  err = 0;

  s626->fd = a4l_sys_open(s626->DeviceName);
  if (s626->fd < 0) {
    err = s626->fd;
    fprintf(stderr, "a4l_sys_open: failed err=%d\n", err);

    goto out_a4l_sys_open;
  }

  err = a4l_sys_attach(s626->fd, &s626->lnkdsc);
  if (err < 0) {
    fprintf(stderr, "a4l_sys_attach: attach failed err=%d\n", err);
    goto out_a4l_sys_attach;
  }

  err = a4l_open(&s626->dsc, s626->DeviceName);
  if (err < 0) {
    fprintf(stderr, "a4l_open: failed err=%d\n", err);
    goto out_a4l_open;
  }

  if (s626->lnkdsc.opts != NULL)
    free(s626->lnkdsc.opts);

  // open the device
  s626->device = rt_dev_open(s626->DeviceName, 0);
  if (s626->device < 0) {
    err = s626->device;
    printf("rt_dev_close: can't open device %s (%s)\n", s626->DeviceName,
        strerror(-s626->device));
    goto out_rt_dev_open;
  }

  s626->dsc.sbdata = malloc(s626->dsc.sbsize);

  err = a4l_fill_desc(&s626->dsc);
  if (err < 0) {
    printf("a4l_fill_desc failed (err=%d)\n",
      err);
  }

  return 0;

  out_rt_dev_open:

  a4l_close(&s626->dsc);

  out_a4l_open:

  a4l_sys_detach(s626->fd);

  out_a4l_sys_attach:

  a4l_sys_close(s626->fd);

  out_a4l_sys_open:

  return err;
}

int s626_close(ts626 * s626) {

  int err;

  // close the device
  err = rt_dev_close(s626->device);
  if (err < 0) {
    printf("rt_dev_close: can't close device %s (%s)\n", s626->DeviceName,
        strerror(-err));
    fflush(stdout);

    return err;
  }

  err = a4l_close(&s626->dsc);
  if( err < 0) {
    fprintf(stderr, "a4l_close: analogy device close failed err=%d\n", err);
    return err;
  }

  err = a4l_sys_detach(s626->fd);
  if (err < 0) {
    fprintf(stderr, "a4l_sys_detach: detach failed err=%d\n", err);
    return err;
  }

  a4l_sys_close(s626->fd);

  free(s626->dsc.sbdata);

  return 0;
}

int s626_gpct_conf_enc(ts626 * s626, unsigned int subd, unsigned int chan) {

  int err;

  s626->config.type = A4L_INSN_CONFIG;
  s626->config.idx_subd = subd;
  s626->config.chan_desc = chan;
  s626->data[0] = A4L_INSN_CONFIG_GPCT_QUADRATURE_ENCODER;
  s626->data[1] = 0;
  s626->data[2] = 0;
  s626->data[3] = 0;
  s626->config.data = &s626->data;
  s626->config.data_size = 4 * sizeof(unsigned int);

  err = a4l_snd_insn(&s626->dsc, &s626->config);
  if (err < 0) {
    printf("ERROR: configuration of subdevice (%d)\n", err);

    return err;
  }

  return 0;

}

int s626_gpct_read_enc(ts626 * s626, unsigned int subd, unsigned int chan,
    int * value) {

  char buf[4];
  int err;

  err = a4l_sync_read(&s626->dsc, subd, CHAN(chan), 0, buf, 4);
  if (err < 0) {
    printf("ERROR: reading from device (%d)\n", err);

    return err;
  }

  *value = *(signed int *) (&buf[0]);
  if (*value & 0x00800000)
    *value = *value | 0xff000000;

  return 0;
}

int s626_get_subd_count(ts626 * s626)
{
  if(s626)
  {
    return s626->dsc.nb_subd;
  }
  else
  {
    return -1;
  }
}

a4l_sbinfo_t * s626_get_subd_info(ts626 * s626, unsigned int subd)
{
  a4l_sbinfo_t * sbinfo;
  int err;

  err = a4l_get_subdinfo(&s626->dsc, subd, &sbinfo);
  if (err < 0) {
    printf(
      "a4l_get_subdinfo failed (err = %d)\n",
      err);
  }

  return sbinfo;
}

int s626_get_subd_type(ts626 * s626, unsigned int subd)
{
  return s626_get_subd_info(s626, subd)->flags & A4L_SUBD_TYPES;
}

int s626_dio_read(ts626 * s626, unsigned int subd, unsigned int mask, int * value)
{
  int err;

  mask = 0;
  *value = 0;

  err = a4l_sync_dio(&s626->dsc, subd, &mask, value);

  if (err < 0) {
    fprintf(stderr,
      "a4l_sync_dio() failed (err=%d)\n", err);

    return err;
  }

  return 0;
}

int s626_dio_write(ts626 * s626, unsigned int subd, unsigned int mask, unsigned int value)
{
  int err;

  err = a4l_sync_dio(&s626->dsc, subd, &mask, &value);

  if (err < 0) {
    printf("a4l_sync_dio() failed (err=%d)\n", err);
  }

  return err;
}


a4l_chinfo_t * s626_adc_get_chan_info(ts626 * s626, unsigned int subd, unsigned int channel)
{
  int err = 0;
  a4l_chinfo_t *chinfo;

  err = a4l_get_chinfo(&s626->dsc, subd, channel, &chinfo);
  if (err < 0) {
    printf(
      "info for channel %d on subdevice %d not available (err=%d)\n",
      channel, subd, err);

    return NULL;
  }

  return chinfo;
}

a4l_rnginfo_t * s626_adc_get_rng_info(ts626 * s626, unsigned int subd, unsigned int channel, int range)
{
  int err;
  a4l_rnginfo_t *rnginfo;

  err = a4l_get_rnginfo(&s626->dsc,
            subd, channel, range, &rnginfo);
  if (err < 0) {
    printf(
      "failed to recover range descriptor\n");

    return NULL;
  }

  //min value inside rnginfo->min
  //max value inside rnginfo->max

  return rnginfo;
}

int s626_adc_get_scan_size(ts626 * s626, unsigned int subd, unsigned int channel)
{
  return a4l_sizeof_chan(s626_adc_get_chan_info(s626, subd, channel));
}

int s626_adc_read(ts626 * s626, unsigned int subd, unsigned int channel, char * buffer, unsigned int count)
{
  int err;

  static int range;

  range = 0x00;

  if( s626->adc_range & (1<<channel))
  	range = 0x010000;

  //we are reading 16 bits which is 2 bytes
  //count is the number of measurements we want to
  //read
  //so total number of bytes is 2 * count
  err = a4l_sync_read(&s626->dsc,
              subd, (channel & 0xFFFF) | range, 0, buffer, count*2);

  return err;
}

int s626_adc_set_range(ts626 * s626, unsigned int mask, unsigned int ranges)
{
	s626->adc_range = (s626->adc_range & ~mask) | (ranges & mask);

	return ranges;
}

int s626_adc_get_range(ts626 * s626)
{
	return s626->adc_range;
}

int s626_dac_write(ts626 * s626, unsigned int subd, unsigned int channel, char * buffer)
{
  int err = 0;

  err = a4l_sync_write(&s626->dsc,
              subd, CHAN(channel), 0, buffer, 2);

  return err;
}
































