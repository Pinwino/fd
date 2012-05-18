#ifndef __FD_LIB_H
#define __FD_LIB_H

#include <stdint.h>

/* Number of fractional bits in the timestamps/time definitions. Must be consistent with the HDL bitstream.  */
#define FDELAY_FRAC_BITS 12


/* fdelay_get_timing_status() return values: */

#define FDELAY_FREE_RUNNING	  0x10		/* local oscillator is free running */
#define FDELAY_WR_OFFLINE	  0x8		/* attached WR core is offline */
#define FDELAY_WR_READY 	  0x1		/* attached WR core is synchronized, we can sync the fine delay core anytime */
#define FDELAY_WR_SYNCING 	  0x2		/* local oscillator is being synchronized with WR clock */
#define FDELAY_WR_SYNCED   	  0x4		/* we are synced. */
#define FDELAY_WR_NOT_PRESENT	  0x20		/* No WR Core present */

/* fdelay_configure_sync() flags */

#define FDELAY_SYNC_LOCAL 	 0x1  	 	/* use local oscillator */
#define FDELAY_SYNC_WR	 	 0x2		/* use White Rabbit */

/* Hardware "handle" structure */
typedef struct fdelay_device
{
  /* Base address of the FD core (relative to the beginning of local writel/readl address spaces) */
  uint32_t base_addr; 

  /* Bus-specific readl/writel functions - so the same library can be used both with
     RawRabbit, VME and Etherbone backends */
  void (*writel)(void *priv, uint32_t data, uint32_t addr);
  uint32_t (*readl)(void *priv, uint32_t addr);
  
  void *priv_fd; /* pointer to Fine Delay library private data */
  void *priv_io; /* pointer to the I/O routines private data */
} fdelay_device_t;

typedef struct {
  int64_t utc, utc_sh;
  int32_t coarse, coarse_sh;
  int32_t start_offset;
  int32_t subcycle_offset;
  int32_t frac;
} fdelay_raw_time_t;

typedef struct 
{
  fdelay_raw_time_t raw;

  int64_t utc; /* TAI seconds */ /* FIXME: replace all UTCs with TAIs or seconds for clarity */
  int32_t coarse; /* 125 MHz counter cycles */
  int32_t frac; /* Fractional part (<8ns) */
  uint16_t seq_id; /* Sequence ID to detect missed timestamps */
} fdelay_time_t;

/* 
--------------------
PUBLIC API 
--------------------
*/


/* Creates a local instance of Fine Delay Core at address base_addr. Returns a non-null fdelay_device_t
   card context on success or null if an error occured */
fdelay_device_t *fdelay_create(const char *device);

/* Does the same as above, but for a card accessible via EtherBone/MiniBone. 
   iface = network interface to which the carrier is connected
   mac_addr = MAC address of the EtherBone/MiniBone core
   base_addr = base address of the FD core (relative to EB/MB address space) */
fdelay_device_t *fdelay_create_minibone(char *iface, char *mac_addr, uint32_t base_addr);

/* Helper functions - converting FD timestamp format from/to plain picoseconds */
fdelay_time_t fdelay_from_picos(const uint64_t ps);
int64_t fdelay_to_picos(const fdelay_time_t t);

/* Initializes and calibrates the device. 0 = success, negative = error */
int fdelay_init(fdelay_device_t *dev);

/* Disables and releases the resources for a given FD Card */
int fdelay_release(fdelay_device_t *dev);

/* Returns an explaination of the last error occured on device dev (TBI) */
char *fdelay_strerror(fdelay_device_t *dev);

/* Sets the timing reference for the card (ref source). Currently there are two choices:
- FDELAY_SYNC_LOCAL 	- use local oscillator 
- FDELAY_SYNC_WR	- use White Rabbit */
int fdelay_set_timing_reference(fdelay_device_t *dev, int ref_source);

/* Polls the current status of the timing source. Returns a combination of 
   .... SYNCED flags. wait_mask can enable/disable waiting for a change of 
   a particular flag or set of flags. For example, calling

   fdelay_get_timing_status(dev, FDELAY_WR_SYNCED) will wait until a change of
   FDELAY_WR_SYNCED bit. */
int fdelay_get_timing_status(fdelay_device_t *dev, int wait_mask);

/* Configures the trigger input (TDC/Delay modes). enable enables the input,
   termination switches on/off the built-in 50 Ohm termination resistor */
   
int fdelay_configure_trigger(fdelay_device_t *dev, int enable, int termination);

/* Configures timestamp buffer capture: enable = TS buffer enabled, channel mask: 
   channels to time tag (bit 0 = TDC, bits 1..4 = outputs 1..4) */

int fdelay_configure_capture (fdelay_device_t *dev, int enable, int channel_mask);

/* Reads how_many timestamps from the buffer. Blocking */
/* TODO: non-blocking version? */
int fdelay_read (fdelay_device_t *dev, fdelay_time_t *timestamps, int how_many);


/* (delay mode only) Configures output(s) selected in channel_mask to work in delay mode. Delta_ps = spacing between
the rising edges of subsequent pulses. */
int fdelay_configure_delay (fdelay_device_t *dev, int channel_mask, int enable, int64_t delay_ps, int64_t width_ps, int64_t delta_ps, int repeat_count);

/* (pulse mode only)  Configures output(s) selected in channel_mask to produce pulse(s) starting at (start) with appropriate width/spacing/repeat_count */
int fdelay_configure_pulse_gen(fdelay_device_t *dev, int channel_mask, int enable, fdelay_time_t start, int64_t width_ps, int64_t delta_ps, int repeat_count);


/* (pulse mode only) Returns non-0 when all of the channels in channel mask have produced their programmed pulses */
int fdelay_outputs_triggered(fdelay_device_t *dev, int channel_mask, int blocking);

void fdelay_set_user_offset(fdelay_device_t *dev,int input, int64_t offset);

int fdelay_get_time(fdelay_device_t *dev, fdelay_time_t *t);
int fdelay_set_time(fdelay_device_t *dev, const fdelay_time_t t);

#endif
