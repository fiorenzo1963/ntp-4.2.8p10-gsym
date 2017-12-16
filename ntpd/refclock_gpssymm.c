/*
 * refclock_gpssymm.c - clock driver GPS disciplined symmetricomm BC635-PCI
 *                      (uses GPS 1PPS as input to 1PPS to BC635-PCI)
 *
 * Fio Cattaneo <fio@cattaneo.us>, Nov 24, 2017
 *
 * based on refclock_nmea.c
 *
 *
 * NOTE: there are no SDK APIs for the two new IOCTLs in BTFP driver,
 * so this code will only work for FreeBSD.
 *
 */
/*
***********************************************************************
*                                                                     *
* Copyright (c) Fio Cattaneo 2017                                     *
*                                                                     *
* All Rights Reserved                                                 *
*                                                                     *
* Redistribution and use in source and binary forms, with or without  *
* modification, are permitted provided that the following conditions  *
* are met:                                                            *
* 1. Redistributions of source code must retain the above copyright   *
*    notice, this list of conditions and the following disclaimer.    *
* 2. Redistributions in binary form must reproduce the above          *
*    copyright notice, this list of conditions and the following      *
*    disclaimer in the documentation and/or other materials provided  *
*    with the distribution.                                           *
*                                                                     *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS  *
* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
* ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE    *
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT   *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR  *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF          *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT           *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE   *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH    *
* DAMAGE.                                                             *
***********************************************************************
*/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "ntp_types.h"

#if defined(REFCLOCK) && defined(CLOCK_GPSSYM)

#include <sys/stat.h>
#include <stdio.h>
#include <ctype.h>
#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif

#include "ntpd.h"
#include "ntp_io.h"
#include "ntp_unixtime.h"
#include "ntp_refclock.h"
#include "ntp_stdlib.h"
#include "ntp_calendar.h"
#include "timespecops.h"

/*
#include "ppsapi_timepps.h"
#include "refclock_atom.h"
*/

#include "bc635_btfp.h"

/*
 *
 * Prototype was refclock_nmea.c, Thanks a lot.
 *
 * server 127.127.25.0 mode X
 * 
 * bit 9/10 - gps receiver type
 *            1 = NEO UBLOX7
 *            2 = Garmin 18x
 *            0 = Generic GPS receiver
 *
 * GPS receiver must be configured as follows:
 * (1) output only GPRMC.
 * (2) PPS enable rising edge for 100 to 200 milliseconds max,
 *     do not output PPS if GPS lock is lost, or minimize it (garmin limits it to 1).
 * (3) stationary mode (if available).
 *
 *
 * bit 4/5/6 - selects the baudrate for serial port :
 *		0 for 4800 (default) 
 *		1 for 9600 
 *		2 for 19200 
 *		3 for 38400 
 *		4 for 57600 
 *		5 for 115200 
 */
#define NMEA_GPS_NEO7_UBLOX		0x00000100U
#define NMEA_GPS_18X_GARMIN		0x00000200U
#define NMEA_GPS_GENERIC		0x00000000U
#define NMEA_GPS_MASK			0x00000F00U

#define NMEA_EXTLOG_MASK	        0x00010000U

#define NMEA_BAUDRATE_MASK	0x00000070U
#define NMEA_BAUDRATE_SHIFT	4

#define NMEA_PROTO_IDLEN	4	/* tag name must be at least 5 chars */
#define NMEA_PROTO_MINLEN	6	/* min chars in sentence, excluding CS */
#define NMEA_PROTO_MAXLEN	100	/* max chars in sentence, excluding CS */
#define NMEA_PROTO_FIELDS	32	/* not official; limit on fields per record */

#define NMEA_GPRMC		1
#define NMEA_PUBX		2
/*
 * We check the timecode format and decode its contents.
 * We only care about GPRMC (and PUBX for UBLOX NEO devices to determine if leap seconds are valid).
 * Note for UBLOX, we set stationary mode, so we do not velocity information.
 *
 *
 *
 * ===================== GARMIN 18X ============================================
 *
 * $GPRMC,hhmmss,a,fddmm.xx,n,dddmmm.xx,w,zz.z,yyy.,ddmmyy,dd,v*CC
 * a =
 * v =
 * $GPRMC,214403,A,4734.0198,N,12207.7026,W,000.1,043.1,251117,016.2,E,D*0E
 *
 *
 * ===================== NEO UBLOX7 ============================================
 *
 * NEO UBLOX7:
 *
 * On UBLOX7, we must look for PUBX,04 message to make sure we are getting
 * gps leap seconds from satellites, otherwise our time will be off
 * (UBLOX7 firmware has a default of 16 seconds, currently it's 18 as of December 2017).
 *
 *
 * $GPRMC,hhmmss.00,a,fddmm.xx,n,dddmmm.xx,w,zz.z,yyy.,ddmmyy,dd,v*CC
 * a:
 *     A = data valid
 *     V = nav receiver warning
 * v:
 *     N = invalid
 *     E = estimated
 *     A = valid autonomous
 *     D = valid differential
 * $GPRMC,214403.00,A,4734.01467,N,12207.70351,W,0.015,,251117,,,A*61
 *
 * (issued on demand, must send proprietary nmea PUBX,04 command)
 * $PUBX,04,hhmmss.00,ddhhyy,
 *                           gpsweek seconds offset,
 *                           gpsweek(does not rollover),
 *                           gps leapseconds
 *                                 (marked with a 'D' suffix if the value
 *                                  is the fw default value, otherwise it
 *                                  is the actual leap seconds value received
 *                                  from GPS constellation),
 *                           receiver clock bias (ns),
 *                           receiver clock driver (ns/s),
 *                           time pulse granularity (ns)
 * $PUBX,04,214403.00,251117,596642.99,1976,18,-693780,-471.533,21*15
 */

/*
 * Definitions
 */
#define	GPS_DEVICE	"/dev/gps0"	/* GPS serial device */
#define SYMM_DEVICE	"/dev/btfp0"	/* Symmetricom BC635 device */
#define	DEF_GPS_RS232	B38400	/* uart speed (38400 bps) fallback */
#define	FREE_PRECISION		(-19)	/* precision assumed without PPS (about 1 us) */
#define	LOCKED_PRECISION	(-21)	/* precision assumed with PPS activre (about 0.250 us) */
#define	REFID		"GSYM"	/* reference id */
#define	DESCRIPTION	"GPS Disciplined Symmetricom BC635 Clock" /* who we are */
#ifndef O_NOCTTY
#define M_NOCTTY	0
#else
#define M_NOCTTY	O_NOCTTY
#endif
#ifndef O_NONBLOCK
#define M_NONBLOCK	0
#else
#define M_NONBLOCK	O_NONBLOCK
#endif
#define PPSOPENMODE	(O_RDWR | M_NOCTTY | M_NONBLOCK)

/* results for 'field_init()'
 *
 * Note: If a checksum is present, the checksum test must pass OK or the
 * sentence is tagged invalid.
 */
#define CHECK_EMPTY  -1	/* no data			*/
#define CHECK_INVALID 0	/* not a valid NMEA sentence	*/
#define CHECK_VALID   1	/* valid but without checksum	*/
#define CHECK_CSVALID 2	/* valid with checksum OK	*/

enum gps_clockstate {
	CS_INVALID_CLOCK,		/* wait valid GPRMC sentence */
	CS_WAIT_CLOCK_LEAP_SECS,	/* for UBLOX7, wait for valid leap secs */
	CS_VALID_CLOCK,			/* valid clock */
};

enum bc635_clockstate {
	BCS_PRE_INITIALIZED,		/* pre initialized */
	BCS_TIME_INITIALIZED,		/* initialized */
	BCS_TIME_PPS_SYNCHRONIZED,	/* initialized, PPS valid */
	BCS_TIME_FREE_RUNNING,		/* initialized, free running (without PPS) */
};

#define YEAR_CENTURY	2000
#define YEAR_1900	1900

#define BC635_MIN_LOCKED_COUNT		5

/*
 * Unit control structure
 *
 * the BC635 hardware has a 32 bit field for seconds time, so there is no need to
 * worry about centuries, we are in 2017 and the BC635 will be obsolete in 2038
 * (so this is why we can simply fix YEAR CENTURY to 2000).
 */
typedef struct {
	l_fp			last_bc635_reftime;	/* last processed reference stamp */
	struct timespec 	last_bc635_t_reftime;
	l_fp    		last_clocktime;		/* last clocktime matching ref stamp */
	struct timespec 	last_t_clocktime;
	enum gps_clockstate	gps_clock_state;
	enum bc635_clockstate	bc635_clock_state;
	int			bc635_time_initialized_count;
	int			bc635_time_locked_count;
	struct gps_tally
	{
		u_int total;             /* total packes */
		u_int valid_ignored;     /* valid, but don't have correct time yet */
		u_int valid_processed;   /* valid, accepted */
		u_int invalid;           /* invalid sentence, or not recognized */
		u_int malformed;         /* valid sentence, but values malformed */
		u_int nofix;             /* no gps fix */
		u_int bc_time_processed;
		u_int bc_time_pps_sync;
		u_int bc_time_free_running;
	} gps_tally;
	int			gps_type;		/* (ttl & NMEA_GPS_MASK) */
	/*
	 * the next four fields are only vaid if (gps_mode_flags & NMEA_GPS_NEO7_UBLOX) is true.
	 */
	int			gps_week_number;	/* gps week number, if available (does not rollover) */
	int			gps_week_seconds;	/* gps week seconds, if available */
	int			gps_leap_seconds;	/* gps leap seconds, if available */
	struct timespec		gps_info_reftime;	/* when the above three fields were obtained, if available */
	int			count;			/* count of received packets. */
	/*
	 * fd to access BC635
	 */
	int			bc635_fd;
	/*
	 * temporary fields being worked on.
	 */
	struct tm		gps_curr_tm;
} gpssymm_unit;

/*
 * helper for faster field access
 */
typedef struct {
	char  *base;	/* buffer base		*/
	char  *cptr;	/* current field ptr	*/
	int    blen;	/* buffer length	*/
	int    cidx;	/* current field index	*/
} gpssymm_data;

/*
 * Function prototypes
 */
static	void	gpssymm_init	(void);
static	int	gpssymm_start	(int, struct peer *);
static	void	gpssymm_shutdown(int, struct peer *);
static	void	gpssymm_receive	(struct recvbuf *);
static	void	gpssymm_poll	(int, struct peer *);
static	void	gpssymm_timer	(int, struct peer *);
static	void	gpssymm_control	(int, const struct refclockstat *, struct refclockstat *, struct peer *);

/* parsing helpers */
static int	field_init	(struct peer *peer, gpssymm_data * data, char * cp, int len);
static char *	field_parse	(gpssymm_data * data, int fn);
static u_char	parse_qual	(gpssymm_data * data, int idx, char tag, int inv);
static int	parse_time	(struct tm *t, long *ns_off, gpssymm_data *, int idx);
static int	parse_date	(struct tm *t, gpssymm_data *, int idx);
/* calendar / date helpers */
static void     save_ltc        (struct refclockproc * const, const char * const, size_t);

/*
 * If we want the driver to ouput sentences, too: re-enable the send
 * support functions by defining NMEA_WRITE_SUPPORT to non-zero...
 */
static	void gps_send(int, const char *, struct peer *);
# ifdef SYS_WINNT
#  undef write	/* ports/winnt/include/config.h: #define write _write */
extern int async_write(int, const void *, unsigned int);
#  define write(fd, data, octets)	async_write(fd, data, octets)
# endif /* SYS_WINNT */

/*
 * -------------------------------------------------------------------
 * Transfer vector
 * -------------------------------------------------------------------
 */
struct refclock refclock_gpssymm = {
	gpssymm_start,		/* start up driver */
	gpssymm_shutdown,	/* shut down driver */
	gpssymm_poll,		/* transmit poll message */
	gpssymm_control,	/* fudge control */
	gpssymm_init,		/* initialize driver */
	noentry,		/* buginfo */
	gpssymm_timer		/* called once per second */
};

/*
 * -------------------------------------------------------------------
 * gpssymm_init - initialise data
 *
 * calculates a few runtime constants that cannot be made compile time
 * constants.
 * -------------------------------------------------------------------
 */
static void
gpssymm_init(void)
{
	DPRINTF(1, ("gpssymm_init\n"));
}

/*
 * -------------------------------------------------------------------
 * gpssymm_start - open the GPS devices and initialize data for processing
 *
 * return 0 on error, 1 on success. Even on error the peer structures
 * must be in a state that permits 'gpssymm_shutdown()' to clean up all
 * resources, because it will be called immediately to do so.
 * -------------------------------------------------------------------
 */
static int
gpssymm_start(
	int		unit,
	struct peer *	peer
	)
{
	struct refclockproc * const	pp = peer->procptr;
	gpssymm_unit * const		up = emalloc_zero(sizeof(*up));
	u_int32				rate;
	int				baudrate;
	const char *			baudtext;
	union btfp_ioctl_out		btfp;



	DPRINTF(1, ("%s gpssymm_start, unit=%d, ttl=0x%x\n",
		refnumtoa(&peer->srcadr), unit, peer->ttl));

	if (unit != 0) {
		msyslog(LOG_ERR, "%s BC635/GPS devices only support unit 0",
			refnumtoa(&peer->srcadr));
		return FALSE; /* buffer overflow */
	}

	/* Get baudrate choice from mode byte bits 4/5/6 */
	rate = (peer->ttl & NMEA_BAUDRATE_MASK) >> NMEA_BAUDRATE_SHIFT;

	switch (rate) {
	case 0:
		baudrate = B4800;
		baudtext = "4800";
		break;
	case 1:
		baudrate = B9600;
		baudtext = "9600";
		break;
	case 2:
		baudrate = B19200;
		baudtext = "19200";
		break;
	case 3:
		baudrate = B38400;
		baudtext = "38400";
		break;
#ifdef B57600
	case 4:
		baudrate = B57600;
		baudtext = "57600";
		break;
#endif
#ifdef B115200
	case 5:
		baudrate = B115200;
		baudtext = "115200";
		break;
#endif
	default:
		baudrate = DEF_GPS_RS232;
		baudtext = "38400 (fallback)";
		break;
	}

	up->bc635_fd = -1;
	/* Allocate and initialize unit structure */
	pp->unitptr = (caddr_t)up;
	pp->io.fd = -1;
	pp->io.clock_recv = gpssymm_receive;
	pp->io.srcclock = peer;
	pp->io.datalen = 0;
	/* force change detection on first valid message */
	memset(&up->last_bc635_reftime, 0xFF, sizeof(up->last_bc635_reftime));
	ZERO(up->gps_tally);

	/* Initialize miscellaneous variables */
	peer->precision = FREE_PRECISION;
	pp->clockdesc = DESCRIPTION;
	memcpy(&pp->refid, REFID, 4);

	/* Open serial port. Use CLK line discipline, if available. */
	pp->io.fd = refclock_open(GPS_DEVICE, baudrate, LDISC_CLK);
	if (pp->io.fd < 0) {
		msyslog(LOG_ERR, "%s cannot open GPS clock device (%s)",
			refnumtoa(&peer->srcadr),
			GPS_DEVICE);
		return FALSE;
	}
	LOGIF(CLOCKINFO, (LOG_NOTICE, "%s GPS clock serial %s opened at %s bps",
	      refnumtoa(&peer->srcadr), GPS_DEVICE, baudtext));

	up->gps_type = peer->ttl & NMEA_GPS_MASK;
	up->gps_clock_state = CS_INVALID_CLOCK;
	up->bc635_clock_state = BCS_PRE_INITIALIZED;

	LOGIF(CLOCKINFO, (LOG_NOTICE, "%s opening timing board %s",
	      refnumtoa(&peer->srcadr), SYMM_DEVICE));
	up->bc635_fd = open(SYMM_DEVICE, O_RDWR);
	if (up->bc635_fd < 0) {
		msyslog(LOG_ERR, "%s cannot open SYMM timing board device (%s)",
			refnumtoa(&peer->srcadr),
			SYMM_DEVICE);
		io_closeclock(&pp->io);
		pp->io.fd = -1;
		return FALSE;
	}

	LOGIF(CLOCKINFO, (LOG_NOTICE, "%s opened timing board %s",
	      refnumtoa(&peer->srcadr), SYMM_DEVICE));

	bzero(&btfp, sizeof (btfp));
	btfp.timemode.id = TFP_TIMEMODE;
	btfp.timemode.mode = TIMEMODE_PPS;
	if (ioctl(up->bc635_fd, BTFP_TIMEMODE, &btfp) != 0) {
		msyslog(LOG_ERR, "%s cannot set PPS timemode for (%s)",
			refnumtoa(&peer->srcadr),
			SYMM_DEVICE);
		io_closeclock(&pp->io);
		pp->io.fd = -1;
		close(up->bc635_fd);
		up->bc635_fd = -1;
		return FALSE;
	}
	LOGIF(CLOCKINFO, (LOG_NOTICE, "%s set timemode to PPS for %s",
	      refnumtoa(&peer->srcadr), SYMM_DEVICE));

	bzero(&btfp, sizeof (btfp));
	btfp.timefmt.id = TFP_TIMEREG_FMT;
	btfp.timefmt.format = UNIX_TIME;
	if (ioctl(up->bc635_fd, BTFP_TIMEREG_FMT, &btfp) != 0) {
		msyslog(LOG_ERR, "%s cannot set UNIX timeformat for (%s)",
			refnumtoa(&peer->srcadr),
			SYMM_DEVICE);
		io_closeclock(&pp->io);
		pp->io.fd = -1;
		close(up->bc635_fd);
		up->bc635_fd = -1;
		return FALSE;
	}
	LOGIF(CLOCKINFO, (LOG_NOTICE, "%s set timeformat to UNIX for %s",
	      refnumtoa(&peer->srcadr), SYMM_DEVICE));

	DPRINTF(1, ("%s gpssymm_start, unit=%d, ttl=0x%x: done\n",
		refnumtoa(&peer->srcadr), unit, peer->ttl));

	/* succeed if this clock can be added */
	return io_addclock(&pp->io) != 0;
}


/*
 * -------------------------------------------------------------------
 * gpssymm_shutdown - shut down a GPS clock
 * 
 * NOTE this routine is called afterk gpssymm_start() returns failure,
 * as well as during a normal shutdown due to ntpq :config unpeer.
 * -------------------------------------------------------------------
 */
static void
gpssymm_shutdown(
	int           unit,
	struct peer * peer
	)
{
	struct refclockproc * const pp = peer->procptr;
	gpssymm_unit	    * const up = (gpssymm_unit *)pp->unitptr;

	DPRINTF(1, ("%s gpssymm_shutdown, unit=%d\n",
		refnumtoa(&peer->srcadr), unit));

	UNUSED_ARG(unit);

	if (up != NULL) {
		free(up);
	}
	pp->unitptr = (caddr_t)NULL;
	if (pp->io.fd != -1) {
		DPRINTF(1, ("%s gpssymm_shutdown: close GPS descriptor\n",
			refnumtoa(&peer->srcadr)));
		io_closeclock(&pp->io);
		pp->io.fd = -1;
	}

	if (up->bc635_fd != -1) {
		DPRINTF(1, ("%s gpssymm_shutdown: close timing board descriptor\n",
			refnumtoa(&peer->srcadr)));
		close(up->bc635_fd);
	}
}

/*
 * -------------------------------------------------------------------
 * gpssymm_control - configure fudge params
 * -------------------------------------------------------------------
 */
static void
gpssymm_control(
	int                         unit,
	const struct refclockstat * in_st,
	struct refclockstat       * out_st,
	struct peer               * peer
	)
{
	struct refclockproc * const pp = peer->procptr;
	gpssymm_unit	    * const up = (gpssymm_unit *)pp->unitptr;

	DPRINTF(1, ("%s gpssymm_control, unit=%d\n",
		refnumtoa(&peer->srcadr), unit));
	
	UNUSED_ARG(up);
	UNUSED_ARG(in_st);
	UNUSED_ARG(out_st);

}

/*
 * -------------------------------------------------------------------
 * gpssymm_timer - called once per second
 *		this only polls (older?) Oncore devices now
 *
 * Usually 'gpssymm_receive()' can get a timestamp every second, but at
 * least one Motorola unit needs prompting each time. Doing so in
 * 'gpssymm_poll()' gives only one sample per poll cycle, which actually
 * defeats the purpose of the median filter. Polling once per second
 * seems a much better idea.
 * -------------------------------------------------------------------
 */
static void
gpssymm_timer(
	int	      unit,
	struct peer * peer
	)
{
	struct timespec t;
	clock_gettime(CLOCK_REALTIME, &t);
	DPRINTF(1, ("%s gpssymm_timer: %ld.%09ld\n",
		refnumtoa(&peer->srcadr), t.tv_sec, t.tv_nsec));
    
	struct refclockproc * const pp = peer->procptr;

	UNUSED_ARG(unit);
	UNUSED_ARG(unit);
	UNUSED_ARG(peer);
}

static void get_bc635_time(struct peer * const peer,
		           gpssymm_unit * const up,
			   struct timespec *bc635_reftime_out,
			   struct timespec *clock_reftime_out,
			   int *locked)
{	
	int ret;
	struct timespec t;
	struct btfp_ioctl_gettime bt;

	memset(&bt, 0, sizeof (bt));
	ret = ioctl(up->bc635_fd, BTFP_READ_UNIX_TIME, &bt);
	clock_gettime(CLOCK_REALTIME, &t);
	DPRINTF(1, ("%s clock_state: %ld.%09ld: read_unix_time=%d\n", refnumtoa(&peer->srcadr), t.tv_sec, t.tv_nsec, ret));
	INVARIANT(ret == 0);
	/*
	 * FIXME: check for timewarps
	 */
	DPRINTF(1, ("%s clock_state: current time=%ld.%09ld\n", refnumtoa(&peer->srcadr), t.tv_sec, t.tv_nsec));
	DPRINTF(1, ("%s clock_state:       bt.bt0=%ld.%09ld\n", refnumtoa(&peer->srcadr), bt.t0.tv_sec, bt.t0.tv_nsec));
	DPRINTF(1, ("%s clock_state:         time=%ld.%09ld\n", refnumtoa(&peer->srcadr), bt.time.tv_sec, bt.time.tv_nsec));
	DPRINTF(1, ("%s clock_state:       locked=%d\n", refnumtoa(&peer->srcadr), bt.locked));
	DPRINTF(1, ("%s clock_state:      freqoff=%d\n", refnumtoa(&peer->srcadr), bt.freqoff));
	DPRINTF(1, ("%s clock_state:      ns_prec=%d\n", refnumtoa(&peer->srcadr), bt.ns_precision));
	DPRINTF(1, ("%s clock_state:       bt.bt1=%ld.%09ld\n", refnumtoa(&peer->srcadr), bt.t1.tv_sec, bt.t1.tv_nsec));
	*locked = bt.locked;

	/* calculate bc635 reference time which matches bt1 */
	t = sub_tspec(bt.t1, bt.t0);
	INVARIANT(t.tv_sec == 0);
	INVARIANT(t.tv_nsec > 0);
	t.tv_nsec /= 2;
	DPRINTF(1, ("%s clock_state:    halfdelay=%ld.%09ld\n", refnumtoa(&peer->srcadr), t.tv_sec, t.tv_nsec));
	*bc635_reftime_out = add_tspec(bt.time, t);
	DPRINTF(1, ("%s clock_state:  bt1_reftime=%ld.%09ld\n", refnumtoa(&peer->srcadr), bc635_reftime_out->tv_sec, bc635_reftime_out->tv_nsec));
	/*
	 * FIXME should add a random offset (srandom() % ns_granularity)
	 */
	DPRINTF(1, ("%s clock_state:          bt1=%ld.%09ld\n", refnumtoa(&peer->srcadr), bt.t1.tv_sec, bt.t1.tv_nsec));
	*clock_reftime_out = bt.t1;
	t = sub_tspec(*bc635_reftime_out, bt.t1);
	DPRINTF(1, ("%s clock_state: bc635ref-bt1=%ld.%09ld\n", refnumtoa(&peer->srcadr), t.tv_sec, t.tv_nsec));
}

/*
 * -------------------------------------------------------------------
 * gpssymm_receive - receive data from the serial interface
 *
 * This is the workhorse for NMEA data evaluation:
 *
 * + it checks all NMEA data, and rejects sentences that are not valid
 *   NMEA sentences
 * + it checks whether a sentence is known and to be used
 * + it parses the time and date data from the NMEA data string and
 *   augments the missing bits. (century in dat, whole date, ...)
 * + it rejects data that is not from the first accepted sentence in a
 *   burst
 * + it eventually replaces the receive time with the PPS edge time.
 * + it feeds the data to the internal processing stages.
 * -------------------------------------------------------------------
 */
static void
gpssymm_receive(
	struct recvbuf * rbufp
	)
{
	/* declare & init control structure ptrs */
	struct peer	    * const peer = rbufp->recv_peer;
	struct refclockproc * const pp = peer->procptr;
	gpssymm_unit	    * const up = (gpssymm_unit*)pp->unitptr;

	/* Use these variables to hold data until we decide its worth keeping */
	gpssymm_data rdata;
	char 	  rd_lastcode[BMAX];
	l_fp 	  rd_timestamp, rd_reftime;
	int	  rd_lencode;
	double	  rd_fudge;
	int 	  ret;
	union     btfp_ioctl_out btfp;
	struct    btfp_ioctl_gettime bt;
	int       locked;

	/* working stuff */
	/* struct calendar date;	** to keep & convert the time stamp */
	/* struct timespec tofs;	** offset to full-second reftime */
	/* gps_weektm      gpsw;	** week time storage */
	/* results of sentence/date/time parsing */
	u_char		sentence;	/* sentence tag */
	int		checkres;
	char *		cp;
	int		rc_date;
	int		rc_time;

	struct timespec t;
	struct timespec bt1_bc635_reftime;

	struct tm	gps_curr_tm;
	long		gps_curr_ns_offset = 0;
	time_t		gps_unix_time;

	/* make sure data has defined pristine state */
	/*ZERO(tofs);*/
	/*ZERO(date);*/
	/*ZERO(gpsw);*/
	ZERO(gps_curr_tm);
	gps_curr_tm.tm_zone = "UTC";

	DPRINTF(1, ("%s gpssymm_receive\n",
		refnumtoa(&peer->srcadr)));

	/* 
	 * Read the timecode and timestamp, then initialise field
	 * processing. The <CR><LF> at the NMEA line end is translated
	 * to <LF><LF> by the terminal input routines on most systems,
	 * and this gives us one spurious empty read per record which we
	 * better ignore silently.
	 */
	rd_lencode = refclock_gtlin(rbufp, rd_lastcode,
				    sizeof(rd_lastcode), &rd_timestamp);

	DPRINTF(1, ("%s gpssymm_receive: rd_lencode=%d\n",
		refnumtoa(&peer->srcadr), rd_lencode));
	clock_gettime(CLOCK_REALTIME, &t);
	DPRINTF(1, ("%s gpssymm_receive: %ld.%09ld: rd_lastcode=%s\n",
		refnumtoa(&peer->srcadr), t.tv_sec, t.tv_nsec, rd_lastcode));


	checkres = field_init(peer, &rdata, rd_lastcode, rd_lencode);

	up->gps_tally.total++;

	switch (checkres) {
	default:
	case CHECK_VALID:
		DPRINTF(1, ("%s gpsread: valid data but no checksum: '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		refclock_report(peer, CEVNT_BADREPLY);
		up->gps_tally.invalid++;
		return;

	case CHECK_INVALID:
		DPRINTF(1, ("%s gpsread: invalid data: '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		refclock_report(peer, CEVNT_BADREPLY);
		up->gps_tally.invalid++;
		return;

	case CHECK_EMPTY:
		DPRINTF(1, ("%s gpsread: empty: '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		refclock_report(peer, CEVNT_TIMEOUT);
		up->gps_tally.invalid++;
		return;

	case CHECK_CSVALID:
		DPRINTF(1, ("%s gpsread: %d '%s': CKSUM VALID: %d\n",
			refnumtoa(&peer->srcadr), rd_lencode, rd_lastcode, checkres));
		break;
	}

	/* 
	 * --> below this point we have a valid NMEA sentence <--
	 *
	 * Check sentence name. Skip first 2 chars (talker ID) in most
	 * cases, to allow for $GLGGA and $GPGGA etc. Since the name
	 * field has at least 5 chars we can simply shift the field
	 * start.
	 */
	sentence = 0;
	cp = field_parse(&rdata, 0);
	if (strncmp(cp, "GPRMC,", 6) == 0) {
		sentence = NMEA_GPRMC;
	} else if (strncmp(cp, "PUBX,", 5) == 0) {
		sentence = NMEA_PUBX;
	} else {
		DPRINTF(1, ("%s unknown timecode '%s', not processing\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		up->gps_tally.invalid++;
		return;	/* not something we know about */
	}

	DPRINTF(1, ("%s gps_clock_state=%d, bc635_clock_state=%d, len=%d, timecode '%s'\n",
		refnumtoa(&peer->srcadr),
		up->gps_clock_state,
		up->bc635_clock_state,
		rd_lencode,
		rd_lastcode));

	/*
	 * Grab fields depending on clock string type and possibly wipe
	 * sensitive data from the last timecode.
	 */
	switch (sentence) {
	case NMEA_GPRMC:
		DPRINTF(1, ("%s processing GPRMC, timecode '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		/* Check quality byte, fetch data & time */
		rc_time	 = parse_time(&gps_curr_tm, &gps_curr_ns_offset, &rdata, 1);
		pp->leap = parse_qual(&rdata, 2, 'A', 0);
		rc_date	 = parse_date(&gps_curr_tm, &rdata, 9);

		/* Check sanity of time-of-day. */
		if (rc_time == 0) {	/* no time or conversion error? */
			checkres = CEVNT_BADTIME;
			up->gps_tally.malformed++;
		}
		/* Check sanity of date. */
		else if (rc_date == 0) {/* no date or conversion error? */
			checkres = CEVNT_BADDATE;
			up->gps_tally.malformed++;
		}
		/* check clock sanity */
		else if (pp->leap == LEAP_NOTINSYNC) { /* no good status? */
			checkres = CEVNT_BADREPLY;
			up->gps_tally.nofix++;
		} else {
			checkres = 0;
		}
		if (checkres != 0) {
			DPRINTF(1, ("%s processing GPRMC, bad timecode '%s': #%d\n",
				refnumtoa(&peer->srcadr), rd_lastcode, checkres));
			up->gps_clock_state = CS_INVALID_CLOCK;
			DPRINTF(1, ("%s set gps_clockstate to INVALID_CLOCK\n",
				refnumtoa(&peer->srcadr)));
			if (up->bc635_clock_state == BCS_TIME_PPS_SYNCHRONIZED) {
				DPRINTF(1, ("%s downgrade bc635_clockstate from PPS_SYNCHRONIZED to FREE_RUNNING\n",
					refnumtoa(&peer->srcadr)));
				up->bc635_clock_state = BCS_TIME_FREE_RUNNING;
				peer->precision = FREE_PRECISION;
			} else {
				up->bc635_clock_state = BCS_PRE_INITIALIZED;
			}
			refclock_report(peer, checkres);
			return;
		}
		/*
		 * time is good, now check if we have/need leap seconds.
		 * the UBLOX unit will not report correct time if it hasn't acquired leap seconds.
		 */
		if (up->gps_leap_seconds == 0 && up->gps_type == NMEA_GPS_NEO7_UBLOX) {
			DPRINTF(1, ("%s UBLOX7 and no leap seconds yet, sending PUBX,04 request\n",
				refnumtoa(&peer->srcadr)));
			up->gps_clock_state = CS_WAIT_CLOCK_LEAP_SECS;
			gps_send(pp->io.fd, "$PUBX,04*37\r\n", peer);
			refclock_report(peer, CEVNT_TIMEOUT);
			return;
		} else {
			DPRINTF(1, ("%s has valid gps clock\n",
				refnumtoa(&peer->srcadr)));
			up->gps_clock_state = CS_VALID_CLOCK;
		}

		break;

	case NMEA_PUBX:
		DPRINTF(1, ("%s processing PUBX, timecode '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		if (up->gps_clock_state == CS_WAIT_CLOCK_LEAP_SECS) {
			float gps_week_secs;
			char comma;
			cp = field_parse(&rdata, 4);
			if (sscanf(cp, "%f%c", &gps_week_secs, &comma) != 2 || comma != ',') {
				DPRINTF(1, ("%s bad field for gps_week_seconds\n",
					refnumtoa(&peer->srcadr)));
				refclock_report(peer, CEVNT_TIMEOUT);
				return;
			}
			up->gps_week_seconds = (int)ceil(gps_week_secs);
			DPRINTF(1, ("%s gps_week_seconds = %d\n",
				refnumtoa(&peer->srcadr), up->gps_week_seconds));
			cp = field_parse(&rdata, 5);
			if (sscanf(cp, "%d%c", &up->gps_week_number, &comma) != 2 || comma != ',') {
				DPRINTF(1, ("%s bad field for gps_week_number\n",
					refnumtoa(&peer->srcadr)));
				refclock_report(peer, CEVNT_TIMEOUT);
				return;
			}
			DPRINTF(1, ("%s gps_week_number = %d\n",
				refnumtoa(&peer->srcadr), up->gps_week_number));
			/*
			 * week 1979 is when this code was written,
			 * week 4152 is when Unix time overflows and the BC635 hardware stops
			 * working (2038-01-17).
			 */
			if (up->gps_week_number < 1979 || up->gps_week_number >= 4152) {
				DPRINTF(1, ("%s illegal value for gps_week_number\n",
					refnumtoa(&peer->srcadr)));
				refclock_report(peer, CEVNT_TIMEOUT);
				return;
			}
			cp = field_parse(&rdata, 6);
			if (sscanf(cp, "%d%c", &up->gps_leap_seconds, &comma) != 2) {
				DPRINTF(1, ("%s bad field for gps_leap_seconds\n",
					refnumtoa(&peer->srcadr)));
				refclock_report(peer, CEVNT_TIMEOUT);
				return;
			}
			DPRINTF(1, ("%s gps_leap_seconds = %d\n",
				refnumtoa(&peer->srcadr), up->gps_leap_seconds));
			if (comma == 'D') {
				DPRINTF(1, ("%s gps_leap_seconds is still stored firmware value\n",
					refnumtoa(&peer->srcadr)));
				refclock_report(peer, CEVNT_TIMEOUT);
				return;
			}
			if (comma != ',') {
				DPRINTF(1, ("%s unexpected character after leap seconds %c\n",
					refnumtoa(&peer->srcadr), comma));
				refclock_report(peer, CEVNT_TIMEOUT);
				return;
			}
			DPRINTF(1, ("%s PUBX,04 valid, gps leap seconds %d - gps clock valid\n",
				refnumtoa(&peer->srcadr), up->gps_leap_seconds));
			clock_gettime(CLOCK_REALTIME, &up->gps_info_reftime);
			/*
			 * has valid clock
			 */
			up->gps_clock_state = CS_VALID_CLOCK;
		} else {
			DPRINTF(1, ("%s got PUBX,04 valid but was not waiting for it\n",
				refnumtoa(&peer->srcadr)));
		}
		refclock_report(peer, CEVNT_TIMEOUT);
		return;

	default:
		INVARIANT(0);	/* Coverity 97123 */
		return;
	}

#if 0
	else
		checkres = -1;

	if (checkres != -1) {
		save_ltc(pp, rd_lastcode, rd_lencode);
		refclock_report(peer, checkres);
		return;
	}
#endif

	/*
	 * GPS clock now valid
	 */

	/*
	 *
	 * FIXME: should recheck with GPS every now and then
	 *
	 */

	DPRINTF(1, ("%s state after parsing: gps_clock_state=%d, bc635_clock_state=%d\n",
		refnumtoa(&peer->srcadr),
		up->gps_clock_state,
		up->bc635_clock_state));

	INVARIANT(up->gps_clock_state == CS_VALID_CLOCK);

	INVARIANT(sentence == NMEA_GPRMC);

	gps_unix_time = timegm(&gps_curr_tm);

	DPRINTF(1, ("%s effective timecode: %04u-%02u-%02u %02d:%02d:%02d [UNIX:%ld]\n",
		refnumtoa(&peer->srcadr),
		gps_curr_tm.tm_year + YEAR_1900,
		gps_curr_tm.tm_mon + 1,
		gps_curr_tm.tm_mday,
		gps_curr_tm.tm_hour,
		gps_curr_tm.tm_min,
		gps_curr_tm.tm_sec,
		gps_unix_time));

	up->gps_tally.bc_time_processed++;

	switch (up->bc635_clock_state) {
	case BCS_PRE_INITIALIZED:
		DPRINTF(1, ("%s clock_state: PRE_INITIALIZED\n", refnumtoa(&peer->srcadr)));
		/*
		 * set UNIX time in BC635 board.
		 */
		memset(&btfp, 0, sizeof (btfp));
		btfp.set_unixtime.unix_time = (uint32_t)gps_unix_time;
		clock_gettime(CLOCK_REALTIME, &t);
		DPRINTF(1, ("%s clock_state: %ld.%09ld: set_unix_time\n", refnumtoa(&peer->srcadr), t.tv_sec, t.tv_nsec));
		ret = ioctl(up->bc635_fd, BTFP_WRITE_UNIX_TIME, &btfp);
		clock_gettime(CLOCK_REALTIME, &t);
		DPRINTF(1, ("%s clock_state: %ld.%09ld: set_unix_time=%d\n", refnumtoa(&peer->srcadr), t.tv_sec, t.tv_nsec, ret));
		INVARIANT(ret == 0);
		up->bc635_clock_state = BCS_TIME_INITIALIZED;
		up->bc635_time_initialized_count = 0;
		up->bc635_time_locked_count = 0;
		refclock_report(peer, CEVNT_TIMEOUT);
		return;
	case BCS_TIME_INITIALIZED:
		DPRINTF(1, ("%s clock_state: TIME_INITIALIZED, #%d, locked=#%d\n", refnumtoa(&peer->srcadr), up->bc635_time_initialized_count, up->bc635_time_locked_count));
		get_bc635_time(peer,
			       up,
			       &up->last_bc635_t_reftime,
			       &up->last_t_clocktime,
			       &locked);

		up->bc635_time_initialized_count++;
		if (locked) {
			up->bc635_time_locked_count++;
		} else {
			up->bc635_time_locked_count = 0;
		}
		if (up->bc635_time_locked_count >= BC635_MIN_LOCKED_COUNT) {
			up->bc635_clock_state = BCS_TIME_PPS_SYNCHRONIZED;
			/* refclock_report(peer, CEVNT_NOMINAL); */
		} else if (up->bc635_time_initialized_count >= BC635_MIN_LOCKED_COUNT * 3) {
			DPRINTF(1, ("%s clock_state: LOCK TIMEOUT -- reset to beginning\n", refnumtoa(&peer->srcadr)));
			up->gps_clock_state = CS_INVALID_CLOCK;
			up->bc635_clock_state = BCS_PRE_INITIALIZED;
			refclock_report(peer, CEVNT_TIMEOUT);
		}
		return;
	case BCS_TIME_PPS_SYNCHRONIZED:
		up->gps_tally.bc_time_pps_sync++;
		DPRINTF(1, ("%s clock_state: PPS_SYNCHRONIZED #%d, locked=#%d\n", refnumtoa(&peer->srcadr), up->bc635_time_initialized_count, up->bc635_time_locked_count));
		get_bc635_time(peer,
			       up,
			       &up->last_bc635_t_reftime,
			       &up->last_t_clocktime,
			       &locked);
		if (locked) {
			up->bc635_time_locked_count++;
			peer->precision = LOCKED_PRECISION;
		} else {
			DPRINTF(1, ("%s clock_state: PPS_SYNCHRONIZED #%d, locked=#%d: lost lock, now freerunning\n", refnumtoa(&peer->srcadr), up->bc635_time_initialized_count, up->bc635_time_locked_count));
			up->bc635_clock_state = BCS_TIME_FREE_RUNNING;
			up->bc635_time_locked_count = 0;
			peer->precision = FREE_PRECISION;
		}
		break;
	case BCS_TIME_FREE_RUNNING:
		up->gps_tally.bc_time_free_running++;
		DPRINTF(1, ("%s clock_state: FREE_RUNNING\n", refnumtoa(&peer->srcadr)));
		get_bc635_time(peer,
			       up,
			       &up->last_bc635_t_reftime,
			       &up->last_t_clocktime,
			       &locked);
		if (locked) {
			DPRINTF(1, ("%s clock_state: PPS_SYNCHRONIZED #%d, locked=#%d: regained lock, now pps_synchronized\n", refnumtoa(&peer->srcadr), up->bc635_time_initialized_count, up->bc635_time_locked_count));
			up->bc635_time_locked_count++;
			peer->precision = LOCKED_PRECISION;
		} else {
			up->bc635_clock_state = BCS_TIME_FREE_RUNNING;
			up->bc635_time_locked_count = 0;
			peer->precision = FREE_PRECISION;
		}
		break;
	default:
		INVARIANT(0);
	}

	up->last_clocktime = tspec_stamp_to_lfp(up->last_t_clocktime);
	up->last_bc635_reftime = tspec_stamp_to_lfp(up->last_bc635_t_reftime);

	rd_fudge = pp->fudgetime2;

	DPRINTF(1, ("%s using '%s'\n",
		    refnumtoa(&peer->srcadr), rd_lastcode));

	/* Data will be accepted. Update stats & log data. */
	save_ltc(pp, rd_lastcode, rd_lencode);
	pp->lastrec = up->last_clocktime;

	/* peer->precision = PPS_PRECISION; */
	peer->flags |= FLAG_PPS;
	DPRINTF(1, ("%s PPS_RELATE_PHASE\n", refnumtoa(&peer->srcadr)));

	DPRINTF(1, ("%s refclock_process_offset(%ld.%09ld, %ld.%09ld, %.3f)\n",
		refnumtoa(&peer->srcadr),
		lfp_uintv_to_tspec(up->last_bc635_reftime).tv_sec,
		lfp_uintv_to_tspec(up->last_bc635_reftime).tv_nsec,
		lfp_uintv_to_tspec(up->last_clocktime).tv_sec,
		lfp_uintv_to_tspec(up->last_clocktime).tv_nsec,
		rd_fudge));
	refclock_process_offset(pp, up->last_bc635_reftime, up->last_clocktime, rd_fudge);
}


/*
 * -------------------------------------------------------------------
 * gpssymm_poll - called by the transmit procedure
 *
 * Does the necessary bookkeeping stuff to keep the reported state of
 * the clock in sync with reality.
 *
 * We go to great pains to avoid changing state here, since there may
 * be more than one eavesdropper receiving the same timecode.
 * -------------------------------------------------------------------
 */
static void
gpssymm_poll(
	int           unit,
	struct peer * peer
	)
{
	struct refclockproc * const pp = peer->procptr;
	gpssymm_unit	    * const up = (gpssymm_unit *)pp->unitptr;

	DPRINTF(1, ("%s gpssymm_poll, unit=%d\n",
		refnumtoa(&peer->srcadr), unit));

	
	/*
	 * Process median filter samples. If none received, declare a
	 * timeout and keep going.
	 */

	/*
	 * If the median filter is empty, claim a timeout. Else process
	 * the input data and keep the stats going.
	 */
	if (pp->coderecv == pp->codeproc) {
		DPRINTF(1, ("%s gpssymm_poll, unit=%d -- reported timeout\n",
			refnumtoa(&peer->srcadr), unit));
		refclock_report(peer, CEVNT_TIMEOUT);
	} else {
		DPRINTF(1, ("%s gpssymm_poll, unit=%d -- reported receive\n",
			refnumtoa(&peer->srcadr), unit));
		pp->polls++;
		pp->lastref = pp->lastrec;
		refclock_receive(peer);
	}
	
	/*
	 * If extended logging is required, write the tally stats to the
	 * clockstats file; otherwise just do a normal clock stats
	 * record. Clear the tally stats anyway.
	*/
	if (peer->ttl & NMEA_EXTLOG_MASK) {
		/* Log & reset counters with extended logging */
		const char *nmea = pp->a_lastcode;
		if (*nmea == '\0') nmea = "(none)";
		mprintf_clock_stats(
		  &peer->srcadr, "%s  %u %u %u %u %u %u - %u %u %u",
		  nmea,
		  up->gps_tally.total,
		  up->gps_tally.valid_ignored,
		  up->gps_tally.valid_processed,
		  up->gps_tally.invalid,
		  up->gps_tally.malformed,
		  up->gps_tally.nofix,
		  up->gps_tally.bc_time_processed,
		  up->gps_tally.bc_time_pps_sync,
		  up->gps_tally.bc_time_free_running);
	} else {
		record_clock_stats(&peer->srcadr, pp->a_lastcode);
	}
	ZERO(up->gps_tally);
}

/*
 * -------------------------------------------------------------------
 * Save the last timecode string, making sure it's properly truncated
 * if necessary and NUL terminated in any case.
 */
static void
save_ltc(
	struct refclockproc * const pp,
	const char * const          tc,
	size_t                      len
	)
{
	if (len >= sizeof(pp->a_lastcode))
		len = sizeof(pp->a_lastcode) - 1;
	pp->lencode = (u_short)len;
	memcpy(pp->a_lastcode, tc, len);
	pp->a_lastcode[len] = '\0';
}


/*
 * -------------------------------------------------------------------
 *  gps_send(fd, cmd, peer)	Sends a command to the GPS receiver.
 *   as in gps_send(fd, "rqts,u", peer);
 *
 * If 'cmd' starts with a '$' it is assumed that this command is in raw
 * format, that is, starts with '$', ends with '<cr><lf>' and that any
 * checksum is correctly provided; the command will be send 'as is' in
 * that case. Otherwise the function will create the necessary frame
 * (start char, chksum, final CRLF) on the fly.
 *
 * We don't currently send any data, but would like to send RTCM SC104
 * messages for differential positioning. It should also give us better
 * time. Without a PPS output, we're Just fooling ourselves because of
 * the serial code paths
 * -------------------------------------------------------------------
 */
static void
gps_send(
	int           fd,
	const char  * cmd,
	struct peer * peer
	)
{
	/* $...*xy<CR><LF><NUL> add 7 */
	char	      buf[NMEA_PROTO_MAXLEN + 7];
	int	      len;
	u_char	      dcs;
	const u_char *beg, *end;

	if (*cmd != '$') {
		/* get checksum and length */
		beg = end = (const u_char*)cmd;
		dcs = 0;
		while (*end >= ' ' && *end != '*')
			dcs ^= *end++;
		len = end - beg;
		/* format into output buffer with overflow check */
		len = snprintf(buf, sizeof(buf), "$%.*s*%02X\r\n",
			       len, beg, dcs);
		if ((size_t)len >= sizeof(buf)) {
			DPRINTF(1, ("%s gps_send: buffer overflow for command '%s'\n",
				    refnumtoa(&peer->srcadr), cmd));
			return;	/* game over player 1 */
		}
		cmd = buf;
	} else {
		len = strlen(cmd);
	}

	DPRINTF(1, ("%s gps_send: '%.*s'\n", refnumtoa(&peer->srcadr),
		len - 2, cmd));

	/* send out the whole stuff */
	if (write(fd, cmd, len) == -1)
		refclock_report(peer, CEVNT_FAULT);
}

/*
 * -------------------------------------------------------------------
 * helpers for faster field splitting
 * -------------------------------------------------------------------
 *
 * set up a field record, check syntax and verify checksum
 *
 * format is $XXXXX,1,2,3,4*ML
 *
 * 8-bit XOR of characters between $ and * noninclusive is transmitted
 * in last two chars M and L holding most and least significant nibbles
 * in hex representation such as:
 *
 *   $GPGLL,5057.970,N,00146.110,E,142451,A*27
 *   $GPVTG,089.0,T,,,15.2,N,,*7F
 *
 * Some other constraints:
 * + The field name must at least 5 upcase characters or digits and must
 *   start with a character.
 * + The checksum (if present) must be uppercase hex digits.
 * + The length of a sentence is limited to 80 characters (not including
 *   the final CR/LF nor the checksum, but including the leading '$')
 *
 * Return values:
 *  + CHECK_INVALID
 *	The data does not form a valid NMEA sentence or a checksum error
 *	occurred.
 *  + CHECK_VALID
 *	The data is a valid NMEA sentence but contains no checksum.
 *  + CHECK_CSVALID
 *	The data is a valid NMEA sentence and passed the checksum test.
 * -------------------------------------------------------------------
 */
static int
field_init(
	struct peer * peer,
	gpssymm_data * data,	/* context structure		       */
	char 	  * cptr,	/* start of raw data		       */
	int	    dlen	/* data len, not counting trailing NUL */
	)
{
	u_char cs_l;	/* checksum local computed	*/
	u_char cs_r;	/* checksum remote given	*/
	char * eptr;	/* buffer end end pointer	*/
	char   tmp;	/* char buffer 			*/

	cs_l = 0;
	cs_r = 0;
	/* some basic input constraints */
	if (dlen < 0)
		dlen = 0;
	eptr = cptr + dlen;
	*eptr = '\0';

	DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s'\n", refnumtoa(&peer->srcadr), dlen, cptr));
	
	/* load data context */	
	data->base = cptr;
	data->cptr = cptr;
	data->cidx = 0;
	data->blen = dlen;

	/* syntax check follows here. check allowed character
	 * sequences, updating the local computed checksum as we go.
	 *
	 * regex equiv: '^\$[A-Z][A-Z0-9]{4,}[^*]*(\*[0-9A-F]{2})?$'
	 */

	/* -*- start character: '^\$' */
	if (*cptr == '\0') {
		DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s': CHECK_EMPTY\n", refnumtoa(&peer->srcadr), dlen, cptr));
		return CHECK_EMPTY;
	}
	if (*cptr++ != '$') {
		DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s': CHECK_INVALID (no $)\n", refnumtoa(&peer->srcadr), dlen, cptr));
		return CHECK_INVALID;
	}

	/* -*- advance context beyond start character */
	data->base++;
	data->cptr++;
	data->blen--;
	
	/* -*- field name: '[A-Z][A-Z0-9]{4,},' */
	if (*cptr < 'A' || *cptr > 'Z') {
		DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s': CHECK_INVALID (first letter of field name)\n", refnumtoa(&peer->srcadr), dlen, cptr));
		return CHECK_INVALID;
	}
	cs_l ^= *cptr++;
	while ((*cptr >= 'A' && *cptr <= 'Z') ||
	       (*cptr >= '0' && *cptr <= '9')  )
		cs_l ^= *cptr++;
	if (*cptr != ',' || (cptr - data->base) < NMEA_PROTO_IDLEN) {
		DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s': CHECK_INVALID (nth letter of field name)\n", refnumtoa(&peer->srcadr), dlen, cptr));
		return CHECK_INVALID;
	}
	cs_l ^= *cptr++;

	/* -*- data: '[^*]*' */
	while (*cptr && *cptr != '*')
		cs_l ^= *cptr++;
	
	/* -*- checksum field: (\*[0-9A-F]{2})?$ */
	if (*cptr == '\0') {
		DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s': CHECK_VALID (no cksum)\n", refnumtoa(&peer->srcadr), dlen, cptr));
		return CHECK_VALID;
	}
	if (*cptr != '*' || cptr != eptr - 3 ||
	    (cptr - data->base) >= NMEA_PROTO_MAXLEN) {
		DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s': CHECK_INVALID (cksum len)\n", refnumtoa(&peer->srcadr), dlen, cptr));
		return CHECK_INVALID;
	}

	for (cptr++; (tmp = *cptr) != '\0'; cptr++) {
		if (tmp >= '0' && tmp <= '9')
			cs_r = (cs_r << 4) + (tmp - '0');
		else if (tmp >= 'A' && tmp <= 'F')
			cs_r = (cs_r << 4) + (tmp - 'A' + 10);
		else
			break;
	}

	/* -*- make sure we are at end of string and csum matches */
	if (cptr != eptr || cs_l != cs_r) {
		DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s': CHECK_INVALID (no eos or bad cksum)\n", refnumtoa(&peer->srcadr), dlen, cptr));
		return CHECK_INVALID;
	}

	DPRINTF(3, ("%s field_init: dlen=%d, cptr='%s': CHECK_CSVALID\n", refnumtoa(&peer->srcadr), dlen, cptr));
	return CHECK_CSVALID;
}

/*
 * -------------------------------------------------------------------
 * fetch a data field by index, zero being the name field. If this
 * function is called repeatedly with increasing indices, the total load
 * is O(n), n being the length of the string; if it is called with
 * decreasing indices, the total load is O(n^2). Try not to go backwards
 * too often.
 * -------------------------------------------------------------------
 */
static char *
field_parse(
	gpssymm_data * data,
	int 	    fn
	)
{
	char tmp;

	if (fn < data->cidx) {
		data->cidx = 0;
		data->cptr = data->base;
	}
	while ((fn > data->cidx) && (tmp = *data->cptr) != '\0') {
		data->cidx += (tmp == ',');
		data->cptr++;
	}
	return data->cptr;
}

/*
 * -------------------------------------------------------------------
 * PARSING HELPERS
 * -------------------------------------------------------------------
 *
 * Check sync status
 *
 * If the character at the data field start matches the tag value,
 * return LEAP_NOWARNING and LEAP_NOTINSYNC otherwise. If the 'inverted'
 * flag is given, just the opposite value is returned. If there is no
 * data field (*cp points to the NUL byte) the result is LEAP_NOTINSYNC.
 * -------------------------------------------------------------------
 */
static u_char
parse_qual(
	gpssymm_data * rd,
	int         idx,
	char        tag,
	int         inv
	)
{
	static const u_char table[2] =
				{ LEAP_NOTINSYNC, LEAP_NOWARNING };
	char * dp;

	dp = field_parse(rd, idx);
	
	return table[ *dp && ((*dp == tag) == !inv) ];
}

/*
 * -------------------------------------------------------------------
 * Parse a time stamp in HHMMSS[.sss] format with error checking.
 *
 * returns 1 on success, 0 on failure
 * -------------------------------------------------------------------
 */
static int
parse_time(
	struct tm	*t,
	long		*ns_off,
	gpssymm_data    *rd,
	int		idx
	)
{
	static const unsigned long weight[4] = {
		0, 100000000, 10000000, 1000000
	};

	int	rc;
	u_int	h;
	u_int	m;
	u_int	s;
	int	p1;
	int	p2;
	u_long	f;
	char  * dp;

	dp = field_parse(rd, idx);
	rc = sscanf(dp, "%2u%2u%2u%n.%3lu%n", &h, &m, &s, &p1, &f, &p2);
	if (rc < 3 || p1 != 6) {
		DPRINTF(1, ("nmea: invalid time code: '%.6s'\n", dp));
		return FALSE;
	}
	
	/* value sanity check */
	if (h > 23 || m > 59 || s > 60) {
		DPRINTF(1, ("nmea: invalid time spec %02u:%02u:%02u\n",
			    h, m, s));
		return FALSE;
	}

	t->tm_hour = h;
	t->tm_min  = m;
	t->tm_sec  = s;

	/* if we have a fraction, scale it up to nanoseconds. */
	if (rc == 4)
		*ns_off = f * weight[p2 - p1 - 1];
	else
		*ns_off = 0;

	return TRUE;
}

/*
 * -------------------------------------------------------------------
 * Parse a date string from an NMEA sentence. This could either be a
 * partial date in DDMMYY format in one field, or DD,MM,YYYY full date
 * spec spanning three fields. This function does some extensive error
 * checking to make sure the date string was consistent.
 *
 * returns 1 on success, 0 on failure
 * -------------------------------------------------------------------
 */
static int
parse_date(
	struct tm	*t,
	gpssymm_data    *rd,
	int		idx
	)
{
	int	rc;
	u_int	y;
	u_int	m;
	u_int	d;
	int	p;
	char  * dp;
	
	dp = field_parse(rd, idx);

	rc = sscanf(dp, "%2u%2u%2u%n", &d, &m, &y, &p);
	if (rc != 3 || p != 6) {
		DPRINTF(1, ("nmea: invalid date code: '%.6s'\n",
			    dp));
		return FALSE;
	}

	/* value sanity check */
	if (d < 1 || d > 31 || m < 1 || m > 12) {
		DPRINTF(1, ("nmea: invalid date spec (YMD) %04u:%02u:%02u\n",
			    y, m, d));
		return FALSE;
	}
	
	/* store results */
	t->tm_mday = d;
	t->tm_mon  = m - 1;
	t->tm_year = (y + YEAR_CENTURY) - YEAR_1900;

	return TRUE;
}

#else
NONEMPTY_TRANSLATION_UNIT
#endif /* REFCLOCK && CLOCK_GPSSYM */
