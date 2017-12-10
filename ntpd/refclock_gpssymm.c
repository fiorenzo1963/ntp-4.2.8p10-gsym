/*
 * refclock_gpssymm.c - clock driver GPS disciplined symmetricomm BC635-PCI
 *                      (uses GPS 1PPS as input to 1PPS to BC635-PCI)
 *
 * Fio Cattaneo <fio@cattaneo.us>, Nov 24, 2017
 *
 * based on refclock_nmea.c
 *
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
#define NMEA_GPS_GENERIC		0x00000200U
#define NMEA_GPS_MASK			0x00000F00U

#define NMEA_BAUDRATE_MASK	0x00000070U
#define NMEA_BAUDRATE_SHIFT	4

#define NMEA_PROTO_IDLEN	4	/* tag name must be at least 5 chars */
#define NMEA_PROTO_MINLEN	6	/* min chars in sentence, excluding CS */
#define NMEA_PROTO_MAXLEN	100	/* max chars in sentence, excluding CS */
#define NMEA_PROTO_FIELDS	32	/* not official; limit on fields per record */

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
#define	SPEED232	B38400	/* uart speed (38400 bps) fallback */
#define	PRECISION	(-22)	/* precision assumed (about 0.125 us) */
#define	PPS_PRECISION	(-22)	/* precision assumed (about 0.125 us) */
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
	BCS_NOT_INITIALIZED,		/* not initialized yet */
	BCS_INITIALIZED,		/* initialized */
	BCS_PPS_SYNCHRONIZED,		/* initialized with PPS valid */
	BCS_FREE_RUNNING,		/* free running (without PPS) */
};

#define YEAR_CENTURY	2000

/*
 * Unit control structure
 *
 * the BC635 hardware has a 32 bit field for seconds time, so there is no need to
 * worry about centuries, we are in 2017 and the BC635 will be obsolete in 2038
 * (so this is why we can simply fix YEAR CENTURY to 2000).
 */
typedef struct {
	l_fp			last_reftime;	/* last processed reference stamp */
	struct timespec 	last_t_reftime;
	l_fp    		last_clocktime;	/* last clocktime matching ref stamp */
	struct timespec 	last_t_clocktime;
	enum gps_clockstate	gps_clock_state;
	enum bc635_clockstate	gps_bc635_clock_state;
	struct gps_tally
	{
		u_int total;             /* total packes */
		u_int valid_ignored;     /* valid, but don't have correct time yet */
		u_int valid_processed;   /* valid, accepted */
		u_int invalid;
	} gps_tally;
	int			gps_mode_flags;		/* (ttl & NMEA_GPS_MASK) */
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
static void	field_wipe	(gpssymm_data * data, ...);
static u_char	parse_qual	(gpssymm_data * data, int idx, char tag, int inv);
static int	parse_time	(struct gpssymm_unit *gu, gpssymm_data *, int idx);
static int	parse_date	(struct gpssymm_unit *gu, gpssymm_data*, int idx);
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
	char				device[20];
	size_t				devlen;
	u_int32				rate;
	int				baudrate;
	const char *			baudtext;

	DPRINTF(1, ("%s gpssymm_start, unit=%d, ttl=0x%x\n",
		refnumtoa(&peer->srcadr), unit, peer->ttl));

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
		baudrate = SPEED232;
		baudtext = "38400 (fallback)";
		break;
	}

	/* Allocate and initialize unit structure */
	pp->unitptr = (caddr_t)up;
	pp->io.fd = -1;
	pp->io.clock_recv = gpssymm_receive;
	pp->io.srcclock = peer;
	pp->io.datalen = 0;
	/* force change detection on first valid message */
	memset(&up->last_reftime, 0xFF, sizeof(up->last_reftime));
	ZERO(up->tally);

	/* Initialize miscellaneous variables */
	peer->precision = PRECISION;
	pp->clockdesc = DESCRIPTION;
	memcpy(&pp->refid, REFID, 4);

	/* Open serial port. Use CLK line discipline, if available. */
	devlen = snprintf(device, sizeof(device), GPS_DEVICE, unit);
	if (devlen >= sizeof(device)) {
		msyslog(LOG_ERR, "%s clock device name too long",
			refnumtoa(&peer->srcadr));
		return FALSE; /* buffer overflow */
	}
	pp->io.fd = refclock_open(device, baudrate, LDISC_CLK);
	if (0 >= pp->io.fd) {
		pp->io.fd = nmead_open(device);
		if (-1 == pp->io.fd)
			return FALSE;
	}
	LOGIF(CLOCKINFO, (LOG_NOTICE, "%s serial %s open at %s bps",
	      refnumtoa(&peer->srcadr), device, baudtext));

	/* succeed if this clock can be added */
	return io_addclock(&pp->io) != 0;
}


/*
 * -------------------------------------------------------------------
 * gpssymm_shutdown - shut down a GPS clock
 * 
 * NOTE this routine is called after gpssymm_start() returns failure,
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
	if (-1 != pp->io.fd)
		io_closeclock(&pp->io);
	pp->io.fd = -1;
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

	char   device[32];
	size_t devlen;

	DPRINTF(1, ("%s gpssymm_control, unit=%d\n",
		refnumtoa(&peer->srcadr), unit));
	
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

	if (-1 != pp->io.fd)
		gps_send(pp->io.fd, "$PUBX,04*37\r\n", peer);
	
	UNUSED_ARG(unit);
	UNUSED_ARG(peer);
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

	/* working stuff */
	struct calendar date;	/* to keep & convert the time stamp */
	struct timespec tofs;	/* offset to full-second reftime */
	gps_weektm      gpsw;	/* week time storage */
	/* results of sentence/date/time parsing */
	u_char		sentence;	/* sentence tag */
	int		checkres;
	char *		cp;
	int		rc_date;
	int		rc_time;
	struct timespec t;

	/* make sure data has defined pristine state */
	ZERO(tofs);
	ZERO(date);
	ZERO(gpsw);

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

	switch (checkres) {

	default:
	case CHECK_VALID:
		DPRINTF(1, ("%s gpsread: valid data but no checksum: '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		refclock_report(peer, CEVNT_BADREPLY);
		return;

	case CHECK_INVALID:
		DPRINTF(1, ("%s gpsread: invalid data: '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		refclock_report(peer, CEVNT_BADREPLY);
		return;

	case CHECK_EMPTY:
		DPRINTF(1, ("%s gpsread: empty: '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		return;

	case CHECK_CSVALID:
		DPRINTF(1, ("%s gpsread: %d '%s': CKSUM VALID: %d\n",
			refnumtoa(&peer->srcadr), rd_lencode, rd_lastcode, checkres));
		break;
	}
	up->tally.total++;

	/* 
	 * --> below this point we have a valid NMEA sentence <--
	 *
	 * Check sentence name. Skip first 2 chars (talker ID) in most
	 * cases, to allow for $GLGGA and $GPGGA etc. Since the name
	 * field has at least 5 chars we can simply shift the field
	 * start.
	 */
	cp = field_parse(&rdata, 0);
	if      (strncmp(cp, "GPRMC,", 6) == 0)
		sentence = NMEA_GPRMC;
	else if (strncmp(cp, "PUBX,", 5) == 0)
		sentence = NMEA_PUBX;
	else
		return;	/* not something we know about */

	DPRINTF(1, ("%s processing %d bytes, timecode '%s'\n",
		refnumtoa(&peer->srcadr), rd_lencode, rd_lastcode));

	/*
	 * Grab fields depending on clock string type and possibly wipe
	 * sensitive data from the last timecode.
	 */
	switch (sentence) {

	case NMEA_GPRMC:
		DPRINTF(1, ("%s processing GPRMC, timecode '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		/* Check quality byte, fetch data & time */
		rc_time	 = parse_time(&date, &tofs.tv_nsec, &rdata, 1);
		pp->leap = parse_qual(&rdata, 2, 'A', 0);
		rc_date	 = parse_date(&date, &rdata, 9, DATE_1_DDMMYY)
			&& unfold_century(&date, rd_timestamp.l_ui);
		if (CLK_FLAG4 & pp->sloppyclockflag)
			field_wipe(&rdata, 3, 4, 5, 6, -1);
		break;

	case NMEA_PUBX:
		DPRINTF(1, ("%s processing PUBX, timecode '%s'\n",
			refnumtoa(&peer->srcadr), rd_lastcode));
		break;
		
	default:
		INVARIANT(0);	/* Coverity 97123 */
		return;
	}

	return;

	/* Check sanity of time-of-day. */
	if (rc_time == 0) {	/* no time or conversion error? */
		checkres = CEVNT_BADTIME;
		up->tally.malformed++;
	}
	/* Check sanity of date. */
	else if (rc_date == 0) {/* no date or conversion error? */
		checkres = CEVNT_BADDATE;
		up->tally.malformed++;
	}
	/* check clock sanity; [bug 2143] */
	else if (pp->leap == LEAP_NOTINSYNC) { /* no good status? */
		checkres = CEVNT_BADREPLY;
		up->tally.rejected++;
	}
	else
		checkres = -1;

	if (checkres != -1) {
		save_ltc(pp, rd_lastcode, rd_lencode);
		refclock_report(peer, checkres);
		return;
	}

	DPRINTF(1, ("%s effective timecode: %04u-%02u-%02u %02d:%02d:%02d\n",
		refnumtoa(&peer->srcadr),
		date.year, date.month, date.monthday,
		date.hour, date.minute, date.second));

	/*
	 * Get the reference time stamp from the calendar buffer.
	 * Process the new sample in the median filter and determine the
	 * timecode timestamp, but only if the PPS is not in control.
	 * Discard sentence if reference time did not change.
	 */
	rd_reftime = eval_gps_time(peer, &date, &tofs, &rd_timestamp);
	if (L_ISEQU(&up->last_reftime, &rd_reftime)) {
		/* Do not touch pp->a_lastcode on purpose! */
		up->tally.filtered++;
		return;
	}
	up->last_reftime = rd_reftime;
	rd_fudge = pp->fudgetime2;

	DPRINTF(1, ("%s using '%s'\n",
		    refnumtoa(&peer->srcadr), rd_lastcode));

	/* Data will be accepted. Update stats & log data. */
	save_ltc(pp, rd_lastcode, rd_lencode);
	pp->lastrec = rd_timestamp;

	refclock_process_offset(pp, rd_reftime, rd_timestamp, rd_fudge);
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

	refclock_report(peer, CEVNT_TIMEOUT);

	DPRINTF(1, ("%s gpssymm_poll, unit=%d -- reported timeout\n",
		refnumtoa(&peer->srcadr), unit));

	return;
	
	/*
	 * Process median filter samples. If none received, declare a
	 * timeout and keep going.
	 */

	/*
	 * If the median filter is empty, claim a timeout. Else process
	 * the input data and keep the stats going.
	 */
	if (pp->coderecv == pp->codeproc) {
		refclock_report(peer, CEVNT_TIMEOUT);
	} else {
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
		  &peer->srcadr, "%s  %u %u %u %u %u %u",
		  nmea,
		  up->tally.total, up->tally.accepted,
		  up->tally.rejected, up->tally.malformed,
		  up->tally.filtered, up->tally.pps_used);
	} else {
		record_clock_stats(&peer->srcadr, pp->a_lastcode);
	}
	ZERO(up->tally);
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
 * Wipe (that is, overwrite with '_') data fields and the checksum in
 * the last timecode.  The list of field indices is given as integers
 * in a varargs list, preferrably in ascending order, in any case
 * terminated by a negative field index.
 *
 * A maximum number of 8 fields can be overwritten at once to guard
 * against runaway (that is, unterminated) argument lists.
 *
 * This function affects what a remote user can see with
 *
 * ntpq -c clockvar <server>
 *
 * Note that this also removes the wiped fields from any clockstats
 * log.	 Some NTP operators monitor their NMEA GPS using the change in
 * location in clockstats over time as as a proxy for the quality of
 * GPS reception and thereby time reported.
 * -------------------------------------------------------------------
 */
static void
field_wipe(
	gpssymm_data * data,
	...
	)
{
	va_list	va;		/* vararg index list */
	int	fcnt;		/* safeguard against runaway arglist */
	int	fidx;		/* field to nuke, or -1 for checksum */
	char  * cp;		/* overwrite destination */
	
	fcnt = 8;
	cp = NULL;
	va_start(va, data);
	do {
		fidx = va_arg(va, int);
		if (fidx >= 0 && fidx <= NMEA_PROTO_FIELDS) {
			cp = field_parse(data, fidx);
		} else {
			cp = data->base + data->blen;
			if (data->blen >= 3 && cp[-3] == '*')
				cp -= 2;
		}
		for ( ; '\0' != *cp && '*' != *cp && ',' != *cp; cp++)
			if ('.' != *cp)
				*cp = '_';
	} while (fcnt-- && fidx >= 0);
	va_end(va);	
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
	struct calendar * jd,	/* result calendar pointer */
	long		* ns,	/* storage for nsec fraction */
	gpssymm_data       * rd,
	int		  idx
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

	jd->hour   = (u_char)h;
	jd->minute = (u_char)m;
	jd->second = (u_char)s;
	/* if we have a fraction, scale it up to nanoseconds. */
	if (rc == 4)
		*ns = f * weight[p2 - p1 - 1];
	else
		*ns = 0;

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
	struct calendar * jd,	/* result pointer */
	gpssymm_data       * rd,
	int		  idx,
	enum date_fmt	  fmt
	)
{
	int	rc;
	u_int	y;
	u_int	m;
	u_int	d;
	int	p;
	char  * dp;
	
	dp = field_parse(rd, idx);
	switch (fmt) {

	case DATE_1_DDMMYY:
		rc = sscanf(dp, "%2u%2u%2u%n", &d, &m, &y, &p);
		if (rc != 3 || p != 6) {
			DPRINTF(1, ("nmea: invalid date code: '%.6s'\n",
				    dp));
			return FALSE;
		}
		break;

	default:
		DPRINTF(1, ("nmea: invalid parse format: %d\n", fmt));
		return FALSE;
	}

	/* value sanity check */
	if (d < 1 || d > 31 || m < 1 || m > 12) {
		DPRINTF(1, ("nmea: invalid date spec (YMD) %04u:%02u:%02u\n",
			    y, m, d));
		return FALSE;
	}
	
	/* store results */
	jd->monthday = (u_char)d;
	jd->month    = (u_char)m;
	jd->year     = (u_short)y;

	return TRUE;
}

#else
NONEMPTY_TRANSLATION_UNIT
#endif /* REFCLOCK && CLOCK_GPSSYM */
