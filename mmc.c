/*-----------------------------------------------------------------------*/
/* MMC/SDC (in SPI mode) control module  (C)ChaN, 2006                   */
/*-----------------------------------------------------------------------*/
/* Only rcvr_spi(), xmit_spi(), disk_timerproc(), disk_initialize () and */
/* some macros are platform dependent.                                   */
/*-----------------------------------------------------------------------*/


#include <avr/io.h>
#include "diskio.h"


/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD9	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD41	(0x40+41)	/* SEND_OP_COND (ACMD) */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */


/* Control signals (Platform dependent) */
#define SELECT()	PORTB &= ~_BV(2)	/* MMC CS = L */
#define	DESELECT()	PORTB |= _BV(2)		/* MMC CS = H */



extern volatile
BYTE MmcTmr[2];		/* 100Hz decrement timers */


/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

static volatile
DSTATUS Stat;	/* Disk status */

static
BYTE CardType;			/* b0:MMC, b1:SDC, b2:Block addressing */



/*-----------------------------------------------------------------------*/
/* Transmit a byte to MMC via SPI  (Platform dependent)                  */
/*-----------------------------------------------------------------------*/

#define xmit_spi(dat) 	SPDR=(dat); loop_until_bit_is_set(SPSR,SPIF)



/*-----------------------------------------------------------------------*/
/* Receive a byte from MMC via SPI  (Platform dependent)                 */
/*-----------------------------------------------------------------------*/

static
BYTE rcvr_spi (void)
{
	SPDR = 0xFF;
	loop_until_bit_is_set(SPSR, SPIF);
	return SPDR;
}

/* Alternative macro to receive data fast */
#define rcvr_spi_m(dst)	SPDR=0xFF; loop_until_bit_is_set(SPSR,SPIF); *(dst)=SPDR



/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
BYTE wait_ready (void)
{
	BYTE res;


	MmcTmr[1] = 50;	/* Wait for ready in timeout of 500ms */
	rcvr_spi();
	do
		res = rcvr_spi();
	while ((res != 0xFF) && MmcTmr[1]);

	return res;
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static
BOOL rcvr_datablock (
	BYTE *buff,			/* Data buffer to store received data */
	UINT btr			/* Byte count (must be even number) */
)
{
	BYTE token;


	MmcTmr[0] = 10;
	do {							/* Wait for data packet in timeout of 100ms */
		token = rcvr_spi();
	} while ((token == 0xFF) && MmcTmr[0]);
	if(token != 0xFE) return FALSE;	/* If not valid data token, retutn with error */

	do {							/* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	} while (btr -= 2);
	rcvr_spi();						/* Discard CRC */
	rcvr_spi();

	return TRUE;					/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
static
BOOL xmit_datablock (
	const BYTE *buff,	/* 512 byte data block to be transmitted */
	BYTE token			/* Data/Stop token */
)
{
	BYTE resp, wc;


	if (wait_ready() != 0xFF) return FALSE;

	xmit_spi(token);					/* Xmit data token */
	if (token != 0xFD) {	/* Is data token */
		wc = 0;
		do {							/* Xmit the 512 byte data block to MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		} while (--wc);
		xmit_spi(0xFF);					/* CRC (Dummy) */
		xmit_spi(0xFF);
		resp = rcvr_spi();				/* Reveive data response */
		if ((resp & 0x1F) != 0x05)		/* If not accepted, return with error */
			return FALSE;
	}

	return TRUE;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (
	BYTE cmd,		/* Command byte */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;


	if (wait_ready() != 0xFF) return 0xFF;

	/* Send command packet */
	xmit_spi(cmd);						/* Command */
	xmit_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xmit_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xmit_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xmit_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0;
	if (cmd == CMD0) n = 0x95;			/* CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command response */
	if (cmd == CMD12) rcvr_spi();		/* Skip a stuff byte when stop reading */
	n = 10;								/* Wait for a valid response in timeout of 10 attempts */
	do
		res = rcvr_spi();
	while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}




/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive nmuber (0) */
)
{
	BYTE n, ty, ocr[4];


	if (drv) return STA_NOINIT;			/* Supports only single drive */

	for (n = 10; n; n--) rcvr_spi();	/* 80 dummy clocks */

	SELECT();				/* CS = L */
	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		MmcTmr[0] = 100;						/* Initialization timeout of 1000 msec */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDC Ver2+ */
			for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {	/* The card can work at vdd range of 2.7-3.6V */
				do {
					if (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 1UL << 30) == 0)	break;	/* ACMD41 with HCS bit */
				} while (MmcTmr[0]);
				if (MmcTmr[0] && send_cmd(CMD58, 0) == 0) {	/* Check CCS bit */
					for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
					ty = (ocr[0] & 0x40) ? 6 : 2;
				}
			}
		} else {							/* SDC Ver1 or MMC */
			ty = (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) <= 1) ? 2 : 1;	/* SDC : MMC */
			do {
				if (ty == 2) {
					if (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) == 0) break;	/* ACMD41 */
				} else {
					if (send_cmd(CMD1, 0) == 0) break;								/* CMD1 */
				}
			} while (MmcTmr[0]);
			if (!MmcTmr[0] || send_cmd(CMD16, 512) != 0)	/* Select R/W block length */
				ty = 0;
		}
	}
	CardType = ty;
	DESELECT();			/* CS = H */
	rcvr_spi();			/* Idle (Release DO) */

	if (ty) {			/* Initialization succeded */
		Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */
	} else {			/* Initialization failed */
		Stat |=  STA_NOINIT;		/* Set STA_NOINIT */
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only single drive */
	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,			/* Physical drive nmuber (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count			/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	if (!(CardType & 4)) sector *= 512;	/* Convert to byte address if needed */

	SELECT();			/* CS = L */

	if (count == 1) {	/* Single block read */
		if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock(buff, 512))
			count = 0;
	}
	else {				/* Multiple block read */
		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}

	DESELECT();			/* CS = H */
	rcvr_spi();			/* Idle (Release DO) */

	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count			/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

	if (!(CardType & 4)) sector *= 512;	/* Convert to byte address if needed */

	SELECT();			/* CS = L */

	if (count == 1) {	/* Single block write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & 2) {
			send_cmd(CMD55, 0); send_cmd(CMD23, count);	/* ACMD23 */
		}
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}

	DESELECT();			/* CS = H */
	rcvr_spi();			/* Idle (Release DO) */

	return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive data block */
)
{
	DRESULT res;
	BYTE n, csd[16], *ptr = buff;
	WORD csize;


	if (drv) return RES_PARERR;

	SELECT();		/* CS = L */

	res = RES_ERROR;
	switch (ctrl) {
		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
				if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
					csize = csd[9] + ((WORD)csd[8] << 8) + 1;
					*(DWORD*)buff = (DWORD)csize << 10;
				} else {					/* MMC or SDC ver 1.XX */
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
					*(DWORD*)buff = (DWORD)csize << (n - 9);
				}
				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE :	/* Get sectors on the disk (WORD) */
			*(WORD*)buff = 512;
			res = RES_OK;
			break;

		case CTRL_SYNC :	/* Make sure that data has been written */
			if (wait_ready() == 0xFF)
				res = RES_OK;
			break;

		case MMC_GET_CSD :	/* Receive CSD as a data block (16 bytes) */
			if (Stat & STA_NOINIT) return RES_NOTRDY;
			if ((send_cmd(CMD9, 0) == 0)	/* READ_CSD */
				&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_CID :	/* Receive CID as a data block (16 bytes) */
			if (Stat & STA_NOINIT) return RES_NOTRDY;
			if ((send_cmd(CMD10, 0) == 0)	/* READ_CID */
				&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_OCR :	/* Receive OCR as an R3 resp (4 bytes) */
			if (Stat & STA_NOINIT) return RES_NOTRDY;
			if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
				for (n = 0; n < 4; n++)
					*ptr++ = rcvr_spi();
				res = RES_OK;
			}
			break;

		default:
			res = RES_PARERR;
	}

	DESELECT();			/* CS = H */
	rcvr_spi();			/* Idle (Release DO) */

	return res;
}



/*---------------------------------------*/
/* Device timer interrupt procedure      */
/* This must be called in period of 10ms */
/* (Platform dependent)                  */

void disk_timerproc (void)
{
	BYTE n;


	n = MmcTmr[0];						/* 100Hz decrement timer */
	if (n) MmcTmr[0] = --n;
	n = MmcTmr[1];
	if (n) MmcTmr[1] = --n;

}

