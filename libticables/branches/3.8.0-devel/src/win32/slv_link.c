/* Hey EMACS -*- win32-c -*- */
/* $Id: slv_link.c 370 2004-03-22 18:47:32Z roms $ */

/*  libticables - Ti Link Cable library, a part of the TiLP project
 *  Copyright (C) 1999-2004  Romain Lievin
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/* TI-GRAPH LINK USB support */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "intl.h"
#include "export.h"
#include "cabl_def.h"
#include "cabl_err.h"
#include "verbose.h"
#include "logging.h"
#include "externs.h"
#include "timeout.h"

/* 
   Some important remarks... (http://lpg.ticalc.org/prj_usb/index.html)
   
   This link cable use Bulk mode with packets. The max size of a packet is 
   32 bytes (MAX_PACKET_SIZE/BULKUSB_MAX_TRANSFER_SIZE). 
   
   This is transparent for the user because the driver manages all these 
   things for us. Nethertheless, this fact has some consequences:
   - it is better (for USB & OS performances) to read/write a set of bytes 
   rather than byte per byte.
   - for reading, we have to read up to 32 bytes at a time (even if we need 
   only 1 byte) and to store them in a buffer for subsequent acesses. 
   In fact, if we try and get byte per byte, it will not work.
   - for writing, we don't store bytes in a buffer. It seems better to send
   data byte per byte (latency ?!).
   - another particular effect (quirk): sometimes (usually when calc need to 
   reply and takes a while), a read call can returns with no data or timeout. 
   Simply retry a read call and it works fine.
*/

/* 
   This part talk with the USB device driver through the TiglUsb library.
   There are 2 other files: TiglUsb.h (interface) & TiglUsb.lib (linkage).
*/

#include <stdio.h>
#include <windows.h>

#include "tiglusb.h"

#define MIN_VERSION "2.2"

static HINSTANCE hDLL = NULL;	// DLL handle on TiglUsb.dll
static dllOk = FALSE;		// Dll loading is OK

TIGLUSB_VERSION dynTiglUsbVersion = NULL;	// Functions pointers for 
TIGLUSB_OPEN dynTiglUsbOpen = NULL;	// dynamic loading
TIGLUSB_FLUSH dynTiglUsbFlush = NULL;
TIGLUSB_READ dynTiglUsbRead = NULL;
TIGLUSB_WRITE dynTiglUsbWrite = NULL;
TIGLUSB_CLOSE dynTiglUsbClose = NULL;
TIGLUSB_SETTIMEOUT dynTiglUsbSetTimeout = NULL;
TIGLUSB_CHECK dynTiglUsbCheck = NULL;

extern int time_out;		// Timeout value for cables in 0.10 seconds

int slv_init()
{
  int ret;

  // Create an handle on library and retrieve symbols
  hDLL = LoadLibrary("TIGLUSB.DLL");
  if (hDLL == NULL) {
    DISPLAY_ERROR
	(_
	 ("TiglUsb library not found. Have you installed the TiglUsb driver ?\n"));
    return ERR_OPEN_USB_DEV;
  }

  dynTiglUsbVersion =
      (TIGLUSB_VERSION) GetProcAddress(hDLL, "TiglUsbVersion");
  if (!dynTiglUsbVersion || (strcmp(dynTiglUsbVersion(), MIN_VERSION) < 0)) {
    char buffer[256];
    sprintf(buffer,
	    _
	    ("TiglUsb.dll: version %s mini needed, got version %s.\nPlease download the latest release on <http://ti-lpg.org/prj_usb>."),
	    MIN_VERSION, dynTiglUsbVersion());
    DISPLAY_ERROR(buffer);
    MessageBox(NULL, buffer, "Error in SilverLink support", MB_OK);
    FreeLibrary(hDLL);
    return ERR_TIGLUSB_VERSION;
  }

  dynTiglUsbOpen = (TIGLUSB_OPEN) GetProcAddress(hDLL, "TiglUsbOpen");
  if (!dynTiglUsbOpen) {
    DISPLAY_ERROR(_("Unable to load TiglUsbOpen symbol.\n"));
    FreeLibrary(hDLL);
    return ERR_FREELIBRARY;
  }

  dynTiglUsbFlush = (TIGLUSB_FLUSH) GetProcAddress(hDLL, "TiglUsbFlush");
  if (!dynTiglUsbOpen) {
    DISPLAY_ERROR(_("Unable to load TiglUsbFlush symbol.\n"));
    FreeLibrary(hDLL);
    return ERR_FREELIBRARY;
  }

  dynTiglUsbRead = (TIGLUSB_READ) GetProcAddress(hDLL, "TiglUsbRead");
  if (!dynTiglUsbRead) {
    DISPLAY_ERROR(_("Unable to load TiglUsbRead symbol.\n"));
    FreeLibrary(hDLL);
    return ERR_FREELIBRARY;
  }

  dynTiglUsbWrite = (TIGLUSB_WRITE) GetProcAddress(hDLL, "TiglUsbWrite");
  if (!dynTiglUsbWrite) {
    DISPLAY_ERROR(_("Unable to load TiglUsbWrite symbol.\n"));
    FreeLibrary(hDLL);
    return ERR_FREELIBRARY;
  }

  dynTiglUsbClose = (TIGLUSB_CLOSE) GetProcAddress(hDLL, "TiglUsbClose");
  if (!dynTiglUsbClose) {
    DISPLAY_ERROR(_("Unable to load TiglUsbClose symbol.\n"));
    FreeLibrary(hDLL);
    return ERR_FREELIBRARY;
  }

  dynTiglUsbSetTimeout = (TIGLUSB_SETTIMEOUT) GetProcAddress(hDLL,
							     "TiglUsbSetTimeout");
  if (!dynTiglUsbSetTimeout) {
    DISPLAY_ERROR(_("Unable to load TiglUsbSetTimeout symbol.\n"));
    FreeLibrary(hDLL);
    return ERR_FREELIBRARY;
  }

  dynTiglUsbCheck = (TIGLUSB_CHECK) GetProcAddress(hDLL, "TiglUsbCheck");
  if (!dynTiglUsbCheck) {
    DISPLAY_ERROR(_("Unable to load TiglUsbCheck symbol.\n"));
    FreeLibrary(hDLL);
    return ERR_FREELIBRARY;
  }

  dllOk = TRUE;

  ret = dynTiglUsbOpen();
  switch (ret) {
  case TIGLERR_DEV_OPEN_FAILED:
    return ERR_OPEN_USB_DEV;
  case TIGLERR_DEV_ALREADY_OPEN:
    return ERR_OPEN_USB_DEV;
  default:
    break;
  }

  dynTiglUsbSetTimeout(time_out);

  START_LOGGING();

  return 0;
}

int slv_open()
{
  int ret;

  if (!hDLL)
    ERR_TIGLUSB_VERSION;

  dynTiglUsbSetTimeout(time_out);
  ret = dynTiglUsbFlush();
  switch (ret) {
  case TIGLERR_FLUSH_FAILED:
    return ERR_IOCTL;
  default:
    break;
  }

  tdr.count = 0;
  toSTART(tdr.start);

  return 0;
}

int slv_put(uint8_t data)
{
  int ret;

  tdr.count++;
  LOG_DATA(data);

  ret = dynTiglUsbWrite(data);
  switch (ret) {
  case TIGLERR_WRITE_TIMEOUT:
    return ERR_WRITE_TIMEOUT;
  default:
    break;
  }

  return 0;
}

int slv_get(uint8_t * data)
{
  int ret;

  ret = dynTiglUsbRead(data);
  switch (ret) {
  case TIGLERR_READ_TIMEOUT:
    return ERR_READ_TIMEOUT;
  default:
    break;
  }

  tdr.count++;
  LOG_DATA(*data);

  return 0;
}

int slv_close()
{
  return 0;
}

int slv_exit()
{
  int ret;

  STOP_LOGGING();

  ret = dynTiglUsbClose();

  /* Free library handle */
  if (hDLL != NULL)
    FreeLibrary(hDLL);
  hDLL = NULL;

  dllOk = FALSE;

  return 0;
}

int slv_probe()
{
  /*
     HANDLE hDev = dynTiglUsbOpen();

     if(hDev == INVALID_HANDLE_VALUE)
     {
     return ERR_PROBE_FAILED;
     }
     else
     {
     CloseHandle(hDev);
     return 0;
     }
   */

  return 0;
}

int slv_check(int *status)
{
  return dynTiglUsbCheck(status);
}

#define swap_bits(a) (((a&2)>>1) | ((a&1)<<1))	// swap the 2 lowest bits

int slv_set_red_wire(int b)
{
  return 0;
}

int slv_set_white_wire(int b)
{
  return 0;
}

int slv_get_red_wire()
{
  return 0;
}

int slv_get_white_wire()
{
  return 0;
}

int slv_supported()
{
  return SUPPORT_ON;
}

int slv_register_cable_1(TicableLinkCable * lc)
{
  lc->init = slv_init;
  lc->open = slv_open;
  lc->put = slv_put;
  lc->get = slv_get;
  lc->close = slv_close;
  lc->exit = slv_exit;
  lc->probe = slv_probe;
  lc->check = slv_check;

  lc->set_red_wire = NULL;
  lc->set_white_wire = NULL;
  lc->get_red_wire = NULL;
  lc->get_white_wire = NULL;

  return 0;
}