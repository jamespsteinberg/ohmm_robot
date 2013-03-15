/**
 * Open Hardware Mobile Manipulator (OHMM) AVR Monitor
 * 
 * Default monitor program entry point.
 *
 * When compiling the ohmm library, this translation unit is not linked so
 * that custom monitor programs can define their own main() function without
 * name collision.
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *  
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (c) 2011 Marsette A. Vona
 **/

/* <file.h> searches in system directories only */
#include <avr/pgmspace.h>

/* "file.h" searches in local directories, then system directories */
#include "pololu/orangutan.h"
#include "ohmm/ohmm.h"
#include "drive.h"

/**
 * Subversion ID for this file.
 *
 * See svnid in ohmm.c for more info.
 **/
static char* svnid PROGMEM = "$Id: main.c 4 2013-01-17 21:53:38Z vona $";

/**
 * Default monitor program entry point.
 **/
int main() {

  /* initialize monitor */
  ohmmInit();

  /* do custom initialization here: register custom tasks, command handlers,
     configure modules, init custom modules, etc. */
  driveRegisterCmds();

    //Initialize drive module
    initState();
        

  /* run the foreground task manager (never returns) */
  taskLoop();

  /* never get here, but return is required to compile */

  return 0; /* just enters infinite loop on AVR */
}

