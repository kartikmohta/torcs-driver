##############################################################################
#
#    file                 : Makefile
#    created              : Mon Apr 6 22:59:11 IST 2009
#    copyright            : (C) 2002 Kartik Mohta
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = kartik
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp opponent.cpp learning.cpp trajectory.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml logo.rgb
SHIPSUBDIRS = 0 1

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-kartik_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-kartik_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
