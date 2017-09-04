## Copyright Cypress Semiconductor Corporation, 2008-2009,
## All Rights Reserved
## UNPUBLISHED, LICENSED SOFTWARE.
##
## CONFIDENTIAL AND PROPRIETARY INFORMATION 
## WHICH IS THE PROPERTY OF CYPRESS.
##
## Use of this file is governed 
## by the license agreement included in the file 
##
##      <install>/license/license.txt
##
## where <install> is the Cypress software
## installation root directory path.
## 

CYU3ROOT=../../..

all:compile

-include $(CYU3ROOT)/build/fwconfig/$(CYCONFIG).mak
include cyfxapp_exe.mak
include $(CYU3ROOT)/build/cyu3fw_common.mak

LIBS =
EXES = $(CYCONFIG)/$(MODULE).$(EXEEXT)

LDLIBS= \
	$(CYU3POUTDIR)/cyu3lpp.a \
	$(CYU3POUTDIR)/cyfxapi.a \
	$(CYU3POUTDIR)/cyu3threadx.a

$(CYCONFIG)/$(MODULE).$(EXEEXT):$(OBJECT) $(LDLIBS)
	$(LinkProgram)

myclean: clean
clean: cleanconfig
compile:$(LIBS) $(EXES)
everything: compile

##[]

