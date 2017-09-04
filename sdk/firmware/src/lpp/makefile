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
include lpp_lib.mak
include $(CYU3ROOT)/build/cyu3fw_common.mak

LIBS = $(CYCONFIG)/$(MODULE).$(LIBEXT)
EXES =

$(CYCONFIG)/$(MODULE).$(LIBEXT): $(OBJECT)
	$(AT) rm -rf $(CYCONFIG)/$@  
	$(AT) $(BuildLibrary)
	$(AT) mkdir -p $(CYU3POUTDIR)
	cp $(CYCONFIG)/$(MODULE).$(LIBEXT) $(CYU3POUTDIR)

clean: cleanconfig
myclean: cleanconfig
compile:$(LIBS) $(EXES)
everything: compile unittest

##[]

