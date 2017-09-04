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
include cyu3system_exe.mak
include $(CYU3ROOT)/build/cyu3fw_common.mak

LIBS = $(CYCONFIG)/$(MODULE).$(LIBEXT)
EXES =

# Take the DMA, PPORT, USB and FX3 device libraries and merge them into a single
# API library which provides all of the required functionality.

LDLIBS= \
	../dma/$(CYCONFIG)/cyu3dma.a \
	../pport/$(CYCONFIG)/cyu3pport.a \
	../usb/$(CYCONFIG)/cyu3uport.a \
	../fx3lib/$(CYCONFIG)/cyfx3lib.a


$(CYCONFIG)/$(MODULE).$(LIBEXT): $(OBJECT) $(LDLIBS)
	for lib in $(LDLIBS); do \
	    cd $(CYCONFIG) && $(CY.AR) -x ../$$lib && cd -; \
	done
	$(AT) rm -rf $@  
	$(AT) $(CY.AR) -cr $@ $(CYCONFIG)/*.o
	$(AT) mkdir -p $(CYU3POUTDIR)
	cp $(CYCONFIG)/$(MODULE).$(LIBEXT) $(CYU3POUTDIR)

myclean: clean
clean: cleanconfig
compile:$(LIBS) $(EXES)
everything: compile

##[]

