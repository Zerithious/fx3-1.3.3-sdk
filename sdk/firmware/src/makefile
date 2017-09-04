## Copyright Cypress Semiconductor Corporation, 2010-2014,
## All Rights Reserved
## UNPUBLISHED, LICENSED SOFTWARE.
##
## CONFIDENTIAL AND PROPRIETARY INFORMATION 
## WHICH IS THE PROPERTY OF CYPRESS.
##
## Use of this file is governed 
## by the license agreement included in the file 
##
##	<install>/license/license.txt
##
## where <install> is the Cypress software
## installation root directory path.
## 

CYU3ROOT=../..

all: compile

-include $(CYU3ROOT)/build/fwconfig/$(CYCONFIG).mak
include $(CYU3ROOT)/build/cyu3fw_common.mak

SUBDIRS=		\
	rtos		\
	dma		\
	lpp		\
	pport		\
	storage		\
	mipicsi		\
	usb		\
	system		\
	fxapp   

clean:
	@echo === $@ ... ===
	@date
	@echo ""
	for subdir in $(SUBDIRS); do \
		$(MAKE) CYTOOLCHAIN=$(CYTOOLCHAIN) -C $$subdir clean;\
	done
	@echo ""
	@date
	@echo === ... $@ ===

myclean:
	@echo === $@ ... ===
	@date
	@echo ""
	for subdir in $(SUBDIRS); do \
		$(MAKE) CYTOOLCHAIN=$(CYTOOLCHAIN) -C $$subdir myclean;\
	done
	@echo ""
	@date
	@echo === ... $@ ===

compile:
	@echo === $@ ... ===
	@date
	@echo ""
	for subdir in $(SUBDIRS); do \
		$(MAKE) CYTOOLCHAIN=$(CYTOOLCHAIN) -C $$subdir $@  ; \
	done
	@echo ""
	@date
	@echo === ... $@ ===

#[]#

