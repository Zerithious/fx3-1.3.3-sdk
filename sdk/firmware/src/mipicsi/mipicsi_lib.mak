## Copyright Cypress Semiconductor Corporation, 2013,
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

MODULE=cyu3mipicsi

Include=-I.\
        -I$(CYU3ROOT)/firmware/include

SOURCE= \
	cyu3mipicsi.c \
	cyu3mipi_gpif.c

##[]

