TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
DB += PIOP_ipl.template 
DB += PIOP_ftp.template
DB += PIOP_msg.template
DB += PIOP_updt.template
DB += PIOP_volts.template
DB += PIOP_status_block.template
DB += SBI.template 
#DB_INSTALLS += PIOPdemo.db
DB_INSTALLS += ../mksu.img

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

