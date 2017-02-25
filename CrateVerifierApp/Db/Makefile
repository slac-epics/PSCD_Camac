#==============================================================
#
#  Abs:  Makefile to build EPICS Database components
#
#  Name: Makefile
#
#  Facility: LCLS CAMAC Controls
#
#  Auth: 29-Jun-2009, Kristi Luchini  (USERNAME)
#  Rev:  dd-mmm-yyyy, First Lastname  (USERNAME)
#--------------------------------------------------------------
#  Mod:
#        dd-mmm-yyyy, First Lastname  (USERNAME)
#          comment
#
#===========================================================
#
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
#
DB += cv_camac_module.template
DB += cv.db
DB += cv_spear.db

# Soft pvs
DB += camac_crat_profile.db
DB += camac_crat.template
DB += camac_module.template

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#========================================
#
# End of file

