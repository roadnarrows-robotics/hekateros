###############################################################################
#
# Package:   RoadNarrows Hekateros Robotic Manipulator Package
#
# Link:      https://github.com/roadnarrows-robotics/hekateros
#
# ROS Node:  hek_*
#
# File:      PanelConfig.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Hekateros panel configuration dialog and XML classes.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2013-2016.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##
# @EulaBegin@
# @EulaEnd@
#
###############################################################################

import sys
import os
import time

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

import xml.parsers.expat as expat

from hekateros_control.Utils import *

# ------------------------------------------------------------------------------
# Class PanelConfigDlg
# ------------------------------------------------------------------------------

#
## \brief Hekateros Panel configuration dialog.
#
class PanelConfigDlg(Toplevel):
  ## \brief User's home directory.
  Home = os.path.expanduser("~")

  ## \brief User-specific configuration directory (in home directory).
  UserDirName = ".roadnarrows"

  ## \brief hek_panel application configuration file name.
  ConfigFileName = "hek_panel.xml"

  ## \brief Fully qualified path name.
  PathNameDft = Home + os.path.sep + \
                UserDirName + os.path.sep + \
                ConfigFileName

  ## \brief Configuration default.
  ConfigDft = \
  {
    'warn_on_calib':    True, # do [not] warn user at calibration start
    'warn_on_release':  True, # do [not] warn user on release
    'force_recalib':    True, # do [not] force recalibration on all joints
  }

  #
  ## \brief Constructor.
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):

    # initialize dialog data
    kw = self.initData(kw)

    Toplevel.__init__(self, master=master, cnf=cnf, **kw)

    self.title("hek_panel configuration")

    # parent widget's window geometry
    if master is not None:
      self.m_parentGeo = [master.winfo_width(), master.winfo_height(),
                          master.winfo_rootx(), master.winfo_rooty()]
    else:
      self.m_parentGeo = [400, 400, 400, 400]

    #print 'DBG: Parent geometry = {0}x{1}+{2}+{3}'.format(*self.m_parentGeo)

    # Set a good location for this dialog overlaying on top of the parent's
    # geometry. This is a compromise in that this dialog's geometry has not
    # been determined yet.
    glist= [self.m_parentGeo[2] + self.m_parentGeo[0]/4,
            self.m_parentGeo[3] + 30]
    self.geometry('+{0}+{1}'.format(*glist))

    # create and show widgets
    self.createWidgets()

    # allows the enter button to fire either button's action
    self.m_bttnCancel.bind('<KeyPress-Return>', func=self.close)

    # allows us to customize what happens when the close button is pressed
    self.protocol("WM_DELETE_WINDOW", self.close)

    #
    # Modal diagle settings.
    #
    # set the focus on dialog window (needed on Windows)
    self.focus_set()

    # make sure events only go to our dialog
    self.grab_set()

    # make sure dialog stays on top of its parent window (if needed)
    self.transient(master)

    # display the window and wait for it to close
    self.wait_window(self)

  #
  ## \brief Initialize class state data.
  ##
  ## \param kw      Keyword options.
  ##
  ## \return Modified keywords sans this specific class.
  ##
  def initData(self, kw):
    self.m_errors   = False
    self.m_errmsg   = ""
    self.m_saved    = False
    self.m_filename = None
    if kw.has_key('config'):
      self.m_config = kw['config'].copy()
      del kw['config']
    else:
      self.m_config = PanelConfig.Dlg.ConfigDft.copy()
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    frame = Frame(self)
    frame.grid(row=0, column=0)

    # top heading
    w = Label(frame)
    helv = tkFont.Font(family="Helvetica",size=24,weight="bold")
    w['font']   = helv
    w['text']   = 'Configuration'
    w['anchor'] = CENTER
    w.grid(row=0, column=0, sticky=E+W)

    # warn on calibrate check button
    row = 1
    self.m_varWarnCalib = IntVar()
    self.m_varWarnCalib.set(self.m_config['warn_on_calib'])
    w = Checkbutton(frame,
        text="Show warning dialog before calibrating arm.",
        variable=self.m_varWarnCalib)
    w.grid(row=row, column=0, padx=5, sticky=W)

    # force recalibration on all joints check button
    row += 1
    self.m_varForceRecalib = IntVar()
    self.m_varForceRecalib.set(self.m_config['force_recalib'])
    w = Checkbutton(frame,
        text="Force (re)calibration for all joints on calibrate action.",
        variable=self.m_varForceRecalib)
    w.grid(row=row, column=0, padx=20, sticky=W)

    row += 1
    spacer = Label(frame, text="  ");
    spacer.grid(row=row, column=0, padx=5, sticky=W)

    # warn on release check button
    row += 1
    self.m_varWarnRelease = IntVar()
    self.m_varWarnRelease.set(self.m_config['warn_on_release'])
    w = Checkbutton(frame,
        text="Show warning dialog before releasing arm.",
        variable=self.m_varWarnRelease)
    w.grid(row=row, column=0, padx=5, sticky=W)

    row += 1
    spacer = Label(frame, text="  ");
    spacer.grid(row=row, column=0, padx=5, sticky=W)

    #
    # buttons
    #
    row += 1
    wframe = Frame(frame)
    wframe.grid(row=row, column=0)

    # defaults button
    w = Button(wframe, width=10, text='Defaults', command=self.setToDefaults)
    w.grid(row=0, column=0, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnCancel = w

    # cancel button
    w = Button(wframe, width=10, text='Cancel', command=self.close)
    w.grid(row=0, column=1, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnCancel = w

    # save button
    w = Button(wframe, width=10, text='Save', command=self.ok)
    w.grid(row=0, column=2, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnOk = w

  #
  ## \brief Check and convert field data.
  ##
  ## \return True on success or False on error.
  #
  def check(self):
    self.m_errmsg = ""
    self.m_errors = True

    if self.m_varWarnCalib.get():
      self.m_config['warn_on_calib'] = True
    else:
      self.m_config['warn_on_calib'] = False
    if self.m_varForceRecalib.get():
      self.m_config['force_recalib'] = True
    else:
      self.m_config['force_recalib'] = False
    if self.m_varWarnRelease.get():
      self.m_config['warn_on_release'] = True
    else:
      self.m_config['warn_on_release'] = False

    self.m_errors = False
    return True

  #
  ## \brief Save configuration to XML file.
  ##
  ## \return True on success or False on error.
  #
  def save(self):
    dirname = PanelConfigDlg.Home + os.path.sep + PanelConfigDlg.UserDirName
    if not os.path.isdir(dirname):
      try:
        os.mkdir(dirname, 0755)
      except OSError, err:
        self.m_errmsg = "%s: %s" % (dirname, err)
        return False
    self.m_filename = dirname + os.path.sep + PanelConfigDlg.ConfigFileName
    xml = PanelConfigXml()
    xml.save(self.m_filename, self.m_config)
    return True

  #
  ## \brief Set to defaults callback.
  #
  def setToDefaults(self):
    self.m_varWarnCalib.set(PanelConfigDlg.ConfigDft['warn_on_calib'])
    self.m_varForceRecalib.set(PanelConfigDlg.ConfigDft['force_recalib'])
    self.m_varWarnRelease.set(PanelConfigDlg.ConfigDft['warn_on_release'])

  #
  ## \brief Save configuration callback.
  #
  def ok(self):
    if self.check() and self.save():
      self.m_saved = True
    self.close()

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()


# ------------------------------------------------------------------------------
# Class PanelConfigXml
# ------------------------------------------------------------------------------

##
## \brief Application hek_panel configuration xml class.
##
class PanelConfigXml():
  def __init__(self):
    self.m_curData      = ""
    self.m_config       = None

  #
  ## \brief Open XML configuration file and parse.
  ##
  ## \return Returns parsed configuration.
  #
  def parse(self, pathname=None):
    if pathname is None:
      pathname = PanelConfigDlg.PathNameDft;
    self.m_config = PanelConfigDlg.ConfigDft.copy()
    try:
      fp = open(pathname, 'r')
    except IOError, err:
      return self.m_config
    parser = expat.ParserCreate()
    parser.returns_unicode      = False
    parser.StartElementHandler  = self.onElemStart
    parser.CharacterDataHandler = self.onElemData
    parser.EndElementHandler    = self.onElemEnd
    self.m_stack = []
    try:
      parser.ParseFile(fp)
    except expat.ExpatError as e:
      print "%s: %s" % (pathname, expat.ErrorString(e.code))
    fp.close()
    return self.m_config

  #
  ## \brief Save to file.
  ##
  ## \return True on success or False on error.
  #
  def save(self, pathname, config):
    try:
      fp = open(pathname, 'w')
    except IOError, err:
      print "%s: %s." % (pathname, err)
      return
    fp.write("""\
<?xml version="1.0" encoding="utf-8"?>
<?xml-stylesheet type="text/xsl" href="http://roadnarrows.com/xml/Hekateros/1.0/hekateros.xsl"?>

<!-- RoadNarrows Hekateros Top-Level Configuration -->
<hekateros xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="http://roadnarrows.com/xml/Hekateros/1.0/hekateros.xsd">

  <!-- Hekateros panel configuration. -->
  <hek_panel>
""")

    self.writeTree(fp, 4, config);

    fp.write("""\
  </hek_panel>

</hekateros>
""")

    fp.close()

  #
  ## \brief Write out XmL tree.
  #
  def writeTree(self, fp, indent, config):
    s = ' ' * indent
    for key,data in config.iteritems():
      if type(data) is dict:
        fp.write("{0}<{1}>\n".format(s, key))
        self.writeTree(fp, indent+2, data)
        fp.write("{0}</{1}>\n".format(s, key))
      else:
        fp.write("{0}<{1}>{2}</{3}>\n".format(s, key, config[key], key))

  #
  ## \brief On element start callback.
  #
  def onElemStart(self, elem, attrs):
    #print "start-of-element", "<%s> attrs=%s" % (elem, repr(attrs))
    self.m_stack.append(elem)
    #print 'onElemStart', self.m_stack
    self.m_curData  = ""

  #
  ## \brief On element data read callback.
  #
  def onElemData(self, data):
    #print "char-data", repr(data)
    self.m_curData += data

  #
  ## \brief On element end callback.
  #
  def onElemEnd(self, elem):
    #print "end-of-element", "<\%s>" % (elem)
    # <hekateros> <hek_panel> x
    if len(self.m_stack) == 3:
      elem = self.m_stack[2]
      if self.m_config.has_key(elem):
        if elem in ['warn_on_calib', 'force_recalib', 'warn_on_release']:
          self.m_config[elem] = self.cvtToBool(self.m_curData.strip())
    try:
      self.m_stack.pop()
    except:
      pass
    #print 'onElemEnd', self.m_stack
    self.m_curData  = ""

  #
  ## \brief Convert element data to boolean.
  ##
  ## \return True or False.
  #
  def cvtToBool(self, data):
    if data.lower() == 'true':
      return True
    elif data.lower() == 'false':
      return False
    else:
      try:
        i = int(data)
        if i:
          return True
        else:
          return False
      except ValueError:
        return False

  #
  ## \brief Convert element data to fpn.
  ##
  ## \return Float.
  #
  def cvtToFloat(self, data):
    try:
      return float(data)
    except (TypeError, ValueError):
      return 0.0
