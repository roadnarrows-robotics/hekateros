###############################################################################
#
# Package:   RoadNarrows Hekateros Robotic Manipulator Package
#
# Link:      https://github.com/roadnarrows-robotics/hekateros
#
# ROS Node:  hek_*
#
# File:      EECam.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Hekateros End Effector camera configuration, run, and stop dialogs
## and XML classes.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2015.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##
# @EulaBegin@
# @EulaEnd@
#
###############################################################################

import sys
import os
import socket
import time

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

import xml.parsers.expat as expat

from hekateros_control.Utils import *
from hekateros_control.Xterm import Xterm

# ------------------------------------------------------------------------------
# Class EECamConfigDlg
# ------------------------------------------------------------------------------

#
## \brief Hekateros EECam teleoperation configuration dialog.
#
class EECamConfigDlg(Toplevel):
  ## \brief User's home directory.
  Home = os.path.expanduser("~")

  ## \brief User-specific configuration directory (in home directory).
  UserDirName = ".roadnarrows"

  ## \brief hek_eecam application configuration file name.
  ConfigFileName = "hek_eecam.xml"

  ## \brief Fully qualified path name.
  PathNameDft = Home + os.path.sep + \
                UserDirName + os.path.sep + \
                ConfigFileName

  ## \brief Configuration default.
  ConfigDft = \
  {
    'host':       socket.gethostname(), # receiving host 
    'port':       4951,                 # UDP port number
    'resolution': 'vga',                # resolution
    'fps':        15,                   # frames per second
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

    self.title("End Effector Camera Configuration")

    # create and show widgets
    self.createWidgets()

    # allows the enter button to fire either button's action
    self.m_bttnCancel.bind('<KeyPress-Return>', func=self.close)

    # center the dialog over parent window
    if master is not None:
      self.update_idletasks()
      x0 = master.winfo_rootx()
      y0 = master.winfo_rooty()
      xp = x0 + (master.winfo_width() - self.winfo_width()) / 2 - 8
      yp = y0 + (master.winfo_height() - self.winfo_height()) / 2 - 20
      glist = [self.winfo_width(), self.winfo_height(), xp, yp]
      #self.withdraw() # hide the dialog until position and size is set
      self.geometry('{0}x{1}+{2}+{3}'.format(*glist))
      #self.deiconify() # now show

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
      self.m_config = EECamConfigDlg.ConfigDft.copy()
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    frame = Frame(self)
    frame.grid(row=0, column=0)

    row = 0

    # top heading
    w = Label(frame)
    helv = tkFont.Font(family="Helvetica",size=14,weight="bold")
    w['font']   = helv
    w['text']   = 'End Effector Camera Configuration'
    w['anchor'] = CENTER
    w.grid(row=row, column=0, columnspan=2, sticky=E+W)

    row += 1

    # receiving host field
    w = Label(frame)
    w['text']   = 'Host:'
    w['anchor'] = 'e'
    w.grid(row=row, column=0, sticky=E)

    self.m_varHost = StringVar()
    self.m_varHost.set(self.m_config['host'])
    w = Entry(frame)
    w['justify']      = LEFT
    w['width']        = 32
    w['textvariable'] = self.m_varHost
    w.grid(row=row, column=1, padx=5, sticky=W)

    row += 1

    # UDP port field
    w = Label(frame)
    w['text']   = 'UDP Port:'
    w['anchor'] = 'e'
    w.grid(row=row, column=0, sticky=E)

    self.m_varPort = IntVar()
    self.m_varPort.set(self.m_config['port'])
    w = Entry(frame)
    w['justify']      = LEFT
    w['width']        = 8
    w['textvariable'] = self.m_varPort
    w.grid(row=row, column=1, padx=5, sticky=W)

    row += 1

    wframe = Frame(frame)
    wframe.grid(row=row, column=0, columnspan=2)

    # resolution field
    lframe = LabelFrame(wframe, text="Resolution")
    lframe.grid(row=row, column=0, padx=5, pady=5, sticky=N)

    radiores = [
        ("QVGA (320x240)",    "qvga"),
        ("VGA (640x480)",     "vga"),
        ("720p (1280x720)",   "720p"),
        ("Other (WxH)",       "other")
    ]

    self.m_varRes = StringVar()
    curres  = self.m_config['resolution'].lower()
    bInit   = False
    frow    = 0
    for text, res in radiores:
      w = Radiobutton(lframe, text=text, variable=self.m_varRes, value=res)
      w.grid(row=frow, column=0, sticky=W)
      if res == curres:
        self.m_varRes.set(res) # initialize
        bInit = True
      frow += 1

    self.m_varResOther = StringVar()
    w = Entry(lframe)
    w['justify']      = LEFT
    w['width']        = 10
    w['textvariable'] = self.m_varResOther
    w.grid(row=frow-1, column=1, padx=5, sticky=W)

    # initialize other
    if not bInit:
      self.m_varRes.set("other")
      self.m_varResOther.set(curres)

    # fps field
    lframe = LabelFrame(wframe, text="Frames/Second")
    lframe.grid(row=row, column=1, padx=5, pady=5, sticky=N)

    radiofps = [
        ("8",     "8"),
        ("15",    "15"),
        ("30",    "30"),
        ("Other", "other")
    ]

    self.m_varFps = StringVar()
    curfps  = repr(self.m_config['fps']).lower()
    bInit   = False
    frow    = 0
    for text, fps in radiofps:
      w = Radiobutton(lframe, text=text, variable=self.m_varFps, value=fps)
      w.grid(row=frow, column=0, sticky=W)
      if fps == curfps:
        self.m_varFps.set(fps) # initialize
        bInit = True
      frow += 1

    self.m_varFpsOther = StringVar()
    w = Entry(lframe)
    w['justify']      = LEFT
    w['width']        = 4
    w['textvariable'] = self.m_varFpsOther
    w.grid(row=frow-1, column=1, padx=5, sticky=W)

    # initialize other
    if not bInit:
      self.m_varFps.set("other")
      self.m_varFpsOther.set(curfps)

    row += 1

    # spacer
    spacer = Label(frame, text="  ");
    spacer.grid(row=row, column=0, columnspan=2, padx=5, sticky=W)

    #
    # buttons
    #
    row += 1

    wframe = Frame(frame)
    wframe.grid(row=row, column=0, columnspan=2)

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

    val = self.m_varHost.get().strip()
    if not val:
      self.m_errmsg = "Host: No name or address specified"
      return False
    self.m_config['host'] = val

    val = self.m_varPort.get()
    if val <= 0 or val > 65535:
      self.m_errmsg = "UDP Port: %s: Out of range" % (val)
      return False
    self.m_config['port'] = val

    val = self.m_varRes.get()
    if val == "other":
      val = self.m_varResOther.get().strip()
      if not val:
        self.m_errmsg = "Resolution: No video resolution specified"
        return False
      wxh = val.split('x')
      if len(wxh) != 2:
        self.m_errmsg = "Resolution: %s: Bad format" % (val)
        return False
      try:
        w = int(wxh[0])
        h = int(wxh[1])
      except ValueError:
        self.m_errmsg = "Resolution: %s: Bad format" % (val)
        return False
    self.m_config['resolution'] = val

    val = self.m_varFps.get()
    if val == "other":
      val = self.m_varFpsOther.get().strip()
      if not val:
        self.m_errmsg = "FPS: No video resolution specified"
        return False
      try:
        fps = int(val)
      except ValueError:
        self.m_errmsg = "FPS: %s: Bad format" % (val)
        return False
    else:
      fps = int(val)
    if fps <= 0:
      self.m_errmsg = "FPS: %s: Out of range" % (val)
      return False
    self.m_config['fps'] = fps

    self.m_errors = False
    return True

  #
  ## \brief Save configuration to XML file.
  ##
  ## \return True on success or False on error.
  #
  def save(self):
    dirname = EECamConfigDlg.Home + os.path.sep + EECamConfigDlg.UserDirName
    if not os.path.isdir(dirname):
      try:
        os.mkdir(dirname, 0755)
      except OSError, err:
        self.m_errors = True
        self.m_errmsg = "%s: %s" % (dirname, err)
        return False
    self.m_filename = dirname + os.path.sep + EECamConfigDlg.ConfigFileName
    xml = EECamConfigXml()
    xml.save(self.m_filename, self.m_config)
    return True

  #
  ## \brief Set to defaults callback.
  #
  def setToDefaults(self):
    self.m_varHost.set(EECamConfigDlg.ConfigDft['host'])
    self.m_varPort.set(EECamConfigDlg.ConfigDft['port'])
    self.m_varRes.set(EECamConfigDlg.ConfigDft['resolution'])
    self.m_varResOther.set("")
    self.m_varFps.set(EECamConfigDlg.ConfigDft['fps'])
    self.m_varFpsOther.set("")

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
# Class EECamConfigXml
# ------------------------------------------------------------------------------

##
## \brief Hekateros end effector camera configuration xml class.
##
class EECamConfigXml():
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
      pathname = EECamConfigDlg.PathNameDft;
    self.m_config = EECamConfigDlg.ConfigDft.copy()
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

  <!-- Hekateros end effector camera configuration -->
  <hek_eecam>
""")

    self.writeTree(fp, 4, config);

    fp.write("""\
  </hek_eecam>

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
    # <hekateros> <hek_eecam> x
    if len(self.m_stack) == 3:
      elem = self.m_stack[2]
      if self.m_config.has_key(elem):
        if elem in ['host', 'resolution']:
          self.m_config[elem] = self.m_curData.strip()
        elif elem in ['port', 'fps']:
          self.m_config[elem] = self.cvtToInt(self.m_curData.strip())
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
  ## \brief Convert element data to integer.
  ##
  ## \return Integer.
  #
  def cvtToInt(self, data):
    try:
      return int(data)
    except (TypeError, ValueError):
      return 0

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


# ------------------------------------------------------------------------------
# Class EECamStartDlg
# ------------------------------------------------------------------------------

class EECamStartDlg(Toplevel):
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
    self.title(self.m_title)

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
    self.m_icons          = {}    # must keep loaded icons referenced
    imageLoader = ImageLoader(py_pkg='hekateros_control.images')
    if kw.has_key('title'):
      self.m_title = kw['title']
      del kw['title']
    else:
      self.m_title = "Start End Effector Camera"
    if kw.has_key('image'):
      self.m_icons['image'] = imageLoader.load(kw['image'])
      del kw['image']
    else:
      self.m_icons['image'] = None
    #if self.m_icons['image'] is None:
    #  self.m_icons['image'] = imageLoader.load('icons/icon_warning.png')
    if kw.has_key('src'):
      self.m_srcHek = kw['src']
      del kw['src']
    else:
      self.m_srcHek = 'hekateros'
    if kw.has_key('config'):
      self.m_config = kw['config'].copy()
      del kw['config']
    else:
      self.m_config = EECamConfigDlg.ConfigDft.copy()
    if kw.has_key('cmd'):
      self.m_cmd = kw['cmd']
      del kw['cmd']
    else:
      self.m_cmd = "\
trap '' SIGHUP; \
hek_eecam_start --src={0} --port={1} --res={2} --fps={3}".format(
        self.m_srcHek, self.m_config['port'], self.m_config['resolution'],
        self.m_config['fps'])
    self.m_result = False
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    frame = Frame(self)
    frame.grid(row=0, column=0)

    row = 0

    # camera image 
    w = Label(frame)
    if self.m_icons['image'] is not None:
      w = Label(frame)
      w['image']  = self.m_icons['image']
    w['anchor'] = CENTER
    w.grid(row=row, column=0, sticky=W+N+S)

    # top heading
    w = Label(frame)
    helv = tkFont.Font(family="Helvetica",size=24,weight="bold")
    w['font']   = helv
    w['text']   = 'Start End Effector Camera'
    w['anchor'] = CENTER
    w.grid(row=row, column=1, sticky=E+W)

    row += 1

    w = Label(frame)
    w['text'] = "Remote UDP Source: {0}:{1}   Camera: {2} resolution at {3} frames/second".format(
        self.m_srcHek, self.m_config['port'], self.m_config['resolution'],
        self.m_config['fps'])
    w['justify'] = LEFT
    w.grid(row=row, column=0, columnspan=2, padx=10, pady=5, sticky=W)

    row += 1

    # xterm frame
    self.m_wXterm = Xterm(master=frame, on_exit=self.destroy,
                            xterm_cmd=self.m_cmd)
    self.m_wXterm.grid(row=row, column=0, columnspan=2, padx=5, pady=5)

    row += 1

    wframe = Frame(frame)
    wframe.grid(row=row, column=0, columnspan=2)

    # cancel button
    w = Button(wframe, width=10, text='Cancel', command=self.close)
    w.grid(row=0, column=0, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnCancel = w

    # start button
    w = Button(wframe, width=10, text='Start', command=self.ok)
    w.grid(row=0, column=1, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnContinue = w

  #
  ## \brief Destroy window callback.
  #
  def ok(self):
    self.m_wXterm.execute()
    self.m_result = True

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()


# ------------------------------------------------------------------------------
# Class EECamStopDlg
# ------------------------------------------------------------------------------

class EECamStopDlg(Toplevel):
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
    self.title(self.m_title)

    # create and show widgets
    self.createWidgets()

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
    self.m_icons          = {}    # must keep loaded icons referenced
    imageLoader = ImageLoader(py_pkg='hekateros_control.images')
    if kw.has_key('title'):
      self.m_title = kw['title']
      del kw['title']
    else:
      self.m_title = "Stop End Effector Camera"
    if kw.has_key('image'):
      self.m_icons['image'] = imageLoader.load(kw['image'])
      del kw['image']
    else:
      self.m_icons['image'] = None
    #if self.m_icons['image'] is None:
    #  self.m_icons['image'] = imageLoader.load('icons/icon_warning.png')
    if kw.has_key('src'):
      self.m_srcHek = kw['src']
      del kw['src']
    else:
      self.m_srcHek = 'hekateros'
    if kw.has_key('cmd'):
      self.m_cmd = kw['cmd']
      del kw['cmd']
    else:
      self.m_cmd = "\
trap '' SIGHUP; \
hek_eecam_stop --src={0}".format(self.m_srcHek)
    self.m_result = False
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    frame = Frame(self)
    frame.grid(row=0, column=0)

    row = 0

    # camera image 
    w = Label(frame)
    if self.m_icons['image'] is not None:
      w = Label(frame)
      w['image']  = self.m_icons['image']
    w['anchor'] = CENTER
    w.grid(row=row, column=0, sticky=W+N+S)

    # top heading
    w = Label(frame)
    helv = tkFont.Font(family="Helvetica",size=24,weight="bold")
    w['font']   = helv
    w['text']   = 'Stop End Effector Camera'
    w['anchor'] = CENTER
    w.grid(row=row, column=1, sticky=E+W)

    row += 1

    w = Label(frame)
    w['text'] = "Hekateros: {0}".format(self.m_srcHek)
    w['justify'] = LEFT
    w.grid(row=row, column=0, columnspan=2, padx=10, pady=5, sticky=W)

    row += 1

    # xterm frame
    self.m_wXterm = Xterm(master=frame, auto_run=True, on_exit=self.destroy,
                            xterm_cmd=self.m_cmd)
    self.m_wXterm.grid(row=row, column=0, columnspan=2, padx=5, pady=5)

  #
  ## \brief Destroy window callback.
  #
  def ok(self):
    self.m_wXterm.execute()
    self.m_result = True

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()
