###############################################################################
#
# Package:   RoadNarrows Hekateros Robotic Manipulator Package
#
# Link:      https://github.com/roadnarrows-robotics/hekateros
#
# ROS Node:  hek_*
#
# File:      Xterm.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Xterm widget.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2015-2016.  RoadNarrows LLC.\n
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
import threading
import subprocess
import shlex

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

from hekateros_control.Utils import *

# ------------------------------------------------------------------------------
# Class Xterm
# ------------------------------------------------------------------------------

class Xterm(Frame):
  #
  ## \brief Constructor.
  ##
  ## \par Xterm Keyword Options
  ## Xterm specific keyword options. All other xterm options can be supported
  ## through application resources.
  ## \htmlonly
  ## <table>
  ## <tr>
  ##  <td>xterm_cmd=pgm</td><td>Execute script or program</td>
  ##    <td>Default: None</td>
  ##  <td>xterm_font=FONT</td><td>Specify terminal normal font</td>
  ##    <td>Default: 9x15</td>
  ##  <td>xterm_geom=WxH</td>
  ##    <td>Width and height of embedded xterm in characters</td>
  ##    <td>Default: 80x20</td>
  ##  <td>xterm_sb=TF</td><td>Enable/disable scrollbar</td>
  ##    <td>Default: True</td>
  ## </tr>
  ## </table>
  ## \endhtmlonly
  ##
  ## \par Run-Time Keyword Options
  ## Run-time execution control options.
  ## \htmlonly
  ## <table>
  ## <tr>
  ##  <td>auto_run=TF</td>
  ##    <td>Do [not] auto-run script or program. Use the execute method to
  ##    manually run.  </td>
  ##    <td>Default: False</td>
  ## </tr>
  ## <tr>
  ##  <td>on_exit=FUNC</td>
  ##    <td>If set, make callback to parent function FUNC on xterm exit.</td>
  ##    <td>Default: None</td>
  ## </tr>
  ## </table>
  ## \endhtmlonly
  ##
  ## \par Frame Keyword Options
  ## All frame options are supported. Common useful options are:
  ## \htmlonly
  ## <table>
  ## <tr>
  ##  <td>borderwidth=pixels</td>
  ##    <td>Frame border width</td><td>Default: 2</td>
  ## <tr>
  ##  <td>height=pixels</td><td>Height of widget</td>
  ##    <td>Default: auto-calculated</td>
  ## </tr><tr>
  ##  <td>padx=pixels</td>X external padding<td></td><td>Default: 0</td>
  ## </tr><tr>
  ##  <td>pady=pixels</td>Y external padding<td></td><td>Default: 0</td>
  ## </tr><tr>
  ##  <td>relief=enum</td><td>Widget relief</td><td>Default: 'ridge'</td>
  ## </tr><tr>
  ##  <td>width=pixels</td><td>Width of widget</td>
  ##    <td>Default: auto-calculated</td>
  ## </tr>
  ## </table>
  ## \endhtmlonly
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    self.m_result = 'ok'

    # initialize keyword data
    kw = self.initData(kw)

    # frame init
    Frame.__init__(self, master=master, cnf=cnf, **kw)

    # frame parameters
    self.m_wXFrame = Frame(self)
    self.m_wXFrame['width']  = kw['width']  - kw['borderwidth']
    self.m_wXFrame['height'] = kw['height'] - kw['borderwidth']
    self.m_wXFrame.grid(row=0, column=0, padx=5, pady=5, sticky=N+S+E+W)

    # state data
    self.m_wId  = self.m_wXFrame.winfo_id()
    self.m_th   = None
    self.m_pipe = None
    self.m_xterm_opts_str = ""

    # build xterm command string
    self.buildXtermCmd()

    # do [not] auto-run xterm command
    if self.m_opts['auto_run']:
      self.execute()

  #
  ## \brief Initialize class keyword options data.
  ##
  ## \param kw  Keyword options.
  ##
  ## \return Modified keywords sans this specific class keywords.
  ##
  def initData(self, kw):
    frameopts = { }

    #
    # xterm default options
    #
    self.m_xopts = { }
    self.m_xopts['geom']  = "80x20"
    self.m_xopts['font']  = "9x15"
    self.m_xopts['sb']    = True
    self.m_xopts['cmd']   = None

    preface = 'xterm_'

    for k,v in kw.iteritems():
      i = k.find(preface)
      if i == 0:
        opt = k[len(preface):]
        self.m_xopts[opt] = v
      else:
        frameopts[k] = v

    #
    # run-time options
    #
    self.m_opts = { }

    k = 'auto_run'
    if kw.has_key(k):
      self.m_opts[k] = kw[k]
      del frameopts[k]
    else:
      self.m_opts[k] = False
    k = 'on_exit'
    if kw.has_key(k):
      self.m_opts[k] = kw[k]
      del frameopts[k]
    else:
      self.m_opts[k] = None

    #
    # frame options
    #
    wxh = self.m_xopts['geom'].split('x')
    try:
      geom_width  = int(wxh[0])
      geom_height = int(wxh[1])
    except ValueError:
      geom_width  = 80
      geom_height = 20

    wxh = self.m_xopts['font'].split('x')
    try:
      font_width = int(wxh[0])
      font_height = int(wxh[1])
    except ValueError:
      font_width  = 9
      font_height = 15

    # border width
    bw  = frameopts.setdefault('borderwidth', 3) # border width

    # scroll bar width
    if self.m_xopts['sb']:
      sbw = 14
    else:
      sbw = 0

    # frame margin
    margin = 2

    v = frameopts.setdefault('padx', 0)
    v = frameopts.setdefault('pady', 0)
    v = frameopts.setdefault('relief', 'ridge')
    v = frameopts.setdefault('height', margin + font_height*geom_height + 2*bw)
    v = frameopts.setdefault('width', 
                                  margin + sbw + font_width*geom_width + 2*bw)

    return frameopts

  #
  ## \brief Build xterm command string
  #
  def buildXtermCmd(self):
    for k,v in self.m_xopts.iteritems():
        if k == "geom":
          self.m_xterm_opts_str += " -geometry %s" % (v)
        elif k == "sb":
          if v:
            self.m_xterm_opts_str += " -sb"
        elif k == "font":
          self.m_xterm_opts_str += " -font %s" % (v)
        elif k == "cmd":
          script = v
        else:
          print "Unknown xterm keyword", preface+k, "ignoring."
    if self.m_xopts['cmd']:
      self.m_xterm_opts_str += " -e \"%s\"" % (self.m_xopts['cmd'])
    self.m_xtermcmd = 'xterm -into %d %s' % (self.m_wId, self.m_xterm_opts_str)
    #print self.m_xtermcmd

  #
  ## \brief Execute xterm command.
  #
  def execute(self):
    self.m_th = threading.Thread(target=self.waitForXterm)
    self.m_th.start()

  #
  ## \brief Wait for xterm command to exit.
  #
  def waitForXterm(self):
    args = shlex.split(self.m_xtermcmd);
    try:
      self.m_pipe = subprocess.Popen(args)
    except AttributeError:
      pass
    ec = self.m_pipe.wait()
    #print 'pipe exit', ec
    if ec != 0:
      self.m_result = 'Error: Failed to execute'
    if self.m_opts['on_exit']:
      self.m_opts['on_exit']()

  #
  ## \brief Destroy this Xterm widget.
  #
  def destroy(self):
    #print 'Xterm destroy'
    self.m_opts['on_exit'] = None   # disable any parent callback
    # poll returns none if process is still running
    if self.m_pipe is not None and self.m_pipe.poll() is None:
      #print 'xterm kill'
      self.m_pipe.kill()
    Frame.destroy(self)


# ------------------------------------------------------------------------------
# Unit test
# ------------------------------------------------------------------------------

if __name__ == "__main__":

  #TestCmd = "echo 'my test'; sleep 5"
  #TestCmd = "gst-launch autovideosrc ! autovideosink"
  TestCmd = "(trap '' SIGHUP; hek_eecam_start --res=720p --fps=30 >o 2>&1 &)"

  class UT(Frame):
    def __init__(self, master=None):
      Frame.__init__(self, master=root)
      self.master.title("Xterm Widget Unit Test")

      self.wXterm = Xterm(master=self, auto_run=True, on_exit=self.destroy,
                            xterm_cmd=TestCmd)
      self.wXterm.grid(row=0, column=0, padx=5, pady=5)

      self.wBttnQuit = Button(self)
      self.wBttnQuit["text"] = "Quit"
      self.wBttnQuit["command"] = self.destroy
      self.wBttnQuit.grid(row=1, column=0)

      self.grid(row=0, column=0, padx=5, pady=5)

    def destroy(self):
      #print 'UT destroy'
      Frame.destroy(self)   # tkinter calls child destroy methods.
      self.quit()

    def getResult(self):
      return self.wXterm.m_result

  # run
  root = Tk()
  root.protocol('WM_DELETE_WINDOW', root.destroy)
  ut = UT(master=root)
  ut.mainloop()
  print ut.getResult()
