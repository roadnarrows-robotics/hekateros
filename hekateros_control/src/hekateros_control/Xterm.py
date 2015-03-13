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
import time

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
  ## Xterm Keyword Options
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
  ## \par Run-time Options
  ## Run-time execution control options.
  ## \htmlonly
  ## <table>
  ## <tr>
  ##  <td>auto_run=TF</td><td>Do [not] auto-run xterm in widget.</td>
  ##    <td>Default: True</td>
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

    Frame.__init__(self, master=master, cnf=cnf, **kw)

    self.m_wXFrame = Frame(self)
    self.m_wXFrame['width'] = kw['width'] - kw['borderwidth']
    self.m_wXFrame['height'] = kw['height'] - kw['borderwidth']
    self.m_wXFrame.grid(row=0, column=0, padx=5, pady=5, sticky=N+S+E+W)

    self.m_wId = self.m_wXFrame.winfo_id()

    if self.m_opts['auto_run']:
      self.execute()

  #
  ## \brief Initialize class state data.
  ##
  ## \param kw      Keyword options.
  ##
  ## \return Modified keywords sans this specific class keywords.
  ##
  def initData(self, kw):
    # xterm default options
    xopts = { }
    xopts['geom']  = "80x20"
    xopts['font']  = "9x15"
    xopts['sb']    = True
    xopts['cmd']    = None

    preface = 'xterm_'

    for k,v in kw.iteritems():
      i = k.find(preface)
      if i == 0:
        opt = k[len(preface):]
        xopts[opt] = v
        del kw[k]

    self.m_xterm_opts_str = ""

    for k,v in xopts.iteritems():
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
    if xopts['cmd']:
      self.m_xterm_opts_str += " -e \"%s\"" % (xopts['e'])

    wxh = xopts['geom'].split('x')
    try:
      geom_width  = int(wxh[0])
      geom_height = int(wxh[1])
    except ValueError:
      geom_width  = 80
      geom_height = 20

    wxh = xopts['font'].split('x')
    try:
      font_width = int(wxh[0])
      font_height = int(wxh[1])
    except ValueError:
      font_width  = 9
      font_height = 15

    # border width
    bw  = kw.setdefault('borderwidth', 3) # border width

    # scroll bar width
    if xopts['sb']:
      sbw = 14
    else:
      sbw = 0

    # frame margin
    margin = 2

    # frame options
    v = kw.setdefault('padx', 0)
    v = kw.setdefault('pady', 0)
    v = kw.setdefault('relief', 'ridge')
    v = kw.setdefault('height', margin + font_height*geom_height + 2 * bw)
    v = kw.setdefault('width',  margin + sbw + font_width*geom_width + 2 * bw)

    # run-time options
    self.m_opts = { }

    k = 'auto_run'
    if kw.has_key(k):
      self.m_opts[k] = kw[k]
      del kw[k]
    else:
      self.m_opts[k] = False

    return kw

  def execute(self):
    ec = os.system('xterm -into %d %s &' % (self.m_wId, self.m_xterm_opts_str))
    if ec != 0:
      self.m_result = 'Failed to execute'
      self.cleanup()
      return

  def cleanup(self):
    pass

# ------------------------------------------------------------------------------
# Unit test
# ------------------------------------------------------------------------------

if __name__ == "__main__":
  class UT(Frame):
    def __init__(self, master=None):
      Frame.__init__(self, master=root)
      self.master.title("Xterm Widget Unit Test")

      self.wXterm = Xterm(master=self, auto_run=True)
      self.wXterm.grid(row=0, column=0, padx=5, pady=5)

      self.wBttnQuit = Button(self)
      self.wBttnQuit["text"] = "Quit"
      self.wBttnQuit["command"] = self.destroy
      self.wBttnQuit.grid(row=1, column=0)

      self.grid(row=0, column=0, padx=5, pady=5)

    def destroy(self):
      self.quit()

  # run
  root = Tk()
  root.protocol('WM_DELETE_WINDOW', root.destroy)
  ut = UT(master=root)
  ut.mainloop()
