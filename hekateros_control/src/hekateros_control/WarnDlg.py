###############################################################################
#
# Package:   RoadNarrows Hekateros Robotic Manipulator Package
#
# Link:      https://github.com/roadnarrows-robotics/hekateros
#
# ROS Node:  hek_*
#
# File:      WarnDlg.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Hekateros warning dialog.
##
## \author Daniel Packard (daniel@roadnarrows.com)
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

from hekateros_control.Utils import *

# ------------------------------------------------------------------------------
# Class WarnDlg
# ------------------------------------------------------------------------------

class WarnDlg(Toplevel):
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
    self.m_icons          = {}    # must keep loaded icons referenced
    imageLoader = ImageLoader(py_pkg='hekateros_control.images')
    if kw.has_key('title'):
      self.m_title = kw['title']
      del kw['title']
    else:
      self.m_title = "Warning"
    if kw.has_key('image'):
      self.m_icons['image'] = imageLoader.load(kw['image'])
      del kw['image']
    else:
      self.m_icons['image'] = None
    if self.m_icons['image'] is None:
      self.m_icons['image'] = imageLoader.load('icons/icon_warning.png')
    if kw.has_key('msg'):
      self.m_msg = kw['msg']
      del kw['msg']
    else:
      self.m_msg = "Warn what???"
    self.m_result = False
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    frame = Frame(self)
    frame.grid(row=0, column=0)

    # warning image 
    w = Label(frame)
    if self.m_icons['image'] is not None:
      w = Label(frame)
      w['image']  = self.m_icons['image']
    w['anchor'] = CENTER
    w.grid(row=0, column=0, rowspan=2, sticky=W+N+S)

    # top heading
    w = Label(frame)
    helv = tkFont.Font(family="Helvetica",size=24,weight="bold")
    w['font']   = helv
    w['text']   = 'Warning'
    w['anchor'] = CENTER
    w.grid(row=0, column=1, sticky=E+W)

    row = 1

    # warning message
    w = Label(frame)
    w['text']   = self.m_msg
    w['anchor'] = W
    w['justify'] = LEFT
    w.grid(row=row, column=1, padx=5, sticky=E+W)

    row += 1

    wframe = Frame(frame)
    wframe.grid(row=row, column=1)

    # cancel button
    w = Button(wframe, width=10, text='Cancel', command=self.close)
    w.grid(row=0, column=0, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnCancel = w

    # ok button
    w = Button(wframe, width=10, text='Continue', command=self.ok)
    w.grid(row=0, column=1, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnContinue = w

  #
  ## \brief Destroy window callback.
  #
  def ok(self):
    self.m_result = True
    self.close()

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()
