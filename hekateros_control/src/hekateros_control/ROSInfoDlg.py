###############################################################################
#
#
# Package:   RoadNarrows Hekateros Robotic Manipulator Package
#
# Link:      https://github.com/roadnarrows-robotics/hekateros
#
# ROS Node:  hek_*
#
# File:      ROSInfoDlg.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief ROS info dialog.
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

import webbrowser

from hekateros_control.Utils import *

# ------------------------------------------------------------------------------
# Class ROSInfoDlg
# ------------------------------------------------------------------------------

#
## \brief Hekateros about dialog.
##
class ROSInfoDlg(Toplevel):
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
    self.title(self.m_rosTitle+" Info")

    # create and show widgets
    self.createWidgets()

    # allows the enter button to fire either button's action
    self.m_bttnOk.bind('<KeyPress-Return>', func=self.close)

    # center the dialog over parent panel
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

    # start with ok button focused
    #self.m_bttnOk.focus_set()

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
    self.m_icons            = {}    # must keep loaded icons referenced
    self.m_rosTitle         = "ROS"
    self.m_rosByline        = "Robot Operating System"
    self.m_rosUrl           = "http://ros.org"
    self.m_ros_distro       = os.getenv("ROS_DISTRO", "?")
    self.m_ros_master_uri   = os.getenv("ROS_MASTER_URI", "?")
    self.m_ros_root         = os.getenv("ROS_ROOT", "?")
    self.m_ros_package_path = os.getenv("ROS_PACKAGE_PATH", "?").replace(":", ":\n")

    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    imageLoader = ImageLoader(py_pkg="hekateros_control.images")

    frame = Frame(self)
    frame.grid(row=0, column=0)

    self.m_icons['ros_logo'] = imageLoader.load("ROSLogo.png")

    row = 0

    # top heading
    w = Label(frame)
    times32     = tkFont.Font(family="Times",size=32,weight="bold")
    w['pady']   = 0
    w['font']   = times32
    w['text']   = self.m_rosTitle
    w['anchor'] = W
    w.grid(row=row, column=1, padx=2, pady=0, sticky=E+W)

    bg = w['bg']

    row += 1

    # byline
    #w = Text(frame)
    #w['wrap']   = WORD
    #w['width']  = lwidth + rwidth + 5
    #w['height']  = 1
    #w['relief']  = 'flat'
    #w['bg']  = bg
    #w.insert(END, self.m_rosByline)
    #w['state']  = 'disabled'
    #w.grid(row=row, column=1, columnspan=1, padx=2, sticky=W)
    w = Label(frame)
    times16     = tkFont.Font(family="Times",size=16,weight="bold")
    w['pady']   = 0
    w['font']   = times16
    w['text']   = self.m_rosByline
    w['anchor'] = W
    w.grid(row=row, column=1, padx=2, pady=0, sticky=W+S)

    row += 1

    # logo
    w = Label(frame)
    if self.m_icons['ros_logo'] is not None:
      w['image']  = self.m_icons['ros_logo']
    w['anchor'] = CENTER
    w.grid(row=0, column=0, rowspan=row, sticky=W+S)

    wframe = Frame(frame)
    wframe.grid(row=row, column=0, columnspan=2)

    lwidth  = 20
    rwidth  = 36
    row     = 0

    # url
    w = Label(wframe)
    w['text']   = 'ROS Organization URL:'
    w['anchor'] = W
    w['width'] = lwidth
    w.grid(row=row, column=0, padx=2, sticky=W)

    w = Button(wframe)
    w['text']   = self.m_rosUrl
    w['fg']   = '#aa0000'
    w['activeforeground']   = '#cc0033'
    w['activebackground']   = w['bg']
    w['cursor'] = 'hand1'
    w['anchor'] = W
    w['justify'] = LEFT
    w['relief']   = 'flat'
    w['borderwidth']   = 0
    w['padx']   = 0
    w['pady']   = 0
    w['width']  = rwidth
    w['command']  = lambda aurl=self.m_rosUrl:webbrowser.open_new(self.m_rosUrl)
    w.grid(row=row, column=1, ipadx=0, ipady=0, padx=2, pady=0, sticky=W)

    row += 1

    # distro
    w = Label(wframe)
    w['text']   = 'ROS_DISTRO:'
    w['anchor'] = W
    w['width']  = lwidth
    w.grid(row=row, column=0, padx=2, sticky=W)

    w = Label(wframe)
    w['text']   = self.m_ros_distro
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    row += 1

    # master uri
    w = Label(wframe)
    w['text']   = 'ROS_MASTER_URI:'
    w['anchor'] = W
    w['width']  = lwidth
    w.grid(row=row, column=0, padx=2, sticky=W)

    w = Label(wframe)
    w['text']   = self.m_ros_master_uri
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    row += 1

    # root
    w = Label(wframe)
    w['text']   = 'ROS_ROOT:'
    w['anchor'] = W
    w['width']  = lwidth
    w.grid(row=row, column=0, padx=2, sticky=W)

    w = Label(wframe)
    w['text']   = self.m_ros_root
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    row += 1

    # package path
    w = Label(wframe)
    w['text']   = 'ROS_PACKAGE_PATH:'
    w['anchor'] = W
    w['width']  = lwidth
    w.grid(row=row, column=0, padx=2, sticky=N+W)

    w = Label(wframe)
    w['text']     = self.m_ros_package_path
    w['anchor']   = W
    w['width']    = rwidth
    w['justify']  = LEFT
    w.grid(row=row, column=1, padx=2, sticky=W)

    row += 1

    # ok button
    w = Button(frame, width=10, text='OK', command=self.close)
    w.grid(row=row, column=0, columnspan=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnOk = w

    row += 1

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()
