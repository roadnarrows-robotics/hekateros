###############################################################################
#
# Package:   RoadNarrows Hekateros Robotic Manipulator Package
#
# Link:      https://github.com/roadnarrows-robotics/hekateros
#
# ROS Node:  hek_*
#
# File:      PasswordDlg.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Hekateros password dialog.
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
# Class PasswordDlg
# ------------------------------------------------------------------------------

class PasswordDlg(Toplevel):
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
    self.m_bttnCancel.bind('<KeyPress-Return>', func=self.cr)

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
      self.m_title = "Password"
    if kw.has_key('image'):
      self.m_icons['image'] = imageLoader.load(kw['image'])
      del kw['image']
    else:
      self.m_icons['image'] = None
    if self.m_icons['image'] is None:
      self.m_icons['image'] = imageLoader.load('icons/icon_password.png')
    if kw.has_key('hekateros'):
      self.m_host = kw['hekateros']
      del kw['hekateros']
    else:
      self.m_host = "hekateros"
    if kw.has_key('user'):
      self.m_user = kw['user']
      del kw['user']
    else:
      self.m_user = "robot"
    self.m_result = False
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    frame = Frame(self)
    frame.grid(row=0, column=0)

    # password image 
    w = Label(frame)
    if self.m_icons['image'] is not None:
      w = Label(frame)
      w['image']  = self.m_icons['image']
    w['anchor'] = CENTER
    w.grid(row=0, column=0, sticky=W+E)

    # top heading
    w = Label(frame)
    helv = tkFont.Font(family="Helvetica",size=24,weight="bold")
    w['font']   = helv
    w['text']   = 'Password'
    w['anchor'] = CENTER
    w.grid(row=0, column=1, sticky=E+W)

    row = 1

    wframe = Frame(frame)
    wframe.grid(row=row, column=0, pady=5, columnspan=2)

    row = 0

    # hekateros host
    w = Label(wframe)
    w['text']   = "Hekateros:"
    w['anchor'] = W
    w['justify'] = LEFT
    w.grid(row=row, column=0, padx=5, sticky=W)

    w = Label(wframe)
    w['text']   = self.m_host;
    w['anchor'] = W
    w['justify'] = LEFT
    w.grid(row=row, column=1, padx=5, sticky=W)

    row += 1

    # user 
    w = Label(wframe)
    w['text']   = "User:"
    w['anchor'] = W
    w['justify'] = LEFT
    w.grid(row=row, column=0, padx=5, sticky=W)

    w = Label(wframe)
    w['text']   = self.m_user;
    w['anchor'] = W
    w['justify'] = LEFT
    w.grid(row=row, column=1, padx=5, sticky=W)

    row += 1

    # password 
    w = Label(wframe)
    w['text']   = "Password:"
    w['anchor'] = W
    w['justify'] = LEFT
    w.grid(row=row, column=0, padx=5, sticky=W)

    w = Entry(wframe, show='*')
    w['justify'] = LEFT
    w['width'] = 32
    w.grid(row=row, column=1, padx=5, sticky=W)
    self.m_wEntryPasswd = w
    self.m_wEntryPasswd.bind("<KeyRelease-Return>", self.cr)

    row += 1

    wframe = Frame(frame)
    wframe.grid(row=row, column=0, columnspan=2)

    # cancel button
    w = Button(wframe, width=10, text='Cancel', command=self.close)
    w.grid(row=0, column=0, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnCancel = w

    # ok button
    w = Button(wframe, width=10, text='Enter', command=self.ok)
    w.grid(row=0, column=1, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnEnter = w

  #
  ## \brief Carriage return callback.
  #
  def cr(self, event):
    self.ok()

  #
  ## \brief Ok window callback.
  #
  def ok(self):
    self.m_password = self.m_wEntryPasswd.get()
    if len(self.m_password) > 0:
      self.m_result = True
    else:
      self.m_password = None
    self.close()

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()


# ------------------------------------------------------------------------------
# Unit test
# ------------------------------------------------------------------------------

if __name__ == "__main__":
  class UT(Frame):
    def __init__(self, master=None):
      Frame.__init__(self, master=root)
      self.master.title("PasswordDlg Unit Test")
      self.wBttnPasswd = Button(self)
      self.wBttnPasswd["text"] = "Password Dialog"
      self.wBttnPasswd["command"] = self.getPassword
      self.wBttnPasswd.grid(row=0, column=0)

      self.wBttnQuit = Button(self)
      self.wBttnQuit["text"] = "Quit"
      self.wBttnQuit["command"] = self.destroy
      self.wBttnQuit.grid(row=0, column=1)

      self.grid(row=0, column=0, padx=5, pady=5)

    def getPassword(self):
      dlg = PasswordDlg(master=self, title="Hekateros Password")
      if dlg.m_result:
        print 'Password entered:', repr(dlg.m_password)
      else:
        print 'No password'

    def destroy(self):
      self.quit()

  # run
  root = Tk()
  root.protocol('WM_DELETE_WINDOW', root.destroy)
  ut = UT(master=root)
  ut.mainloop()
