###############################################################################
#
# Package:   RoadNarrows Hekateros Robotic Manipulator Package
#
# Link:      https://github.com/roadnarrows-robotics/hekateros
#
# ROS Node:  hek_*
#
# File:      MoveDlg.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Hekateros move dialog.
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
import math

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

import roslib; roslib.load_manifest('hekateros_control')
import rospy
import trajectory_msgs.msg


from hekateros_control.Utils import *


# ------------------------------------------------------------------------------
# Class MoveDlg
# ------------------------------------------------------------------------------

class MoveDlg(Toplevel):
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
    glist= [self.m_parentGeo[2] + 150,
            self.m_parentGeo[3] + self.m_parentGeo[1] - 120]
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
    if kw.has_key('title'):
      self.m_title = kw['title']
      del kw['title']
    else:
      self.m_title = "Move Hekateros To..."
    if kw.has_key('image'):
      imageLoader = ImageLoader(py_pkg='hekateros_control.images')
      self.m_icons['image'] = imageLoader.load(kw['image'])
      del kw['image']
    else:
      self.m_icons['image'] = None
    self.m_trajectory = kw['trajectory']
    self.m_vals = { }
    del kw['trajectory']
    self.m_resultTraj  = None
    self.m_result = False
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    frame = Frame(self)
    frame.grid(row=0, column=0)

    # move image 
    w = Label(frame)
    if self.m_icons['image'] is not None:
      w = Label(frame)
      w['image']  = self.m_icons['image']
    w['anchor'] = CENTER
    w.grid(row=0, column=0, rowspan=2, sticky=W+N+S)

    # top heading
    w = Label(frame)
    helv = tkFont.Font(family="Helvetica",size=12,weight="bold")
    w['font']   = helv
    w['text']   = 'Specify Trajectory'
    w['anchor'] = CENTER
    w.grid(row=0, column=1, sticky=E+W)

    width = 12
    padx  = 2
    pady  = 3
    row   = 1

    wframe = Frame(frame)
    wframe.grid(row=row, column=1)

    #
    # Trajectory Table
    #
    row   = 0

    # left column of labels
    for text in [' ', 'Position(deg):', 'Velocity(deg/sec):',
                  'Group Velocity:']:
      w = Label(wframe, width=width+2, padx=padx, pady=pady, anchor=W,
          text=text)
      w.grid(row=row, column=0, sticky=W+S)
      row += 1

    col         = 1
    trajPoint   = self.m_trajectory.points[0]
    startGroupV = 0.0

    i = 0
    for name in self.m_trajectory.joint_names:
      # joint label
      row = 0
      w = Label(wframe, width=width, padx=padx, pady=pady, anchor=W, text=name)
      w.grid(row=row, column=col, sticky=W)
      self.m_vals[name] = { }

      # joint position
      row += 1
      var = DoubleVar()
      var.set(round10th(radToDeg(trajPoint.positions[i])))
      w = Spinbox(wframe, justify=RIGHT, textvar=var,
                        increment=0.1, from_=-50000.0, to=50000.0)
      w['width'] = 7
      w.grid(row=row, column=col, padx=1, pady=0, sticky=W)
      d = {'var': var, 'w': w}
      self.m_vals[name]['position'] = d
    
      # joint velocity
      row += 1
      var = DoubleVar()
      var.set(round10th(radToDeg(trajPoint.velocities[i])))
      startGroupV = var.get()
      w = Spinbox(wframe, justify=RIGHT, textvar=var,
                        increment=1.0, from_=-250.0, to=250.0)
      w['width'] = 7
      w.grid(row=row, column=col, padx=1, pady=0, sticky=W)
      d = {'var': var, 'w': w}
      self.m_vals[name]['velocity'] = d
    
      col += 1
      i += 1

    if i > 0:
      self.update_idletasks()
      name = self.m_trajectory.joint_names[0]
      x0 = self.m_vals[name]['position']['w'].winfo_rootx()
      name = self.m_trajectory.joint_names[i-1]
      x1 = self.m_vals[name]['position']['w'].winfo_rootx() + \
           self.m_vals[name]['position']['w'].winfo_width()
      length = x1 - x0
    else:
      i = 1
      length = 100

    # delta group velocity
    self.m_varGroupVel = DoubleVar()
    self.m_varGroupVel.set(round10th(startGroupV))
    w = Scale(wframe, variable=self.m_varGroupVel,
                        from_=-250.0, to=250.0, resolution=1.0,
                        orient=HORIZONTAL, command=self.setGroupV)
    w['width']  = 10
    w['length'] = length
    w.grid(row=3, column=1, columnspan=i, padx=1, pady=0, sticky=W)

    #
    # Dialog Buttons
    #
    row = 2

    wframe = Frame(frame)
    wframe.grid(row=row, column=1)

    # cancel button
    w = Button(wframe, width=10, text='Cancel', command=self.close)
    w.grid(row=0, column=0, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnCancel = w

    # ok button
    w = Button(wframe, width=10, text='Move', command=self.ok)
    w.grid(row=0, column=1, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnContinue = w

  #
  ## \brief Delta velocity callback
  #
  def setGroupV(self, v):
    vel = self.m_varGroupVel.get()
    for name in self.m_vals:
      self.m_vals[name]['velocity']['var'].set(vel)

  #
  ## \brief Destroy window callback.
  #
  def ok(self):
    self.m_resultTraj = trajectory_msgs.msg.JointTrajectory()
    trajPoint = trajectory_msgs.msg.JointTrajectoryPoint()  
    for name in self.m_trajectory.joint_names:
      pos = degToRad(self.m_vals[name]['position']['var'].get())
      vel = degToRad(self.m_vals[name]['velocity']['var'].get())
      if vel != 0.0:
        self.m_resultTraj.joint_names.append(name)
        trajPoint.positions.append(pos)
        trajPoint.velocities.append(vel)
        trajPoint.accelerations.append(0.0)
    if len(self.m_resultTraj.joint_names) > 0:
      self.m_resultTraj.points.append(trajPoint)
      self.m_result = True
    else:
      self.m_result = False
    self.close()

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()
