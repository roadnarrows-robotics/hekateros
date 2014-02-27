###############################################################################
#
# Package:   RoadNarrows Hekateros Robotic Manipulator Package
#
# Link:      https://github.com/roadnarrows-robotics/hekateros
#
# ROS Node:  hek_*
#
# File:      Utils.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Utilities.
##
## \author Daniel Packard (daniel@roadnarrows.com)
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2013-2014.  RoadNarrows LLC.\n
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

from pkg_resources import *

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

from PIL import Image, ImageTk

# ------------------------------------------------------------------------------
# Class ImageLoader
# ------------------------------------------------------------------------------

#
## \brief Class to handle image loading.
#
class ImageLoader:

  #
  ## \brief Constructor
  ##
  ## \param py_pkg      Python resource (e.g. "hekateros_control.images").
  ## \param image_paths List of directory paths to search for the image.
  #
  def __init__(self, py_pkg=None, image_paths=[]):
    self.m_pyPkg = py_pkg
    if len(image_paths) > 0:
      self.m_imagePaths = image_paths
    else:
      self.m_imagePaths = ['.']
  
  #
  ## \brief Class to handle image loading.
  #Load icon image from file name.
  ##
  ## \param filename    Icon file name.
  ##
  ## \return Returns icon widget on success, None on failure.
  #
  def load(self, filename):
    # no file name
    if filename is None or len(filename) == 0:
      return None;
    # absolute file name
    if filename[0] == os.path.sep:
      try:
        return ImageTk.PhotoImage(Image.open(filename))
      except IOError:
        return None
    # relative file name - try python resource(s) first
    if self.m_pyPkg:
      try:
        fqname = resource_filename(self.m_pyPkg, filename)
        try:
          return ImageTk.PhotoImage(Image.open(fqname))
        except IOError:
          pass
      except ImportError:
        pass
    # relative file name - search path for file
    for path in self.m_imagePaths:
      fqname = path + os.path.sep + filename
      try:
        return ImageTk.PhotoImage(Image.open(fqname))
      except IOError:
        continue
    return None


# ------------------------------------------------------------------------------
# Misc. Utilities
# ------------------------------------------------------------------------------

#
#
## Round to nearest 100th.
#
def round100th(x):
  return math.floor((x + 0.005) * 100.0) / 100.0

#
## Round to nearest 10th.
#
def round10th(x):
  return math.floor((x + 0.05) * 10.0) / 10.0

#
## Degrees to radians.
#
def degToRad(deg):
  return deg / 180.0 * math.pi

#
## Radians to degrees.
#
def radToDeg(rad):
  return rad / math.pi * 180.0
