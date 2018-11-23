#!/usr/bin/python
# -*- coding: utf-8 -*-

#------------------------------------------------------------------------------*

import os, sys, subprocess, shutil

#------------------------------------------------------------------------------*

def runCommand (command) :
  s = "+"
  for t in command :
    s += ' "' + t + '"'
  print (s)
  childProcess = subprocess.Popen (command)
  childProcess.wait ()
  if childProcess.returncode != 0 :
    sys.exit (childProcess.returncode)

#------------------------------------------------------------------------------*

def copyFile (sourceFile, destinationDir) :
  runCommand (["cp", sourceFile, destinationDir])

#------------------------------------------------------------------------------*

def compileArduinoSketch (scriptDir, sketch) :
  buildPath = scriptDir + "/build-path/"  + sketch
  if not os.path.exists (buildPath):
    os.makedirs (buildPath)
#---
  command = [
    "/Applications/Arduino.app/Contents/Java/arduino-builder",
    "-quiet",
    "-compile",
    "-logger=machine",
    "-hardware", "/Applications/Arduino.app/Contents/Java/hardware",
    "-hardware", "/Users/pierremolinaro/Library/Arduino15/packages",
    "-hardware", "/Users/pierremolinaro/Documents/Arduino/hardware",
    "-tools", "/Applications/Arduino.app/Contents/Java/tools-builder",
    "-tools", "/Applications/Arduino.app/Contents/Java/hardware/tools/avr",
    "-tools", "/Users/pierremolinaro/Library/Arduino15/packages",
    "-built-in-libraries", "/Applications/Arduino.app/Contents/Java/libraries",
    "-libraries", "/Users/pierremolinaro/Documents/Arduino-dev/libraries",
    "-fqbn=teensy:avr:teensy35:usb=serial,speed=120,opt=oslto,keys=en-us",
    "-ide-version=10805",
    "-build-path", buildPath,
    "-warnings=all",
    scriptDir + "/sample-code/" + sketch + "/" + sketch + ".ino"
  ]
  runCommand (command)

#------------------------------------------------------------------------------*

#--- Get script absolute path
scriptDir = os.path.dirname (os.path.abspath (sys.argv [0]))
os.chdir (scriptDir)
#--- Compile sketches
compileArduinoSketch (scriptDir, "TestWithACAN")
compileArduinoSketch (scriptDir, "LoopBackDemo")
compileArduinoSketch (scriptDir, "LoopBackDemoBitRateSettings")
compileArduinoSketch (scriptDir, "LoopBackFilterDataByte")
compileArduinoSketch (scriptDir, "LoopBackUsingFilters")
#--- Compile OSX code
os.chdir (scriptDir + "/test-ACAN2515Settings-on-desktop")
runCommand ([
  "xcodebuild",
  "-alltargets",
  "-configuration", "Default"
])
runCommand (["build/Release/test-ACAN2515Settings-on-desktop"])
os.chdir (scriptDir)
#--- Compile latex doc
runCommand ([scriptDir + "/documentation-in-latex/-build.command"])
#--- Copy files in the distribution directory
distributionDirectory = scriptDir + "/../releases/acan2515"
if not os.path.exists (distributionDirectory + "/extras"):
  os.makedirs (distributionDirectory + "/extras")
copyFile (scriptDir + "/documentation-in-latex/acan2515.pdf", distributionDirectory + "/extras/")
copyFile (scriptDir + "/library-sources/library.properties", distributionDirectory)
copyFile (scriptDir + "/library-sources/keywords.txt", distributionDirectory)
if os.path.exists (distributionDirectory + "/src"):
  shutil.rmtree (distributionDirectory + "/src")
shutil.copytree (scriptDir + "/library-sources/src", distributionDirectory + "/src")
if os.path.exists (distributionDirectory + "/examples"):
  shutil.rmtree (distributionDirectory + "/examples")
shutil.copytree (scriptDir + "/sample-code", distributionDirectory + "/examples")

#------------------------------------------------------------------------------*
