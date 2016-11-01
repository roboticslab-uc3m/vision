#!/usr/bin/env python

# 2016 (c) edits by Santiago Morante, Juan G Victores and Raul de Santos. 

# Copyright (c) 2008 Carnegie Mellon University.
#
# You may modify and redistribute this file under the same terms as
# the CMU Sphinx system.  See
# http://cmusphinx.sourceforge.net/html/LICENSE for more information.

from gi import pygtkcompat
import gi

gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
GObject.threads_init()
Gst.init(None)
    
gst = Gst
    
print("Using pygtkcompat and Gst from gi")

pygtkcompat.enable() 
pygtkcompat.enable_gtk(version='3.0')

import gtk

import yarp
import os.path

##
#
# @ingroup speechRecognition
#
# @brief Speech Recognition.
class SpeechRecognition(object):
    """Based on GStreamer/PocketSphinx Demo Application"""
    def __init__(self):
        """Initialize a SpeechRecognition object"""
        rf = yarp.ResourceFinder()
        rf.setVerbose(True)
        rf.setDefaultContext('speechRecognition')
        rf.setDefaultConfigFile('speechRecognition1.ini')
        self.my_lm = rf.findFileByName('words-20150720.lm')
        self.my_dic = rf.findFileByName('words-20150720.dic')
        self.outPort = yarp.Port()
        self.outPort.open('/speechRecognition:o')
        self.init_gst()

    def init_gst(self):
        """Initialize the speech components"""
#        self.pipeline = gst.parse_launch('gconfaudiosrc ! audioconvert ! audioresample '
#                                        + '! vader name=vad auto-threshold=true '
#                                        + '! pocketsphinx name=asr ! fakesink')

	""" Configuring the decoder and improving accuracy """
        self.pipeline = gst.parse_launch('autoaudiosrc ! audioconvert ! audioresample '
                                        + '! pocketsphinx name=asr beam=1e-20 ! fakesink')
        asr = self.pipeline.get_by_name('asr')
        asr.connect('result', self.asr_result)
        asr.set_property('lm', self.my_lm )
        asr.set_property('dict', self.my_dic )
        asr.set_property('configured', True)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()

        self.pipeline.set_state(gst.STATE_PLAYING)

    def asr_result(self, asr, text, uttid):
        """Forward result signals on the bus to the main thread."""
        print '---'
        b = yarp.Bottle()
        #s = text.lower().split()
        #for elem in s:
        #    b.addString(elem)
        #    print elem
        #if b.size() != 0:
        #    self.outPort.write(b)
        print text.lower()
        b.addString(text.lower())
        if text != "":
            self.outPort.write(b)

##
#
# @ingroup teo_head_programs
#
# \defgroup speechRecognition speechRecognition.py
#
# @brief Creates an instance of SpeechRecognition.
yarp.Network.init()
if yarp.Network.checkNetwork() != True:
    print '[asr] error: found no yarp network (try running "yarpserver &"), bye!'
    quit()

app = SpeechRecognition()
gtk.main()
