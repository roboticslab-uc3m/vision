#!/usr/bin/env python

# 2016 (c) edits by Santiago Morante, Juan G Victores and Raul de Santos. 

# Copyright (c) 2008 Carnegie Mellon University.
#
# You may modify and redistribute this file under the same terms as
# the CMU Sphinx system.  See
# http://cmusphinx.sourceforge.net/html/LICENSE for more information.

from gi import pygtkcompat
import gi

import gobject

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

class DataProcessor(yarp.PortReader):
    def setRefToFather(self,value):
        self.refToFather = value
    def read(self,connection):
        print("in DataProcessor.read")
        if not(connection.isValid()):
            print("Connection shutting down")
            return False
        bottleIn = yarp.Bottle()
        bOut = yarp.Bottle() 
        print("Trying to read from connection")
        ok = bottleIn.read(connection)
        if not(ok):
            print("Failed to read input")
            return False
        # Code goes here :-)
        print("Received [%s]"%bottleIn.toString())
        if bottleIn.get(0).asString() == "setDictionary":
                # follow-me dictionary:
                if bottleIn.get(1).asString() == "follow-me":
                        # follow-me english
			if bottleIn.get(2).asString() == "english":
				print("follow-me demo configured in english")
				self.refToFather.setDictionary('dictionary/follow-me-english.lm','dictionary/follow-me-english.dic', 'model/en-us')
                        # follow-me spanish
			elif bottleIn.get(2).asString() == "spanish":
				print("follow-me demo configured in spanish")
                                print("dictionary not found")
                                self.refToFather.setDictionary('dictionary/follow-me-spanish.lm','dictionary/follow-me-spanish.dic','model/es')
                
                # waiter dictionary:
                elif bottleIn.get(1).asString() == "waiter":
                        # waiter english:
  			if bottleIn.get(2).asString() == "english":
                                print("waiter demo configured in english")
				self.refToFather.setDictionary('waiter-english.lm','waiter-english.dic')
                        # waiter spanish:
                       	elif bottleIn.get(2).asString() == "spanish":
                                print("waiter demo configured in spanish")
                                print("dictionary not found")
				# self.refToFather.setDictionary('words-20160617.lm','words-20160617.dic')


        bOut.addString("ok")
        writer = connection.getWriter()
        if writer==None:
            print("No one to reply to")
            return True
        return bOut.write(writer)


##
#
# @ingroup speechRecognition
#
# @brief Speech Recognition.
class SpeechRecognition(object):
    """Based on GStreamer/PocketSphinx Demo Application"""
    def __init__(self):
        """Initialize a SpeechRecognition object"""
        self.rf = yarp.ResourceFinder()
        self.rf.setVerbose(True)
        self.rf.setDefaultContext('speechRecognition')
        self.rf.setDefaultConfigFile('speechRecognition.ini')
        self.my_lm = self.rf.findFileByName('dictionary/follow-me-english.lm')
        self.my_dic = self.rf.findFileByName('dictionary/follow-me-english.dic')
	self.my_model = self.rf.findPath('model/en-us/')
        self.outPort = yarp.Port()
        self.configPort = yarp.RpcServer()  # Use Port() if not Python wrapper not existent!
        self.dataProcessor = DataProcessor() 
        self.dataProcessor.setRefToFather(self) # it pass reference to DataProcessor 
        self.configPort.setReader(self.dataProcessor)       
        self.outPort.open('/speechRecognition:o')
        self.configPort.open('/speechRecognition/rpc:s')
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
        # asr.connect('result', self.asr_result) (it's not running with Gstreamer 1.0)
        asr.set_property('lm', self.my_lm )
        asr.set_property('dict', self.my_dic )
	asr.set_property('hmm', self.my_model )
        #asr.set_property('configured', "true")      

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::element', self.element_message) # new

        self.pipeline.set_state(gst.State.PLAYING)

    def element_message(self, bus, msg):
        """Receive element messages from the bus."""
        print "---"
        b = yarp.Bottle()
        msgtype = msg.get_structure().get_name()
        print msgtype # pocketsphinx 
        
        if msgtype != 'pocketsphinx':
            return

        print "hypothesis= '%s'  confidence=%s final=%s\n" % (msg.get_structure().get_value('hypothesis'), msg.get_structure().get_value('confidence'), msg.get_structure().get_value('final'))
        if msg.get_structure().get_value('final') is True:       
                text = msg.get_structure().get_value('hypothesis')        
                print text.lower()
                b.addString(text.lower())
                if text != "":
                        self.outPort.write(b)

    def setDictionary(self, lm, dic, hmm):
        print "Changing Dictionary...."
        self.my_lm = self.rf.findFileByName(lm)
        self.my_dic = self.rf.findFileByName(dic)
	self.my_model = self.rf.findFileByName(hmm)
        
        self.pipeline.set_state(gst.State.NULL)
        self.pipeline = gst.parse_launch('autoaudiosrc ! audioconvert ! audioresample '
                                         + '! pocketsphinx name=asr beam=1e-20 ! fakesink')

        asr = self.pipeline.get_by_name('asr')
	asr.set_property('lm', self.my_lm)
	asr.set_property('dict', self.my_dic)
	asr.set_property('hmm', self.my_model )
        print("Dictionary changed successfully (%s) (%s) (%s)"%(self.my_lm,self.my_dic,self.my_model))

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::element', self.element_message)

        self.pipeline.set_state(gst.State.PLAYING)

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
# enter into a mainloop
loop = GObject.MainLoop()
loop.run()
