#!/usr/bin/env python
#
# #check if dir exist if not create it

from __future__ import absolute_import
import os
import re
import time
from io import open

class Logger(object):
    def __init__(self):
        print "Logger initialized"

    def log_file_creator(self, filepath, group_nr):
        self.file_name = os.path.join(filepath, "group_"+unicode(group_nr)+".txt")
        directory = os.path.dirname(self.file_name)
        self.count = 0
        if not os.path.exists(directory):
            os.makedirs(directory)
            print self.file_name, "is created"
        else:
            self.file_object = open(self.file_name, 'w+')
            self.file_object.truncate(0)
            self.file_object.close()


    def save(self, record):
        print record
        self.file_object = open(self.file_name, 'a')
        record_time = unicode(time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()))
        self.file_object.write(record_time)
        self.file_object.write(unicode('\n'))
        self.file_object.write(unicode(record))
        self.file_object.write(unicode('\n'))
        self.count+=1
        self.file_object.close()
        print self.count, " record saved to ",self.file_name
