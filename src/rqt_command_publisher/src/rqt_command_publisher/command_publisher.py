#!/usr/bin/env python
from __future__ import division
import math
import random
import time

import genpy
import rospy
import roslib

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Slot, QSignalMapper, QTimer

from rqt_command_publisher.command_publisher_widget import CommandPublisherWidget


class CommandPublisher(Plugin):

    def __init__(self, context):
        super(CommandPublisher, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Command Publisher')

        # Create widget
        self._widget = CommandPublisherWidget()
        self._widget.start_publisher.connect(self.start_publisher)
        self._widget.stop_publisher.connect(self.clean_up_publishers)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Create context for the expression eval statement
        self._eval_locals = {'i': 0}
        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']

        self._publishers = {}
        self._id_counter = 0

        self._timeout_mapper = QSignalMapper(self)
        self._timeout_mapper.mapped[int].connect(self.publish_once)

        # Add widget to the user interface
        context.add_widget(self._widget)

    @Slot(str, str, float, object)
    def start_publisher(self, topic_name, command_name, rate, data):
        publisher_info = {
            'topic_name': str(topic_name),
            'type_name': str('std_msgs/UInt16MultiArray'),
            'command_name': str(command_name),
            'rate': float(rate),
            'data': data
        }
        self._start_publisher(publisher_info)

    def _start_publisher(self, publisher_info):
        publisher_info['publisher_id'] = self._id_counter
        self._id_counter += 1
        publisher_info['counter'] = 0

        publisher_info['message_instance'] = roslib.message.get_message_class(publisher_info['type_name'])()
        if publisher_info['message_instance'] is None:
            return

        # create publisher and timer
        try:
            publisher_info['publisher'] = rospy.Publisher(publisher_info['topic_name'],
                                                          type(publisher_info['message_instance']),
                                                          queue_size=100)
        except TypeError:
            publisher_info['publisher'] = rospy.Publisher(publisher_info['topic_name'],
                                                          type(publisher_info['message_instance']))
        publisher_info['timer'] = QTimer(self)

        # add publisher info to _publishers dict and create signal mapping
        self._publishers[publisher_info['publisher_id']] = publisher_info
        self._timeout_mapper.setMapping(publisher_info['timer'], publisher_info['publisher_id'])
        publisher_info['timer'].timeout.connect(self._timeout_mapper.map)
        if publisher_info['rate'] > 0:
            publisher_info['timer'].start(int(1000.0 / publisher_info['rate']))

    @Slot(int)
    def publish_once(self, publisher_id):
        rospy.loginfo('publish_once: {}'.format(publisher_id))
        publisher_info = self._publishers.get(publisher_id, None)
        rospy.loginfo(publisher_info)
        if publisher_info is not None:
            publisher_info['counter'] += 1
            publisher_info['publisher'].publish(data=publisher_info['data'])

    def clean_up_publishers(self):
        # self._widget.publisher_tree_widget.model().clear()
        for publisher_info in self._publishers.values():
            publisher_info['timer'].stop()
            publisher_info['publisher'].unregister()
        self._publishers = {}

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
        self.clean_up_publishers()
