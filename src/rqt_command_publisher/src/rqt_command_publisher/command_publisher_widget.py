#!/usr/bin/env python

from __future__ import division
import os
import yaml

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget

import roslib
import rosmsg
import rospkg
import rospy

from qt_gui_py_common.worker_thread import WorkerThread
from rqt_py_common.extended_combo_box import ExtendedComboBox

COMMAND_DATA_LENGTH = 14


class CommandPublisherWidget(QWidget):
    start_publisher = Signal(str, str, float, object)
    stop_publisher = Signal()
    # clean_up_publishers = Signal()

    # Get path to UI file which should be in the "resource" folder of this package
    _ui_file_path = os.path.join(rospkg.RosPack().get_path('rqt_command_publisher'),
                                 'resource',
                                 'CommandPublisher.ui')
    _command_file_path = os.path.join(rospkg.RosPack().get_path('zstt'),
                                      'resource',
                                      'command.yaml')

    def __init__(self, parent=None):
        super(CommandPublisherWidget, self).__init__(parent)
        self._update_thread = WorkerThread(self._update_thread_run, self._update_finished)
        self._rospack = rospkg.RosPack()

        # Extend the widget with all attributes and children from UI file
        loadUi(self._ui_file_path, self)

        self.refresh_button.setIcon(QIcon.fromTheme('view-refresh'))
        self.refresh_button.clicked.connect(self.refresh_combo_boxes)

        self.start_publisher_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.start_publisher_button.setEnabled(False)
        self.stop_publisher_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.stop_publisher_button.setEnabled(False)

        self.save_button.setIcon(QIcon.fromTheme('document-save'))
        self.save_button.clicked.connect(self._save_command_data)
        self.reset_button.setIcon(QIcon.fromTheme('edit-undo'))
        self.reset_button.clicked.connect(self._reset_command_data)

        self.refresh_combo_boxes()
        self.command_combo_box.currentTextChanged.connect(self.on_command_combo_box_changed)

        # self.remove_publisher_button.clicked.connect(self.publisher_tree_widget.remove_selected_publishers)

    def shutdown_plugin(self):
        self._update_thread.kill()

    @Slot()
    def refresh_combo_boxes(self):
        self._update_thread.kill()
        self.command_combo_box.setEnabled(False)
        self.topic_combo_box.setEnabled(False)
        self.command_combo_box.setEditText('updating...')
        self.topic_combo_box.setEditText('updating...')
        self._update_thread.start()

    # this runs in a non-gui thread, so don't access widgets here directly
    def _update_thread_run(self):
        try:
            with file(self._command_file_path, 'r') as stream:
                self.command_data = yaml.load(stream)
            if self.command_data is None:
                self.command_data = {}
        except:
            self.command_data = {}

        # update_command_combo_box
        commands = self.command_data.get('commands', {})
        self.command_combo_box.setItems.emit(sorted(commands.keys()))

        # update topic_combo_box
        self.topic_combo_box.setItems.emit(['/ctl_robot'])

    @Slot()
    def _update_finished(self):
        self.command_combo_box.setEnabled(True)
        self.topic_combo_box.setEnabled(True)
        self.start_publisher_button.setEnabled(True)

    @Slot()
    def on_start_publisher_button_clicked(self):
        topic_name = str(self.topic_combo_box.currentText())
        command_name = str(self.command_combo_box.currentText())
        rate = float(self.frequency_combo_box.currentText())
        data = self._get_command_spinbox_values()
        self.start_publisher.emit(topic_name, command_name, rate, data)
        self.start_publisher_button.setEnabled(False)
        self.stop_publisher_button.setEnabled(True)

    @Slot()
    def on_stop_publisher_button_clicked(self):
        self.stop_publisher.emit()
        self.start_publisher_button.setEnabled(True)
        self.stop_publisher_button.setEnabled(False)

    def _set_command_spinbox_values(self, data):
        if len(data) < COMMAND_DATA_LENGTH:
            return

        for i in range(0, COMMAND_DATA_LENGTH):
            spin_box = getattr(self, 'spinBox_{}'.format(i))
            spin_box.setValue(data[i])

    def _get_command_spinbox_values(self):
        data = []
        for i in range(0, COMMAND_DATA_LENGTH):
            spin_box = getattr(self, 'spinBox_{}'.format(i))
            data.append(spin_box.value())
        return data

    @Slot(str)
    def on_command_combo_box_changed(self, text):
        rospy.loginfo(text)
        command = self.command_data.get('commands', {})
        command = command.get(str(text), [])
        rospy.loginfo(command)
        self._set_command_spinbox_values(command)

    @Slot()
    def _save_command_data(self):
        command_name = str(self.command_combo_box.currentText())
        new_data = self._get_command_spinbox_values()

        try:
            with file(self._command_file_path, 'w') as stream:
                self.command_data['commands'][command_name] = new_data
                yaml.dump(self.command_data, stream, explicit_start=True, encoding='utf-8')
                rospy.loginfo('Success save command_data')
        except:
            rospy.logerr('Error to save command_data')

    @Slot()
    def _reset_command_data(self):
        command_name = str(self.command_combo_box.currentText())
        command = self.command_data.get('commands', {})[command_name]
        self._set_command_spinbox_values(command)
