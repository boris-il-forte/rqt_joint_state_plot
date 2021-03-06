import rospy
import rospkg

import threading

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget, QTreeWidgetItem

from sensor_msgs.msg import JointState
from .plot_widget import PlotWidget


class MainWidget(QWidget):
    draw_curves = Signal(object, object)
    _redraw_interval = 40

    def __init__(self):
        super(MainWidget, self).__init__()
        self.setObjectName('MainWidget')

        rospack = rospkg.RosPack()
        ui_file = rospack.get_path(
            'rqt_joint_state_plot') + '/resource/JointTrajectoryPlot.ui'
        loadUi(ui_file, self)

        self.refresh_button.setIcon(QIcon.fromTheme('view-refresh'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))

        self.handler = None
        self.joint_names = []

        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        self._lock = threading.Lock()

        self.plot_widget = PlotWidget(self)
        self.plot_layout.addWidget(self.plot_widget)
        self.draw_curves.connect(self.plot_widget.draw_curves)

        self._start_time = rospy.get_time()
        self.time = {}
        (self.pos, self.vel, self.eff) = ({}, {}, {})

        # refresh topics list in the combobox
        self.refresh_topics()
        self.change_topic()

        self.refresh_button.clicked.connect(self.refresh_topics)
        self.topic_combox.currentIndexChanged.connect(self.change_topic)
        self.select_tree.itemChanged.connect(self.update_checkbox)

    def refresh_topics(self):
        self.enable_timer(False)
        self._lock.acquire()

        self.time = {}
        (self.pos, self.vel, self.eff) = ({}, {}, {})

        topic_list = rospy.get_published_topics()
        if topic_list is None:
            return
        self.topic_combox.clear()
        self.plot_widget.clear_plot()
        for (name, type) in topic_list:
            if type == 'sensor_msgs/JointState':
                self.topic_combox.addItem(name)

        self._lock.release()
        self.enable_timer()

    def change_topic(self):
        topic_name = self.topic_combox.currentText()
        if not topic_name:
            return
        if self.handler:
            self.handler.unregister()
        self.joint_names = []
        self.handler = rospy.Subscriber(topic_name, JointState, self.callback)

    def close(self):
        if self.handler:
            self.handler.unregister()
            self.handler = None

    def refresh_tree(self):
        self.select_tree.itemChanged.disconnect()
        self.select_tree.clear()
        for joint_name in self.joint_names:
            item = QTreeWidgetItem(self.select_tree)
            item.setText(0, joint_name)
            item.setCheckState(0, Qt.Unchecked)
            item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            for measure_name in ['position', 'velocity', 'effort']:
                sub_item = QTreeWidgetItem(item)
                sub_item.setText(0, measure_name)
                sub_item.setCheckState(0, Qt.Unchecked)
                sub_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
        if self.joint_names:
            self.select_tree.itemChanged.connect(self.update_checkbox)

    def callback(self, msg):
        if self.pause_button.isChecked():
            self.plot_widget.set_autoscroll(False)
            return
        else:
            self.plot_widget.set_autoscroll(True)

        self._lock.acquire()

        new_joints_set = sorted(list(set(self.joint_names) | set(msg.name)))
        if self.joint_names != new_joints_set:
            self.joint_names = new_joints_set
            self.refresh_tree()

            for joint_name in msg.name:
                if joint_name not in self.time:
                    self.time[joint_name] = []
                if joint_name not in self.pos:
                    self.pos[joint_name] = []
                if joint_name not in self.vel:
                    self.vel[joint_name] = []
                if joint_name not in self.eff:
                    self.eff[joint_name] = []

        dt = msg.header.stamp.to_sec() - self._start_time

        for i, joint_name in enumerate(msg.name):
            self.time[joint_name].append(dt)

            if msg.position:
                self.pos[joint_name].append(msg.position[i])
            else:
                self.pos[joint_name].append(0.0)
            if msg.velocity:
                self.vel[joint_name].append(msg.velocity[i])
            else:
                self.vel[joint_name].append(0.0)
            if msg.effort:
                self.eff[joint_name].append(msg.effort[i])
            else:
                self.eff[joint_name].append(0.0)

        self._lock.release()

    def update_plot(self):
        curve_names = []
        data = {}
        data_list = [self.pos, self.vel, self.eff]
        measure_names = ['position', 'velocity', 'effort']

        for i in range(self.select_tree.topLevelItemCount()):
            joint_item = self.select_tree.topLevelItem(i)

            self._lock.acquire()
            joint_name = joint_item.text(0)
            for n in range(len(measure_names)):
                item = joint_item.child(n)
                if item.checkState(0):
                    curve_name = joint_name + ' ' + measure_names[n]
                    curve_names.append(curve_name)
                    new_data = data_list[n][joint_name]
                    data[curve_name] = (self.time[joint_name][-len(new_data):], new_data)

                    data_list[n][joint_name] = []
            self._lock.release()

        self.draw_curves.emit(curve_names, data)

    def update_checkbox(self, item, column):
        self.enable_timer(False)
        self.recursive_check(item)
        self.enable_timer()

    def recursive_check(self, item):
        check_state = item.checkState(0)
        for i in range(item.childCount()):
            item.child(i).setCheckState(0, check_state)
            self.recursive_check(item.child(i))

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
