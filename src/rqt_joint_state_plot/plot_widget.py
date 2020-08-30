from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from rqt_plot.data_plot import DataPlot


class PlotWidget(QWidget):
    def __init__(self, parent=None):
        super(PlotWidget, self).__init__(parent)
        # create widgets
        self._data_plot = DataPlot(self)

        # disable autoscaling of X, and set a sane default range
        self._data_plot.set_autoscale(x=False)
        self._data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND | DataPlot.SCALE_VISIBLE)
        self._data_plot.autoscroll(True)
        self._data_plot.set_xlim([0, 10.0])

        vbox = QVBoxLayout()
        vbox.addWidget(self._data_plot)
        self.setLayout(vbox)

        # Set of current topics
        self._current_curves = set()

    def set_autoscroll(self, autoscroll):
        self._data_plot.autoscroll(autoscroll)

    def draw_curves(self, curve_names, data):
        current_curves = self._current_curves.copy()
        for name in current_curves:
            if name not in curve_names:
                self._data_plot.remove_curve(name)
                self._current_curves.remove(name)

        for name in curve_names:
            t, v = data[name]
            if name in self._current_curves:
                self._data_plot.update_values(name, t, v)
            else:
                self._data_plot.add_curve(name, name, t, v)
                self._current_curves.add(name)
        self._data_plot.redraw()

    def remove_topic(self, name):
        self._current_curves.remove(name)
        self._data_plot.remove_curve(name)

    def clear_plot(self):
        for name in self._current_curves:
            self._data_plot.clear_values(name)
            self._data_plot.remove_curve(name)

        self._current_curves.clear()
        self._data_plot.redraw()
