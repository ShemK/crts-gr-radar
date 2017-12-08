#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Usrp Echotimer Dual Cw
# Generated: Fri Dec  8 16:41:18 2017
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import qtgui
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.qtgui import Range, RangeWidget
from optparse import OptionParser
import radar
import sip
import sys
from gnuradio import qtgui
from config_editor import ConfigEditor


class usrp_echotimer_dual_cw(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Usrp Echotimer Dual Cw")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Usrp Echotimer Dual Cw")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass

        self.configEditor = ConfigEditor()
        self.radar = self.configEditor.get_configuration('basic_radar.cfg')
        self.radar = self.radar['radar']
        
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "usrp_echotimer_dual_cw")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = self.radar['samp_rate']
        self.packet_len = packet_len = self.radar['packet_len']
        self.freq_res = freq_res = samp_rate/float(packet_len)
        self.freq = freq = self.radar['freq']
        self.center_freq = center_freq = self.radar['center_freq']
        self.v_res = v_res = freq_res*3e8/2/center_freq
        self.time_res = time_res = packet_len/float(samp_rate)
        self.threshold = threshold = -50
        self.samp_protect = samp_protect = 1
        self.range_time = range_time = 30
        self.range_res = range_res = 3e8/2/float((freq[1]-freq[0]))
        self.range_add = range_add = -1
        self.min_output_buffer = min_output_buffer = int(packet_len*2)
        self.max_output_buffer = max_output_buffer = 0
        self.gain_tx = gain_tx = self.radar['tx_gain']
        self.gain_rx = gain_rx = self.radar['rx_gain']
        self.delay_samp = delay_samp = self.radar['delay_samp']
        self.decim_fac = decim_fac = self.radar['decim_fac']
        self.amplitude = amplitude = 0.5

        ##################################################
        # Blocks
        ##################################################
        self._threshold_range = Range(-200, 100, 1, -50, 200)
        self._threshold_win = RangeWidget(self._threshold_range, self.set_threshold, 'Find peak threshold', "counter_slider", float)
        self.top_grid_layout.addWidget(self._threshold_win, 1,0)
        self._samp_protect_range = Range(0, 10, 1, 1, 200)
        self._samp_protect_win = RangeWidget(self._samp_protect_range, self.set_samp_protect, 'Find peak protected samples', "counter_slider", float)
        self.top_grid_layout.addWidget(self._samp_protect_win, 1,1)
        self._range_add_range = Range(-range_res, range_res, 0.1, -1, 200)
        self._range_add_win = RangeWidget(self._range_add_range, self.set_range_add, 'Add range', "counter_slider", float)
        self.top_grid_layout.addWidget(self._range_add_win, 2,1)
        self._gain_tx_range = Range(0, 100, 1, 40, 200)
        self._gain_tx_win = RangeWidget(self._gain_tx_range, self.set_gain_tx, 'TX gain', "counter_slider", float)
        self.top_grid_layout.addWidget(self._gain_tx_win, 0,0)
        self._gain_rx_range = Range(0, 100, 1, 20, 200)
        self._gain_rx_win = RangeWidget(self._gain_rx_range, self.set_gain_rx, 'RX gain', "counter_slider", float)
        self.top_grid_layout.addWidget(self._gain_rx_win, 0,1)
        self._delay_samp_range = Range(0, 100, 1, 28, 200)
        self._delay_samp_win = RangeWidget(self._delay_samp_range, self.set_delay_samp, 'Number delay samples', "counter_slider", float)
        self.top_grid_layout.addWidget(self._delay_samp_win, 2,0)
        self.rational_resampler_xxx_0_0 = filter.rational_resampler_ccc(
                interpolation=1,
                decimation=decim_fac,
                taps=None,
                fractional_bw=None,
        )
        self.rational_resampler_xxx_0 = filter.rational_resampler_ccc(
                interpolation=1,
                decimation=decim_fac,
                taps=None,
                fractional_bw=None,
        )
        self.radar_usrp_echotimer_cc_0 = radar.usrp_echotimer_cc(samp_rate, center_freq, int(delay_samp), self.radar['tx_addr'], '', self.radar['tx_clock_source'], \
                                            self.radar['tx_time_source'], self.radar['tx_antenna'], gain_tx, self.radar['tx_timeout'], 0.05, self.radar['tx_lo_offset'], \
                                                self.radar['rx_addr'], '',self.radar['rx_clock_source'], self.radar['rx_time_source'], self.radar['rx_antenna'], gain_rx, \
                                                    self.radar['rx_timeout'], 0.05, self.radar['rx_lo_offset'], "packet_len")
        (self.radar_usrp_echotimer_cc_0).set_min_output_buffer(4194304)
        self.radar_ts_fft_cc_0_0 = radar.ts_fft_cc(packet_len/decim_fac,  "packet_len")
        (self.radar_ts_fft_cc_0_0).set_min_output_buffer(4194304)
        self.radar_ts_fft_cc_0 = radar.ts_fft_cc(packet_len/decim_fac,  "packet_len")
        (self.radar_ts_fft_cc_0).set_min_output_buffer(4194304)
        self.radar_signal_generator_cw_c_0_0 = radar.signal_generator_cw_c(packet_len, samp_rate, (freq[1], ), amplitude, "packet_len")
        (self.radar_signal_generator_cw_c_0_0).set_min_output_buffer(4194304)
        self.radar_signal_generator_cw_c_0 = radar.signal_generator_cw_c(packet_len, samp_rate, (freq[0], ), amplitude, "packet_len")
        (self.radar_signal_generator_cw_c_0).set_min_output_buffer(4194304)
        self.radar_qtgui_time_plot_1 = radar.qtgui_time_plot(100, 'velocity', (-3,3), range_time, '')
        self.radar_qtgui_time_plot_0 = radar.qtgui_time_plot(100, 'range', (0,range_res), range_time, '')
        self.radar_print_results_0 = radar.print_results(False, "")
        self.radar_msg_manipulator_0 = radar.msg_manipulator(('range',), (range_add, ), (1, ))
        self.radar_find_max_peak_c_0 = radar.find_max_peak_c(samp_rate/decim_fac, threshold, int(samp_protect), ((-300,300)), True, "packet_len")
        self.radar_estimator_fsk_0 = radar.estimator_fsk(center_freq, (freq[1]-freq[0]), False)
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
        	packet_len/decim_fac, #size
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	0, #fc
        	samp_rate/decim_fac, #bw
        	'QT GUI Plot', #name
        	2 #number of inputs
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-140, 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(False)

        if not True:
          self.qtgui_freq_sink_x_0.disable_legend()

        if "complex" == "float" or "complex" == "msg_float":
          self.qtgui_freq_sink_x_0.set_plot_pos_half(not True)

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(2):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_0_win)
        self.blocks_tagged_stream_multiply_length_0_0 = blocks.tagged_stream_multiply_length(gr.sizeof_gr_complex*1, "packet_len", 1.0/float(decim_fac))
        (self.blocks_tagged_stream_multiply_length_0_0).set_min_output_buffer(4194304)
        self.blocks_tagged_stream_multiply_length_0 = blocks.tagged_stream_multiply_length(gr.sizeof_gr_complex*1, "packet_len", 1.0/float(decim_fac))
        (self.blocks_tagged_stream_multiply_length_0).set_min_output_buffer(4194304)
        self.blocks_multiply_conjugate_cc_1 = blocks.multiply_conjugate_cc(1)
        (self.blocks_multiply_conjugate_cc_1).set_min_output_buffer(4194304)
        self.blocks_multiply_conjugate_cc_0_0 = blocks.multiply_conjugate_cc(1)
        (self.blocks_multiply_conjugate_cc_0_0).set_min_output_buffer(4194304)
        self.blocks_multiply_conjugate_cc_0 = blocks.multiply_conjugate_cc(1)
        (self.blocks_multiply_conjugate_cc_0).set_min_output_buffer(4194304)
        self.blocks_add_xx_1 = blocks.add_vcc(1)
        (self.blocks_add_xx_1).set_min_output_buffer(4194304)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.radar_estimator_fsk_0, 'Msg out'), (self.radar_msg_manipulator_0, 'Msg in'))
        self.msg_connect((self.radar_find_max_peak_c_0, 'Msg out'), (self.radar_estimator_fsk_0, 'Msg in'))
        self.msg_connect((self.radar_msg_manipulator_0, 'Msg out'), (self.radar_print_results_0, 'Msg in'))
        self.msg_connect((self.radar_msg_manipulator_0, 'Msg out'), (self.radar_qtgui_time_plot_0, 'Msg in'))
        self.msg_connect((self.radar_msg_manipulator_0, 'Msg out'), (self.radar_qtgui_time_plot_1, 'Msg in'))
        self.connect((self.blocks_add_xx_1, 0), (self.radar_usrp_echotimer_cc_0, 0))
        self.connect((self.blocks_multiply_conjugate_cc_0, 0), (self.rational_resampler_xxx_0, 0))
        self.connect((self.blocks_multiply_conjugate_cc_0_0, 0), (self.rational_resampler_xxx_0_0, 0))
        self.connect((self.blocks_multiply_conjugate_cc_1, 0), (self.radar_find_max_peak_c_0, 0))
        self.connect((self.blocks_tagged_stream_multiply_length_0, 0), (self.radar_ts_fft_cc_0, 0))
        self.connect((self.blocks_tagged_stream_multiply_length_0_0, 0), (self.radar_ts_fft_cc_0_0, 0))
        self.connect((self.radar_signal_generator_cw_c_0, 0), (self.blocks_add_xx_1, 0))
        self.connect((self.radar_signal_generator_cw_c_0, 0), (self.blocks_multiply_conjugate_cc_0, 1))
        self.connect((self.radar_signal_generator_cw_c_0_0, 0), (self.blocks_add_xx_1, 1))
        self.connect((self.radar_signal_generator_cw_c_0_0, 0), (self.blocks_multiply_conjugate_cc_0_0, 1))
        self.connect((self.radar_ts_fft_cc_0, 0), (self.blocks_multiply_conjugate_cc_1, 0))
        self.connect((self.radar_ts_fft_cc_0_0, 0), (self.blocks_multiply_conjugate_cc_1, 1))
        self.connect((self.radar_usrp_echotimer_cc_0, 0), (self.blocks_multiply_conjugate_cc_0, 0))
        self.connect((self.radar_usrp_echotimer_cc_0, 0), (self.blocks_multiply_conjugate_cc_0_0, 0))
        self.connect((self.rational_resampler_xxx_0, 0), (self.blocks_tagged_stream_multiply_length_0, 0))
        self.connect((self.rational_resampler_xxx_0, 0), (self.qtgui_freq_sink_x_0, 0))
        self.connect((self.rational_resampler_xxx_0_0, 0), (self.blocks_tagged_stream_multiply_length_0_0, 0))
        self.connect((self.rational_resampler_xxx_0_0, 0), (self.qtgui_freq_sink_x_0, 1))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "usrp_echotimer_dual_cw")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_time_res(self.packet_len/float(self.samp_rate))
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate/self.decim_fac)
        self.set_freq_res(self.samp_rate/float(self.packet_len))

    def get_packet_len(self):
        return self.packet_len

    def set_packet_len(self, packet_len):
        self.packet_len = packet_len
        self.set_time_res(self.packet_len/float(self.samp_rate))
        self.set_min_output_buffer(int(self.packet_len*2))
        self.set_freq_res(self.samp_rate/float(self.packet_len))

    def get_freq_res(self):
        return self.freq_res

    def set_freq_res(self, freq_res):
        self.freq_res = freq_res
        self.set_v_res(self.freq_res*3e8/2/self.center_freq)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.set_range_res(3e8/2/float((self.freq[1]-self.freq[0])))

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.set_v_res(self.freq_res*3e8/2/self.center_freq)

    def get_v_res(self):
        return self.v_res

    def set_v_res(self, v_res):
        self.v_res = v_res

    def get_time_res(self):
        return self.time_res

    def set_time_res(self, time_res):
        self.time_res = time_res

    def get_threshold(self):
        return self.threshold

    def set_threshold(self, threshold):
        self.threshold = threshold
        self.radar_find_max_peak_c_0.set_threshold(self.threshold)

    def get_samp_protect(self):
        return self.samp_protect

    def set_samp_protect(self, samp_protect):
        self.samp_protect = samp_protect
        self.radar_find_max_peak_c_0.set_samp_protect(int(self.samp_protect))

    def get_range_time(self):
        return self.range_time

    def set_range_time(self, range_time):
        self.range_time = range_time

    def get_range_res(self):
        return self.range_res

    def set_range_res(self, range_res):
        self.range_res = range_res

    def get_range_add(self):
        return self.range_add

    def set_range_add(self, range_add):
        self.range_add = range_add
        self.radar_msg_manipulator_0.set_const_add((self.range_add, ))

    def get_min_output_buffer(self):
        return self.min_output_buffer

    def set_min_output_buffer(self, min_output_buffer):
        self.min_output_buffer = min_output_buffer

    def get_max_output_buffer(self):
        return self.max_output_buffer

    def set_max_output_buffer(self, max_output_buffer):
        self.max_output_buffer = max_output_buffer

    def get_gain_tx(self):
        return self.gain_tx

    def set_gain_tx(self, gain_tx):
        self.gain_tx = gain_tx
        self.radar_usrp_echotimer_cc_0.set_tx_gain(self.gain_tx)

    def get_gain_rx(self):
        return self.gain_rx

    def set_gain_rx(self, gain_rx):
        self.gain_rx = gain_rx
        self.radar_usrp_echotimer_cc_0.set_rx_gain(self.gain_rx)

    def get_delay_samp(self):
        return self.delay_samp

    def set_delay_samp(self, delay_samp):
        self.delay_samp = delay_samp
        self.radar_usrp_echotimer_cc_0.set_num_delay_samps(int(self.delay_samp))

    def get_decim_fac(self):
        return self.decim_fac

    def set_decim_fac(self, decim_fac):
        self.decim_fac = decim_fac
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate/self.decim_fac)
        self.blocks_tagged_stream_multiply_length_0_0.set_scalar(1.0/float(self.decim_fac))
        self.blocks_tagged_stream_multiply_length_0.set_scalar(1.0/float(self.decim_fac))

    def get_amplitude(self):
        return self.amplitude

    def set_amplitude(self, amplitude):
        self.amplitude = amplitude


def main(top_block_cls=usrp_echotimer_dual_cw, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
