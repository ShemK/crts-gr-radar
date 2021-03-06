#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Python Txrx
# Generated: Thu Dec  7 17:48:34 2017
##################################################

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser


class python_txrx(gr.top_block):

    def __init__(self, interface_name='tunCRTS', rx_server_address='192.168.1.56', rx_server_port='52002', tx_server_address='192.168.1.55', tx_server_port='52001'):
        gr.top_block.__init__(self, "Python Txrx")

        ##################################################
        # Parameters
        ##################################################
        self.interface_name = interface_name
        self.rx_server_address = rx_server_address
        self.rx_server_port = rx_server_port
        self.tx_server_address = tx_server_address
        self.tx_server_port = tx_server_port

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 32000

        ##################################################
        # Blocks
        ##################################################
        self.blocks_tuntap_pdu_0 = blocks.tuntap_pdu(interface_name, 10000, True)

    def get_interface_name(self):
        return self.interface_name

    def set_interface_name(self, interface_name):
        self.interface_name = interface_name

    def get_rx_server_address(self):
        return self.rx_server_address

    def set_rx_server_address(self, rx_server_address):
        self.rx_server_address = rx_server_address

    def get_rx_server_port(self):
        return self.rx_server_port

    def set_rx_server_port(self, rx_server_port):
        self.rx_server_port = rx_server_port

    def get_tx_server_address(self):
        return self.tx_server_address

    def set_tx_server_address(self, tx_server_address):
        self.tx_server_address = tx_server_address

    def get_tx_server_port(self):
        return self.tx_server_port

    def set_tx_server_port(self, tx_server_port):
        self.tx_server_port = tx_server_port

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate


def argument_parser():
    parser = OptionParser(usage="%prog: [options]", option_class=eng_option)
    parser.add_option(
        "", "--interface-name", dest="interface_name", type="string", default='tunCRTS',
        help="Set interface_name [default=%default]")
    parser.add_option(
        "", "--rx-server-address", dest="rx_server_address", type="string", default='192.168.1.56',
        help="Set rx_server_address [default=%default]")
    parser.add_option(
        "", "--rx-server-port", dest="rx_server_port", type="string", default='52002',
        help="Set rx_server_port [default=%default]")
    parser.add_option(
        "", "--tx-server-address", dest="tx_server_address", type="string", default='192.168.1.55',
        help="Set tx_server_address [default=%default]")
    parser.add_option(
        "", "--tx-server-port", dest="tx_server_port", type="string", default='52001',
        help="Set tx_server_port [default=%default]")
    return parser


def main(top_block_cls=python_txrx, options=None):
    if options is None:
        options, _ = argument_parser().parse_args()

    tb = top_block_cls(interface_name=options.interface_name, rx_server_address=options.rx_server_address, rx_server_port=options.rx_server_port, tx_server_address=options.tx_server_address, tx_server_port=options.tx_server_port)
    tb.start()
    tb.wait()


if __name__ == '__main__':
    main()
