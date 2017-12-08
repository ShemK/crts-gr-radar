import io,libconf
import socket
import struct
import fcntl
import os

class ConfigEditor(object):
    #   initialize ConfigEditor
    def __init__(self):
        #self.read_config_file('basic.cfg')
        self.output = dict()
        pass

    #   read a configuration file and return a dictionary
    #   if failure, return empty dict
    def read_config_file(self,file_path):
        try:
            with io.open(file_path) as f:
                input_config  = libconf.load(f)
                return input_config
        except Exception as e:
            input_config = dict()
            print 'Failed to read file ',e
            #raise
            return input_config

    #   read basic information that every node should have
    def get_configuration(self,config_file):
        config_path = os.path.dirname(os.path.abspath(__file__))
        config_path = config_path + '/'+config_file
        return self.read_config_file(config_path)




def main():
    configEditor = ConfigEditor()
    radar = configEditor.get_configuration('basic_radar.cfg')
    
if __name__ == '__main__':
    main()
