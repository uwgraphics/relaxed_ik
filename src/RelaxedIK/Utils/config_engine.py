import numpy as np
from colors import bcolors
from os import listdir
from sklearn.externals import joblib
from os.path import isfile, join
from neural_net_trainer import Collision_NN_Trainer
import os


class Config_Engine:
    def __init__(self, collision_graph, config_fn='', override=False):
        self.collision_graph = collision_graph
        self.config_fn = config_fn
        dirname = os.path.dirname(__file__)
        self.path = os.path.join(dirname, '../Config/')

        config_file = self.check_for_config_file()
        if config_file == None or override:
            self.robot_name, self.collision_nn = self.generate_config_file()


        else:
            self.config_data = joblib.load(self.path + config_file)
            self.robot_name = self.config_data[0]
            self.collision_nn = self.config_data[1]

    def check_for_config_file(self):
        files = [f for f in listdir(self.path) if isfile(join(self.path, f))]
        if self.config_fn in files:
            return self.config_fn

        elif 'ur5.config' in files:
            return 'ur5.config'

        for f in files:
            f_arr = f.split('.')
            ext = f_arr[-1]
            if ext == 'config':
                response = raw_input(bcolors.OKGREEN + 'Found saved config file ' + f + '.  Would you like to use this config file? (y or n): ' + bcolors.ENDC)
                if response == 'y':
                    return f
                else:
                    response = raw_input(bcolors.OKGREEN + 'Would you like to generate a new config file? (y or n): ' + bcolors.ENDC)
                    if response == 'y':
                        return None
                    else:
                        print bcolors.FAIL + 'Exiting.  Please manually specify which config file you would like to use on the next run.' + bcolors.ENDC
                        exit(1)

        response = raw_input(bcolors.OKBLUE + 'Config file not found, generating a new one!  This will take some time.  Continue?  (y or n): ' + bcolors.ENDC)
        if response == 'y':
            return None
        else:
            exit(1)

    def generate_config_file(self):
        trainer = Collision_NN_Trainer(self.collision_graph)
        collision_nn = trainer.clf
        robot_name = trainer.robot.__name__

        file_vars = [robot_name, collision_nn]

        joblib.dump(file_vars,self.path + 'ur5.config')

        return robot_name, collision_nn

if __name__ == '__main__':
    ce = Config_Engine()
    print ce.check_for_config_file()
