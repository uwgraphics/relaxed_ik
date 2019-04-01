import numpy as np
from colors import bcolors
from os import listdir
from sklearn.externals import joblib
from os.path import isfile, join
from neural_net_trainer import Collision_NN_Trainer
import os


class Config_Engine:
    def __init__(self, collision_graph, vars, path_to_src, nn_file_name, config_fn='', override=False):
        self.collision_graph = collision_graph
        self.config_fn = config_fn
        self.path_to_src = path_to_src
        self.nn_file_name = nn_file_name
        self.vars = vars
        # dirname = os.path.dirname(__file__)
        # self.path = os.path.join(dirname, '../Config/')
        self.path = path_to_src + '/RelaxedIK/Config/collision_nn_python/'

        if override:
            self.robot_name, self.collision_nn, self.init_state, self.full_joint_lists, self.fixed_ee_joints, \
               self.joint_order, self.urdf_path, self.collision_file = self.generate_config_file()
        else:
            config_file = self.check_for_config_file()
            if config_file == None:
                self.robot_name, self.collision_nn, self.init_state, self.full_joint_lists, self.fixed_ee_joints, \
                self.joint_order, self.urdf_path, self.collision_file = self.generate_config_file()

            else:
                # self.collision_file = joblib.load(self.path + config_file)
                # self.robot_name = self.config_data[0]
                self.collision_nn = joblib.load(self.path + config_file)
                # self.init_state = self.config_data[2]
                # self.full_joint_lists = self.config_data[3]
                # self.fixed_ee_joints = self.config_data[4]
                # self.joint_order = self.config_data[5]
                # self.urdf_path = self.config_data[6]
                # self.collision_file = self.config_data[7]

    def check_for_config_file(self):
        files = [f for f in listdir(self.path) if isfile(join(self.path, f))]

        for f in files:
            if f == self.nn_file_name:
                return f

        response = raw_input(bcolors.OKBLUE + 'Config file not found, generating a new one!  This will take some time.  Continue?  (y or n): ' + bcolors.ENDC)
        if response == 'y':
            return None
        else:
            exit(1)

    def generate_config_file(self):
        trainer = Collision_NN_Trainer(self.collision_graph)
        collision_nn = trainer.clf
        # robot_name = trainer.robot.__name__
        robot_name = 'robot'

        # file_vars = [robot_name, collision_nn, self.vars.init_state, self.vars.full_joint_lists, self.vars.fixed_ee_joints, \
        #        self.vars.joint_order, self.vars.urdf_path, self.vars.collision_file]

        print self.nn_file_name
        joblib.dump(collision_nn, self.path + self.nn_file_name)

        return robot_name, collision_nn, self.vars.init_state, self.vars.full_joint_lists, self.vars.fixed_ee_joints, \
               self.vars.joint_order, self.vars.urdf_path, self.vars.collision_file

if __name__ == '__main__':
    ce = Config_Engine()
    print ce.check_for_config_file()
