import arm
import numpy as np
from ..Utils.transformations import quaternion_from_matrix

class Robot:
    def __init__(self, arms, full_joint_lists, joint_order):
        self.arms = arms
        self.full_joint_lists = full_joint_lists
        self.joint_order = joint_order

        self.numChains = len(self.arms)
        self.numDOF = len(self.joint_order)

        self.subchain_indices = []
        self.__initialize_subchain_indices()

        self.bounds = []
        self.__initialize_bounds()

        self.velocity_limits = []
        self.__initialize_velocity_limits()

        # self.__name__ = self.arms[0].__name__

    def __initialize_subchain_indices(self):
        for i in range(self.numChains):
            self.subchain_indices.append([])

        for i in range(self.numChains):
            for j in self.full_joint_lists[i]:
                idx = self.__get_index_from_joint_order(j)
                if not idx == None:
                    self.subchain_indices[i].append(idx)

    def __get_index_from_joint_order(self, jt_name):
        for j,joint in enumerate(self.joint_order):
            if jt_name == self.joint_order[j]:
                return j

        return None
        # raise Exception('Error in Robot Class.  It appears your full_joint_lists and joint_order lists are not congruent '
        #                'The joint {} was not found in joint_order.  Is there perhaps a misspelling?'.format(jt_name) )

    def __initialize_bounds(self):
        bounds = self.numDOF*[0.0]

        for i,a in enumerate(self.arms):
            sub_bounds = a.joint_limits
            for j,l in enumerate(sub_bounds):
                idx = self.subchain_indices[i][j]
                bounds[idx] = l

        self.bounds = bounds

    def __initialize_velocity_limits(self):
        velocity_limits = self.numDOF*[0.0]

        for i,a in enumerate(self.arms):
            sub_vl = a.velocity_limits
            for j,l in enumerate(sub_vl):
                idx = self.subchain_indices[i][j]
                velocity_limits[idx] = l

        self.velocity_limits = velocity_limits

    def split_state_into_subchains(self, x):
        subchains = []

        for i in xrange(self.numChains):
            subchain = []

            for s in self.subchain_indices[i]:
                subchain.append(x[s])

            subchains.append(subchain)

        return subchains

    def get_ee_positions(self, x):
        frames = self.getFrames(x)
        positions = []

        for f in frames:
            positions.append(f[0][-1])

        return positions

    def get_ee_rotations(self, x, quaternions=True):
        frames = self.getFrames(x)
        rotations = []

        for f in frames:
            if quaternions:
                rotations.append(quaternion_from_matrix(f[1][-1]))
            else:
                rotations.append(f[1][-1])

        return rotations

    def getFrames(self, x):
        all_frames = []

        chains = self.split_state_into_subchains(x)

        for i,c in enumerate(chains):
            frames = self.arms[i].getFrames(c)
            all_frames.append(frames)

        return all_frames

    def getMatrixConditioningMeasure(self, x):
        condition_measures = []
        subchains = self.split_state_into_subchains(x)
        for i in xrange(self.numChains):
            condition_measures.append(self.arms[i].getMatrixConditioningMeasure(subchains[i]))
        return np.min( np.array(condition_measures) )

    def getYoshikawaMeasure(self, x):
        yoshikawa_measures = []
        subchains = self.split_state_into_subchains(x)
        for i in xrange(self.numChains):
            yoshikawa_measures.append(self.arms[i].getYoshikawaMeasure(subchains[i]))
        return np.min( np.array(yoshikawa_measures) )

