import rospy
import yaml

def get_relaxedIK_yaml_obj(path_to_src):
    info_file_loaded = rospy.get_param('relaxedIK/info_file_loaded', default=False)
    if info_file_loaded:
        info_file_name = rospy.get_param('relaxedIK/loaded_info_file_name')
        info_file_path = path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name
        info_file = open(info_file_path, 'r')
        y = yaml.load(info_file)
        return y
    else:
        return None