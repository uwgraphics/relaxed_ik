import yaml
import os

def __get_collision_file_yaml(info_file_yaml):
    collision_file_name = info_file_yaml['collision_file_name']
    fp = path_to_src + '/RelaxedIK/Config/collision_files/' + collision_file_name
    collision_file = open(fp)
    y = yaml.load(collision_file)
    return y

def __get_robot_link_radius_string(collision_file_yaml):
    out_str = 'robot_link_radius: '
    out_str += '{}'.format(collision_file_yaml['robot_link_radius'])
    return out_str

def __get_boxes_string(collision_file_yaml, info_file_yaml):
    out_str = 'boxes: '
    if collision_file_yaml['boxes'] == None:
        return out_str
    else:
        for i in xrange(len(collision_file_yaml['boxes'])):
            out_str += __get_box_string(collision_file_yaml, info_file_yaml, i)

    return out_str

def __get_box_string(collision_file_yaml, info_file_yaml, idx):
    box = collision_file_yaml['boxes'][idx]
    out_str = '\n'
    out_str += '  - name: ' + box['name'] + '\n'
    p = box['parameters']
    out_str += '    parameters: [{}, {}, {}]'.format(p[0]/2.,p[1]/2.,p[2]/2.) + '\n'
    out_str += '    coordinate_frame: {}'.format(__get_coordinate_frame_string(info_file_yaml, box['coordinate_frame'])) + '\n'
    r = box['rotation']
    out_str += '    rotation: [{}, {}, {}]'.format(float(r[0]), float(r[1]), float(r[2])) + '\n'
    t = box['translation']
    out_str += '    translation: [{}, {}, {}]'.format(float(t[0]), float(t[1]), float(t[2]))
    return out_str

def __get_spheres_string(collision_file_yaml, info_file_yaml):
    out_str = 'spheres: '
    if collision_file_yaml['spheres'] == None:
        return out_str
    else:
        for i in xrange(len(collision_file_yaml['spheres'])):
            out_str += __get_sphere_string(collision_file_yaml, info_file_yaml, i)

    return out_str

def __get_sphere_string(collision_file_yaml, info_file_yaml, idx):
    sphere = collision_file_yaml['spheres'][idx]
    out_str = '\n'
    out_str += '  - name: ' + sphere['name'] + '\n'
    p = sphere['parameters']
    out_str += '    parameters: {}'.format(p) + '\n'
    out_str += '    coordinate_frame: {}'.format(__get_coordinate_frame_string(info_file_yaml, sphere['coordinate_frame'])) + '\n'
    r = sphere['rotation']
    out_str += '    rotation: [{}, {}, {}]'.format(float(r[0]), float(r[1]), float(r[2])) + '\n'
    t = sphere['translation']
    out_str += '    translation: [{}, {}, {}]'.format(float(t[0]), float(t[1]), float(t[2]))
    return out_str

def __get_coordinate_frame_string(info_file_yaml, idx):
    if idx == 0:
        return 'static'

    joint_ordering = info_file_yaml['joint_ordering']
    if idx > len(joint_ordering):
        return joint_ordering[-1]

    else:
        return joint_ordering[idx - 1]

def __output_collision_file(path_to_src, info_file_yaml, out_str):
    collision_file_name = info_file_yaml['collision_file_name']
    fp = path_to_src + '/RelaxedIK/Config/collision_files_rust/' + collision_file_name
    out_file = open(fp, 'w')
    out_file.write(out_str)

def transfer_collision_file(path_to_src, info_file_yaml):
    collision_file_yaml = __get_collision_file_yaml(info_file_yaml)
    out_str = ''
    out_str += __get_robot_link_radius_string(collision_file_yaml) + '\n'
    out_str += __get_boxes_string(collision_file_yaml, info_file_yaml) + '\n'
    out_str += __get_spheres_string(collision_file_yaml, info_file_yaml) + '\n'

    print out_str
    __output_collision_file(path_to_src, info_file_yaml, out_str)

def transfer_from_loaded_info_file(path_to_src):
    fp = path_to_src + '/RelaxedIK/Config/loaded_robot'
    loaded_robot = open(fp).readline()
    info_file_fp = path_to_src + '/RelaxedIK/Config/info_files/' + loaded_robot
    info_file_yaml = yaml.load(open(info_file_fp))
    transfer_collision_file(path_to_src, info_file_yaml)

def transfer_from_info_file(path_to_src, info_file_name):
    info_file_fp = path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name
    info_file_yaml = yaml.load(open(info_file_fp))
    transfer_collision_file(path_to_src, info_file_yaml)



if __name__ == '__main__':
    path_to_src = os.path.dirname(__file__)
    transfer_from_info_file(path_to_src, 'sawyer_info.yaml')


