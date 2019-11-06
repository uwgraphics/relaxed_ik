use std::fs::File;
use std::io::prelude::*;
use yaml_rust::{YamlLoader, Yaml};
use nalgebra;
use nalgebra::{DMatrix, DVector};

pub fn get_yaml_obj(fp: String) -> Vec<Yaml> {
    let mut file = File::open(fp.as_str()).unwrap();
    let mut contents = String::new();
    let res = file.read_to_string(&mut contents).unwrap();

    let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
    docs
}

pub struct InfoFileParser {
    pub urdf_file_name: String,
    pub fixed_frame: String,
    pub joint_names: Vec<Vec<String>>,
    pub joint_ordering: Vec<String>,
    pub ee_fixed_joints: Vec<String>,
    pub starting_config: Vec<f64>,
    pub collision_file_name: String,
    pub collision_nn_file: String,
    pub path_to_src: String,
    pub axis_types: Vec<Vec<String>>,
    pub velocity_limits: Vec<f64>,
    pub joint_limits: Vec< [f64; 2] >,
    pub displacements: Vec<Vec<nalgebra::Vector3<f64>>>,
    pub disp_offsets: Vec<nalgebra::Vector3<f64>>,
    pub rot_offsets: Vec<Vec<Vec<f64>>>,
    pub joint_types: Vec<Vec<String>>,
    pub joint_state_define_func_file: String,
}

impl InfoFileParser {
    pub fn from_yaml_path(fp: String) -> InfoFileParser {
        let docs = get_yaml_obj(fp);
        let doc = &docs[0];

        let urdf_file_name = String::from(doc["urdf_file_name"].as_str().unwrap());
        let fixed_frame = String::from(doc["fixed_frame"].as_str().unwrap());
        let mut joint_names: Vec<Vec<String>> = Vec::new();
        let mut joint_ordering: Vec<String> = Vec::new();
        let mut ee_fixed_joints: Vec<String> = Vec::new();
        let mut starting_config: Vec<f64> = Vec::new();
        let collision_file_name = String::from(doc["collision_file_name"].as_str().unwrap());
        let collision_nn_file = String::from(doc["collision_nn_file"].as_str().unwrap());
        let path_to_src = String::from(doc["path_to_src"].as_str().unwrap());
        let mut axis_types: Vec<Vec<String>> = Vec::new();
        let mut velocity_limits: Vec<f64> = Vec::new();
        let mut joint_limits: Vec<[ f64; 2] > = Vec::new();
        let mut displacements: Vec<Vec<nalgebra::Vector3<f64>>> = Vec::new();
        let mut disp_offsets: Vec<nalgebra::Vector3<f64>> = Vec::new();
        let mut rot_offsets: Vec<Vec<Vec<f64>>> = Vec::new();
        let mut joint_types: Vec<Vec<String>> = Vec::new();
        let joint_state_define_func_file = String::from(doc["joint_state_define_func_file"].as_str().unwrap() );

        let joint_names_arr = doc["joint_names"].as_vec().unwrap();
        for i in 0..joint_names_arr.len() {
            let mut joint_names2: Vec<String> = Vec::new();
            joint_names.push(joint_names2);
            let joint_names_arr2 = joint_names_arr[i].as_vec().unwrap();
            for j in 0..joint_names_arr2.len() {
                joint_names[i].push( String::from(joint_names_arr2[j].as_str().unwrap()) );
            }
        }

        let joint_ordering_arr = doc["joint_ordering"].as_vec().unwrap();
        for i in 0..joint_ordering_arr.len() {
            joint_ordering.push( String::from(joint_ordering_arr[i].as_str().unwrap()) );
        }

        let ee_fixed_joints_arr = doc["ee_fixed_joints"].as_vec().unwrap();
        for i in 0..ee_fixed_joints_arr.len() {
            ee_fixed_joints.push( String::from(ee_fixed_joints_arr[i].as_str().unwrap()) );
        }

        let starting_config_arr = doc["starting_config"].as_vec().unwrap();
        for i in 0..starting_config_arr.len() {
            starting_config.push(starting_config_arr[i].as_f64().unwrap());
        }

        let axis_types_arr = doc["axis_types"].as_vec().unwrap();
        for i in 0..axis_types_arr.len() {
            let mut str_vec: Vec<String> = Vec::new();
            axis_types.push(str_vec);
            let axis_types_arr2 = axis_types_arr[i].as_vec().unwrap();
            for j in 0..axis_types_arr2.len() {
                axis_types[i].push(String::from(axis_types_arr2[j].as_str().unwrap() ) );
            }
        }

        let velocity_limits_arr = doc["velocity_limits"].as_vec().unwrap();
        for i in 0..velocity_limits_arr.len() {
            velocity_limits.push(velocity_limits_arr[i].as_f64().unwrap());
        }

        let joint_limits_arr = doc["joint_limits"].as_vec().unwrap();
        for i in 0..joint_limits_arr.len() {
            joint_limits.push( [ joint_limits_arr[i][0].as_f64().unwrap(), joint_limits_arr[i][1].as_f64().unwrap() ] )
        }

        let displacememts_arr = doc["displacements"].as_vec().unwrap();
        for i in 0..displacememts_arr.len() {
            let vec3_vec: Vec<nalgebra::Vector3<f64>> = Vec::new();
            displacements.push(vec3_vec);
            let displacememts_arr2 = displacememts_arr[i].as_vec().unwrap();
            for j in 0..displacememts_arr2.len() {
                displacements[i].push( nalgebra::Vector3::new( displacememts_arr2[j][0].as_f64().unwrap(), displacememts_arr2[j][1].as_f64().unwrap(), displacememts_arr2[j][2].as_f64().unwrap() ) )
            }
        }

        let disp_offsets_arr = doc["disp_offsets"].as_vec().unwrap();
        for i in 0..disp_offsets_arr.len() {
            disp_offsets.push( nalgebra::Vector3::new( disp_offsets_arr[i][0].as_f64().unwrap(), disp_offsets_arr[i][1].as_f64().unwrap(), disp_offsets_arr[i][2].as_f64().unwrap()  ) )
        }

        let rot_offsets_arr = doc["rot_offsets"].as_vec().unwrap();
        for i in 0..rot_offsets_arr.len() {
            let r: Vec<Vec<f64>> = Vec::new();
            rot_offsets.push(r);
            let rot_offsets_arr2 = rot_offsets_arr[i].as_vec().unwrap();
            for j in 0..rot_offsets_arr2.len() {
                rot_offsets[i].push( vec![ rot_offsets_arr2[j][0].as_f64().unwrap(), rot_offsets_arr2[j][1].as_f64().unwrap(), rot_offsets_arr2[j][2].as_f64().unwrap() ] )
            }
        }

        let joint_types_arr = doc["joint_types"].as_vec().unwrap();
        for i in 0..joint_types_arr.len() {
            let str_vec: Vec<String> = Vec::new();
            joint_types.push(str_vec);
            let joint_types_arr2 = joint_types_arr[i].as_vec().unwrap();
            for j in 0..joint_types_arr2.len() {
                joint_types[i].push(String::from(joint_types_arr2[j].as_str().unwrap() ) );
            }
        }


        InfoFileParser{urdf_file_name, fixed_frame, joint_names, joint_ordering, ee_fixed_joints, starting_config, collision_file_name, collision_nn_file, path_to_src, axis_types, velocity_limits,
            joint_limits, displacements, disp_offsets, rot_offsets, joint_types, joint_state_define_func_file}
    }
}


pub struct CollisionFileParser {
    pub states: Vec<Vec<f64>>,
    pub jt_pts: Vec<Vec<f64>>,
    pub collision_scores: Vec<f64>
}

impl CollisionFileParser {
    pub fn from_yaml_path(fp: String) -> Self {
        let docs = get_yaml_obj(fp);
        let doc = &docs[0];

        let mut states: Vec<Vec<f64>> = Vec::new();
        let mut jt_pts: Vec<Vec<f64>> = Vec::new();
        let mut collision_scores: Vec<f64> = Vec::new();

        let mut states_arr = doc["states"].as_vec().unwrap();
        for i in 0..states_arr.len() {
            let s: Vec<f64> = Vec::new();
            states.push(s);
            let states_arr_2 = states_arr[i].as_vec().unwrap();
            for j in 0..states_arr_2.len() {
                states[i].push( states_arr_2[j].as_f64().unwrap() )
            }
        }

        let jt_pts_arr = doc["jt_pts"].as_vec().unwrap();
        for i in 0..jt_pts_arr.len() {
            let s: Vec<f64> = Vec::new();
            jt_pts.push(s);
            let jt_pts_arr_2 = jt_pts_arr[i].as_vec().unwrap();
            for j in 0..jt_pts_arr_2.len() {
                jt_pts[i].push(jt_pts_arr_2[j].as_f64().unwrap());
            }
        }

        let collision_values_arr = doc["collision_scores"].as_vec().unwrap();
        for i in 0..collision_values_arr.len() {
            collision_scores.push(collision_values_arr[i].as_f64().unwrap());
        }

        CollisionFileParser{states, jt_pts, collision_scores}
    }
}



pub struct CollisionNeuralNetParser {
    pub coefs: Vec<Vec<Vec<f64>>>,
    pub intercepts: Vec<Vec<f64>>,
    pub coef_matrices: Vec<DMatrix<f64>>,
    pub intercept_vectors: Vec<DMatrix<f64>>
}

impl CollisionNeuralNetParser {
    pub fn from_yaml_path(fp: String) -> Self {
        let docs = get_yaml_obj(fp);
        let doc = &docs[0];

        let mut coefs: Vec<Vec<Vec<f64>>> = Vec::new();
        let mut intercepts: Vec<Vec<f64>> = Vec::new();
        let mut coef_matrices: Vec<DMatrix<f64>> = Vec::new();
        let mut intercept_vectors: Vec<DMatrix<f64>> = Vec::new();

        coefs = parse_list_of_floats_3(&doc["coefs"]);
        intercepts = parse_list_of_floats_2(&doc["intercepts"]);

        for i in 0..coefs.len() {
            let mut m = DMatrix::from_element( coefs[i].len(), coefs[i][0].len(), 0.0 );
            for j in 0..coefs[i].len() {
                for k in 0..coefs[i][j].len() {
                    m[(j,k)] = coefs[i][j][k];
                }
            }
            coef_matrices.push(m);
        }

        for i in 0..intercepts.len() {
            let mut v = DMatrix::from_element(1, intercepts[i].len(),0.0);
            for j in 0..intercepts[i].len() {
                v[j] = intercepts[i][j];
            }
            intercept_vectors.push(v);
        }

        Self{coefs, intercepts, coef_matrices, intercept_vectors}
    }
}


pub fn parse_list_of_floats_1(y: &Yaml) -> Vec<f64> {
    let mut ret: Vec<f64> = Vec::new();

    let v1 = y.as_vec().unwrap();
    for i in 0..v1.len() {
        ret.push(v1[i].as_f64().unwrap());
    }
    ret
}

pub fn parse_list_of_floats_2(y: &Yaml) -> Vec<Vec<f64>> {
    let mut ret: Vec<Vec<f64>> = Vec::new();

    let v1 = y.as_vec().unwrap();
    for i in 0..v1.len() {
        ret.push( parse_list_of_floats_1(&v1[i]) );
    }
    ret
}

pub fn parse_list_of_floats_3(y: &Yaml) -> Vec<Vec<Vec<f64>>> {
    let mut ret: Vec<Vec<Vec<f64>>> = Vec::new();

    let v1 = y.as_vec().unwrap();
    for i in 0..v1.len() {
        ret.push( parse_list_of_floats_2(&v1[i]) );
    }
    ret
}

pub fn parse_list_of_floats_4(y: &Yaml) -> Vec<Vec<Vec<Vec<f64>>>> {
    let mut ret: Vec<Vec<Vec<Vec<f64>>>> = Vec::new();

    let v1 = y.as_vec().unwrap();
    for i in 0..v1.len() {
        ret.push( parse_list_of_floats_3(&v1[i]) );
    }
    ret
}