extern crate yaml_rust;

use std::env;
use std::fs::File;
use std::io::prelude::*;
use yaml_rust::{YamlLoader};

use nn::{NN, HaltCondition};
use std::time::{Duration, Instant};

fn main() {
    let path = env::current_dir().unwrap();
    let s = path.to_str().unwrap();
    let s1 = String::from(s);
    let path_to_src = s1 + "/../";
    println!("{}", path_to_src);

    // let mut file = File::open("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml").unwrap();
    let mut file = File::open(path_to_src + "RelaxedIK/Config/info_files/ur5_info.yaml").unwrap();
    let mut contents = String::new();
    let res = file.read_to_string(&mut contents).unwrap();
    // let mut file = File::open("/home/rakita/Desktop/test.jl");

    let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
    let doc = &docs[0];
    println!("{:?}", doc["joint_limits"][0][0].as_f64().unwrap());
    assert_eq!(doc["urdf_file_name"].as_str().unwrap(), "ur5.urdf");

    println!("{:?}", contents);
    // let mut contents = String::new();
    // file.read_to_string(&mut contents)?;


}
