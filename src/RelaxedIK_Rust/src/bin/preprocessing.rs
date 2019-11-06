use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::fs::read_dir;
pub mod lib;
use lib::utils_rust::file_utils;
use lib::{utils_rust::yaml_utils};
use nn::{NN, HaltCondition};
extern crate rand;
use rand::distributions::{IndependentSample, Range};
extern crate time;
use time::Duration;


fn package_examples(cfp: &yaml_utils::CollisionFileParser) -> Vec<(Vec<f64>, Vec<f64>)> {
    let mut out: Vec<(Vec<f64>, Vec<f64>)> = Vec::new();
    for i in 0..cfp.states.len() {
        out.push((cfp.jt_pts[i].clone(), vec![cfp.collision_scores[i].clone() ]));
    }
    out
}

fn get_examples_from_multiple_files(all_files: &Vec<String>, num_files: usize, top_dir: &String) -> Vec<(Vec<f64>, Vec<f64>)> {
    let mut out: Vec<(Vec<f64>, Vec<f64>)> = Vec::new();
    let total_num_files = all_files.len();
    let between = Range::new(0usize, total_num_files-1);
    let mut rng = rand::thread_rng();

    for i in 0..num_files {
        let a = between.ind_sample(&mut rng);
        println!("loading in file {}, rng {}", i, a);
        let cfp = yaml_utils::CollisionFileParser::from_yaml_path(top_dir.clone() + "/" + all_files[a].as_str());
        let examples = package_examples(&cfp);
        for j in 0..examples.len() {
            out.push(examples[j].clone());
        }
    }

    out
}

fn network_exists(ifp: &yaml_utils::InfoFileParser) -> bool {
    let path_to_src = file_utils::get_path_to_src();
    let all_net_files = file_utils::get_all_files_in_directory(path_to_src.clone() + "RelaxedIK/Config/collision_nn_rust");
    let mut out = false;

    for i in 0..all_net_files.len() {
        if all_net_files[i] == ifp.collision_nn_file.to_string() {
            out = true;
        }
    }

    out
}

fn load_network(ifp: &yaml_utils::InfoFileParser) -> NN {
    let path_to_src = file_utils::get_path_to_src();
    let contents = file_utils::get_file_contents(path_to_src + "RelaxedIK/Config/collision_nn_rust/" + ifp.collision_nn_file.as_str());
    NN::from_json(contents.as_str())
}

fn save_network(net: &NN, ifp: &yaml_utils::InfoFileParser) {
    let path_to_src = file_utils::get_path_to_src();
    let mut file = File::create(path_to_src + "RelaxedIK/Config/collision_nn_rust/" + ifp.collision_nn_file.as_str()).unwrap();
    file.write(net.to_json().as_bytes());
}

fn main() {
    let path_to_src = file_utils::get_path_to_src();
    let info_file_name = file_utils::get_file_contents(path_to_src.clone() + "RelaxedIK/Config/loaded_robot");
    let ifp = yaml_utils::InfoFileParser::from_yaml_path(path_to_src.clone() + "RelaxedIK/Config/info_files/" + info_file_name.as_str());
    let urdf_file_name = ifp.urdf_file_name.clone();
    let vec: Vec<&str> = urdf_file_name.split(".").collect();
    let robot_name = vec[0].to_string();

    let top_dir = path_to_src.clone() + "RelaxedIK/Config/collision_inputs_and_outputs/" + robot_name.as_str();
    let all_files = file_utils::get_all_files_in_directory(top_dir.clone());

    let cfp = yaml_utils::CollisionFileParser::from_yaml_path(top_dir.clone() + "/" + all_files[0].as_str());
    let input_length = cfp.jt_pts[0].len() as u32;

    let mut net = NN::new(&[input_length, 30, 20, 1]);
    if network_exists(&ifp) {
        net = load_network(&ifp);
    }

    let examples = get_examples_from_multiple_files(&all_files, 100, &top_dir);
    println!("{:?}", examples.len());


    net.train(examples.as_slice())
    .halt_condition(HaltCondition::Timer(Duration::minutes(1)))
    .log_interval(Some(1))
    .momentum(0.0001)
    .rate(1000.0)
    .go();

    save_network(&net, &ifp);


    let res = net.run(examples[0].0.as_slice());
    println!("{:?}", res);
    println!("{:?}", examples[0].1);
    
}