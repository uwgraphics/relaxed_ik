use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::fs::read_dir;


pub fn get_path_to_src() -> String {
    let path = env::current_dir().unwrap();
    let s = path.to_str().unwrap();
    let s1 = String::from(s);
    let path_to_src = s1 + "/../";
    path_to_src
}

pub fn get_file_contents(fp: String) -> String {
    let mut file = File::open(fp.as_str()).unwrap();
    let mut contents = String::new();
    let res = file.read_to_string(&mut contents).unwrap();
    contents
}


pub fn get_all_files_in_directory(fp: String) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    let it = read_dir(fp);
    for i in it.unwrap() {
        out.push(i.unwrap().file_name().into_string().unwrap());
    }
    out
}