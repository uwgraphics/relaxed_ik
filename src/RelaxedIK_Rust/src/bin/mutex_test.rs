use std::sync::{Arc, Mutex};
use std::thread;
use std::sync::mpsc::channel;
use rosrust;

mod msg {
    rosrust::rosmsg_include!(std_msgs/UInt64);
}

pub struct T {
    a: u64
}

fn main() {
    let lock = Arc::new(Mutex::new(T{a: 0}));
    let c_lock = lock.clone();

    rosrust::init("listener");
    let _subscriber_raii = rosrust::subscribe("cc", 100, move |v: msg::std_msgs::UInt64| {
        let mut g = lock.lock().unwrap();
        g.a = v.data;
    });


    let r = rosrust::rate(10.);
    while rosrust::is_ok() {
        println!("{:?}", c_lock.lock().unwrap().a);
        r.sleep();
    }
}