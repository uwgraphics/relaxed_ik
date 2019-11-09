use rosrust;
use std::thread;
use std::sync::mpsc::channel;
use std::sync::mpsc::{Sender, Receiver};

mod msg {
    rosrust::rosmsg_include!(std_msgs/UInt64);
}


pub struct SubTest {
    pub val: u64
}
impl SubTest {
    pub fn new(val: u64) -> Self {
        Self{val: val}
    }
}

pub struct RosChannel {
    tx: Sender<u64>,
    rx: Receiver<u64>,
}
impl RosChannel {
    pub fn new() -> Self {
        let (tx, rx) = channel();
        Self{tx, rx}
    }

    pub fn initialize(&self, topic_name: &str) {
        let _subscriber_raii = rosrust::subscribe(topic_name, 100, move |v: msg::std_msgs::UInt64| {
        // Callback for handling received messages
        // rosrust::ros_info!("Received: {}", v.data);
        // tx.send(SubTest::new(v.data)).unwrap();
        // &c.tx.send(SubTest::new(v.data)).unwrap();
        // tx.send(v.data).unwrap();
            self.tx.send(v.data).unwrap();
        });
    }
}


fn main() {
    // Initialize node
    rosrust::init("listener");

    let r = RosChannel::new();


    let mut idx = 0;
    while rosrust::is_ok() {
        // println!("{:?}", rx.recv().unwrap().val);
        // println!("{:?}", &c.rx.try_recv());
        // idx += 1;
        // println!("{:?}", idx);
        // let data = rx.try_recv();
        // if data.err().is_some() {
        // } else {
        //     println!("{:?}", data.unwrap());
        // }
    }
}