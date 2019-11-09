use rosrust;

mod msg {
    rosrust::rosmsg_include!(std_msgs / String, relaxed_ik / EEPoseGoals, geometry_msgs / Pose);
}

fn main() {

}