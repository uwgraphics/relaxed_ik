use nalgebra;
use nalgebra::{UnitQuaternion, Unit};

#[derive(Clone, Debug)]
pub struct Arm {
    pub axis_types: Vec<String>,
    pub displacements: Vec<nalgebra::Vector3<f64>>,
    pub disp_offset: nalgebra::Vector3<f64>,
    pub pos_offsets: Vec<nalgebra::Vector3<f64>>,
    pub rot_offsets: Vec<Vec<f64>>,
    pub rot_offset_matrices: Vec<nalgebra::Matrix3<f64>>,
    pub rot_offset_quats: Vec<nalgebra::UnitQuaternion<f64>>,
    pub joint_types: Vec<String>,
    pub num_dof: usize,
    pub out_positions: Vec<nalgebra::Vector3<f64>>,
    pub out_rot_mats: Vec<nalgebra::Matrix3<f64>>,
    pub out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>>,
    __do_rot_offset: Vec<bool>,
    __is_prismatic: Vec<bool>,
    __is_revolute_or_continuous: Vec<bool>,
    __is_fixed: Vec<bool>,
    __is_x: Vec<bool>,
    __is_y: Vec<bool>,
    __is_z: Vec<bool>,
    __is_neg_x: Vec<bool>,
    __is_neg_y: Vec<bool>,
    __is_neg_z: Vec<bool>,
    __aux_matrix: nalgebra::Matrix3<f64>
}

impl Arm{
    pub fn new(axis_types: Vec<String>,
        displacements: Vec<nalgebra::Vector3<f64>>, disp_offset: nalgebra::Vector3<f64>,
        rot_offsets: Vec<Vec<f64>>, joint_types: Vec<String>) -> Arm {

        let num_dof = axis_types.len();

        let mut __do_rot_offset: Vec<bool> = Vec::new();
        for i in 0..rot_offsets.len() {
            if rot_offsets[i][0] == 0.0 && rot_offsets[i][1] == 0.0 && rot_offsets[i][2] == 0.0 {
                __do_rot_offset.push(false);
            } else {
                __do_rot_offset.push(true);
            }
        }

        let mut pos_offsets: Vec<nalgebra::Vector3<f64>> = Vec::new();
        pos_offsets.push(disp_offset.clone());
        for i in 0..displacements.len() {
            pos_offsets.push(displacements[i].clone());
        }

        let mut rot_offset_matrices: Vec<nalgebra::Matrix3<f64>> = Vec::new();
        let mut rot_offset_quats: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();
        for i in 0..rot_offsets.len() {
            rot_offset_matrices.push( euler_triple_to_3x3(&rot_offsets[i]) );
            let r = nalgebra::Rotation3::from_matrix_unchecked(rot_offset_matrices[i]);
            rot_offset_quats.push( UnitQuaternion::from_rotation_matrix(&r)  );
        }

        let mut out_positions: Vec<nalgebra::Vector3<f64>> = Vec::new();
        let mut out_rot_mats: Vec<nalgebra::Matrix3<f64>> = Vec::new();
        let mut out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();
        for i in 0..rot_offsets.len() {
            out_positions.push(nalgebra::Vector3::new(0.,0.,0.));
            out_rot_mats.push(nalgebra::Matrix3::identity());
            out_rot_quats.push(nalgebra::UnitQuaternion::identity());
        }
        out_positions[0] = disp_offset;
        out_rot_mats[0] = rot_offset_matrices[0];
        out_rot_quats[0] = rot_offset_quats[0];

        let mut __is_prismatic: Vec<bool> = Vec::new();
        let mut __is_revolute_or_continuous: Vec<bool> = Vec::new();
        let mut __is_fixed: Vec<bool> = Vec::new();
        for i in 0..joint_types.len() {
            if joint_types[i] == String::from("prismatic") {
                __is_prismatic.push(true);
                __is_revolute_or_continuous.push(false);
                __is_fixed.push(false);
            }
            else if joint_types[i] == String::from("continuous") || joint_types[i] == String::from("revolute") {
                __is_prismatic.push(false);
                __is_revolute_or_continuous.push(true);
                __is_fixed.push(false);
            }
            else if joint_types[i] == String::from("fixed")  {
                __is_prismatic.push(false);
                __is_revolute_or_continuous.push(false);
                __is_fixed.push(true);
            }
        }

        let __aux_matrix: nalgebra::Matrix3<f64> = nalgebra::Matrix3::identity();

        let mut __is_x: Vec<bool> = Vec::new();
        let mut __is_y: Vec<bool> = Vec::new();
        let mut __is_z: Vec<bool> = Vec::new();
        let mut __is_neg_x: Vec<bool> = Vec::new();
        let mut __is_neg_y: Vec<bool> = Vec::new();
        let mut __is_neg_z: Vec<bool> = Vec::new();
        for i in 0..axis_types.len() {
            __is_x.push(false);
            __is_y.push(false);
            __is_z.push(false);
            __is_neg_x.push(false);
            __is_neg_y.push(false);
            __is_neg_z.push(false);
            if axis_types[i] == String::from("X") || axis_types[i] == String::from("x") {
                __is_x[i] = true;
            }
            else if axis_types[i] == String::from("X") || axis_types[i] == String::from("x") {
                __is_x[i] = true;
            }
            else if axis_types[i] == String::from("Y") || axis_types[i] == String::from("y") {
                __is_y[i] = true;
            }
            else if axis_types[i] == String::from("Z") || axis_types[i] == String::from("z") {
                __is_z[i] = true;
            }
            else if axis_types[i] == String::from("-x"){
                __is_neg_x[i] = true;
            }
            else if axis_types[i] == String::from("-y"){
                __is_neg_y[i] = true;
            }
            else if axis_types[i] == String::from("-z"){
                __is_neg_z[i] = true;
            }
        }


        Arm{axis_types, displacements, disp_offset, pos_offsets, rot_offsets, rot_offset_matrices, rot_offset_quats,
            joint_types, num_dof, out_positions, out_rot_mats, out_rot_quats, __do_rot_offset, __is_prismatic,
            __is_revolute_or_continuous, __is_fixed, __is_x, __is_y, __is_z, __is_neg_x, __is_neg_y,
            __is_neg_z, __aux_matrix}
    }

    pub fn get_frames(&mut self, x: &[f64]) {
        let mut joint_idx: usize = 0;
        for i in 0..self.displacements.len() {

            if self.__is_revolute_or_continuous[i] || self.__is_prismatic[i] {
                self.__update_frames(i, x[joint_idx], self.__do_rot_offset[i + 1],
                                     self.__is_prismatic[i], self.__is_revolute_or_continuous[i],
                                     self.__is_fixed[i], self.__is_x[joint_idx], self.__is_y[joint_idx],
                                     self.__is_z[joint_idx], self.__is_neg_x[joint_idx],
                                     self.__is_neg_y[joint_idx], self.__is_neg_z[joint_idx]);
            } else {
                if self.__do_rot_offset[i+1] {
                    self.__update_fixed_ro(i)
                } else {
                    self.__update_fixed(i)
                }
            }

            if self.__is_revolute_or_continuous[i] || self.__is_prismatic[i] {
                joint_idx += 1;
            }
        }
    }

    pub fn get_frames_immutable(&self, x: &[f64]) -> (Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>) {
        let mut out_positions: Vec<nalgebra::Vector3<f64>> = Vec::new();
        let mut out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();

        let mut pt: nalgebra::Vector3<f64> = self.disp_offset.clone();
        let mut rot_quat = self.rot_offset_quats[0].clone();
        out_positions.push(pt);
        out_rot_quats.push(rot_quat);

        let mut joint_idx: usize = 0;
        for i in 0..self.displacements.len() {
            if self.__is_revolute_or_continuous[i] {
                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    rot_quat = rot_quat * get_quat_x(joint_val);
                } else if self.__is_y[joint_idx] {
                    rot_quat = rot_quat * get_quat_y(joint_val);
                } else if self.__is_z[joint_idx] {
                    rot_quat = rot_quat * get_quat_z(joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    rot_quat = rot_quat * get_quat_x(-joint_val);
                } else if self.__is_neg_y[joint_idx] {
                    rot_quat = rot_quat * get_quat_y(-joint_val);
                } else if self.__is_neg_z[joint_idx] {
                    rot_quat = rot_quat * get_quat_z(-joint_val);
                }

                pt = rot_quat * self.displacements[i] + pt;
                if self.__do_rot_offset[i+1] {
                    rot_quat = rot_quat * self.rot_offset_quats[i + 1];
                }
                out_positions.push(pt.clone());
                out_rot_quats.push(rot_quat.clone());

                joint_idx += 1;
            }
            else if self.__is_prismatic[i] {
                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(joint_val, 0., 0.);
                } else if self.__is_y[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., joint_val, 0.);
                } else if self.__is_z[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., 0., joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(-joint_val, 0., 0.);
                } else if self.__is_neg_y[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., -joint_val, 0.);
                } else if self.__is_neg_z[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., 0., -joint_val);
                }
                if self.__do_rot_offset[i+1] {
                    rot_quat = rot_quat * self.rot_offset_quats[i + 1];
                }
                out_positions.push(pt.clone());
                out_rot_quats.push(rot_quat.clone());
                joint_idx += 1;
            }
            else {
                pt = rot_quat * self.displacements[i] + pt;
                if self.__do_rot_offset[i+1] {
                    rot_quat = rot_quat * self.rot_offset_quats[i+1];
                }
                out_positions.push(pt.clone());
                out_rot_quats.push(rot_quat.clone());
            }
        }

        (out_positions, out_rot_quats)
    }

    pub fn get_ee_pos_and_quat_immutable(&self, x: &[f64]) -> (nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>) {
        let mut pt: nalgebra::Vector3<f64> = self.disp_offset.clone();
        let mut rot_quat = self.rot_offset_quats[0].clone();

        let mut joint_idx: usize = 0;
        for i in 0..self.displacements.len() {
            if self.__is_revolute_or_continuous[i] {
                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    rot_quat = rot_quat * get_quat_x(joint_val);
                } else if self.__is_y[joint_idx] {
                    rot_quat = rot_quat * get_quat_y(joint_val);
                } else if self.__is_z[joint_idx] {
                    rot_quat = rot_quat * get_quat_z(joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    rot_quat = rot_quat * get_quat_x(-joint_val);
                } else if self.__is_neg_y[joint_idx] {
                    rot_quat = rot_quat * get_quat_y(-joint_val);
                } else if self.__is_neg_z[joint_idx] {
                    rot_quat = rot_quat * get_quat_z(-joint_val);
                }

                pt = rot_quat * self.displacements[i] + pt;
                if self.__do_rot_offset[i+1] {
                    rot_quat = rot_quat * self.rot_offset_quats[i + 1];
                }

                joint_idx += 1;
            }
            else if self.__is_prismatic[i] {
                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(joint_val, 0., 0.);
                } else if self.__is_y[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., joint_val, 0.);
                } else if self.__is_z[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., 0., joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(-joint_val, 0., 0.);
                } else if self.__is_neg_y[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., -joint_val, 0.);
                } else if self.__is_neg_z[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., 0., -joint_val);
                }
                if self.__do_rot_offset[i+1] {
                    rot_quat = rot_quat * self.rot_offset_quats[i + 1];
                }
                joint_idx += 1;
            }
            else {
                pt = rot_quat * self.displacements[i] + pt;
                if self.__do_rot_offset[i+1] {
                    rot_quat = rot_quat * self.rot_offset_quats[i+1];
                }
            }
        }

        (pt, rot_quat)
    }

    pub fn get_ee_position(&mut self, x: &[f64]) -> nalgebra::Vector3<f64> {
        self.get_frames(x);
        let last_index = self.out_positions.len() - 1;
        self.out_positions[last_index].clone()
    }

    pub fn get_ee_rot_mat(&mut self, x: &[f64]) -> nalgebra::Matrix3<f64> {
        self.get_frames(x);
        let last_index = self.out_rot_mats.len() - 1;
        self.out_rot_mats[last_index].clone()
    }

    pub fn get_ee_quat(&mut self, x: &[f64]) -> nalgebra::UnitQuaternion<f64> {
        self.get_frames(x);
        let last_index = self.out_rot_quats.len() - 1;
        self.out_rot_quats[last_index].clone()
    }

    fn __update_frames(&mut self, i: usize, joint_val: f64, __do_rot_offset: bool, __is_prismatic: bool,
                       __is_revolute_or_continuous: bool, __is_fixed: bool, __is_x: bool, __is_y: bool,
                       __is_z: bool, __is_neg_x: bool, __is_neg_y: bool, __is_neg_z: bool) {
        if __is_prismatic {
            if __do_rot_offset {
                self.__update_prismatic_ro(i, joint_val, __is_x, __is_y, __is_z, __is_neg_x, __is_neg_y, __is_neg_z);
            }
            else {
                self.__update_prismatic(i, joint_val, __is_x, __is_y, __is_z, __is_neg_x, __is_neg_y, __is_neg_z);
            }
        }
        else if __is_revolute_or_continuous {
            if __do_rot_offset {
                self.__update_revolute_or_continuous_ro(i, joint_val, __is_x, __is_y, __is_z, __is_neg_x, __is_neg_y, __is_neg_z);
            }
            else {
                self.__update_revolute_or_continuous(i, joint_val, __is_x, __is_y, __is_z, __is_neg_x, __is_neg_y, __is_neg_z);
            }
        }
        else if __is_fixed {
            if __do_rot_offset {
                self.__update_fixed_ro(i);
            }
            else {
                self.__update_fixed(i);
            }
        }
    }

    fn __update_prismatic(&mut self, i: usize, joint_val: f64, __is_x: bool, __is_y: bool,
                       __is_z: bool, __is_neg_x: bool, __is_neg_y: bool, __is_neg_z: bool) {

        self.out_rot_mats[i + 1] = self.out_rot_mats[i];
        self.out_rot_quats[i + 1] = self.out_rot_quats[i];

        if __is_x {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(joint_val, 0., 0.);
        }
        else if __is_y {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(0., joint_val, 0.);
        }
        else if __is_z {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(0., 0., joint_val);
        }
        else if __is_neg_x {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(-joint_val, 0., 0.);
        }
        else if __is_neg_y {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(0., -joint_val, 0.);
        }
        else if __is_neg_z {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(0., 0., -joint_val);
        }
    }

    fn __update_prismatic_ro(&mut self, i: usize, joint_val: f64, __is_x: bool, __is_y: bool,
                       __is_z: bool, __is_neg_x: bool, __is_neg_y: bool, __is_neg_z: bool) {

        self.out_rot_quats[i + 1] = self.out_rot_quats[i];

        if __is_x {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(joint_val, 0., 0.);
        }
        else if __is_y {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(0., joint_val, 0.);
        }
        else if __is_z {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(0., 0., joint_val);
        }
        else if __is_neg_x {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(-joint_val, 0., 0.);
        }
        else if __is_neg_y {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(0., -joint_val, 0.);
        }
        else if __is_neg_z {
            self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i] + nalgebra::Vector3::new(0., 0., -joint_val);
        }

        self.out_rot_quats[i + 1] = self.out_rot_quats[i + 1] * self.rot_offset_quats[i+1];
        self.out_rot_mats[i + 1] = *self.out_rot_quats[i + 1].to_rotation_matrix().matrix()

    }

    fn __update_revolute_or_continuous(&mut self, i: usize, joint_val: f64, __is_x: bool, __is_y: bool,
                       __is_z: bool, __is_neg_x: bool, __is_neg_y: bool, __is_neg_z: bool) {
        if __is_x {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_rot_x(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_quat_x(joint_val);
        }
        else if __is_y {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_rot_y(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_quat_y(joint_val);
        }
        else if __is_z {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_rot_z(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_quat_z(joint_val);
        }
        else if __is_neg_x {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_neg_rot_x(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_neg_quat_x(joint_val);
        }
        else if __is_neg_y {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_neg_rot_y(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_neg_quat_y(joint_val);
        }
        else if __is_neg_z {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_neg_rot_z(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_neg_quat_x(joint_val);
        }

        self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i];
        self.out_rot_mats[i + 1] = *self.out_rot_quats[i + 1].to_rotation_matrix().matrix();
    }

    fn __update_revolute_or_continuous_ro(&mut self, i: usize, joint_val: f64, __is_x: bool, __is_y: bool,
                       __is_z: bool, __is_neg_x: bool, __is_neg_y: bool, __is_neg_z: bool) {
        if __is_x {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_rot_x(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_quat_x(joint_val);
        }
        else if __is_y {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_rot_y(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_quat_y(joint_val);
        }
        else if __is_z {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_rot_z(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_quat_z(joint_val);
        }
        else if __is_neg_x {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_neg_rot_x(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_neg_quat_x(joint_val);
        }
        else if __is_neg_y {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_neg_rot_y(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_neg_quat_y(joint_val);
        }
        else if __is_neg_z {
            // self.out_rot_mats[i + 1] = self.out_rot_mats[i] * get_neg_rot_z(joint_val);
            self.out_rot_quats[i + 1] = self.out_rot_quats[i] * get_neg_quat_x(joint_val);
        }

        self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i];
        self.out_rot_quats[i + 1] = self.out_rot_quats[i + 1] * self.rot_offset_quats[i+1];
        self.out_rot_mats[i + 1] = *self.out_rot_quats[i + 1].to_rotation_matrix().matrix();
    }

    fn __update_fixed(&mut self, i: usize) {
        self.out_rot_quats[i + 1] = self.out_rot_quats[i];
        self.out_positions[i + 1] = self.out_rot_quats[i + 1] * self.displacements[i] + self.out_positions[i];
        self.out_rot_mats[i + 1] = *self.out_rot_quats[i].to_rotation_matrix().matrix();
    }

    fn __update_fixed_ro(&mut self, i: usize) {
        self.out_positions[i + 1] = self.out_rot_quats[i] * self.displacements[i] + self.out_positions[i];
        self.out_rot_quats[i + 1] = self.out_rot_quats[i] * self.rot_offset_quats[i];
        self.out_rot_mats[i + 1] = *self.out_rot_quats[i].to_rotation_matrix().matrix();
    }
}

pub fn get_rot_x(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(1., 0., 0., 0., val.cos(), -val.sin(), 0.0, val.sin(), val.cos())
}

pub fn get_rot_y(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(val.cos(), 0.0, val.sin(), 0., 1., 0., -val.sin(), 0., val.cos())
}

pub fn get_rot_z(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(val.cos(), -val.sin(), 0., val.sin(), val.cos(), 0., 0., 0., 1.)
}

pub fn get_neg_rot_x(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_x(-val)
}

pub fn get_neg_rot_y(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_y(-val)
}

pub fn get_neg_rot_z(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_z(-val)
}

pub fn get_quat_x(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(val, 0., 0.)
}

pub fn get_quat_y(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(0., val, 0.)
}

pub fn get_quat_z(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(0., 0., val)
}

pub fn get_neg_quat_x(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_x(-val)
}

pub fn get_neg_quat_y(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_y(-val)
}

pub fn get_neg_quat_z(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_z(-val)
}

pub fn euler_triple_to_3x3(t: &Vec<f64>) -> nalgebra::Matrix3<f64>{
    let xm = get_rot_x(t[0]);
    let ym = get_rot_y(t[1]);
    let zm = get_rot_z(t[2]);

    let zy = zm*ym;
    zy*xm
}

/*
pub fn get_frame_closure(axis: &String, joint_type: &String, rot_offset_matrix: &nalgebra::Matrix3<f64>, disp: &nalgebra::Vector3<f64>, do_rot_offset: bool) -> impl Fn(f64) -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
    let m: nalgebra::Matrix3<f64> = nalgebra::Matrix3::identity();
    let v: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.,0.,0.);

    move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
        (m, v)
    }

}
*/

/*
pub fn get_revolute_closure(axis: &String, rot_offset_matrix: nalgebra::Matrix3<f64>, disp: nalgebra::Vector3<f64>, do_rot_offset: bool) -> impl Fn(f64) -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
    if do_rot_offset {
        let m: nalgebra::Matrix3<f64> = rot_offset_matrix.clone();

        if *axis == String::from("X") || *axis == String::from("x") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (m * get_rot_x(val), disp.clone())
            }
        }
        else if *axis == String::from("Y") || *axis == String::from("y") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (m * get_rot_y(val), disp.clone())
            }
        }
        else if *axis == String::from("Z") || *axis == String::from("z") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (m * get_rot_z(val), disp.clone())
            }
        }
        else if *axis == String::from("-x") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (m * get_rot_x(-val), disp.clone())
            }
        }
        else if *axis == String::from("-y") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (m * get_rot_y(-val), disp.clone())
            }
        }
        else if *axis == String::from("-z") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (m * get_rot_z(-val), disp.clone())
            }
        }
        else {
            move  |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (get_rot_z(-val), disp.clone())
            }
        }
    }
    else {
        if *axis == String::from("X") || *axis == String::from("x") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (get_rot_x(val), disp.clone())
            }
        }
        else if *axis == String::from("Y") || *axis == String::from("y") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (get_rot_y(val), disp.clone())
            }
        }
        else if *axis == String::from("Z") || *axis == String::from("z") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (get_rot_z(val), disp.clone())
            }
        }
        else if *axis == String::from("-x") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (get_rot_x(-val), disp.clone())
            }
        }
        else if *axis == String::from("-y") {
            move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (get_rot_y(-val), disp.clone())
            }
        }
        else if *axis == String::from("-z") {
            move  |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (get_rot_z(-val), disp.clone())
            }
        }
        else {
            move  |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
                (get_rot_z(-val), disp.clone())
            }
        }
    }

}
*/

/*
pub fn get_revolute_closure(axis: &String, rot_offset_matrix: nalgebra::Matrix3<f64>, disp: nalgebra::Vector3<f64>, do_rot_offset: bool) -> Box<Fn(f64) -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>)> {
    if true {
        Box::new(move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
            (get_rot_x(val).clone(), disp.clone())
        })
    } else {
        Box::new(move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
            (get_rot_x(val).clone(), disp.clone())
        })
    }
}
*/

/*
pub fn get_revolute_closure() -> impl Fn(f64) -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
    let m: nalgebra::Matrix3<f64> = nalgebra::Matrix3::identity();
    let v: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.,0.,0.);

    move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
        (m, v)
    }
}

pub fn get_revolute_closure2() -> impl Fn(f64) -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
    let m: nalgebra::Matrix3<f64> = nalgebra::Matrix3::identity();
    let v: nalgebra::Vector3<f64> = nalgebra::Vector3::new(1.,1.,0.);

    move |val: f64| -> (nalgebra::Matrix3<f64>, nalgebra::Vector3<f64>) {
        (m*m, v)
    }
}


fn get_fixed_closure(axis: &String, rot_offset_matrix: &nalgebra::Matrix3<f64>, disp: &nalgebra::Vector3<f64>, do_rot_offset: bool) {

}

fn get_prismatic_closure(axis: &String, rot_offset_matrix: &nalgebra::Matrix3<f64>, disp: &nalgebra::Vector3<f64>, do_rot_offset: bool) {

}
*/