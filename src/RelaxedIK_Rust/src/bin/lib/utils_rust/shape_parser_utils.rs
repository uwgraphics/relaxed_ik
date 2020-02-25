
#[derive(Clone, Debug)]
pub struct Cuboid {
    pub name: String,
    pub x_halflength: f64,
    pub y_halflength: f64,
    pub z_halflength: f64,
    pub coordinate_frame: String,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64
}
impl Cuboid {
    pub fn new(name: String, x_halflength: f64, y_halflength: f64, z_halflength: f64, coordinate_frame: String,
        rx: f64, ry: f64, rz: f64, tx: f64, ty: f64, tz: f64) -> Self {
        Self {name, x_halflength, y_halflength, z_halflength, coordinate_frame, rx, ry, rz, tx, ty, tz}
    }
}

#[derive(Clone, Debug)]
pub struct Sphere {
    pub name: String,
    pub radius: f64,
    pub coordinate_frame: String,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64
}
impl Sphere {
    pub fn new(name: String, radius: f64, coordinate_frame: String, tx: f64, ty: f64, tz: f64) -> Self {
        Self {name, radius, coordinate_frame, tx, ty, tz}
    }
}

