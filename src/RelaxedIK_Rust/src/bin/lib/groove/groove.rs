
pub struct Groove<'a> {
    pub init_state: &'a[f64],
}

impl<'a> Groove<'a> {
    pub fn new(init_state: &'a [f64]) -> Groove<'a> {
        Groove{init_state}
    }
}