use std::fmt::Write;

pub type Vec3 = nalgebra::Vector3<f64>;

/// Dimensionless circular restricted 3 body system
#[derive(Clone)]
pub struct Cr3bs {
    // π₂ = m₂ / (m₁ + m₂)
    pub mass_ratio: f64,
    pub pos_log: Vec<Vec3>,
    pub vel_log: Vec<Vec3>,
}

impl From<[f64; 7]> for Cr3bs {
    fn from(vals: [f64; 7]) -> Self {
        Self::new(
            vals[0],
            [1, 2, 3].map(|i| vals[i]).into(),
            [4, 5, 6].map(|i| vals[i]).into(),
        )
    }
}

impl Cr3bs {
    pub fn new(mass_ratio: f64, pos0: Vec3, vel0: Vec3) -> Self {
        Self {
            mass_ratio,
            pos_log: vec![pos0],
            vel_log: vec![vel0],
        }
    }

    fn sigma(&self, pos: Vec3) -> Vec3 {
        Vec3::x() * self.mass_ratio + pos
    }

    fn psi(&self, pos: Vec3) -> Vec3 {
        Vec3::x() * (self.mass_ratio - 1.0) + pos
    }

    /// Jacobi constant
    pub fn jacobi(&self, pos: Vec3, vel: Vec3) -> f64 {
        let pi2 = self.mass_ratio;
        let pi1 = 1.0 - pi2;
        let sigma = self.sigma(pos).norm();
        let psi = self.psi(pos).norm();

        // Multiple sources
        // - https://en.wikipedia.org/wiki/Jacobi_integral
        // - https://www.colorado.edu/faculty/bosanac/sites/default/files/2024-09/MicBosKar_2024ASC_NeptuneTrajectories.pdf
        // - https://gereshes.com/2018/11/26/jacobi-and-his-constant-the-3-body-problem/
        let mut ppos = pos;
        ppos.z = 0.0;
        ppos.norm_squared() + 2.0 * pi1 / sigma + 2.0 * pi2 / psi - vel.norm_squared()
    }

    fn calc_accel(&self, pos_i: usize, vel_i: usize) -> Vec3 {
        let pos = self.pos_log[pos_i];
        let vel = self.vel_log[vel_i];
        let pi2 = self.mass_ratio;
        let pi1 = 1.0 - pi2;

        let sigma3 = self.sigma(pos).norm().powi(3);
        let psi3 = self.psi(pos).norm().powi(3);

        // Equations of motion sourced from below
        // https://orbital-mechanics.space/the-n-body-problem/circular-restricted-three-body-problem.html#equation-eq-non-dim-scalar-eom-cr3bp
        Vec3::new(
            pos.x + 2.0 * vel.y - pi1 * (pos.x + pi2) / sigma3 - pi2 * (pos.x - pi1) / psi3,
            pos.y - 2.0 * vel.x - pi1 * pos.y / sigma3 - pi2 * pos.y / psi3,
            -pi1 * pos.z / sigma3 - pi2 * pos.z / psi3,
        )
    }

    #[allow(dead_code)]
    pub fn euler(&mut self, dt: f64, steps: usize) {
        self.pos_log.reserve(steps);
        let init_len = self.pos_log.len();
        for step in 0..steps {
            let i = init_len + step - 1;
            self.pos_log.push(self.pos_log[i] + self.vel_log[i] * dt);
            let acc = self.calc_accel(i, i);
            self.vel_log.push(self.vel_log[i] + acc * dt);
        }
    }

    #[allow(dead_code)]
    pub fn modified_euler(&mut self, dt: f64, steps: usize) {
        self.pos_log.reserve(steps);
        let init_len = self.pos_log.len();
        for step in 0..steps {
            let i = init_len + step - 1;
            let acc = self.calc_accel(i, i);
            let vel = self.vel_log[i] + acc * dt;
            let pos = self.pos_log[i] + vel * dt;
            self.vel_log.push(vel);
            self.pos_log.push(pos);
        }
    }

    pub fn serialize_pos_log(&self) -> String {
        serialize_log(&self.pos_log)
    }

    pub fn serialize_vel_log(&self) -> String {
        serialize_log(&self.vel_log)
    }
}

fn serialize_log(log: &[Vec3]) -> String {
    let mut s = String::new();
    for entry in log {
        writeln!(s, "{},{},{}", entry.x, entry.y, entry.z).unwrap();
    }
    s
}
