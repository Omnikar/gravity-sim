use std::fmt::Write;

pub type Vec3 = nalgebra::Vector3<f64>;

/// Dimensionless circular restricted 3 body system
#[derive(Clone)]
pub struct Cr3bs {
    // π₂ = m₂ / (m₁ + m₂)
    pub mass_ratio: f64,
    pub pos_log: Vec<Vec3>,
    pub vel_log: Vec<Vec3>,
    pub jacobi_log: Vec<f64>,
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
        let mut new = Self {
            mass_ratio,
            pos_log: vec![pos0],
            vel_log: vec![vel0],
            jacobi_log: Vec::new(),
        };
        let init_jacobi = new.jacobi(pos0, vel0);
        new.jacobi_log.push(init_jacobi);
        new
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

    pub fn log_jacobi(&mut self) {
        // TODO: Don't clear the log every time?
        self.jacobi_log.clear();
        for (&pos, &vel) in self.pos_log.iter().zip(self.vel_log.iter()) {
            let jacobi = self.jacobi(pos, vel);
            self.jacobi_log.push(jacobi);
        }
    }

    fn calc_accel(&self, pos: Vec3, vel: Vec3) -> Vec3 {
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
        self.vel_log.reserve(steps);
        let init_len = self.pos_log.len();
        for step in 0..steps {
            let i = init_len + step - 1;
            self.pos_log.push(self.pos_log[i] + self.vel_log[i] * dt);
            let acc = self.calc_accel(self.pos_log[i], self.vel_log[i]);
            self.vel_log.push(self.vel_log[i] + acc * dt);
        }

        self.log_jacobi();
    }

    #[allow(dead_code)]
    pub fn modified_euler(&mut self, dt: f64, steps: usize) {
        self.pos_log.reserve(steps);
        self.vel_log.reserve(steps);
        let init_len = self.pos_log.len();
        for step in 0..steps {
            let i = init_len + step - 1;
            let acc = self.calc_accel(self.pos_log[i], self.vel_log[i]);
            let vel = self.vel_log[i] + acc * dt;
            let pos = self.pos_log[i] + vel * dt;
            self.vel_log.push(vel);
            self.pos_log.push(pos);
        }

        self.log_jacobi();
    }

    #[allow(dead_code)]
    pub fn verlet(&mut self, dt: f64, steps: usize) {
        self.pos_log.reserve(steps);
        self.vel_log.reserve(steps);
        let init = self.pos_log.len();
        for i in init - 1..init + steps - 1 {
            let acc = self.calc_accel(self.pos_log[i], self.vel_log[i]);
            let int_vel_change = 0.5 * acc * dt;
            let new_pos = self.pos_log[i] + self.vel_log[i] * dt + int_vel_change * dt;
            self.pos_log.push(new_pos);
            let int_vel = self.vel_log[i] + 0.5 * acc * dt;
            let new_acc = self.calc_accel(new_pos, int_vel);
            let new_vel = int_vel + 0.5 * new_acc * dt;
            self.vel_log.push(new_vel);
        }

        self.log_jacobi();
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
