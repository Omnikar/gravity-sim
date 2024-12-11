pub type Vec3 = nalgebra::Vector3<f64>;

/// Dimensionless circular restricted 3 body system
#[derive(Clone)]
pub struct Cr3bs {
    // π₂ = m₂ / (m₁ + m₂)
    pub mass_ratio: f64,
    pub time_log: Vec<f64>,
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
            time_log: vec![0.0],
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
    pub fn euler(&mut self, dt: f64, t: f64) {
        let steps = (t / dt).ceil() as usize + 1;
        self.time_log.reserve(steps);
        self.pos_log.reserve(steps);
        self.vel_log.reserve(steps);
        let init_len = self.pos_log.len();
        for step in 0..steps {
            let i = init_len + step - 1;
            self.time_log.push(i as f64 * dt);
            self.pos_log.push(self.pos_log[i] + self.vel_log[i] * dt);
            let acc = self.calc_accel(self.pos_log[i], self.vel_log[i]);
            self.vel_log.push(self.vel_log[i] + acc * dt);
        }

        self.log_jacobi();
    }

    #[allow(dead_code)]
    pub fn modified_euler(&mut self, dt: f64, t: f64) {
        let steps = (t / dt).ceil() as usize + 1;
        self.time_log.reserve(steps);
        self.pos_log.reserve(steps);
        self.vel_log.reserve(steps);
        let init_len = self.pos_log.len();
        for step in 0..steps {
            let i = init_len + step - 1;
            self.time_log.push(i as f64 * dt);
            let acc = self.calc_accel(self.pos_log[i], self.vel_log[i]);
            let vel = self.vel_log[i] + acc * dt;
            let pos = self.pos_log[i] + vel * dt;
            self.vel_log.push(vel);
            self.pos_log.push(pos);
        }

        self.log_jacobi();
    }

    #[allow(dead_code)]
    pub fn verlet(&mut self, dt: f64, t: f64) {
        let steps = (t / dt).ceil() as usize + 1;
        self.time_log.reserve(steps);
        self.pos_log.reserve(steps);
        self.vel_log.reserve(steps);
        let init = self.pos_log.len();
        for i in init - 1..init + steps - 1 {
            self.time_log.push(i as f64 * dt);
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

    #[allow(dead_code)]
    pub fn rkf45(&mut self, dt0: f64, epsilon: f64, t: f64) {
        let t0 = self.time_log.last().unwrap();

        type Vec6 = nalgebra::Vector6<f64>;
        fn mk_state(pos: Vec3, vel: Vec3) -> Vec6 {
            Vec6::from_iterator(pos.iter().copied().chain(vel.iter().copied()))
        }

        let deriv = |state: Vec6| {
            let pos = Vec3::from_column_slice(&state.as_slice()[..3]);
            let vel = Vec3::from_column_slice(&state.as_slice()[3..]);
            let acc = self.calc_accel(pos, vel);
            mk_state(vel, acc)
        };

        let mut elapsed = 0.0;
        let mut dt = dt0;
        let mut state = mk_state(*self.pos_log.last().unwrap(), *self.vel_log.last().unwrap());

        let mut time_log = Vec::new();
        let mut state_log = Vec::new();

        while elapsed < t {
            let k1 = dt * deriv(state);
            let k2 = dt * deriv(state + 1.0 / 4.0 * k1);
            let k3 = dt * deriv(state + (3.0 * k1 + 9.0 * k2) / 32.0);
            let k4 = dt * deriv(state + (1932.0 * k1 - 7200.0 * k2 + 7296.0 * k3) / 2197.0);
            let k5 = dt
                * deriv(
                    state + 439.0 / 216.0 * k1 - 8.0 * k2 + 3680.0 / 513.0 * k3
                        - 845.0 / 4104.0 * k4,
                );
            let k6 = dt
                * deriv(
                    state - 8.0 / 27.0 * k1 + 2.0 * k2 - 3544.0 / 2565.0 * k3
                        + 1859.0 / 4104.0 * k4
                        - 11.0 / 40.0 * k5,
                );

            let ord4 = state + 25.0 / 216.0 * k1 + 1408.0 / 2565.0 * k3 + 2197.0 / 4104.0 * k4
                - 1.0 / 5.0 * k5;
            let ord5 = state + 16.0 / 135.0 * k1 + 6656.0 / 12825.0 * k3 + 28561.0 / 56430.0 * k4
                - 9.0 / 50.0 * k5
                + 2.0 / 55.0 * k6;

            let error = (ord5 - ord4).norm() / dt;
            if error <= epsilon {
                state = ord4;
                state_log.push(state);
                elapsed += dt;
                time_log.push(t0 + elapsed);
            }
            dt *= 0.84 * (epsilon / error).powf(0.25);
        }

        self.time_log.append(&mut time_log);
        for state in state_log {
            let pos = Vec3::from_column_slice(&state.as_slice()[..3]);
            let vel = Vec3::from_column_slice(&state.as_slice()[3..]);
            self.pos_log.push(pos);
            self.vel_log.push(vel);
        }

        self.log_jacobi();
    }

    pub fn timed_pos(&self) -> impl Iterator<Item = (f64, Vec3)> + '_ {
        self.time_log
            .iter()
            .copied()
            .zip(self.pos_log.iter().copied())
    }

    #[allow(dead_code)]
    pub fn timed_vel(&self) -> impl Iterator<Item = (f64, Vec3)> + '_ {
        self.time_log
            .iter()
            .copied()
            .zip(self.vel_log.iter().copied())
    }

    pub fn timed_jacobi(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.time_log
            .iter()
            .copied()
            .zip(self.jacobi_log.iter().copied())
    }

    pub fn write_time_log(&self, w: &mut impl std::io::Write) {
        write_1d_log(&self.time_log, w);
    }

    pub fn write_jacobi_log(&self, w: &mut impl std::io::Write) {
        write_1d_log(&self.jacobi_log, w);
    }

    pub fn write_pos_log(&self, w: &mut impl std::io::Write) {
        write_3d_log(&self.pos_log, w);
    }

    pub fn write_vel_log(&self, w: &mut impl std::io::Write) {
        write_3d_log(&self.vel_log, w);
    }
}

fn write_1d_log(log: &[f64], w: &mut impl std::io::Write) {
    for entry in log {
        writeln!(w, "{entry}").unwrap();
    }
}

fn write_3d_log(log: &[Vec3], w: &mut impl std::io::Write) {
    for entry in log {
        writeln!(w, "{},{},{}", entry.x, entry.y, entry.z).unwrap();
    }
}
