use nalgebra_lapack::Eigen;
use rand::Rng;
use std::io::Write;
use std::sync::mpsc;
use std::thread::{spawn, JoinHandle};

pub type Vec3 = nalgebra::Vector3<f64>;
type Vec6 = nalgebra::Vector6<f64>;
type Mat6 = nalgebra::Matrix6<f64>;
type Maneuver = (f64, Vec3);

/// Dimensionless circular restricted 3 body system
#[derive(Clone)]
pub struct Cr3bs {
    // μ₂ = m₂ / (m₁ + m₂)
    pub mass_ratio: f64,
    pub period: f64,
    pub maneuvers: Vec<Maneuver>,
    pub time_log: Vec<f64>,
    pub pos_log: Vec<Vec3>,
    pub vel_log: Vec<Vec3>,
    pub jacobi_log: Vec<f64>,
    pub orbit_report: Option<OrbitReport>,
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
            period: f64::INFINITY,
            maneuvers: Vec::new(),
            time_log: vec![0.0],
            pos_log: vec![pos0],
            vel_log: vec![vel0],
            jacobi_log: Vec::new(),
            orbit_report: None,
        };
        let init_jacobi = new.jacobi(pos0, vel0);
        new.jacobi_log.push(init_jacobi);
        new
    }

    pub fn with_period(mut self, period: f64) -> Self {
        self.period = period;
        self
    }

    fn r1(&self, pos: Vec3) -> Vec3 {
        Vec3::x() * self.mass_ratio + pos
    }

    fn r2(&self, pos: Vec3) -> Vec3 {
        Vec3::x() * (self.mass_ratio - 1.0) + pos
    }

    /// Jacobi constant
    pub fn jacobi(&self, pos: Vec3, vel: Vec3) -> f64 {
        let mu2 = self.mass_ratio;
        let mu1 = 1.0 - mu2;
        let r1 = self.r1(pos).norm();
        let r2 = self.r2(pos).norm();

        let mut ppos = pos;
        ppos.z = 0.0;
        ppos.norm_squared() + 2.0 * mu1 / r1 + 2.0 * mu2 / r2 - vel.norm_squared()
    }

    pub fn log_jacobi(&mut self) {
        self.jacobi_log.clear();
        for (&pos, &vel) in self.pos_log.iter().zip(self.vel_log.iter()) {
            let jacobi = self.jacobi(pos, vel);
            self.jacobi_log.push(jacobi);
        }
    }

    fn calc_accel(&self, pos: Vec3, vel: Vec3) -> Vec3 {
        let mu2 = self.mass_ratio;
        let mu1 = 1.0 - mu2;

        let r1p3 = self.r1(pos).norm().powi(3);
        let r2p3 = self.r2(pos).norm().powi(3);

        Vec3::new(
            pos.x + 2.0 * vel.y - mu1 * (pos.x + mu2) / r1p3 - mu2 * (pos.x - mu1) / r2p3,
            pos.y - 2.0 * vel.x - mu1 * pos.y / r1p3 - mu2 * pos.y / r2p3,
            -mu1 * pos.z / r1p3 - mu2 * pos.z / r2p3,
        )
    }

    pub fn pos_at(&self, t: f64) -> Option<Vec3> {
        pos_at(&self.time_log, &self.pos_log, t)
    }

    pub fn vel_at(&self, t: f64) -> Option<Vec3> {
        vel_at(&self.time_log, &self.vel_log, t)
    }

    pub fn state_at(&self, t: f64) -> Option<Vec6> {
        self.pos_at(t)
            .and_then(|p| self.vel_at(t).map(|v| mk_state(p, v)))
    }

    fn jacobian(&self, t: f64) -> Option<Mat6> {
        jacobian(self.mass_ratio, &self.time_log, &self.pos_log, t)
    }

    // fn ababa(&self, t: f64) -> Option<Mat6> {
    //     let pos = pos_at(time_log, pos_log, t)?;

    //     let mu2 = mass_ratio;
    //     let mu1 = 1.0 - mu2;

    //     let r1p3r = r1(mass_ratio, pos).norm().powi(3).recip();
    //     let r1p5r = r1(mass_ratio, pos).norm().powi(5).recip();
    //     let r2p3r = r2(mass_ratio, pos).norm().powi(3).recip();
    //     let r2p5r = r2(mass_ratio, pos).norm().powi(5).recip();

    //     let [x, y, z] = <[f64; 3]>::from(pos);
    //     let xd1 = x - mu1;
    //     let xd2 = x + mu2;

    //     // Hessian partial derivatives (supplied by C.G.)
    //     let xx =
    //         1.0 - mu1 * (r1p3r - 3.0 * xd2 * xd2 * r1p5r) - mu2 * (r2p3r - 3.0 * xd1 * xd1 * r2p5r);
    //     let xy = mu1 * 3.0 * y * xd2 * r1p5r + mu2 * 3.0 * y * xd1 * r2p5r;
    //     let xz = mu1 * 3.0 * z * xd2 * r1p5r + mu2 * 3.0 * z * xd1 * r2p5r;
    //     let yy = 1.0 - mu1 * (r1p3r - 3.0 * y * y * r1p5r) - mu2 * (r2p3r - 3.0 * y * y * r2p5r);
    //     let yz = mu1 * 3.0 * y * z * r1p5r + mu2 * 3.0 * y * z * r2p5r;
    //     let zz = -mu1 * (r1p3r - 3.0 * z * z * r1p5r) - mu2 * (r2p3r - 3.0 * z * z * r2p5r);

    //     Some(Mat6::from_rows(
    //         &[
    //             [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    //             [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    //             [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    //             [xx, xy, xz, 0.0, 2.0, 0.0],
    //             [xy, yy, yz, -2.0, 0.0, 0.0],
    //             [xz, yz, zz, 0.0, 0.0, 0.0],
    //         ]
    //         .map(From::from),
    //     ))
    // }

    pub fn monodromy(&self, t: f64, dt0: f64, epsilon: f64) -> Option<Mat6> {
        self.stm(t, t + self.period, dt0, epsilon)
    }

    /// State transition matrix
    pub fn stm(&self, t0: f64, t1: f64, dt0: f64, epsilon: f64) -> Option<Mat6> {
        if t1 <= t0 {
            return None;
        }
        let mut t = t0;
        let mut state = Mat6::identity();
        let mut dt = dt0;

        while t < t1 {
            dt = dt.min(t1 - t);

            let jacobian = self.jacobian(t)?;
            let (ord4, ord5) = calc_rkf45_step(dt, state, |s| jacobian * s);
            let error = (ord4 - ord5).norm_squared() / 6.0;
            if error <= epsilon {
                state = ord4;
                t += dt;
            }
            if error.is_nan() {
                println!("method stm");
                dbg!(ord4);
                dbg!(ord5);
            }
            adjust_rkf45_dt(&mut dt, epsilon, error);
        }

        Some(state)
    }

    #[allow(dead_code)]
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

    #[allow(dead_code)]
    pub fn timed_jacobi(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.time_log
            .iter()
            .copied()
            .zip(self.jacobi_log.iter().copied())
    }

    pub fn gen_orbit_report(&mut self) {
        use uiua::{Array, Uiua};
        let mut uiua = Uiua::with_native_sys();
        uiua.push(Array::from_row_arrays_infallible(
            self.pos_log.iter().map(|pos| pos.as_slice().into()),
        ));
        uiua.push(self.mass_ratio);
        uiua.push(Array::from(&*self.time_log));
        let res = uiua.run_file("src/orbit_report.ua");
        if res.is_ok() {
            self.orbit_report = Some(OrbitReport {
                period: uiua.pop_num().unwrap(),
                periapsis: uiua.pop_num().unwrap(),
                apoapsis: uiua.pop_num().unwrap(),
                destabilizes: uiua.pop_num().unwrap(),
            });
        } else if let Err(e) = res {
            self.orbit_report = None;
            eprintln!("Failed to generate orbit report: {e:?}");
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct OrbitReport {
    pub period: f64,
    pub periapsis: f64,
    pub apoapsis: f64,
    pub destabilizes: f64,
}

impl std::fmt::Display for OrbitReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "--- Orbit Report ---")?;
        writeln!(f, "period: {}", self.period)?;
        writeln!(f, "periapsis: {}", self.periapsis)?;
        writeln!(f, "apoapsis: {}", self.apoapsis)?;
        write!(f, "destabilizes at: {}", self.destabilizes)?;
        Ok(())
    }
}

type SimOut = (Vec<f64>, Vec<Vec6>, Vec<Maneuver>);

pub struct Cr3bsMt {
    pub inner: Cr3bs,
    pub save_handle: Option<JoinHandle<()>>,
    pub sim_handle: Option<JoinHandle<SimOut>>,
}

impl From<Cr3bs> for Cr3bsMt {
    fn from(inner: Cr3bs) -> Self {
        Self {
            inner,
            save_handle: None,
            sim_handle: None,
        }
    }
}

impl Cr3bsMt {
    pub fn save(&mut self) {
        let system = self.inner.clone();
        self.inner.gen_orbit_report();
        let orbit_report = self.inner.orbit_report;
        self.save_handle = Some(spawn(move || {
            use std::fs::File;
            use std::io::BufWriter;
            fn f(s: &str) -> BufWriter<File> {
                BufWriter::new(File::create(s).unwrap())
            }
            let mut time_f = f("artifacts/time_log.csv");
            write_1d_log(&system.time_log, &mut time_f);
            time_f.flush().unwrap();
            let mut pos_f = f("artifacts/pos_log.csv");
            write_3d_log(&system.pos_log, &mut pos_f);
            pos_f.flush().unwrap();
            let mut vel_f = f("artifacts/vel_log.csv");
            write_3d_log(&system.vel_log, &mut vel_f);
            vel_f.flush().unwrap();
            let mut jacobi_f = f("artifacts/jacobi_log.csv");
            write_1d_log(&system.jacobi_log, &mut jacobi_f);
            jacobi_f.flush().unwrap();
            let mut maneuvers_f = f("artifacts/maneuver_log.csv");
            write_4d_log(&system.maneuvers, &mut maneuvers_f);
            maneuvers_f.flush().unwrap();

            if let Some(orbit_report) = orbit_report {
                let mut report_file = f("artifacts/orbit_report.txt");
                writeln!(report_file, "{orbit_report}").unwrap();
                report_file.flush().unwrap();
            }
        }));
    }

    #[allow(dead_code)]
    pub fn rkf45_threaded(
        &mut self,
        dt0: f64,
        epsilon: f64,
        t: f64,
    ) -> (mpsc::Receiver<f64>, mpsc::Sender<()>) {
        let t0 = *self.inner.time_log.last().unwrap();

        let system = self.inner.clone();
        let deriv = move |state: Vec6| {
            let pos = Vec3::from_column_slice(&state.as_slice()[..3]);
            let vel = Vec3::from_column_slice(&state.as_slice()[3..]);
            let acc = system.calc_accel(pos, vel);
            mk_state(vel, acc)
        };

        let mass_ratio = self.inner.mass_ratio;
        let period = self.inner.period;

        let mut time_log = self.inner.time_log.clone();
        let mut state_log = Vec::new();
        for (pos, vel) in self.inner.pos_log.iter().zip(self.inner.vel_log.iter()) {
            state_log.push(mk_state(*pos, *vel));
        }

        let mut elapsed = 0.0;
        let mut state = *state_log.last().unwrap();
        let mut dt = dt0;

        let mut maneuvers = self.inner.maneuvers.clone();

        let cadence = period / 3.0;

        let (status_tx, status_rx) = mpsc::channel();
        let (end_tx, end_rx) = mpsc::channel();

        let thread_code = move || {
            let mut maneuver_i = maneuvers.len();

            while elapsed < t {
                if end_rx.try_recv().is_ok() {
                    break;
                }

                let mut maneuver = None;
                if let Some(next_maneuver) = maneuvers.get(maneuver_i) {
                    if next_maneuver.0 < t0 + elapsed {
                        maneuver = Some(*next_maneuver);
                        time_log.pop();
                        state_log.pop();
                        elapsed = *time_log.last().expect("time_log should not be empty") - t0;
                        dt = next_maneuver.0 - elapsed - t0;
                        state = *state_log.last().expect("state_log should not be empty");
                        maneuver_i += 1;
                    }
                }
                let (ord4, ord5) = calc_rkf45_step(dt, state, &deriv);

                let error = (ord5 - ord4).norm() / dt;
                if error <= epsilon {
                    let old_state = state;
                    state = ord4;

                    if let Some(maneuver) = maneuver {
                        state += mk_state(Vec3::default(), maneuver.1);
                    }

                    state_log.push(state);
                    elapsed += dt;
                    time_log.push(t0 + elapsed);
                    if let Err(e) = status_tx.send(t0 + elapsed) {
                        eprintln!("WARN: Failed to send status update from simulation thread\n{e}");
                    }

                    let hf_pd = period / 2.0;
                    let check_i = ((t0 + elapsed) / hf_pd).floor();
                    if ((t0 + elapsed - dt) / hf_pd).floor() < check_i {
                        let last_maneuver_t = maneuvers.last().map(|m| m.0).unwrap_or(0.0);
                        let m_t = last_maneuver_t + cadence;
                        if (m_t / hf_pd).floor() + 4.0 <= check_i {
                            let check_t = check_i * hf_pd;
                            let pos_log = state_log
                                .iter()
                                .map(|st| st.remove_fixed_rows::<3>(3))
                                .collect::<Vec<_>>();
                            if let Some(stm) =
                                stm(mass_ratio, &time_log, &pos_log, m_t, check_t, dt0, epsilon)
                            {
                                let check_state = state_at(&time_log, &state_log, check_t)
                                    .unwrap()
                                    .remove_row(4)
                                    .remove_row(2)
                                    .remove_row(0);
                                let eigen = Eigen::new(stm, false, true)
                                    .expect("STM eigendecomposition failed");
                                let evecs = eigen.eigenvectors.unwrap();
                                let evec_i = eigen
                                    .eigenvalues_re
                                    .into_iter()
                                    .zip(eigen.eigenvalues_im.into_iter())
                                    .enumerate()
                                    .filter(|&(_, (&r, &i))| i == 0.0 && r.abs() > 1.0)
                                    .max_by(|&(_, (&a, _)), &(_, (&b, _))| {
                                        a.abs().total_cmp(&b.abs())
                                    })
                                    .map(|tup| tup.0);
                                if let Some(evec_i) = evec_i {
                                    let unstable_evecs = evecs
                                        .column(evec_i)
                                        .remove_row(4)
                                        .remove_row(2)
                                        .remove_row(0);

                                    let sq_mags =
                                        unstable_evecs.component_mul(&unstable_evecs).row_sum_tr();
                                    let dots = unstable_evecs.transpose() * check_state;
                                    let projected_error =
                                        unstable_evecs * dots.component_div(&sq_mags);

                                    let sub_stm = stm
                                        .remove_row(4)
                                        .remove_row(2)
                                        .remove_row(0)
                                        .remove_fixed_columns::<3>(0);
                                    let correction =
                                        -sub_stm.try_inverse().expect("STM inversion failed")
                                            * projected_error;

                                    let perturbation_strength = 0.0;
                                    let mul = 1.0
                                        + perturbation_strength
                                            * (rand::rng().random::<f64>() * 2.0 - 1.0);

                                    maneuvers.push((m_t, correction * mul));

                                    let trunc_i = time_log.partition_point(|&t| t <= m_t);
                                    time_log.truncate(trunc_i);
                                    state_log.truncate(trunc_i);
                                    elapsed =
                                        *time_log.last().expect("time_log should not be empty")
                                            - t0;
                                    state =
                                        *state_log.last().expect("state_log should not be empty");
                                }
                            }
                        }
                    }
                }

                if error.is_nan() {
                    println!("main sim");
                    dbg!(ord4, ord5, dt, state);
                }
                adjust_rkf45_dt(&mut dt, epsilon, error);
            }
            (time_log, state_log, maneuvers)
        };

        self.sim_handle = Some(spawn(thread_code));

        (status_rx, end_tx)
    }

    pub fn update(&mut self) -> bool {
        if let Some(handle) = self.sim_handle.take_if(|h| h.is_finished()) {
            let (time_log, state_log, maneuvers) = handle.join().unwrap();
            self.inner.time_log = time_log;
            self.inner.pos_log.clear();
            self.inner.vel_log.clear();
            for state in state_log {
                let pos = Vec3::from_column_slice(&state.as_slice()[..3]);
                let vel = Vec3::from_column_slice(&state.as_slice()[3..]);
                self.inner.pos_log.push(pos);
                self.inner.vel_log.push(vel);
            }
            self.inner.maneuvers = maneuvers;
            self.inner.log_jacobi();
            self.inner.gen_orbit_report();
            if let Some(orbit_report) = self.inner.orbit_report {
                println!("{orbit_report}");
            } else {
                println!("No orbit report produced");
            }
            true
        } else {
            false
        }
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

fn write_4d_log(log: &[(f64, Vec3)], w: &mut impl std::io::Write) {
    for entry in log {
        writeln!(w, "{},{},{},{}", entry.0, entry.1.x, entry.1.y, entry.1.z).unwrap();
    }
}

use std::ops::{Add, Div, Mul, Sub};
fn calc_rkf45_step<S>(dt: f64, state: S, deriv: impl Fn(S) -> S) -> (S, S)
where
    S: Copy + Add<Output = S> + Sub<Output = S> + Mul<f64, Output = S> + Div<f64, Output = S>,
    f64: Mul<S, Output = S>,
{
    let k1 = dt * deriv(state);
    let k2 = dt * deriv(state + 1.0 / 4.0 * k1);
    let k3 = dt * deriv(state + (3.0 * k1 + 9.0 * k2) / 32.0);
    let k4 = dt * deriv(state + (1932.0 * k1 - 7200.0 * k2 + 7296.0 * k3) / 2197.0);
    let k5 = dt
        * deriv(state + 439.0 / 216.0 * k1 - 8.0 * k2 + 3680.0 / 513.0 * k3 - 845.0 / 4104.0 * k4);
    let k6 = dt
        * deriv(
            state - 8.0 / 27.0 * k1 + 2.0 * k2 - 3544.0 / 2565.0 * k3 + 1859.0 / 4104.0 * k4
                - 11.0 / 40.0 * k5,
        );

    let ord4 =
        state + 25.0 / 216.0 * k1 + 1408.0 / 2565.0 * k3 + 2197.0 / 4104.0 * k4 - 1.0 / 5.0 * k5;
    let ord5 = state + 16.0 / 135.0 * k1 + 6656.0 / 12825.0 * k3 + 28561.0 / 56430.0 * k4
        - 9.0 / 50.0 * k5
        + 2.0 / 55.0 * k6;

    (ord4, ord5)
}
fn adjust_rkf45_dt(dt: &mut f64, epsilon: f64, error: f64) {
    if !error.is_subnormal() && error.is_finite() {
        *dt *= 0.84 * (epsilon / error).powf(0.25);
    }
    if dt.is_nan() {
        *dt = 0.001;
    } else if dt.is_infinite() {
        *dt = 100.0;
    }
}

fn mk_state(pos: Vec3, vel: Vec3) -> Vec6 {
    Vec6::from_iterator(pos.iter().copied().chain(vel.iter().copied()))
}

pub fn pos_at(time_log: &[f64], pos_log: &[Vec3], t: f64) -> Option<Vec3> {
    let j = time_log.partition_point(|&cur_t| cur_t <= t);
    if j == time_log.len() {
        return None;
    }
    let i = j - 1;
    let lerp_factor = (t - time_log[i]) / (time_log[j] - time_log[i]);
    Some(pos_log[i] + lerp_factor * (pos_log[j] - pos_log[i]))
}

pub fn vel_at(time_log: &[f64], vel_log: &[Vec3], t: f64) -> Option<Vec3> {
    let j = time_log.partition_point(|&cur_t| cur_t <= t);
    if j == time_log.len() {
        return None;
    }
    let i = j - 1;
    let lerp_factor = (t - time_log[i]) / (time_log[j] - time_log[i]);
    Some(vel_log[i] + lerp_factor * (vel_log[j] - vel_log[i]))
}

pub fn state_at(time_log: &[f64], state_log: &[Vec6], t: f64) -> Option<Vec6> {
    let j = time_log.partition_point(|&cur_t| cur_t <= t);
    if j == time_log.len() {
        return None;
    }
    let i = j - 1;
    let lerp_factor = (t - time_log[i]) / (time_log[j] - time_log[i]);
    Some(state_log[i] + lerp_factor * (state_log[j] - state_log[i]))
}

fn r1(mass_ratio: f64, pos: Vec3) -> Vec3 {
    Vec3::x() * mass_ratio + pos
}

fn r2(mass_ratio: f64, pos: Vec3) -> Vec3 {
    Vec3::x() * (mass_ratio - 1.0) + pos
}
fn jacobian(mass_ratio: f64, time_log: &[f64], pos_log: &[Vec3], t: f64) -> Option<Mat6> {
    let pos = pos_at(time_log, pos_log, t)?;

    let mu2 = mass_ratio;
    let mu1 = 1.0 - mu2;

    let r1p3r = r1(mass_ratio, pos).norm().powi(3).recip();
    let r1p5r = r1(mass_ratio, pos).norm().powi(5).recip();
    let r2p3r = r2(mass_ratio, pos).norm().powi(3).recip();
    let r2p5r = r2(mass_ratio, pos).norm().powi(5).recip();

    let [x, y, z] = <[f64; 3]>::from(pos);
    let xd1 = x - mu1;
    let xd2 = x + mu2;

    // Hessian partial derivatives (supplied by C.G.)
    let xx =
        1.0 - mu1 * (r1p3r - 3.0 * xd2 * xd2 * r1p5r) - mu2 * (r2p3r - 3.0 * xd1 * xd1 * r2p5r);
    let xy = mu1 * 3.0 * y * xd2 * r1p5r + mu2 * 3.0 * y * xd1 * r2p5r;
    let xz = mu1 * 3.0 * z * xd2 * r1p5r + mu2 * 3.0 * z * xd1 * r2p5r;
    let yy = 1.0 - mu1 * (r1p3r - 3.0 * y * y * r1p5r) - mu2 * (r2p3r - 3.0 * y * y * r2p5r);
    let yz = mu1 * 3.0 * y * z * r1p5r + mu2 * 3.0 * y * z * r2p5r;
    let zz = -mu1 * (r1p3r - 3.0 * z * z * r1p5r) - mu2 * (r2p3r - 3.0 * z * z * r2p5r);

    Some(Mat6::from_rows(
        &[
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [xx, xy, xz, 0.0, 2.0, 0.0],
            [xy, yy, yz, -2.0, 0.0, 0.0],
            [xz, yz, zz, 0.0, 0.0, 0.0],
        ]
        .map(From::from),
    ))
}

/// State transition matrix
pub fn stm(
    mass_ratio: f64,
    time_log: &[f64],
    pos_log: &[Vec3],
    t0: f64,
    t1: f64,
    dt0: f64,
    epsilon: f64,
) -> Option<Mat6> {
    if t1 <= t0 {
        return None;
    }
    let mut t = t0;
    let mut state = Mat6::identity();
    let mut dt = dt0;

    let start_i = time_log.partition_point(|&t| t < t0).saturating_sub(1);
    let end_i = (time_log.partition_point(|&t| t < t1) + 1).min(time_log.len());
    let time_log = &time_log[start_i..end_i];
    let pos_log = &pos_log[start_i..end_i];

    while t < t1 {
        dt = dt.min(t1 - t);

        let jacobian = jacobian(mass_ratio, time_log, pos_log, t)?;
        let (ord4, ord5) = calc_rkf45_step(dt, state, |s| jacobian * s);
        let error = (ord4 - ord5).norm_squared() / 6.0;
        if error <= epsilon {
            state = ord4;
            t += dt;
        }
        if error.is_nan() {
            println!("stm");
            dbg!(ord4);
            dbg!(ord5);
        }
        adjust_rkf45_dt(&mut dt, epsilon, error);
    }

    Some(state)
}
