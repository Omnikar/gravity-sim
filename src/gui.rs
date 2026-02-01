use eframe::egui::{
    self, Color32, ColorImage, Context, DragValue, Key, Label, Sense, Slider, Spinner,
};
use egui_plot::{HLine, Line, Plot, PlotPoints, Points, VLine};
use image::ImageBuffer;
use itertools::Itertools;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, SQRT_2};
use std::io::Write;

use crate::cr3bp::{Cr3bs, Cr3bsMt, Vec3};

const BG_COLOR: Color32 = Color32::BLACK;
// const EXPORT_BG_COLOR: Color32 = Color32::TRANSPARENT;
const EXPORT_BG_COLOR: Color32 = Color32::BLACK;

const M1_COLOR: Color32 = Color32::BLUE;
const M2_COLOR: Color32 = Color32::GRAY;

// const M1_COLOR: Color32 = Color32::LIGHT_YELLOW;
// const M2_COLOR: Color32 = Color32::ORANGE;

const M3_COLOR: Color32 = Color32::RED;

const TRAIL_COLOR: Color32 = Color32::from_rgb(0xff, 0, 0xff);

const VIEWPORT_SIZE: usize = 512;
const EXPORT_SIZE: usize = 512;
// const EXPORT_SIZE: usize = 4096;

type Ui<'a> = &'a mut egui::Ui;

pub struct App {
    pub system: Cr3bsMt,
    pub autosave: bool,
    pub export_size: usize,
    pub dt0: f64,
    pub epsilon_order: i32,
    pub duration: f64,
    pub playback_speed: f64,
    pub paused: bool,
    pub t: f64,
    pub view_configs: ViewConfigs,
    pub history: f64,
    pub full_history: bool,
    sim_needed: bool,
    sim_rx_tx: Option<(std::sync::mpsc::Receiver<f64>, std::sync::mpsc::Sender<()>)>,
    sim_progress: f64,
    // save_handle: Option<thread::JoinHandle<()>>,
    // pub system_alt: Cr3bs,
}

#[derive(Clone, Copy)]
pub struct ViewConfigs {
    pub inertial_vcs: [ViewConfig; 4],
    pub rotating_vcs: [ViewConfig; 4],
}
impl Default for ViewConfigs {
    fn default() -> Self {
        Self {
            inertial_vcs: [
                // xy
                [0.0, 0.0, 1.0, 0.0, 0.0],
                // yz
                [0.0, 0.0, 1.0, FRAC_PI_2, FRAC_PI_2],
                // xz
                [0.0, 0.0, 1.0, 0.0, FRAC_PI_2],
                // isometric
                [0.0, 0.0, 1.0, FRAC_PI_4, SQRT_2.atan()],
            ]
            .map(Into::into),
            rotating_vcs: [
                [1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, FRAC_PI_2, FRAC_PI_2],
                [1.0, 0.0, 0.0, 0.0, FRAC_PI_2],
                [0.7, -0.4, -0.25, FRAC_PI_4, SQRT_2.atan()],
            ]
            .map(Into::into),
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
pub struct ViewConfig {
    cx: f64,
    cy: f64,
    scale: f64,
    theta: f64,
    phi: f64,
    show_controls: bool,
}
impl From<[f64; 5]> for ViewConfig {
    fn from([cx, cy, scale, theta, phi]: [f64; 5]) -> Self {
        Self {
            cx,
            cy,
            scale,
            theta,
            phi,
            show_controls: false,
        }
    }
}

impl App {
    pub fn new(system: Cr3bs, dt0: f64, epsilon_order: i32, duration: f64) -> Self {
        // let system_alt = system.clone();
        Self {
            system: system.into(),
            autosave: true,
            export_size: EXPORT_SIZE,
            dt0,
            epsilon_order,
            duration,
            playback_speed: 1.0,
            paused: true,
            t: 0.0,
            history: 10.0,
            full_history: true,
            view_configs: Default::default(),
            sim_needed: true,
            sim_rx_tx: None,
            sim_progress: 0.0,
            // save_handle: None,
            // system_alt,
        }
    }

    pub fn run(mut self) -> eframe::Result {
        let options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default().with_inner_size([1420.0, 770.0]),
            ..Default::default()
        };

        eframe::run_simple_native("CR3BP Simulation", options, move |ctx, _frame| {
            egui::CentralPanel::default().show(ctx, |ui| self.event_loop(ctx, ui));
        })
    }

    fn event_loop(&mut self, ctx: &Context, ui: Ui) {
        ui.horizontal(|ui| self.draw_controls(ui, ctx));

        self.handle_playback(ctx);

        if !self.sim_needed {
            let size = ui.available_width() / 4.08;
            // let size = ui.available_height() * 0.97;

            for inertial in [true, false] {
                ui.horizontal(|ui| {
                    for perspective in 0..4 {
                        // if !inertial && perspective == 2 {
                        //     // self.draw_jacobi_plot(ui, size);
                        //     // self.draw_period_plot(ui, size);
                        //     // self.draw_proximity_plot(ui, size);
                        //     // self.draw_deltav_plot(ui, size);
                        //     self.draw_cumul_deltav_plot(ui, size);
                        // } else if inertial && perspective == 1 {
                        //     self.draw_period_error_plot(ui, size);
                        // } else if inertial && perspective == 2 {
                        //     self.draw_deltav_plot(ui, size);
                        // // } else if !inertial && perspective == 1 {
                        // //     self.draw_jacobi_plot(ui, size);

                        if inertial && perspective == 0 {
                            self.draw_period_error_plot(ui, size);
                        } else if inertial && perspective == 1 {
                            self.draw_cumul_deltav_plot(ui, size);
                        } else if inertial && perspective == 2 {
                            self.draw_deltav_plot(ui, size);
                        } else {
                            self.draw_view(ctx, ui, inertial, perspective, size);
                        }
                    }
                });
            }
            // self.draw_view(ctx, ui, false, 3, size);
        } else if let Some((rx, tx)) = self.sim_rx_tx.take() {
            if self.system.update() {
                // println!("final value: {}", rx.recv().unwrap());
                if self.autosave {
                    self.save();
                }
                self.sim_needed = false;
            } else {
                // let progress = rx
                //     .recv()
                //     .map(|v| v.to_string())
                //     .unwrap_or_else(|_| "-".to_owned());
                // let progress = rx.try_iter().last();
                // let progress_s = progress
                //     .map(|v| format!("{v:.3}"))
                //     .unwrap_or_else(|| "-".to_owned());
                if let Some(progress) = rx.try_iter().last() {
                    self.sim_progress = progress;
                }
                ui.label(format!("{:.3}/{}", self.sim_progress, self.duration));

                if ui.button("Stop").clicked() {
                    tx.send(()).unwrap();
                    self.duration = self.sim_progress;
                }

                self.sim_rx_tx = Some((rx, tx));
                ctx.request_repaint();
            }
        } else if ui
            .button(egui::RichText::new("Simulate").size(30.0))
            .clicked()
            || ctx.input(|i| i.key_pressed(Key::Space))
        {
            self.run_sim();
            // self.save();
            // self.sim_needed = false;
        }
    }

    fn handle_playback(&mut self, ctx: &Context) {
        self.paused ^= ctx.input(|i| i.key_pressed(Key::Space));
        self.paused |= self.sim_needed;

        if ctx.input(|i| i.key_pressed(Key::Equals)) {
            self.t = 0.0;
        }

        let playback_keys = |step: f64, left: Key, right: Key| {
            step * self.playback_speed
                * ctx.input(|i| {
                    i.key_pressed(right) as i64 as f64 - i.key_pressed(left) as i64 as f64
                })
        };

        self.t += playback_keys(0.1, Key::Comma, Key::Period);
        self.t += playback_keys(1.0, Key::ArrowLeft, Key::ArrowRight);
        self.t += playback_keys(5.0, Key::J, Key::L);
        self.t = self.t.clamp(0.0, self.duration);

        if !self.paused {
            self.t += self.playback_speed * ctx.input(|i| i.stable_dt) as f64;
            self.t = self.t.rem_euclid(self.duration);
            ctx.request_repaint();
        }
    }

    fn draw_controls(&mut self, ui: Ui, ctx: &Context) {
        if let Some(handle) = self.system.save_handle.take() {
            if handle.is_finished() {
                handle.join().unwrap();
            } else {
                self.system.save_handle = Some(handle);
                ui.label("Saving");
                ui.add(Spinner::new());
            }
        } else if ctx.input(|i| i.key_pressed(Key::S)) {
            self.save();
        }

        // if ui.button("Report").clicked() {
        //     let report = self.system.inner.orbit_report();
        //     println!("{report:?}");
        // }
        // if ui.button("Monodromy").clicked() {
        //     // if let Some(monodromy) =
        //     //     self.system
        //     //         .inner
        //     //         .monodromy(self.t, self.dt0, 10f64.powi(-self.epsilon_order))
        //     // {
        //     //     println!("{monodromy}");
        //     //     // let a = monodromy.bidiagonalize();
        //     //     // println!("{a:?}");
        //     //     // let b = monodromy.complex_eigenvalues();
        //     //     // println!("{b}");
        //     //     // let c = monodromy.symmetric_eigen();
        //     //     // println!("{c:?}");
        //     //     // let d = monodromy.transpose().symmetric_eigen();
        //     //     // println!("{d:?}");
        //     //     use nalgebra_lapack::Eigen;
        //     //     if let Some(eigen) = Eigen::new(monodromy, false, true) {
        //     //         println!("{}", eigen.eigenvalues_re);
        //     //         println!("{}", eigen.eigenvalues_im);
        //     //         let eigenvectors = eigen.eigenvectors.unwrap();
        //     //         println!("{eigenvectors}");
        //     //     }
        //     // } else {
        //     //     println!("Monodromy matrix creation failed");
        //     // }
        //     if let Some(stm) = self.system.inner.stm(
        //         0.1,
        //         2.4829089190914457,
        //         self.dt0,
        //         10f64.powi(-self.epsilon_order),
        //     ) {
        //         let state = self
        //             .system
        //             .inner
        //             .state_at(2.4829089190914457)
        //             .unwrap()
        //             .remove_rows_at(&[0, 2, 4]);
        //         let stm = stm.remove_columns_at(&[0, 2, 4]).remove_fixed_rows::<3>(0);
        //         let correction = stm.pseudo_inverse(1e-14).expect("STM inversion failed") * state;
        //         println!("{correction}");
        //     }
        // }

        ui.checkbox(&mut self.autosave, "Autosave");

        ui.add(Label::new("Export size"));
        ui.add(DragValue::new(&mut self.export_size));

        let dt_before = self.dt0;
        ui.add(Label::new("Δt₀"));
        ui.add(
            DragValue::new(&mut self.dt0)
                .range(0.0..=0.05)
                .speed(0.0001),
        );

        let eps_ord_before = self.epsilon_order;
        // ui.add(Label::new("-log(ε)"));
        ui.add(Label::new("pε"));
        ui.add(DragValue::new(&mut self.epsilon_order).range(0..=12));

        if self.dt0 != dt_before || self.epsilon_order != eps_ord_before {
            self.sim_needed = true;
            self.system.inner.time_log.truncate(1);
            self.system.inner.pos_log.truncate(1);
            self.system.inner.vel_log.truncate(1);
            self.system.inner.jacobi_log.truncate(1);

            // self.system_alt.time_log.truncate(1);
            // self.system_alt.pos_log.truncate(1);
            // self.system_alt.vel_log.truncate(1);
            // self.system_alt.jacobi_log.truncate(1);
        }

        ui.add(Label::new("Duration"));
        ui.add(DragValue::new(&mut self.duration).speed(0.1));
        self.duration = self.duration.max(0.0);
        self.sim_needed |= *self.system.inner.time_log.last().unwrap() < self.duration;

        ui.add(Label::new("Trail"));
        ui.add(DragValue::new(&mut self.history).speed(0.1));
        self.history = self.history.max(0.0);

        ui.checkbox(&mut self.full_history, "Full");
        if self.full_history {
            self.history = self.duration;
        }

        ui.add(Label::new("Playback Speed"));
        ui.add(DragValue::new(&mut self.playback_speed).speed(0.1));

        self.paused ^= ui.button(if self.paused { "⏵" } else { "⏸" }).clicked();

        ui.style_mut().spacing.slider_width = ui.available_width() - 60.0;
        ui.add(
            Slider::new(&mut self.t, 0.0..=self.duration)
                .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.1 }),
        );
    }

    fn draw_period_plot(&self, ui: Ui, size: f32) {
        let sq_dists = self
            .system
            .inner
            .pos_log
            .iter()
            .map(|v| (v - (1.0 - self.system.inner.mass_ratio) * Vec3::x()).norm_squared());
        let period_points: PlotPoints = self
            .system
            .inner
            .time_log
            .iter()
            .copied()
            .zip(sq_dists)
            .skip_while(|&(t, _)| t < self.t - self.history)
            .take_while(|&(t, _)| t < self.t)
            .tuple_windows()
            .filter_map(|((_, r0), (t1, r1), (_, r2))| ((r0 < r1) ^ (r1 < r2)).then_some(t1))
            .tuple_windows()
            .map(|(a, b)| (b - a) * 2.0)
            .enumerate()
            .map(|(i, p)| [i as f64, p])
            .collect();
        let line = Line::new(period_points);

        self.draw_plot(
            ui,
            size,
            [line],
            [],
            [],
            [],
            "orbital period",
            "Period Plot",
        );
    }

    fn draw_period_error_plot(&self, ui: Ui, size: f32) {
        let sq_dists = self
            .system
            .inner
            .pos_log
            .iter()
            .map(|v| (v - (1.0 - self.system.inner.mass_ratio) * Vec3::x()).norm_squared());
        let period_points: PlotPoints = self
            .system
            .inner
            .time_log
            .iter()
            .copied()
            .zip(sq_dists)
            .skip_while(|&(t, _)| t < self.t - self.history)
            .take_while(|&(t, _)| t < self.t)
            .tuple_windows()
            .filter_map(|((_, r0), (t1, r1), (_, r2))| ((r0 < r1) ^ (r1 < r2)).then_some(t1))
            .tuple_windows()
            .map(|(a, b)| (b - a) * 2.0)
            .map(|p| (p - self.system.inner.period).abs().log10())
            .filter(|&e| e >= -15.0)
            .enumerate()
            .map(|(i, p)| [i as f64, p])
            .collect();
        let line = Line::new(period_points);

        self.draw_plot(
            ui,
            size,
            [line],
            [],
            [],
            [],
            "period error log",
            "Period Error Log Plot",
        );
    }

    fn draw_proximity_plot(&self, ui: Ui, size: f32) {
        // let sq_dists = self.system.inner.time_log(self.system.inner.pos_log.iter().map(|v| v.norm()));
        let points: PlotPoints = self
            .system
            .inner
            .timed_pos()
            .skip_while(|&(t, _)| t < self.t - self.history)
            .take_while(|&(t, _)| t < self.t)
            .map(|(t, r)| {
                [
                    t,
                    (r - (1.0 - self.system.inner.mass_ratio) * Vec3::x()).norm(),
                ]
            })
            // .map_while(|(t, r)| (t < self.t).then_some([t, r.norm()]))
            .collect();
        let line = Line::new(points);

        let mut hlines = Vec::new();
        let vlines;
        if let Some(report) = self.system.inner.orbit_report {
            hlines.push(HLine::new(report.periapsis));
            hlines.push(HLine::new(report.apoapsis));

            // vlines = (((self.t - self.history) / report.period).ceil().max(1.0) as usize
            //     ..(self.t / report.period).ceil() as usize)
            //     // .map(|i| dbg!(dbg!(i) as f64 * report.period))
            //     .map(|i| i as f64 * report.period)
            //     .map(VLine::new)
            //     .collect();
            vlines = Vec::new();
        } else {
            vlines = Vec::new();
        }

        self.draw_plot(
            ui,
            size,
            [line],
            hlines,
            vlines,
            [],
            "mass 2 proximity",
            "Proximity Plot",
        );
    }

    fn draw_jacobi_plot(&self, ui: Ui, size: f32) {
        let init_jacobi = self.system.inner.jacobi_log[0];
        let jacobi_points: PlotPoints = self
            .system
            .inner
            .timed_jacobi()
            .skip_while(|&(t, _)| t < self.t - self.history)
            .map_while(|(t, j)| (t < self.t).then_some([t, j - init_jacobi]))
            .collect();
        let line = Line::new(jacobi_points);

        // let dt_points: PlotPoints = self
        //     .system
        //     .time_log
        //     .iter()
        //     .copied()
        //     .zip(self.system.time_log.windows(2).map(|w| w[1] - w[0]))
        //     .map_while(|(t, dt)| (t < self.t).then_some([t, dt]))
        //     .collect();
        // let line = Line::new(dt_points);

        // let jacobi_points_alt: PlotPoints = self
        //     .system_alt
        //     .timed_jacobi()
        //     .map_while(|(t, j)| (t < self.t).then_some([t, j - init_jacobi]))
        //     .collect();
        // let line_alt = Line::new(jacobi_points_alt);

        // ui.vertical(|ui| {
        //     ui.label("jacobi error");
        //     Plot::new("Jacobi Plot")
        //         .view_aspect(1.0)
        //         .width(size)
        //         .y_axis_formatter(|v, _| format!("{:.0e}", v.value))
        //         .show(ui, |plot_ui| {
        //             plot_ui.line(line);
        //             // plot_ui.line(line_alt);
        //         });
        // });

        self.draw_plot(ui, size, [line], [], [], [], "jacobi error", "Jacobi Plot");
    }

    fn draw_deltav_plot(&self, ui: Ui, size: f32) {
        let (times, burns): (Vec<f64>, Vec<Vec3>) =
            self.system.inner.maneuvers.iter().copied().unzip();
        let deltavs = burns.iter().map(Vec3::norm).collect_vec();
        let points: PlotPoints = times
            .into_iter()
            .zip(deltavs)
            .skip_while(|&(t, _)| t < self.t - self.history)
            .take_while(|&(t, _)| t < self.t)
            .map(Into::into)
            .collect();
        let points = Points::new(points);
        // let line = Line::new(points);

        self.draw_plot(ui, size, [], [], [], [points], "Δv usage", "Delta V Plot");
    }

    fn draw_cumul_deltav_plot(&self, ui: Ui, size: f32) {
        let (times, burns): (Vec<f64>, Vec<Vec3>) =
            self.system.inner.maneuvers.iter().copied().unzip();
        let deltavs = burns.iter().map(Vec3::norm).collect_vec();
        let cumul_deltavs = deltavs
            .into_iter()
            .scan(0.0, |st, x| {
                *st += x;
                Some(*st)
            })
            .collect_vec();
        let points: PlotPoints = times
            .into_iter()
            .zip(cumul_deltavs)
            .skip_while(|&(t, _)| t < self.t - self.history)
            .take_while(|&(t, _)| t < self.t)
            .map(Into::into)
            .collect();
        let line = Line::new(points).color(Color32::ORANGE);

        self.draw_plot(
            ui,
            size,
            [line],
            [],
            [],
            [],
            "Δv used",
            "Cumulative Delta V Plot",
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_plot(
        &self,
        ui: Ui,
        size: f32,
        lines: impl IntoIterator<Item = Line>,
        hlines: impl IntoIterator<Item = HLine>,
        vlines: impl IntoIterator<Item = VLine>,
        points: impl IntoIterator<Item = Points>,
        ui_label: &str,
        plot_id: &str,
    ) {
        ui.vertical(|ui| {
            ui.label(ui_label);
            Plot::new(plot_id)
                .view_aspect(1.0)
                .width(size)
                .y_axis_formatter(|v, _| format!("{:.0e}", v.value))
                .show(ui, |plot_ui| {
                    // plot_ui.line(line);
                    // plot_ui.hline(HLine::new(0.011641076714751517));
                    // plot_ui.hline(HLine::new(0.19049717400503915));

                    // plot_ui.hline(HLine::new(0.04378571485476416));
                    // plot_ui.hline(HLine::new(0.22278304755446823));

                    for line in lines {
                        plot_ui.line(line);
                    }
                    for hline in hlines {
                        plot_ui.hline(hline);
                    }
                    for vline in vlines {
                        plot_ui.vline(vline);
                    }
                    for points in points {
                        plot_ui.points(points);
                    }
                });
        });
    }

    fn draw_view(&mut self, ctx: &Context, ui: Ui, inertial: bool, perspective: usize, size: f32) {
        let label = format!(
            "{} {}",
            ["xy", "yz", "xz", "isometric"][perspective],
            ["rotating", "inertial"][inertial as usize]
        );
        let image = self.render_view(inertial, perspective, VIEWPORT_SIZE, BG_COLOR, false);
        let texture = ui.ctx().load_texture(&label, image, Default::default());
        let vcs = if inertial {
            &mut self.view_configs.inertial_vcs
        } else {
            &mut self.view_configs.rotating_vcs
        };
        let vc = &mut vcs[perspective];

        let scale = vc.scale.exp();

        let viewport_control_bar = |vc: &mut ViewConfig, ui: Ui| -> bool {
            let mut should_export = false;
            let controls_button_label = if vc.show_controls {
                for val in [&mut vc.cx, &mut vc.cy] {
                    ui.add(DragValue::new(val).speed(vc.scale.exp() / 100.0));
                }
                for val in [&mut vc.theta, &mut vc.phi] {
                    ui.add(DragValue::new(val).speed(0.02));
                }
                ui.add(DragValue::new(&mut vc.scale).speed(0.03));
                "✔"
            } else {
                ui.label(label);
                should_export = ui.button("E").clicked();
                "#"
            };
            vc.show_controls ^= ui.button(controls_button_label).clicked();

            let def_vcs = ViewConfigs::default();
            let mut def_vc = if inertial {
                def_vcs.inertial_vcs
            } else {
                def_vcs.rotating_vcs
            }[perspective];
            def_vc.show_controls = vc.show_controls;
            if *vc != def_vc && ui.button("↺").clicked() {
                *vc = def_vc;
            }
            should_export
        };

        let should_export = ui.vertical(|ui| {
            let should_export = ui.horizontal(|ui| viewport_control_bar(vc, ui)).inner;

            let image_size = [size; 2].into();
            let response = ui.allocate_response(image_size, Sense::click_and_drag());
            ui.painter().image(
                texture.id(),
                response.rect,
                egui::Rect::from_min_max((0.0, 0.0).into(), (1.0, 1.0).into()),
                Color32::WHITE,
            );

            let delta = response.drag_delta() / size;

            // Default to rotation rather than panning for isometric perspective
            if !ctx.input(|i| i.modifiers.shift) ^ (perspective == 3) {
                let delta = <[f32; 2]>::from(delta).map(|v| v as f64 * scale);
                vc.cx -= delta[0];
                vc.cy += delta[1];
            } else {
                let old_proj = calc_proj_mat(vc.theta, vc.phi).transpose();
                vc.theta -= delta[0] as f64 * PI;
                vc.phi -= delta[1] as f64 * PI;
                let new_proj = calc_proj_mat(vc.theta, vc.phi);
                let mut up_projected = old_proj * nalgebra::Vector2::new(vc.cx, vc.cy);
                let mut z_vector = old_proj.column(0).cross(&old_proj.column(1));
                if (z_vector.z - 0.0).abs() > 1e-6 {
                    z_vector /= z_vector.z;
                    z_vector *= up_projected.z;
                    up_projected -= z_vector;
                }
                let new_center = new_proj * up_projected;
                vc.cx = new_center[0];
                vc.cy = new_center[1];
            }

            if let Some(mouse_pos) = response.hover_pos() {
                let scroll = -ctx.input(|i| i.smooth_scroll_delta.y as f64) / 70.0;
                vc.scale += scroll;

                let mut zoom_pos =
                    <[f32; 2]>::from(mouse_pos - response.rect.min).map(|v| (v / size) as f64);
                zoom_pos[0] -= 0.5;
                zoom_pos[1] -= 0.5;
                zoom_pos[1] *= -1.0;

                let fact = 1.0 - scroll.exp();
                vc.cx += zoom_pos[0] * scale * fact;
                vc.cy += zoom_pos[1] * scale * fact;
            }
            should_export
        });

        if should_export.inner {
            let image = self.render_view(
                inertial,
                perspective,
                self.export_size,
                EXPORT_BG_COLOR,
                true,
            );
            let buffer: ImageBuffer<image::Rgba<u8>, _> =
                ImageBuffer::from_raw(image.width() as u32, image.height() as u32, image.as_raw())
                    .expect("Failed to create image buffer");
            if let Err(e) = buffer.save("artifacts/viewport.png") {
                eprintln!("Failed to save image: {e}");
            }
        }
    }

    fn render_view(
        &self,
        inertial: bool,
        perspective: usize,
        px_size: usize,
        bg_color: Color32,
        print_history: bool,
    ) -> ColorImage {
        let config = if inertial {
            self.view_configs.inertial_vcs
        } else {
            self.view_configs.rotating_vcs
        }[perspective];

        let proj_mat = calc_proj_mat(config.theta, config.phi);
        let project_viewport = |v: Vec3| {
            let v = proj_mat * v;
            [v[0], v[1]]
        };

        let history = self
            .system
            .inner
            .timed_pos()
            .skip_while(|&(t, _)| t < self.t - self.history)
            .take_while_inclusive(|&(t, _)| t <= self.t)
            .map(|(t, p)| rotate_inertial(p, t, inertial))
            .map(project_viewport)
            .collect_vec();

        // let history_alt: Vec<_> = self
        //     .system_alt
        //     .timed_pos()
        //     .skip_while(|&(t, _)| t < self.t - self.history)
        //     .take_while_inclusive(|&(t, _)| t <= self.t)
        //     .map(|(t, p)| rotate_inertial(p, t, inertial))
        //     .map(project_viewport)
        //     .collect();

        // let px_size = VIEWPORT_SIZE;
        let mut image = ColorImage::new([px_size; 2], bg_color);
        // let mut image = ColorImage::new([px_size; 2], Color32::TRANSPARENT);
        let mut put_px = |row, col, color| {
            if row >= 0 && row < px_size as isize && col >= 0 && col < px_size as isize {
                let width = image.width();
                image.pixels[width * row as usize + col as usize] = color;
            }
        };

        let scale = config.scale.exp();
        let world2px = |p: [f64; 2]| {
            [
                ((config.cy - p[1]) / scale + 0.5) * px_size as f64,
                ((p[0] - config.cx) / scale + 0.5) * px_size as f64,
            ]
        };

        // FIXME: Trail fading doesn't work. Fix or remove it.
        let mut draw_trail = |history: &[[f64; 2]], mut color: Color32, fade: bool| {
            let iter = history.windows(2).enumerate();
            let segs = iter.len() as f64;
            for (seg_i, ends) in iter {
                let [start, end] = [ends[0], ends[1]].map(world2px);
                let size = px_size as f64;
                if [start, end]
                    .as_flattened()
                    .iter()
                    .any(|&v| v < 0.0 || v >= size)
                {
                    continue;
                }
                let steps = (end[0] - start[0])
                    .abs()
                    .max((end[1] - start[1]).abs())
                    .floor() as usize
                    + 1;
                for i in 0..steps {
                    let lerp = i as f64 / steps as f64;
                    let row = (1.0 - lerp) * start[0] + lerp * end[0];
                    let col = (1.0 - lerp) * start[1] + lerp * end[1];
                    if fade {
                        let fade_val = 1f64.min((seg_i as f64 + lerp) / segs);
                        color = color.gamma_multiply(fade_val as f32);
                    }
                    put_px(row as isize, col as isize, color);
                }
            }
        };

        draw_trail(&history, TRAIL_COLOR, false /*true*/);
        // draw_trail(&history_alt, Color32::ORANGE, false /*true*/);

        if inertial {
            let circle_steps = 1024;
            let circle_history = (0..=circle_steps)
                .map(|i| i as f64 * std::f64::consts::TAU / circle_steps as f64)
                .map(|t| Vec3::new(t.cos(), t.sin(), 0.0));
            let m1_history = circle_history
                .clone()
                .map(|v| v * self.system.inner.mass_ratio)
                .map(project_viewport)
                .collect_vec();
            let m2_history = circle_history
                .clone()
                .map(|v| v * (1.0 - self.system.inner.mass_ratio))
                .map(project_viewport)
                .collect_vec();
            draw_trail(&m1_history, M1_COLOR, false);
            draw_trail(&m2_history, M2_COLOR, false);
        }

        let m1_pos = project_viewport(rotate_inertial(
            -self.system.inner.mass_ratio * Vec3::x(),
            self.t,
            inertial,
        ));
        let m2_pos = project_viewport(rotate_inertial(
            (1.0 - self.system.inner.mass_ratio) * Vec3::x(),
            self.t,
            inertial,
        ));
        let m3_pos = *history.last().unwrap();
        let circles = [
            m1_pos, m2_pos, m3_pos, /* *history_alt.last().unwrap()*/
        ]
        .into_iter()
        .map(world2px)
        .zip([M1_COLOR, M2_COLOR, M3_COLOR, Color32::GREEN]);
        for ([row, col], color) in circles {
            let r = 5;
            (-r..=r)
                .flat_map(|i| (-r..=r).map(move |j| (i, j)))
                .filter(|(i, j)| i * i + j * j < r * r)
                .for_each(|(i, j)| put_px(row as isize + i, col as isize + j, color));
        }

        if print_history {
            use std::io::Write;
            // println!("{history:?}");
            let mut history_f = std::io::BufWriter::new(
                std::fs::File::create("artifacts/rendered_orbit.csv").unwrap(),
            );
            // writeln!(history_f, "{history:?}").unwrap();
            writeln!(
                history_f,
                "{},{}",
                config.cx - scale / 2.0,
                config.cx + scale / 2.0
            )
            .unwrap();
            writeln!(
                history_f,
                "{},{}",
                config.cy - scale / 2.0,
                config.cy + scale / 2.0
            )
            .unwrap();
            writeln!(history_f, "{},{}", m1_pos[0], m1_pos[1],).unwrap();
            writeln!(history_f, "{},{}", m2_pos[0], m2_pos[1],).unwrap();
            for [x, y] in history.iter() {
                writeln!(history_f, "{x},{y}").unwrap();
            }
            // println!("x: {}_{}", config.cx - scale / 2.0, config.cx + scale / 2.0);
            // println!("y: {}_{}", config.cy - scale / 2.0, config.cy + scale / 2.0);
        }

        image
    }

    fn run_sim(&mut self) {
        let remaining_t = self.duration - self.system.inner.time_log.last().unwrap();
        // self.system.inner.rkf45(self.dt0, 1e-10, remaining_t); // TODO: async this
        self.sim_rx_tx = Some(self.system.rkf45_threaded(
            self.dt0,
            10f64.powi(-self.epsilon_order),
            remaining_t,
        ));

        // self.system_alt.verlet(self.dt0, remaining_t);
    }

    // fn save(&mut self) {
    //     // let system = self.system.clone();

    //     // self.save_handle = Some(thread::spawn(move || {
    //     //     let mut pos_f = std::fs::File::create("pos_log.csv").unwrap();
    //     //     system.write_pos_log(&mut pos_f);
    //     //     let mut vel_f = std::fs::File::create("vel_log.csv").unwrap();
    //     //     system.write_vel_log(&mut vel_f);
    //     //     let mut time_f = std::fs::File::create("time_log.csv").unwrap();
    //     //     system.write_time_log(&mut time_f);
    //     //     let mut jacobi_f = std::fs::File::create("jacobi_log.csv").unwrap();
    //     //     system.write_jacobi_log(&mut jacobi_f);
    //     // }));

    //     // let mut pos_f_alt = std::fs::File::create("pos_log_alt.csv").unwrap();
    //     // self.system.write_pos_log(&mut pos_f_alt);
    //     // let mut vel_f_alt = std::fs::File::create("vel_log_alt.csv").unwrap();
    //     // self.system.write_vel_log(&mut vel_f_alt);
    //     // let mut time_f_alt = std::fs::File::create("time_log_alt.csv").unwrap();
    //     // self.system.write_time_log(&mut time_f_alt);
    //     // let mut jacobi_f_alt = std::fs::File::create("jacobi_log_alt.csv").unwrap();
    //     // self.system.write_jacobi_log(&mut jacobi_f_alt);
    // }

    fn save(&mut self) {
        self.system.save();
        let mut metadata_f = std::fs::File::create("artifacts/metadata.txt").unwrap();
        writeln!(
            metadata_f,
            "dt0: {}\npe: {}\nduration: {}",
            self.dt0, self.epsilon_order, self.duration
        )
        .unwrap();
    }
}

fn rotate(v: Vec3, a: f64) -> Vec3 {
    nalgebra::Rotation3::from_axis_angle(&Vec3::z_axis(), a) * v
}

fn rotate_inertial(v: Vec3, a: f64, inertial: bool) -> Vec3 {
    if inertial {
        rotate(v, a)
    } else {
        v
    }
}

fn calc_proj_mat(theta: f64, phi: f64) -> nalgebra::Matrix2x3<f64> {
    (nalgebra::Rotation3::from_axis_angle(&Vec3::z_axis(), theta)
        * nalgebra::Rotation3::from_axis_angle(&Vec3::x_axis(), phi))
    .matrix()
    .transpose()
    .remove_row(2)
}
