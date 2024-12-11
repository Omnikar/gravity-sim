use eframe::egui::{self, Color32, ColorImage, Context, DragValue, Key, Label, Sense, Slider};
use itertools::Itertools;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, SQRT_2};

use crate::cr3bp::{Cr3bs, Vec3};

const M1_COLOR: Color32 = Color32::BLUE;
const M2_COLOR: Color32 = Color32::GRAY;
const M3_COLOR: Color32 = Color32::RED;
const TRAIL_COLOR: Color32 = Color32::from_rgb(0xff, 0, 0xff);

const VIEWPORT_SIZE: usize = 512;

type Ui<'a> = &'a mut egui::Ui;

pub struct App {
    pub system: Cr3bs,
    pub dt0: f64,
    pub duration: f64,
    pub playback_speed: f64,
    pub paused: bool,
    pub t: f64,
    pub view_configs: ViewConfigs,
    pub history: f64,
    sim_needed: bool,
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
    pub fn new(system: Cr3bs, dt0: f64, duration: f64) -> Self {
        // let system_alt = system.clone();
        Self {
            system,
            dt0,
            duration,
            playback_speed: 1.0,
            paused: true,
            t: 0.0,
            history: 10.0,
            view_configs: Default::default(),
            sim_needed: true,
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
        ui.horizontal(|ui| self.draw_controls(ui));

        self.handle_playback(ctx);

        if !self.sim_needed {
            let size = ui.available_width() / 4.08;

            for inertial in [true, false] {
                ui.horizontal(|ui| {
                    for perspective in 0..4 {
                        if !inertial && perspective == 2 {
                            self.draw_jacobi_plot(ui, size);
                        } else {
                            self.draw_view(ctx, ui, inertial, perspective, size);
                        }
                    }
                });
            }
        } else if ui
            .button(egui::RichText::new("Simulate").size(30.0))
            .clicked()
            || ctx.input(|i| i.key_pressed(Key::Space))
        {
            self.run_sim();
            self.sim_needed = false;
        }
    }

    fn handle_playback(&mut self, ctx: &Context) {
        self.paused ^= ctx.input(|i| i.key_pressed(Key::Space));
        self.paused |= self.sim_needed;

        if ctx.input(|i| i.key_pressed(Key::Num0)) {
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

    fn draw_controls(&mut self, ui: Ui) {
        let dt_before = self.dt0;
        ui.add(Label::new("Δt₀"));
        ui.add(
            DragValue::new(&mut self.dt0)
                .range(0.0..=0.05)
                .speed(0.0001),
        );
        if self.dt0 != dt_before {
            self.sim_needed = true;
            self.system.time_log.truncate(1);
            self.system.pos_log.truncate(1);
            self.system.vel_log.truncate(1);
            self.system.jacobi_log.truncate(1);

            // self.system_alt.time_log.truncate(1);
            // self.system_alt.pos_log.truncate(1);
            // self.system_alt.vel_log.truncate(1);
            // self.system_alt.jacobi_log.truncate(1);
        }

        ui.add(Label::new("Duration"));
        ui.add(DragValue::new(&mut self.duration).speed(0.1));
        self.duration = self.duration.max(0.0);
        self.sim_needed |= *self.system.time_log.last().unwrap() < self.duration;

        ui.add(Label::new("Trail"));
        ui.add(DragValue::new(&mut self.history).speed(0.1));
        self.history = self.history.max(0.0);

        ui.add(Label::new("Playback Speed"));
        ui.add(DragValue::new(&mut self.playback_speed).speed(0.1));

        if ui.button(if self.paused { "⏵" } else { "⏸" }).clicked() {
            self.paused ^= true;
        }

        ui.style_mut().spacing.slider_width = ui.available_width() - 60.0;
        ui.add(
            Slider::new(&mut self.t, 0.0..=self.duration)
                .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.1 }),
        );
    }

    fn draw_jacobi_plot(&self, ui: Ui, size: f32) {
        use egui_plot::{Line, Plot, PlotPoints};
        let jacobi_points: PlotPoints = self
            .system
            .timed_jacobi()
            .map_while(|(t, j)| (t < self.t).then_some([t, j]))
            .collect();
        let line = Line::new(jacobi_points);

        // let jacobi_points_alt: PlotPoints = self
        //     .system_alt
        //     .timed_jacobi()
        //     .map_while(|(t, j)| (t < self.t).then_some([t, j]))
        //     .collect();
        // let line_alt = Line::new(jacobi_points_alt);

        ui.vertical(|ui| {
            ui.label("jacobi constant");
            Plot::new("Jacobi Plot")
                .view_aspect(1.0)
                .width(size)
                .show(ui, |plot_ui| {
                    plot_ui.line(line);
                    // plot_ui.line(line_alt);
                });
        });
    }

    fn draw_view(&mut self, ctx: &Context, ui: Ui, inertial: bool, perspective: usize, size: f32) {
        let label = format!(
            "{} {}",
            ["xy", "yz", "xz", "isometric"][perspective],
            ["rotating", "inertial"][inertial as usize]
        );
        let image = self.render_view(self.history, inertial, perspective);
        let texture = ui.ctx().load_texture(&label, image, Default::default());
        let vcs = if inertial {
            &mut self.view_configs.inertial_vcs
        } else {
            &mut self.view_configs.rotating_vcs
        };
        let vc = &mut vcs[perspective];

        let scale = vc.scale.exp();

        let viewport_control_bar = |vc: &mut ViewConfig, ui: Ui| {
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
        };

        ui.vertical(|ui| {
            ui.horizontal(|ui| viewport_control_bar(vc, ui));

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
        });
    }

    fn render_view(
        &self,
        history_duration: f64, /*history_len: usize*/
        inertial: bool,
        perspective: usize,
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

        let history: Vec<_> = self
            .system
            .timed_pos()
            .skip_while(|&(t, _)| t < self.t - history_duration)
            .take_while_inclusive(|&(t, _)| t <= self.t)
            .map(|(t, p)| rotate_inertial(p, t, inertial))
            .map(project_viewport)
            .collect();

        // let history_alt: Vec<_> = self
        //     .system_alt
        //     .timed_pos()
        //     .skip_while(|&(t, _)| t < self.t - history_duration)
        //     .take_while_inclusive(|&(t, _)| t <= self.t)
        //     .map(|(t, p)| rotate_inertial(p, t, inertial))
        //     .map(project_viewport)
        //     .collect();

        let px_size = VIEWPORT_SIZE;
        let mut image = ColorImage::new([px_size; 2], Color32::BLACK);
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
                    let t = i as f64 / steps as f64;
                    let row = (1.0 - t) * start[0] + t * end[0];
                    let col = (1.0 - t) * start[1] + t * end[1];
                    if fade {
                        let fade_val = 1f64.min((seg_i as f64 + t) / segs + 1.0);
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
                .map(|v| v * self.system.mass_ratio)
                .map(project_viewport)
                .collect::<Vec<_>>();
            let m2_history = circle_history
                .clone()
                .map(|v| v * (1.0 - self.system.mass_ratio))
                .map(project_viewport)
                .collect::<Vec<_>>();
            draw_trail(&m1_history, M1_COLOR, false);
            draw_trail(&m2_history, M2_COLOR, false);
        }

        let m1_pos = project_viewport(rotate_inertial(
            -self.system.mass_ratio * Vec3::x(),
            self.t,
            inertial,
        ));
        let m2_pos = project_viewport(rotate_inertial(
            (1.0 - self.system.mass_ratio) * Vec3::x(),
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

        image
    }

    fn run_sim(&mut self) {
        let remaining_t = self.duration - self.system.time_log.last().unwrap();
        self.system.rkf45(self.dt0, 1e-10, remaining_t);

        // self.system_alt.verlet(self.dt0, remaining_t);

        let mut pos_f = std::fs::File::create("pos_log.csv").unwrap();
        self.system.write_pos_log(&mut pos_f);
        let mut vel_f = std::fs::File::create("vel_log.csv").unwrap();
        self.system.write_vel_log(&mut vel_f);
        let mut time_f = std::fs::File::create("time_log.csv").unwrap();
        self.system.write_time_log(&mut time_f);
        let mut jacobi_f = std::fs::File::create("jacobi_log.csv").unwrap();
        self.system.write_jacobi_log(&mut jacobi_f);

        // let mut pos_f_alt = std::fs::File::create("pos_log_alt.csv").unwrap();
        // self.system.write_pos_log(&mut pos_f_alt);
        // let mut vel_f_alt = std::fs::File::create("vel_log_alt.csv").unwrap();
        // self.system.write_vel_log(&mut vel_f_alt);
        // let mut time_f_alt = std::fs::File::create("time_log_alt.csv").unwrap();
        // self.system.write_time_log(&mut time_f_alt);
        // let mut jacobi_f_alt = std::fs::File::create("jacobi_log_alt.csv").unwrap();
        // self.system.write_jacobi_log(&mut jacobi_f_alt);
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
