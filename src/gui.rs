use eframe::egui::{self, Color32, ColorImage, Context, DragValue, Key, Label, Sense, Slider};
use nalgebra::Matrix2x3;
use std::io::Write;

use crate::cr3bp::{Cr3bs, Vec3};

const M1_COLOR: Color32 = Color32::BLUE;
const M2_COLOR: Color32 = Color32::GRAY;
const M3_COLOR: Color32 = Color32::RED;
const TRAIL_COLOR: Color32 = Color32::from_rgb(0xff, 0, 0xff);

const VIEWPORT_SIZE: usize = 512;

type Ui<'a> = &'a mut egui::Ui;

pub struct App {
    pub system: Cr3bs,
    pub dt: f64,
    pub duration: f64,
    pub playback_speed: f64,
    pub paused: bool,
    pub t: f64,
    pub view_configs: ViewConfigs,
    pub history: f64,
    sim_needed: bool,
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
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 1.0),
            ]
            .map(Into::into),
            rotating_vcs: [
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.7, -0.4, -0.25),
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
}
impl From<(f64, f64, f64)> for ViewConfig {
    fn from((cx, cy, scale): (f64, f64, f64)) -> Self {
        Self { cx, cy, scale }
    }
}

impl App {
    pub fn new(system: Cr3bs, dt: f64, duration: f64) -> Self {
        Self {
            system,
            dt,
            duration,
            playback_speed: 1.0,
            paused: true,
            t: 0.0,
            history: 10.0,
            view_configs: Default::default(),
            sim_needed: true,
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

        if !self.sim_needed {
            let size = ui.available_width() / 4.08;

            for inertial in [true, false] {
                ui.horizontal(|ui| {
                    (0..4).for_each(|perspect| self.draw_view(ctx, ui, inertial, perspect, size));
                });
            }
        } else if ui
            .button(egui::RichText::new("Simulate").size(30.0))
            .clicked()
        {
            self.run_sim();
            self.sim_needed = false;
        }

        self.handle_playback(ctx);
    }

    fn handle_playback(&mut self, ctx: &Context) {
        self.paused ^= ctx.input(|i| i.key_pressed(Key::Space));
        self.paused |= self.sim_needed;

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
        let dt_before = self.dt;
        ui.add(Label::new("Δt"));
        ui.add(DragValue::new(&mut self.dt).range(0.0..=0.05).speed(0.0001));
        if self.dt != dt_before {
            self.sim_needed = true;
            self.system.pos_log.truncate(1);
            self.system.vel_log.truncate(1);
        }

        ui.add(Label::new("Duration"));
        ui.add(DragValue::new(&mut self.duration).speed(0.1));
        self.duration = self.duration.max(0.0);
        self.sim_needed |= (self.duration / self.dt).ceil() as usize > self.system.pos_log.len();

        ui.add(Label::new("Trail"));
        ui.add(DragValue::new(&mut self.history).speed(0.1));

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

    fn draw_view(&mut self, ctx: &Context, ui: Ui, inertial: bool, perspective: usize, size: f32) {
        let label = format!(
            "{} {}",
            ["xy", "yz", "xz", "isometric"][perspective],
            ["rotating", "inertial"][inertial as usize]
        );
        let history_len = (self.history / self.dt).floor() as usize + 1;
        let image = self.render_view(history_len, inertial, perspective);
        let texture = ui.ctx().load_texture(&label, image, Default::default());
        let vcs = if inertial {
            &mut self.view_configs.inertial_vcs
        } else {
            &mut self.view_configs.rotating_vcs
        };
        let vc = &mut vcs[perspective];

        let scale = vc.scale.exp();

        let viewport_control_bar = |vc: &mut ViewConfig, ui: Ui| {
            ui.label(label);
            for val in [&mut vc.cx, &mut vc.cy] {
                ui.add(DragValue::new(val).speed(vc.scale.exp() / 100.0));
            }
            ui.add(DragValue::new(&mut vc.scale).speed(0.03));
            let def_vcs = ViewConfigs::default();
            let def_vc = if inertial {
                def_vcs.inertial_vcs
            } else {
                def_vcs.rotating_vcs
            }[perspective];
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

            let delta = <[f32; 2]>::from(response.drag_delta() / size).map(|v| v as f64 * scale);
            vc.cx -= delta[0];
            vc.cy += delta[1];
            if response.hovered() {
                let scroll = ctx.input(|i| i.smooth_scroll_delta.y as f64) / 70.0;
                vc.scale -= scroll;
            }
        });
    }

    fn render_view(&self, history_len: usize, inertial: bool, perspective: usize) -> ColorImage {
        let config = if inertial {
            self.view_configs.inertial_vcs
        } else {
            self.view_configs.rotating_vcs
        }[perspective];

        use std::f64::consts::FRAC_1_SQRT_2;
        let frac_sqrt_3_2 = 3f64.sqrt() / 2.0;
        let frac_sqrt_3_4 = frac_sqrt_3_2 / 2.0;
        let proj_mats = [
            Matrix2x3::identity(),
            Matrix2x3::new(0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            Matrix2x3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            Matrix2x3::new(
                FRAC_1_SQRT_2,
                FRAC_1_SQRT_2,
                0.0,
                -frac_sqrt_3_4,
                frac_sqrt_3_4,
                frac_sqrt_3_2,
            ),
        ];
        let project_viewport = |v: Vec3| {
            let v = proj_mats[perspective] * v;
            [v[0], v[1]]
        };

        let end_i = ((self.t / self.dt) as usize + 1).clamp(0, self.system.pos_log.len() - 1);
        let start_i = end_i.saturating_sub(history_len);
        let history = self.system.pos_log[start_i..end_i]
            .iter()
            .copied()
            .enumerate()
            .map(|(i, v)| rotate_inertial(v, (i + start_i) as f64 * self.dt, inertial))
            .map(project_viewport)
            .collect::<Vec<_>>();

        let px_size = VIEWPORT_SIZE;
        let mut image = ColorImage::new([px_size; 2], Color32::BLACK);
        let mut put_px = |row, col, color| {
            if row >= 0 && row < px_size as isize && col >= 0 && col < px_size as isize {
                let width = image.width();
                image.pixels[width * row as usize + col as usize] = color;
            }
        };

        let world2px = |p: [f64; 2]| {
            [
                ((config.cy - p[1]) / config.scale.exp() + 0.5) * px_size as f64,
                ((p[0] - config.cx) / config.scale.exp() + 0.5) * px_size as f64,
            ]
        };

        let mut draw_trail = |history: &[[f64; 2]], color: Color32| {
            for ends in history.windows(2) {
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
                    put_px(row as isize, col as isize, color);
                }
            }
        };

        draw_trail(&history, TRAIL_COLOR);

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
            draw_trail(&m1_history, M1_COLOR);
            draw_trail(&m2_history, M2_COLOR);
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
        let m3_pos = history.last().copied().unwrap();
        let circles = [m1_pos, m2_pos, m3_pos]
            .into_iter()
            .map(world2px)
            .zip([M1_COLOR, M2_COLOR, M3_COLOR]);
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
        let remaining_steps =
            ((self.duration / self.dt).ceil() as usize).saturating_sub(self.system.pos_log.len());
        self.system
            // TODO: Better integration method
            .modified_euler(self.dt, remaining_steps);

        let pos_s = self.system.serialize_pos_log();
        let vel_s = self.system.serialize_vel_log();

        let mut pos_f = std::fs::File::create("pos_log.csv").unwrap();
        write!(pos_f, "{}", pos_s).unwrap();
        let mut vel_f = std::fs::File::create("vel_log.csv").unwrap();
        write!(vel_f, "{}", vel_s).unwrap();
        let mut jacobi_f = std::fs::File::create("jacobi_log.csv").unwrap();
        for (pos, vel) in self.system.pos_log.iter().zip(self.system.vel_log.iter()) {
            writeln!(jacobi_f, "{}", self.system.jacobi(*pos, *vel)).unwrap();
        }
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
