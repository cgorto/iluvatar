//! Debug UI for the simulator using egui
//!
//! Provides real-time controls for all the tunable parameters:
//! - Voxel extraction (percentile threshold, min camera contributors)
//! - Grid settings (clear each frame, decay rate)
//! - DBSCAN clustering (epsilon, min points)
//! - Tracking (association threshold, max missed frames)
//! - Visualization options

use bevy::gizmos::AppGizmoBuilder;
use bevy::gizmos::config::{DefaultGizmoConfigGroup, GizmoConfig};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use crate::render_layers::debug_layers;
use crate::targets::{MovementPattern, Target, TargetPath, spawn_target};
use crate::tracking::{TrackingConfig, TrackingMetrics, TrackingState};
use crate::voxels::{SimulatorConfig, VoxelGridResource};

pub struct DebugUiPlugin;

impl Plugin for DebugUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(EguiPlugin::default())
            .init_resource::<VisualizationConfig>()
            .init_resource::<TargetUiState>()
            // Configure gizmos to render only on the debug layer (layer 2)
            // This prevents render cameras from seeing voxel cubes, trails, etc.
            .insert_gizmo_config(
                DefaultGizmoConfigGroup,
                GizmoConfig {
                    render_layers: debug_layers(),
                    ..default()
                },
            )
            .add_systems(EguiPrimaryContextPass, debug_ui_system);
    }
}

/// Visualization toggles that aren't part of the core configs
#[derive(Resource)]
pub struct VisualizationConfig {
    /// Show velocity vectors on tracked objects
    pub show_velocity_vectors: bool,
    /// Show trail history for tracks
    pub show_trails: bool,
    /// Show bounding boxes around tracked objects
    pub show_bounding_boxes: bool,
    /// Show raw voxels
    pub show_voxels: bool,
    /// Show camera frustums
    pub show_cameras: bool,
    /// Show ground truth targets
    pub show_targets: bool,
    /// Show the bounding box of the voxel grid
    pub show_grid_bounds: bool,
}

impl Default for VisualizationConfig {
    fn default() -> Self {
        Self {
            show_velocity_vectors: true,
            show_trails: true,
            show_bounding_boxes: true,
            show_voxels: true,
            show_cameras: true,
            show_targets: true,
            show_grid_bounds: true,
        }
    }
}

#[derive(Resource)]
struct TargetUiState {
    selected: Option<Entity>,
    spawn_origin: Vec3,
    spawn_speed: f32,
    spawn_time_offset: f32,
    spawn_pattern: MovementPattern,
}

impl Default for TargetUiState {
    fn default() -> Self {
        Self {
            selected: None,
            spawn_origin: Vec3::new(0.0, 200.0, 0.0),
            spawn_speed: 0.5,
            spawn_time_offset: 0.0,
            spawn_pattern: MovementPattern::Circle {
                radius: 120.0,
                axis: Vec3::Y,
            },
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PatternKind {
    Linear,
    Circle,
    Spiral,
    Lissajous,
}

// Bevy injects each ECS resource/query as a system parameter; grouping them would
// hide access patterns from the scheduler without simplifying the system.
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn debug_ui_system(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut contexts: EguiContexts,
    mut sim_config: ResMut<SimulatorConfig>,
    mut tracking_config: ResMut<TrackingConfig>,
    mut vis_config: ResMut<VisualizationConfig>,
    mut target_ui: ResMut<TargetUiState>,
    grid_res: Res<VoxelGridResource>,
    tracking_state: Res<TrackingState>,
    metrics: Res<TrackingMetrics>,
    mut targets: ParamSet<(
        Query<(Entity, &Target, &TargetPath)>,
        Query<&mut TargetPath>,
    )>,
) -> Result {
    let ctx = contexts.ctx_mut()?;

    egui::Window::new("Detection & Tracking")
        .default_width(320.0)
        .show(ctx, |ui| {
            // === Extraction Section ===
            ui.heading("Extraction");
            ui.separator();

            ui.checkbox(
                &mut tracking_config.use_percentile_extraction,
                "Use percentile extraction",
            );

            ui.horizontal(|ui| {
                ui.label("Percentile:");
                ui.add(
                    egui::Slider::new(&mut tracking_config.extraction_percentile, 0.5..=0.99)
                        .fixed_decimals(2),
                );
            });

            ui.horizontal(|ui| {
                ui.label("Min contributors:");
                let mut min_contrib = tracking_config.detection.min_contributors as i32;
                if ui.add(egui::Slider::new(&mut min_contrib, 1..=4)).changed() {
                    tracking_config.detection.min_contributors = min_contrib as u8;
                }
            });

            ui.checkbox(
                &mut tracking_config.clear_grid_each_frame,
                "Clear grid each frame",
            );

            ui.add_space(8.0);

            // === Grid Settings ===
            ui.heading("Grid");
            ui.separator();

            ui.horizontal(|ui| {
                ui.label("Decay rate:");
                ui.add(
                    egui::Slider::new(&mut sim_config.decay_rate, 0.0..=20.0)
                        .suffix(" /s")
                        .fixed_decimals(1),
                );
            });

            ui.horizontal(|ui| {
                ui.label("Viz threshold:");
                ui.add(
                    egui::Slider::new(&mut sim_config.visualization_threshold, 0.0..=10.0)
                        .fixed_decimals(1),
                );
            });

            ui.horizontal(|ui| {
                ui.label("Ray intensity:");
                ui.add(
                    egui::Slider::new(&mut sim_config.ray_intensity, 1.0..=50.0).fixed_decimals(1),
                );
            });

            ui.add_space(8.0);

            // === Clustering Section ===
            ui.heading("Clustering (DBSCAN)");
            ui.separator();

            ui.horizontal(|ui| {
                ui.label("Epsilon:");
                ui.add(
                    egui::Slider::new(&mut tracking_config.detection.cluster_epsilon, 0.5..=15.0)
                        .suffix(" m")
                        .fixed_decimals(1),
                );
            });

            ui.horizontal(|ui| {
                ui.label("Min points:");
                let mut min_pts = tracking_config.detection.cluster_min_points as i32;
                if ui.add(egui::Slider::new(&mut min_pts, 1..=10)).changed() {
                    tracking_config.detection.cluster_min_points = min_pts as usize;
                }
            });

            ui.add_space(8.0);

            // === Tracking Section ===
            ui.heading("Tracking");
            ui.separator();

            ui.horizontal(|ui| {
                ui.label("Association threshold:");
                ui.add(
                    egui::Slider::new(&mut tracking_config.association_threshold, 1.0..=30.0)
                        .suffix(" m")
                        .fixed_decimals(1),
                );
            });

            ui.horizontal(|ui| {
                ui.label("Max missed frames:");
                let mut max_missed = tracking_config.max_missing_frames as i32;
                if ui
                    .add(egui::Slider::new(&mut max_missed, 1..=120))
                    .changed()
                {
                    tracking_config.max_missing_frames = max_missed as u32;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Trail length:");
                let mut trail = tracking_config.trail_length as i32;
                if ui.add(egui::Slider::new(&mut trail, 10..=300)).changed() {
                    tracking_config.trail_length = trail as usize;
                }
            });

            ui.add_space(12.0);

            // === Visualization Section ===
            ui.heading("Visualization");
            ui.separator();

            ui.horizontal(|ui| {
                ui.checkbox(&mut vis_config.show_velocity_vectors, "Velocity vectors");
                ui.checkbox(&mut vis_config.show_trails, "Trails");
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut vis_config.show_bounding_boxes, "Bounding boxes");
                ui.checkbox(&mut vis_config.show_voxels, "Voxels");
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut vis_config.show_cameras, "Cameras");
                ui.checkbox(&mut vis_config.show_targets, "Targets");
            });
            ui.checkbox(&mut vis_config.show_grid_bounds, "Grid Bounds");

            ui.add_space(12.0);

            // === Targets Section ===
            ui.heading("Targets");
            ui.separator();

            ui.collapsing("Spawn Target", |ui| {
                vec3_drag(ui, "Origin", &mut target_ui.spawn_origin, 1.0);

                ui.horizontal(|ui| {
                    ui.label("Speed");
                    ui.add(
                        egui::DragValue::new(&mut target_ui.spawn_speed)
                            .speed(0.05)
                            .range(-5.0..=5.0),
                    );
                });

                ui.horizontal(|ui| {
                    ui.label("Time offset");
                    ui.add(egui::DragValue::new(&mut target_ui.spawn_time_offset).speed(0.1));
                });

                ui.add_space(4.0);
                edit_movement_pattern(
                    ui,
                    "spawn_pattern",
                    target_ui.spawn_origin,
                    &mut target_ui.spawn_pattern,
                );

                ui.add_space(4.0);
                if ui.button("Spawn target").clicked() {
                    let next_id = next_target_id(&targets.p0());
                    let path = TargetPath {
                        origin: target_ui.spawn_origin,
                        params: target_ui.spawn_pattern,
                        speed: target_ui.spawn_speed,
                        time_offset: target_ui.spawn_time_offset,
                    };
                    let entity = spawn_target(&mut commands, &asset_server, next_id, path);
                    target_ui.selected = Some(entity);
                }
            });

            ui.add_space(8.0);

            ui.collapsing("Active Targets", |ui| {
                let mut target_count = 0usize;
                let mut selected_exists = false;
                let mut selected_id = None;

                {
                    let target_query = targets.p0();
                    egui::Grid::new("targets_grid")
                        .striped(true)
                        .spacing([12.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("ID");
                            ui.label("Pattern");
                            ui.label("Speed");
                            ui.label("");
                            ui.end_row();

                            for (entity, target, path) in target_query.iter() {
                                target_count += 1;
                                let is_selected = target_ui.selected == Some(entity);
                                if is_selected {
                                    selected_exists = true;
                                    selected_id = Some(target.id);
                                }

                                if ui
                                    .selectable_label(is_selected, format!("{}", target.id))
                                    .clicked()
                                {
                                    target_ui.selected = Some(entity);
                                    selected_exists = true;
                                    selected_id = Some(target.id);
                                }

                                ui.label(pattern_label(&path.params));
                                ui.label(format!("{:.2}", path.speed));
                                let despawn_clicked = ui.button("Despawn").clicked();
                                ui.end_row();

                                if despawn_clicked {
                                    commands.entity(entity).despawn();
                                    if target_ui.selected == Some(entity) {
                                        target_ui.selected = None;
                                        selected_exists = false;
                                        selected_id = None;
                                    }
                                }
                            }
                        });
                }

                if target_count == 0 {
                    ui.label("No active targets.");
                }

                if !selected_exists {
                    target_ui.selected = None;
                }

                if let Some(selected) = target_ui.selected {
                    ui.add_space(8.0);
                    ui.separator();
                    ui.label(format!(
                        "Selected target{}",
                        selected_id.map(|id| format!(" {}", id)).unwrap_or_default()
                    ));
                    ui.add_space(4.0);

                    {
                        let mut target_paths = targets.p1();
                        if let Ok(mut path) = target_paths.get_mut(selected) {
                            ui.horizontal(|ui| {
                                ui.label("Speed");
                                ui.add(
                                    egui::DragValue::new(&mut path.speed)
                                        .speed(0.05)
                                        .range(-5.0..=5.0),
                                );
                            });
                            vec3_drag(ui, "Origin", &mut path.origin, 1.0);
                            ui.horizontal(|ui| {
                                ui.label("Time offset");
                                ui.add(egui::DragValue::new(&mut path.time_offset).speed(0.1));
                                if ui.button("Reset").clicked() {
                                    path.time_offset = 0.0;
                                }
                            });

                            ui.add_space(4.0);
                            edit_movement_pattern(
                                ui,
                                format!("selected_pattern_{selected:?}"),
                                path.origin,
                                &mut path.params,
                            );

                            ui.add_space(4.0);
                            if ui.button("Despawn selected").clicked() {
                                commands.entity(selected).despawn();
                                target_ui.selected = None;
                            }
                        } else {
                            target_ui.selected = None;
                        }
                    }
                } else if target_count > 0 {
                    ui.label("Select a target to edit.");
                }
            });

            ui.add_space(12.0);

            // === Stats Section ===
            ui.heading("Stats");
            ui.separator();

            egui::Grid::new("stats_grid")
                .num_columns(2)
                .spacing([20.0, 4.0])
                .show(ui, |ui| {
                    ui.label("Active voxels:");
                    ui.label(format!("{}", grid_res.grid.active_count()));
                    ui.end_row();

                    ui.label("Detected points:");
                    ui.label(format!("{}", metrics.detected_points));
                    ui.end_row();

                    ui.label("Clusters:");
                    ui.label(format!("{}", metrics.cluster_count));
                    ui.end_row();

                    ui.label("Tracks:");
                    ui.label(format!("{}", tracking_state.tracked_objects.len()));
                    ui.end_row();

                    ui.label("Position error:");
                    ui.label(format!("{:.2} m", metrics.avg_position_error));
                    ui.end_row();

                    ui.label("Velocity error:");
                    ui.label(format!("{:.2} m/s", metrics.avg_velocity_error));
                    ui.end_row();

                    ui.label("Matched:");
                    ui.label(format!(
                        "{}/{}",
                        metrics.matched_count, metrics.ground_truth_count
                    ));
                    ui.end_row();
                });

            // Show individual track info if we have tracks
            if !tracking_state.tracked_objects.is_empty() {
                let voxel_size = sim_config.voxel_size;
                let grid_origin = sim_config.grid_origin;

                ui.add_space(8.0);
                ui.collapsing("Track Details", |ui| {
                    for obj in &tracking_state.tracked_objects {
                        let vel_str = obj
                            .velocity
                            .map(|v| format!("{:.1} m/s", v.length()))
                            .unwrap_or_else(|| "—".to_string());

                        // World position (grid-local centroid + grid origin)
                        let world_pos = obj.centroid + grid_origin;

                        // Voxel indices (centroid in grid-local coords / voxel_size)
                        let vx = (obj.centroid.x / voxel_size).floor() as i32;
                        let vy = (obj.centroid.y / voxel_size).floor() as i32;
                        let vz = (obj.centroid.z / voxel_size).floor() as i32;

                        ui.group(|ui| {
                            ui.label(format!(
                                "Track {} (conf: {:.2}, {} pts, intensity: {:.1})",
                                obj.id, obj.confidence, obj.point_count, obj.total_intensity
                            ));
                            ui.horizontal(|ui| {
                                ui.label("World:");
                                ui.monospace(format!(
                                    "({:.1}, {:.1}, {:.1})",
                                    world_pos.x, world_pos.y, world_pos.z
                                ));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Voxel:");
                                ui.monospace(format!("[{}, {}, {}]", vx, vy, vz));
                            });
                            ui.horizontal(|ui| {
                                ui.label("BBox:");
                                let bb_min = obj.bounding_box.min + grid_origin;
                                let bb_max = obj.bounding_box.max + grid_origin;
                                ui.monospace(format!(
                                    "({:.0},{:.0},{:.0})-({:.0},{:.0},{:.0})",
                                    bb_min.x, bb_min.y, bb_min.z, bb_max.x, bb_max.y, bb_max.z
                                ));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Vel:");
                                if let Some(vel) = obj.velocity {
                                    ui.monospace(format!(
                                        "({:.1}, {:.1}, {:.1}) |{}|",
                                        vel.x, vel.y, vel.z, vel_str
                                    ));
                                } else {
                                    ui.label("—");
                                }
                            });
                        });
                    }
                });
            }
        });

    Ok(())
}

const PATTERN_KINDS: [PatternKind; 4] = [
    PatternKind::Linear,
    PatternKind::Circle,
    PatternKind::Spiral,
    PatternKind::Lissajous,
];

impl PatternKind {
    fn label(self) -> &'static str {
        match self {
            PatternKind::Linear => "Linear",
            PatternKind::Circle => "Circle",
            PatternKind::Spiral => "Spiral",
            PatternKind::Lissajous => "Lissajous",
        }
    }
}

fn pattern_kind(pattern: &MovementPattern) -> PatternKind {
    match pattern {
        MovementPattern::Linear { .. } => PatternKind::Linear,
        MovementPattern::Circle { .. } => PatternKind::Circle,
        MovementPattern::Spiral { .. } => PatternKind::Spiral,
        MovementPattern::Lissajous { .. } => PatternKind::Lissajous,
    }
}

fn pattern_label(pattern: &MovementPattern) -> &'static str {
    pattern_kind(pattern).label()
}

fn default_pattern(kind: PatternKind, origin: Vec3) -> MovementPattern {
    match kind {
        PatternKind::Linear => MovementPattern::Linear {
            end: origin + Vec3::new(200.0, 0.0, 0.0),
        },
        PatternKind::Circle => MovementPattern::Circle {
            radius: 120.0,
            axis: Vec3::Y,
        },
        PatternKind::Spiral => MovementPattern::Spiral {
            radius: 60.0,
            height: 40.0,
            frequency: 1.0,
        },
        PatternKind::Lissajous => MovementPattern::Lissajous {
            freq_x: 0.7,
            freq_z: 0.9,
            amp_x: 200.0,
            amp_z: 200.0,
        },
    }
}

fn next_target_id(query: &Query<(Entity, &Target, &TargetPath)>) -> u32 {
    query
        .iter()
        .map(|(_, target, _)| target.id)
        .max()
        .unwrap_or(0)
        .saturating_add(1)
}

fn vec3_drag(ui: &mut egui::Ui, label: &str, value: &mut Vec3, speed: f32) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.label("x");
        ui.add(egui::DragValue::new(&mut value.x).speed(speed));
        ui.label("y");
        ui.add(egui::DragValue::new(&mut value.y).speed(speed));
        ui.label("z");
        ui.add(egui::DragValue::new(&mut value.z).speed(speed));
    });
}

fn edit_movement_pattern<I: std::hash::Hash>(
    ui: &mut egui::Ui,
    id_source: I,
    origin: Vec3,
    pattern: &mut MovementPattern,
) {
    ui.push_id(id_source, |ui| {
        let current_kind = pattern_kind(pattern);
        let mut selected_kind = current_kind;

        egui::ComboBox::from_id_salt("pattern_kind")
            .selected_text(current_kind.label())
            .show_ui(ui, |ui| {
                for kind in PATTERN_KINDS {
                    ui.selectable_value(&mut selected_kind, kind, kind.label());
                }
            });

        if selected_kind != current_kind {
            *pattern = default_pattern(selected_kind, origin);
        }

        match pattern {
            MovementPattern::Linear { end } => {
                vec3_drag(ui, "End", end, 1.0);
            }
            MovementPattern::Circle { radius, axis: _ } => {
                ui.horizontal(|ui| {
                    ui.label("Radius");
                    ui.add(egui::DragValue::new(radius).speed(1.0).range(0.0..=2000.0));
                });
            }
            MovementPattern::Spiral {
                radius,
                height,
                frequency,
            } => {
                ui.horizontal(|ui| {
                    ui.label("Radius");
                    ui.add(egui::DragValue::new(radius).speed(1.0).range(0.0..=2000.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Height");
                    ui.add(egui::DragValue::new(height).speed(1.0).range(0.0..=2000.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Frequency");
                    ui.add(egui::DragValue::new(frequency).speed(0.1).range(0.0..=10.0));
                });
            }
            MovementPattern::Lissajous {
                freq_x,
                freq_z,
                amp_x,
                amp_z,
            } => {
                ui.horizontal(|ui| {
                    ui.label("Freq X");
                    ui.add(egui::DragValue::new(freq_x).speed(0.1).range(0.0..=10.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Freq Z");
                    ui.add(egui::DragValue::new(freq_z).speed(0.1).range(0.0..=10.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Amp X");
                    ui.add(egui::DragValue::new(amp_x).speed(1.0).range(0.0..=2000.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Amp Z");
                    ui.add(egui::DragValue::new(amp_z).speed(1.0).range(0.0..=2000.0));
                });
            }
        }
    });
}
