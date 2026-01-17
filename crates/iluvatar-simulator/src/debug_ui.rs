//! Debug UI for the simulator using egui
//!
//! Provides real-time controls for all the tunable parameters:
//! - Voxel extraction (percentile threshold, min camera contributors)
//! - Grid settings (clear each frame, decay rate)  
//! - DBSCAN clustering (epsilon, min points)
//! - Tracking (association threshold, max missed frames)
//! - Visualization options

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use crate::tracking::{TrackingConfig, TrackingMetrics, TrackingState};
use crate::voxels::{SimulatorConfig, VoxelGridResource};

pub struct DebugUiPlugin;

impl Plugin for DebugUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(EguiPlugin::default())
            .init_resource::<VisualizationConfig>()
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
        }
    }
}

fn debug_ui_system(
    mut contexts: EguiContexts,
    mut sim_config: ResMut<SimulatorConfig>,
    mut tracking_config: ResMut<TrackingConfig>,
    mut vis_config: ResMut<VisualizationConfig>,
    grid_res: Res<VoxelGridResource>,
    tracking_state: Res<TrackingState>,
    metrics: Res<TrackingMetrics>,
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
                ui.add_space(8.0);
                ui.collapsing("Track Details", |ui| {
                    for obj in &tracking_state.tracked_objects {
                        let vel_str = obj
                            .velocity
                            .map(|v| format!("{:.1} m/s", v.length()))
                            .unwrap_or_else(|| "—".to_string());

                        ui.horizontal(|ui| {
                            ui.label(format!("Track {}:", obj.id));
                            ui.label(format!(
                                "({:.0}, {:.0}, {:.0})",
                                obj.centroid.x, obj.centroid.y, obj.centroid.z
                            ));
                            ui.label(format!("v={}", vel_str));
                        });
                    }
                });
            }
        });

    Ok(())
}
