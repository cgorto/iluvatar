//! Kalman filter for position/velocity tracking.
//!
//! Implements a simple 1D Kalman filter with constant-velocity motion model.
//! For 3D tracking, use three independent filters (one per axis).

use glam::Vec3;

/// 1D Kalman filter for position + velocity estimation.
///
/// State vector: [position, velocity]
/// Motion model: constant velocity (x' = x + v*dt, v' = v)
/// Measurement: position only
#[derive(Debug, Clone)]
pub struct Kalman1D {
    // State estimate
    pub x: f32, // position
    pub v: f32, // velocity

    // Error covariance matrix P (2x2, stored as elements)
    // P = [[p00, p01], [p10, p11]]
    p00: f32,
    p01: f32,
    p10: f32,
    p11: f32,

    // Process noise (acceleration variance)
    q_pos: f32,
    q_vel: f32,

    // Measurement noise (position variance)
    r: f32,
}

impl Kalman1D {
    /// Create a new filter initialized at a position with zero velocity.
    ///
    /// # Arguments
    /// * `initial_pos` - Initial position estimate
    /// * `process_noise` - Process noise (acceleration std dev, m/s²)
    /// * `measurement_noise` - Measurement noise (position std dev, m)
    pub fn new(initial_pos: f32, process_noise: f32, measurement_noise: f32) -> Self {
        // Initial covariance tuned for fast velocity convergence
        // High p11 = uncertain about initial velocity (will learn quickly)
        // High p01/p10 = position info transfers strongly to velocity estimate
        Self {
            x: initial_pos,
            v: 0.0,
            p00: 4.0,    // 2m position std dev initially
            p01: 50.0,   // High cross-covariance for fast velocity learning
            p10: 50.0,   // Symmetric
            p11: 2500.0, // 50 m/s velocity std dev initially
            q_pos: process_noise * process_noise * 0.25,
            q_vel: process_noise * process_noise,
            r: measurement_noise * measurement_noise,
        }
    }

    /// Predict step: advance state by dt seconds.
    ///
    /// Uses constant-velocity model: x' = x + v*dt, v' = v
    pub fn predict(&mut self, dt: f32) {
        // State prediction: x' = x + v*dt, v' = v
        self.x += self.v * dt;
        // v stays the same

        // Covariance prediction: P' = F * P * F^T + Q
        // where F = [[1, dt], [0, 1]]
        //
        // F * P = [[p00 + dt*p10, p01 + dt*p11],
        //         [p10,          p11         ]]
        //
        // F * P * F^T:
        // [[p00 + dt*p10 + dt*(p01 + dt*p11), p01 + dt*p11],
        //  [p10 + dt*p11,                     p11         ]]
        let dt2 = dt * dt;
        let new_p00 = self.p00 + dt * (self.p01 + self.p10) + dt2 * self.p11 + self.q_pos * dt2;
        let new_p01 = self.p01 + dt * self.p11;
        let new_p10 = self.p10 + dt * self.p11;
        let new_p11 = self.p11 + self.q_vel * dt;

        self.p00 = new_p00;
        self.p01 = new_p01;
        self.p10 = new_p10;
        self.p11 = new_p11;
    }

    /// Update step: incorporate a position measurement.
    ///
    /// Returns the innovation (measurement - prediction) for diagnostics.
    pub fn update(&mut self, measured_pos: f32) -> f32 {
        // Innovation (measurement residual)
        let y = measured_pos - self.x;

        // Innovation covariance: S = H * P * H^T + R
        // where H = [1, 0], so S = p00 + R
        let s = self.p00 + self.r;

        // Kalman gain: K = P * H^T * S^-1
        // K = [[p00], [p10]] / s
        let k0 = self.p00 / s;
        let k1 = self.p10 / s;

        // State update: x = x + K * y
        self.x += k0 * y;
        self.v += k1 * y;

        // Covariance update: P = (I - K * H) * P
        // (I - K * H) = [[1-k0, 0], [-k1, 1]]
        let new_p00 = (1.0 - k0) * self.p00;
        let new_p01 = (1.0 - k0) * self.p01;
        let new_p10 = -k1 * self.p00 + self.p10;
        let new_p11 = -k1 * self.p01 + self.p11;

        self.p00 = new_p00;
        self.p01 = new_p01;
        self.p10 = new_p10;
        self.p11 = new_p11;

        y
    }

    /// Get current position estimate
    pub fn position(&self) -> f32 {
        self.x
    }

    /// Get current velocity estimate
    pub fn velocity(&self) -> f32 {
        self.v
    }

    /// Get position uncertainty (standard deviation)
    pub fn position_std(&self) -> f32 {
        self.p00.sqrt()
    }

    /// Get velocity uncertainty (standard deviation)
    pub fn velocity_std(&self) -> f32 {
        self.p11.sqrt()
    }
}

/// 3D Kalman filter using three independent 1D filters.
///
/// This works well for tracking in Cartesian coordinates where axes are independent.
#[derive(Debug, Clone)]
pub struct Kalman3D {
    pub x: Kalman1D,
    pub y: Kalman1D,
    pub z: Kalman1D,
}

impl Kalman3D {
    /// Create a new 3D filter at the given initial position.
    pub fn new(initial_pos: Vec3, process_noise: f32, measurement_noise: f32) -> Self {
        Self {
            x: Kalman1D::new(initial_pos.x, process_noise, measurement_noise),
            y: Kalman1D::new(initial_pos.y, process_noise, measurement_noise),
            z: Kalman1D::new(initial_pos.z, process_noise, measurement_noise),
        }
    }

    /// Predict step for all axes.
    pub fn predict(&mut self, dt: f32) {
        self.x.predict(dt);
        self.y.predict(dt);
        self.z.predict(dt);
    }

    /// Update step with a 3D position measurement.
    pub fn update(&mut self, measured_pos: Vec3) {
        self.x.update(measured_pos.x);
        self.y.update(measured_pos.y);
        self.z.update(measured_pos.z);
    }

    /// Get current position estimate.
    pub fn position(&self) -> Vec3 {
        Vec3::new(self.x.position(), self.y.position(), self.z.position())
    }

    /// Get current velocity estimate.
    pub fn velocity(&self) -> Vec3 {
        Vec3::new(self.x.velocity(), self.y.velocity(), self.z.velocity())
    }

    /// Get predicted position after dt seconds (without modifying state).
    pub fn predicted_position(&self, dt: f32) -> Vec3 {
        Vec3::new(
            self.x.x + self.x.v * dt,
            self.y.x + self.y.v * dt,
            self.z.x + self.z.v * dt,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_1d_static() {
        // Object at rest, should converge to true position
        let mut kf = Kalman1D::new(0.0, 1.0, 0.5);

        // Feed measurements near position 10
        for _ in 0..100 {
            kf.predict(1.0 / 60.0);
            kf.update(10.0);
        }

        assert!((kf.position() - 10.0).abs() < 0.1);
        assert!(kf.velocity().abs() < 0.5);
    }

    #[test]
    fn test_kalman_1d_constant_velocity() {
        // Object moving at 2 units/sec
        let mut kf = Kalman1D::new(0.0, 1.0, 0.5);
        let velocity = 2.0;
        let dt = 1.0 / 60.0;

        let mut true_pos = 0.0;
        for _ in 0..60 {
            true_pos += velocity * dt;
            kf.predict(dt);
            kf.update(true_pos);
        }

        // After 1 second, should be near true position and velocity
        assert!((kf.position() - true_pos).abs() < 0.1);
        assert!((kf.velocity() - velocity).abs() < 0.5);
    }

    #[test]
    fn test_kalman_3d_tracking() {
        let mut kf = Kalman3D::new(Vec3::ZERO, 1.0, 0.5);
        let velocity = Vec3::new(2.0, 2.0, 0.0);
        let dt = 1.0 / 60.0;

        let mut true_pos = Vec3::ZERO;
        for _ in 0..60 {
            true_pos += velocity * dt;
            kf.predict(dt);
            kf.update(true_pos);
        }

        let pos_error = (kf.position() - true_pos).length();
        let vel_error = (kf.velocity() - velocity).length();

        assert!(pos_error < 0.2);
        assert!(vel_error < 0.5);
    }
}
