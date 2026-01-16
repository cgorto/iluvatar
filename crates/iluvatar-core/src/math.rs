use glam::Vec3;

/// Epsilon for safe division to handle parallel rays
pub const EPSILON: f32 = 1e-12;

/// Safe division that returns infinity for near-zero denominators
/// Prevents NaN and handles edge cases with parallel rays
#[inline]
pub fn safe_div(num: f32, den: f32) -> f32 {
    if den.abs() < EPSILON {
        f32::INFINITY
    } else {
        num / den
    }
}

/// Ray-AABB (Axis-Aligned Bounding Box) intersection
/// Returns (t_min, t_max) if the ray intersects the box, None otherwise
/// t_min is clamped to be >= 0 (ray starts at origin)
pub fn ray_aabb_intersection(
    ray_origin: Vec3,
    ray_dir: Vec3,
    box_min: Vec3,
    box_max: Vec3,
) -> Option<(f32, f32)> {
    let mut t_min = 0.0f32;
    let mut t_max = f32::INFINITY;

    // Check each axis
    for i in 0..3 {
        let origin = match i {
            0 => ray_origin.x,
            1 => ray_origin.y,
            _ => ray_origin.z,
        };
        let dir = match i {
            0 => ray_dir.x,
            1 => ray_dir.y,
            _ => ray_dir.z,
        };
        let mn = match i {
            0 => box_min.x,
            1 => box_min.y,
            _ => box_min.z,
        };
        let mx = match i {
            0 => box_max.x,
            1 => box_max.y,
            _ => box_max.z,
        };

        if dir.abs() < EPSILON {
            // Ray is parallel to this slab
            if origin < mn || origin > mx {
                return None; // No intersection
            }
        } else {
            let t1 = (mn - origin) / dir;
            let t2 = (mx - origin) / dir;
            let t_near = t1.min(t2);
            let t_far = t1.max(t2);

            if t_near > t_min {
                t_min = t_near;
            }
            if t_far < t_max {
                t_max = t_far;
            }
            if t_min > t_max {
                return None;
            }
        }
    }

    // Clamp t_min to be non-negative (ray starts at origin)
    if t_min < 0.0 {
        t_min = 0.0;
    }

    Some((t_min, t_max))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_safe_div_normal() {
        assert!((safe_div(10.0, 2.0) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_safe_div_zero() {
        assert!(safe_div(10.0, 0.0).is_infinite());
        assert!(safe_div(10.0, 1e-15).is_infinite());
    }

    #[test]
    fn test_ray_aabb_hit() {
        let origin = Vec3::new(-5.0, 0.0, 0.0);
        let dir = Vec3::new(1.0, 0.0, 0.0);
        let box_min = Vec3::new(0.0, -1.0, -1.0);
        let box_max = Vec3::new(2.0, 1.0, 1.0);

        let result = ray_aabb_intersection(origin, dir, box_min, box_max);
        assert!(result.is_some());
        let (t_min, t_max) = result.unwrap();
        assert!((t_min - 5.0).abs() < 1e-6); // Enter at x=0
        assert!((t_max - 7.0).abs() < 1e-6); // Exit at x=2
    }

    #[test]
    fn test_ray_aabb_miss() {
        let origin = Vec3::new(-5.0, 5.0, 0.0); // Above the box
        let dir = Vec3::new(1.0, 0.0, 0.0);
        let box_min = Vec3::new(0.0, -1.0, -1.0);
        let box_max = Vec3::new(2.0, 1.0, 1.0);

        let result = ray_aabb_intersection(origin, dir, box_min, box_max);
        assert!(result.is_none());
    }

    #[test]
    fn test_ray_aabb_inside() {
        let origin = Vec3::new(1.0, 0.0, 0.0); // Inside the box
        let dir = Vec3::new(1.0, 0.0, 0.0);
        let box_min = Vec3::new(0.0, -1.0, -1.0);
        let box_max = Vec3::new(2.0, 1.0, 1.0);

        let result = ray_aabb_intersection(origin, dir, box_min, box_max);
        assert!(result.is_some());
        let (t_min, t_max) = result.unwrap();
        assert!(t_min.abs() < 1e-6); // Already inside
        assert!((t_max - 1.0).abs() < 1e-6); // Exit at x=2
    }
}
