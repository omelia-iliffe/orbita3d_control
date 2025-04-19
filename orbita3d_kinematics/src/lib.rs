//! Kinematics model for Orbita3d
//!
//! ### Model definition
//! * _thetas_ disk angles
//! * _rot_ platform orientation
//!
//! All values are expressed in radians.
//!
//! See the [README.md](./README.md) for more information.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(any(feature = "std", test)))]
use nalgebra::ComplexField;
use nalgebra::{Matrix3, Rotation3, Vector3};

pub mod conversion;
mod jacobian;
mod position;
mod torque;
mod velocity;

pub use position::InverseSolutionErrorKind;
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Deserialize, Serialize)]
/// Kinematics model for Orbita3d
pub struct Orbita3dKinematicsModel {
    pub alpha: f64,
    pub gamma_min: f64,
    pub offset: f64,
    pub beta: f64,
    pub gamma_max: f64,
    pub passiv_arms_direct: bool,
}

impl Default for Orbita3dKinematicsModel {
    /// Creates a new Orbita3dKinematicsModel with default values corresponding to Reachy's neck.
    fn default() -> Self {
        Orbita3dKinematicsModel {
            alpha: 54.0_f64.to_radians(),
            gamma_min: 0.0_f64.to_radians(),
            offset: 0.0_f64.to_radians(),
            beta: 90.0_f64.to_radians(),
            gamma_max: 180.0_f64.to_radians(),
            passiv_arms_direct: true,
        }
    }
}

impl Orbita3dKinematicsModel {
    fn platform_unit_vectors_from_mat(&self, rot: Rotation3<f64>) -> Matrix3<f64> {
        // Compute the unit vectors of the platform in the base frame, from an input rotation matrix
        let delta_v = 120.0_f64.to_radians();
        let mut beta = self.beta;

        if !self.passiv_arms_direct {
            beta *= -1.0;
        }
        let v_initial1 = Vector3::from_row_slice(&[beta.cos(), beta.sin(), 0.]);
        let v_initial2 =
            Vector3::from_row_slice(&[(beta + delta_v).cos(), (beta + delta_v).sin(), 0.]);
        let v_initial3 =
            Vector3::from_row_slice(&[(beta - delta_v).cos(), (beta - delta_v).sin(), 0.]);

        let roffset = Rotation3::from_euler_angles(0., 0., self.offset);

        let v_initial1 = roffset * v_initial1;
        let v_initial2 = roffset * v_initial2;
        let v_initial3 = roffset * v_initial3;

        let v_rotation = Matrix3::from_columns(&[v_initial1, v_initial2, v_initial3]);

        rot * v_rotation
    }

    pub fn calculate_max_angle(&self) -> Option<f64> {
        let angle = 60.0_f64.to_radians() - (self.gamma_min / 2.0);
        let angle = angle * 0.99;
        let [_, pitch1, _] = self
            .compute_forward_kinematics_rpy_multiturn([0.0, angle, -angle])
            .inspect_err(|_| {
                log::error!(
                    "failed to find max pitch for gamma min position {}",
                    self.gamma_min.to_degrees()
                )
            })
            .ok()?;
        let angle = (self.gamma_max - 120.0_f64.to_radians()) / 2.0;
        let angle = angle * 0.99;
        let [_, pitch2, _] = self
            .compute_forward_kinematics_rpy_multiturn([0.0, angle, -angle])
            .inspect_err(|_| {
                log::error!(
                    "failed to find max pitch for gamma max {}",
                    self.gamma_max.to_degrees()
                )
            })
            .ok()?;
        Some(pitch1.min(pitch2))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn calculate_max_angle() {
        let mut model = Orbita3dKinematicsModel {
            alpha: 45.0_f64.to_radians(),
            gamma_min: 45_f64.to_radians(),
            offset: 0.0,
            beta: 90.0_f64.to_radians(),
            gamma_max: 175.0_f64.to_radians(),
            passiv_arms_direct: true,
        };
        let max = model.calculate_max_angle().unwrap();
        dbg!(max);
    }
    #[test]
    fn calculate_max_angle_50() {
        let mut model = Orbita3dKinematicsModel {
            alpha: 45.0_f64.to_radians(),
            gamma_min: 50_f64.to_radians(),
            offset: 0.0,
            beta: 90.0_f64.to_radians(),
            gamma_max: 175.0_f64.to_radians(),
            passiv_arms_direct: true,
        };

        // model.gamma_min = 60.0_f64.to_radians();
        let angle = 60.0_f64.to_radians() - (model.gamma_min / 2.0).next_up();
        dbg!(angle.to_degrees());
        let max = model.calculate_max_angle().unwrap();
        dbg!(max);
    }
}
