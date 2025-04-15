use criterion::{black_box, criterion_group, criterion_main, Criterion};
use orbita3d_kinematics::{
    conversion::intrinsic_roll_pitch_yaw_to_matrix, Orbita3dKinematicsModel,
};
pub fn inverse_kinematics(c: &mut Criterion) {
    let orb = Orbita3dKinematicsModel::default();

    let rot = intrinsic_roll_pitch_yaw_to_matrix(0.0, 0.0, 0.0);

    c.bench_function("inverse_kinematics", |b| {
        b.iter(|| {
            let thetas = orb.compute_inverse_kinematics(black_box(rot)).unwrap();
        })
    });
}

pub fn forward_kinematics(c: &mut Criterion) {
    let orb = Orbita3dKinematicsModel::default();

    c.bench_function("forward_kinematics", |b| {
        b.iter(|| {
            let thetas = orb.compute_forward_kinematics(black_box([0.0; 3])).unwrap();
        })
    });
}

criterion_group!(benches, inverse_kinematics, forward_kinematics);
criterion_main!(benches);
