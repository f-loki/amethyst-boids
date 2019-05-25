use amethyst::{
    prelude::*,
    assets::{Handle},
    renderer::{Camera, Projection, Light, SunLight, Mesh},
    core::transform::{Transform},
};
use crate::{
    boids::{ClosenessThreshold, SeparationDistance, CentreOfFlockValue, Pos, Vel, Vec2, make_a_boid}
};
use nalgebra::{Point2};
use rand::prelude::*;

pub struct MainScene {
    handle: Handle<Mesh>
} 

impl MainScene {
    pub fn new(handle: Handle<Mesh>) -> MainScene {
        MainScene { handle: handle }
    }
}

impl SimpleState for MainScene {
    fn on_start(&mut self, data: StateData<'_, GameData<'_, '_>>) {
        data.world.add_resource(ClosenessThreshold(10.0));
        data.world.add_resource(SeparationDistance(2.0));
        data.world.add_resource(CentreOfFlockValue(Point2::origin()));
        let mut rng = rand::thread_rng();
        data.world.create_entity().with(Camera::from(Projection::orthographic(0.0, 100.0, 100.0, 0.0))).with(Transform::default()).build();
        data.world.create_entity().with(Light::Sun(SunLight::default())).with(Transform::default()).build();
        for _ in 0..5 {
            make_a_boid(data.world, Pos(Point2::origin()), Vel(Vec2::new(rng.gen_range(-5.75, 5.75), rng.gen_range(-5.75, 5.75))), self.handle.clone());
        }
    }
}