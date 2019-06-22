use amethyst::{
    prelude::*,
    assets::{Handle},
    renderer::{
        camera::{Camera, Projection},  
        light::{PointLight, Light},
    },
    core::transform::{Transform},
    gltf::*,
};
use crate::{
    boids::{ClosenessThreshold, SeparationDistance, CentreOfFlockValue, Pos, Vel, Vec2, make_a_boid, make_an_asset_entity}
};
use nalgebra::{Point2};
use rand::prelude::*;

pub struct MainScene {
    gltf_scene: Handle<GltfSceneAsset>
} 

impl MainScene {
    pub fn new(scene: Handle<GltfSceneAsset>) -> MainScene {
        MainScene { gltf_scene: scene }
    }
}

impl SimpleState for MainScene {
    fn on_start(&mut self, data: StateData<'_, GameData<'_, '_>>) {
        data.world.add_resource(ClosenessThreshold(50.0));
        data.world.add_resource(SeparationDistance(2.0));
        data.world.add_resource(CentreOfFlockValue(Point2::origin()));
        let mut rng = rand::thread_rng();
        let mut camera_transform = Transform::default();
        camera_transform.set_translation_xyz(0.0, 0.0, 10.0);
        data.world.create_entity().with(Camera::from(Projection::perspective(1.3, 1.0471975512, 0.1, 2000.0))).with(camera_transform).build();
        data.world.create_entity().with(Light::Point(PointLight::default())).with(Transform::default()).build();
        for _ in 0..20 {
            make_a_boid(data.world, Pos(Point2::new(rng.gen_range(-3.75, 3.75), rng.gen_range(-5.75, 5.75))), Vel(Vec2::new(rng.gen_range(-5.75, 5.75), rng.gen_range(-5.75, 5.75))), self.gltf_scene.clone());
            // make_an_asset_entity(data.world, Transform::default(), self.gltf_scene.clone());
        }
    }
}