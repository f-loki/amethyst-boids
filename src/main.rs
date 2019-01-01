extern crate amethyst;

use amethyst::{
    prelude::*,
    renderer::{DisplayConfig, DrawFlat, Pipeline, PosNormTex, RenderBundle, Stage},
    utils::application_root_dir,
    ecs::*,
};

use nalgebra::Vector2;

struct Example;

impl SimpleState for Example {}

fn main() -> amethyst::Result<()> {
    amethyst::start_logger(Default::default());

    let path = format!(
        "{}/resources/display_config.ron",
        application_root_dir()
    );
    let config = DisplayConfig::load(&path);

    let pipe = Pipeline::build().with_stage(
        Stage::with_backbuffer()
            .clear_target([0.00196, 0.23726, 0.21765, 1.0], 1.0)
            .with_pass(DrawFlat::<PosNormTex>::new()),
    );

    let game_data =
        GameDataBuilder::default().with_bundle(RenderBundle::new(pipe, Some(config)))?;
    let mut game = Application::new("./", Example, game_data)?;

    game.run();

    Ok(())
}

type Vec2 = Vector2<f32>;

// components

// The very vanilla pos/vel components, as this doesn't hook into nphysics

struct Pos(Vec2);

impl Component for Pos {
    type Storage = VecStorage<Self>;
}

// The velocity also determines the heading!
struct Vel(Vec2);

impl Component for Vel {
    type Storage = VecStorage<Self>;
}

// Systems

struct Cohesion;

impl <'a> System<'a> for Cohesion {

    type SystemData = (ReadStorage<'a, Pos>, ReadStorage<'a, Vel>);

    fn run(&mut self, data: Self::SystemData) {
        
    }

}

//
