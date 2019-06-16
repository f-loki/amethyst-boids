extern crate amethyst;
mod states;
mod boids;
mod render;

use amethyst::{
    prelude::*,
    renderer::{RenderingSystem, types::DefaultBackend, visibility::VisibilitySortingSystem},
    utils::application_root_dir,
    core::transform::{bundle::TransformBundle},
    window::WindowBundle,
};

use render::BoidyGraphBuilder;
use states::loadassets::LoadAssets;
use boids::BoidsBundle;

pub struct Dummy;

impl SimpleState for Dummy {}
 
fn main() -> amethyst::Result<()> {
    amethyst::start_logger(Default::default());

    let path = application_root_dir()?;
    let assets = path.join("resources");
    let display_config_path = assets.join("display_config.ron");

    let game_data = GameDataBuilder::default()
        .with_bundle(WindowBundle::from_config_path(display_config_path))?
        .with_bundle(TransformBundle::new())?
        .with_bundle(BoidsBundle)?
        // the visibility bundle is necessary for the PBR rendering pass
        // but there seems to be a lack of an explicitly declared dependency
        // and that which worries me
        .with(VisibilitySortingSystem::new(), "visibility_system", &["transform_system"])
        .with_thread_local(RenderingSystem::<DefaultBackend, _>::new(BoidyGraphBuilder::default()));

    let mut game = Application::new(assets, LoadAssets::new(), game_data)?;

    game.run();
    // let game_data =
    //     GameDataBuilder::default()
    //     .with_bundle(RenderBundle::new(pipe, Some(config)))?
    //     .with_bundle(TransformBundle::new())?
    //     .with_bundle(boids::BoidsBundle)?;
    // let mut game = Application::new("./", states::loadassets::LoadAssets::new(), game_data)?;

    // game.run();

    Ok(())
}
