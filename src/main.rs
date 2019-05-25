extern crate amethyst;
mod states;
mod boids;

use amethyst::{
    prelude::*,
    renderer::{DisplayConfig, DrawShaded, Pipeline, PosNormTex, RenderBundle, Stage},
    utils::application_root_dir,
    core::transform::{bundle::TransformBundle},
};

 
fn main() -> amethyst::Result<()> {
    amethyst::start_logger(Default::default());

    let path = format!(
        "{}/resources/display_config.ron",
        application_root_dir()
    );
    let config = DisplayConfig::load(&path);

    let pipe = Pipeline::build().with_stage(
        Stage::with_backbuffer()
            .clear_target([0.0, 0.0, 0.0, 1.0], 1.0)
            .with_pass(DrawShaded::<PosNormTex>::new())
    );

    let game_data =
        GameDataBuilder::default()
        .with_bundle(RenderBundle::new(pipe, Some(config)))?
        .with_bundle(TransformBundle::new())?
        .with_bundle(boids::BoidsBundle)?;
    let mut game = Application::new("./", states::loadassets::LoadAssets::new(), game_data)?;

    game.run();

    Ok(())
}
