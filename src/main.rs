extern crate amethyst;
mod states;
mod boids;
mod render;

use amethyst::{
    prelude::*,
    renderer::{RenderingSystem, types::DefaultBackend, visibility::VisibilitySortingSystem},
    utils::application_root_dir,
    core::transform::{bundle::TransformBundle, Transform},
    assets::{PrefabLoaderSystem},
    gltf::{GltfSceneLoaderSystem, GltfPrefab},
    window::WindowBundle,
    animation::{AnimationBundle, VertexSkinningBundle},
};

use render::BoidyGraphBuilder;
use states::loadassets::LoadAssets;
use boids::BoidsBundle;
 
fn main() -> amethyst::Result<()> {
    amethyst::start_logger(Default::default());

    let path = application_root_dir()?;
    let assets = path.join("resources");
    let display_config_path = assets.join("display_config.ron");

    let game_data = GameDataBuilder::default()
        .with_bundle(WindowBundle::from_config_path(display_config_path))?
        .with(PrefabLoaderSystem::<GltfPrefab>::default(), "scene_loader", &[])
        .with_bundle(BoidsBundle)?
        .with(GltfSceneLoaderSystem::default(), "gltf_loader", &["scene_loader"])
        // the visibility bundle is necessary for the PBR rendering pass
        // but there seems to be a lack of an explicitly declared dependency
        // and that which worries me
        .with_bundle(AnimationBundle::<usize, Transform>::new("animation_control", "sampler_interpolation").with_dep(&["gltf_loader"]))?
        .with_bundle(TransformBundle::new().with_dep(&["animation_control", "sampler_interpolation"]))?
        .with_bundle(VertexSkinningBundle::new().with_dep(&["transform_system", "animation_control", "sampler_interpolation"]))?
        .with(VisibilitySortingSystem::new(), "visibility_system", &["transform_system"])
        
        .with_thread_local(RenderingSystem::<DefaultBackend, _>::new(BoidyGraphBuilder::default()));

    let mut game = Application::new(assets, LoadAssets::new(), game_data)?;

    game.run();

    Ok(())
}
