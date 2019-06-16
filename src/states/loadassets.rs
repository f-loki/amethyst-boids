use amethyst::{
    prelude::*,
    assets::{PrefabLoader, Handle, ProgressCounter, Completion},
    gltf::*
};

pub struct LoadAssets {
    progress: ProgressCounter,
    gltf_scene: Option<Handle<GltfSceneAsset>>,
}

impl LoadAssets {
    pub fn new() -> LoadAssets {
        LoadAssets { 
            progress: ProgressCounter::new(), 
            gltf_scene: None, 
        }
    }
}

impl SimpleState for LoadAssets {
    fn on_start(&mut self, data: StateData<'_, GameData<'_, '_>>) {
        let handle: Handle<GltfSceneAsset> = data.world.exec(|loader: PrefabLoader<GltfPrefab>| {
            loader.load("boid.gltf", GltfSceneFormat::default(), &mut self.progress)
        });
        self.gltf_scene = Some(handle)
    }

    fn update(&mut self, _data: &mut StateData<'_, GameData<'_, '_>>) -> SimpleTrans {
        match self.progress.complete() {
            Completion::Complete => {
                match &self.gltf_scene {
                    Some(gltf_scene) => {
                        Trans::Switch(Box::new(crate::states::scene::MainScene::new(gltf_scene.clone())))
                    }
                    None => {
                        println!("Asset handle not aquired");
                        Trans::Quit
                    }
                }
            }
            Completion::Failed => {
                println!("Failed to load assets");
                Trans::Quit
            }
            Completion::Loading => {
                Trans::None
            }
        }
    }
}