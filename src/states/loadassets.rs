use amethyst::{
    prelude::*,
    assets::{AssetStorage, Loader, Handle, ProgressCounter, Completion},
    renderer::{ObjFormat, Mesh}
};



pub struct LoadAssets {
    progress: ProgressCounter,
    handle: Option<Handle<Mesh>>
}

impl LoadAssets {
    pub fn new() -> LoadAssets {
        LoadAssets { progress: ProgressCounter::new(), handle: None }
    }
}

impl SimpleState for LoadAssets {
    fn on_start(&mut self, data: StateData<'_, GameData<'_, '_>>) {
        let handle = {
            let meshstore = data.world.write_resource::<AssetStorage<Mesh>>();
            let loader = data.world.read_resource::<Loader>();
            loader.load("resources/boid.obj", ObjFormat, (), &mut self.progress, &meshstore)
        };
        self.handle = Some(handle)
    }

    fn update(&mut self, _data: &mut StateData<'_, GameData<'_, '_>>) -> SimpleTrans {
        match self.progress.complete() {
            Completion::Complete => {
                match &self.handle {
                    Some(handle) => {
                        Trans::Switch(Box::new(crate::states::scene::MainScene::new(handle.clone())))
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