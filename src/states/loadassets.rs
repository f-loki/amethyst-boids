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

// pub struct LoadGlb<T, E> {
//     progress: ProgressCounter,
//     handle: Option<Handle<Mesh>>,
//     url: String,
//     constructor: Box<Fn (&LoadGlb<T, E>) -> Box<State<T, E>>>
// }

// impl <T, E: Send + Sync> LoadGlb<T, E> {
//     pub fn new(name: String, constructor: Box<Fn (&LoadGlb<T, E>) -> Box<State<T, E>>>) -> LoadGlb<T, E> {
//         LoadGlb { 
//             progress: ProgressCounter::new(), 
//             handle: None, 
//             url: name,
//             constructor: constructor 
//         }
//     }
// }

// impl SimpleState for LoadGlb<GameData<'static, 'static>, StateEvent> {
//     fn on_start(&mut self, data: StateData<'_, GameData<'_, '_>>) {
//         let handle = {
//             let meshstore = data.world.write_resource::<AssetStorage<Mesh>>();
//             let loader = data.world.read_resource::<Loader>();
//             loader.load(self.url.clone(), ObjFormat, (), &mut self.progress, &meshstore)
//         };
//         self.handle = Some(handle)
//     }

//     fn update(&mut self, _data: &mut StateData<'_, GameData<'_, '_>>) -> SimpleTrans {
//         match self.progress.complete() {
//             Completion::Complete => {
//                 match &self.handle {
//                     Some(_) => {
//                         Trans::Switch((self.constructor)(&self))
//                     }
//                     None => {
//                         println!("Asset handle not aquired");
//                         Trans::Quit
//                     }
//                 }
//             }
//             Completion::Failed => {
//                 println!("Failed to load assets");
//                 Trans::Quit
//             }
//             Completion::Loading => {
//                 Trans::None
//             }
//         }
//     }
// }