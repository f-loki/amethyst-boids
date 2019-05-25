extern crate amethyst;
mod loadassets;

use amethyst::{
    prelude::*,
    assets::{AssetStorage, Loader, Handle, Asset, ProgressCounter, Completion},
    renderer::{DisplayConfig, DrawFlat, DrawShaded, Pipeline, PosNormTex, PosTex, RenderBundle, Stage, Shape, MeshHandle, Camera, Material, Projection, Light, SunLight, MaterialDefaults, ObjFormat, Mesh},
    utils::application_root_dir,
    ecs::*,
    core::transform::{Transform, bundle::TransformBundle},
};
use nalgebra::{Vector2, Vector3, Point2, UnitQuaternion, Unit, Translation};
use rayon::iter::ParallelIterator;
use amethyst::core::SystemBundle;
use rand::prelude::*;

struct LoadAssets {
    progress: ProgressCounter,
    handle: Option<Handle<Mesh>>
}

impl LoadAssets {
    fn new() -> LoadAssets {
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

    fn update(&mut self, data: &mut StateData<'_, GameData<'_, '_>>) -> SimpleTrans {
        match self.progress.complete() {
            Completion::Complete => {
                match &self.handle {
                    Some(handle) => {
                        Trans::Switch(Box::new(Example { handle: handle.clone() }))
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

struct Example {
    handle: Handle<Mesh>
}

impl SimpleState for Example {
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
        .with_bundle(BoidsBundle)?;
    let mut game = Application::new("./", LoadAssets::new(), game_data)?;

    game.run();

    Ok(())
}

type Vec2 = Vector2<f32>;

// components

// The very vanilla pos/vel components, as this doesn't hook into nphysics

#[derive(Debug)]
struct Pos(Point2<f32>);

impl Component for Pos {
    type Storage = VecStorage<Self>;
}


#[derive(Debug)]
// The velocity also determines the heading!
struct Vel(Vec2);

impl Component for Vel {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
struct Closest(Vec<Entity>);

impl Component for Closest {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
struct SeparationVector(Vec2);

impl Component for SeparationVector {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
struct CohesionVector(Vec2);

impl Component for CohesionVector {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
struct AlignmentVector(Vec2);

impl Component for AlignmentVector {
    type Storage = VecStorage<Self>;
}

// Systems

#[derive(Debug)]
struct ClosenessThreshold(f32);

impl Default for ClosenessThreshold {
    fn default() -> Self {
        ClosenessThreshold(10.0)
    }
}

/// System that computes all the entities within a threshold.
struct ComputeClose;

impl <'a> System<'a> for ComputeClose {

    type SystemData =
        ( Entities<'a>
        , ReadStorage<'a, Pos>
        , WriteStorage<'a, Closest>
        , Read<'a, ClosenessThreshold>
    );

    fn run(&mut self, (entities, posdata, mut closedata, threshold): Self::SystemData) {
        for (entity, position, close) in (&entities, &posdata, &mut closedata).join() {
            close.0.clear();
            for (check_entity, check_position) in (&entities, &posdata).join() {
                if check_entity != entity && nalgebra::distance(&position.0, &check_position.0) <= threshold.0 {
                    close.0.push(check_entity);
                    if close.0.len() >= 8 {
                        break;
                    }
                }
            }
        }
    }
}

#[derive(Debug)]
struct CentreOfFlockValue(Point2<f32>);

/// Computes the centre of the flock, where flock is defined as all boids in the program.
struct CentreOfFlock;

impl <'a> System<'a> for CentreOfFlock {

    type SystemData = (ReadStorage<'a, Pos>, Write<'a, Option<CentreOfFlockValue>>);

    fn run(&mut self, (posdata, mut centre_of_mass): Self::SystemData) {
        let mut ongoing = (0, Vec2::zeros());
        for pos in posdata.join() {
            ongoing.0 += 1;
            ongoing.1 += pos.0.coords;
        }
        let total = if ongoing.0 == 0 {ongoing.1} else {ongoing.1 / (ongoing.0 as f32)};
        *centre_of_mass = Some(CentreOfFlockValue(Point2::from(total)));
    }
}

#[derive(Debug)]
struct SeparationDistance(f32);

impl Default for SeparationDistance {
    fn default() -> Self {
        SeparationDistance(0.0)
    }
}

struct Separation;

impl <'a> System<'a> for Separation {
    type SystemData =
        ( Entities<'a>
        , Read<'a, SeparationDistance>
        , ReadStorage<'a, Closest>
        , ReadStorage<'a, Pos>
        , WriteStorage<'a, SeparationVector>
    );

    fn run(&mut self, (entities, septhresh, closedata, posdata, mut sepdata): Self::SystemData) {
        for (closest, pos, separation) in (&closedata, &posdata, &mut sepdata).join() {
            separation.0 = closest.0.iter().filter_map(|close| {
                posdata.join().get(*close, &entities).and_then(|a| {
                    if nalgebra::distance(&a.0, &pos.0) <= septhresh.0 {
                        Some(pos.0 - a.0)
                    } else {
                        None
                    }
                })
            }).fold(Vec2::zeros(), |acc, a| acc - a);
        }
    }
}

struct Cohesion;

impl <'a> System<'a> for Cohesion {
    type SystemData =
        ( Entities<'a>
        , ReadStorage<'a, Closest>
        , ReadStorage<'a, Pos>
        , WriteStorage<'a, CohesionVector>
    );

    fn run(&mut self, (entities, closedata, posdata, mut cohdata): Self::SystemData) {
        for (closest, pos, cohesion) in (&closedata, &posdata, &mut cohdata).join() {
            if !closest.0.is_empty() {
                // An amalgam of mathematics, that supposedly produces a vector
                // that shoves towards the centre of the nearby flockmates
                cohesion.0 = closest.0.iter().filter_map(|close| {
                    posdata.join().get(*close, &entities).map(|a| a.0.coords + pos.0.coords)
                }).sum::<Vec2>() / closest.0.len() as f32 - pos.0.coords;
            } else {
                cohesion.0 *= 0.0;
            }
        }
    }
}

struct Alignment;

impl <'a> System<'a> for Alignment {
    type SystemData =
        ( Entities<'a>
        , ReadStorage<'a, Closest>
        , ReadStorage<'a, Vel>
        , WriteStorage<'a, AlignmentVector>
    );

    fn run(&mut self, (entities, closedata, veldata, mut alidata): Self::SystemData) {
        for (closest, vel, alignment) in (&closedata, &veldata, &mut alidata).join() {
            if !closest.0.is_empty() {
                alignment.0 = closest.0.iter().filter_map(|close| {
                    veldata.join().get(*close, &entities).map(|a| a.0)
                }).sum::<Vec2>() / closest.0.len() as f32 - vel.0;
            } else {
                alignment.0 *= 0.0;
            }
        }
    }
}

struct ApplyAdjustments;

impl <'a> System<'a> for ApplyAdjustments {
    type SystemData =
        ( WriteStorage<'a, Vel>
        , ReadStorage<'a, SeparationVector>
        , ReadStorage<'a, CohesionVector>
        , ReadStorage<'a, AlignmentVector>
    );

    fn run(&mut self, (mut veldata, sepdata, cohdata, alidata): Self::SystemData) {
        (&mut veldata, &sepdata, &cohdata, &alidata).par_join().for_each(
            |(vel, separation, cohesion, alignment)| {
                vel.0 += (separation.0 * 1.0) + (cohesion.0 * 0.01) + (alignment.0 * 1.0);
                vel.0.normalize_mut();
            }
        );
    }
}

struct Movement;

impl <'a> System<'a> for Movement {
    type SystemData =
        ( ReadStorage<'a, Vel>
        , WriteStorage<'a, Pos>
    );

    fn run(&mut self, (veldata, mut posdata): Self::SystemData) {
        (&veldata, &mut posdata).par_join().for_each(|(vel, pos)| {
            pos.0.coords += vel.0 * (1. / 60.0)
        });
    }
}

struct ReportEndCycle;

impl <'a> System<'a> for ReportEndCycle {
    type SystemData = Read<'a, Option<CentreOfFlockValue>>;

    fn run(&mut self, centre: Self::SystemData) {
        println!("Flock centre: {:?}", *centre);
    }
}

struct SyncWithTransform;

impl <'a> System<'a> for SyncWithTransform {
    type SystemData =
        ( WriteStorage<'a, Transform>
        , ReadStorage<'a, Pos>
        , ReadStorage<'a, Vel>
    );

    fn run(&mut self, (mut transformdata, posdata, veldata): Self::SystemData) {
        for (transform, pos, vel) in (&mut transformdata, &posdata, &veldata).join() {
            let t_pos = Vector3::new(pos.0.x, pos.0.y,0.0);
            let rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(vel.0.x, vel.0.y, 0.0)), 0.0);
            transform.set_position(t_pos);
            transform.set_rotation(rot);
        }
    }
}

struct SyncCameraWithCentre;

impl <'a> System<'a> for SyncCameraWithCentre {
    type SystemData =
        ( WriteStorage<'a, Transform>
        , ReadStorage<'a, Camera>
        , Read<'a, Option<CentreOfFlockValue>>
    );

    fn run(&mut self, (mut transdata, cameradata, flockval_opt): Self::SystemData) {
        if let Some(ref flockval) = *flockval_opt {
            for (transform, _) in (&mut transdata, &cameradata).join() {
                let transform_mut = transform.translation_mut();
                transform_mut.x = flockval.0.coords.x;
                transform_mut.y = flockval.0.coords.y;
            }
        }
    }
}

struct BoidsBundle;

impl <'a, 'b> SystemBundle<'a, 'b> for BoidsBundle {
    fn build(self, builder: &mut DispatcherBuilder<'a, 'b>) -> amethyst::core::bundle::Result<()> {
        builder.add(ComputeClose, "compute_close", &[]);
        builder.add(Separation, "separation", &["compute_close"]);
        builder.add(Alignment, "alignment", &["compute_close"]);
        builder.add(Cohesion, "cohesion", &["compute_close"]);
        builder.add(ApplyAdjustments, "apply_adjustments", &["separation", "alignment", "cohesion"]);
        builder.add(Movement, "movement", &["apply_adjustments"]);
        builder.add(SyncWithTransform, "sync_with_transform", &["movement"]);
        builder.add(CentreOfFlock, "centre_of_flock", &[]);
        builder.add(SyncCameraWithCentre, "sync_camera_with_centre", &["centre_of_flock"]);
        // builder.add(ReportEndCycle, "report_end", &["movement", "centre_of_flock", "sync_with_transform", "sync_camera_with_centre"]);
        Ok(())
    }
}

fn make_a_boid(world: &mut World, position: Pos, velocity: Vel, handle: Handle<Mesh>) {
    let transform = {
        let translation = Translation::from(Vector3::new(position.0.x, position.0.y, 0.0));
        let rotation = UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(velocity.0.x, velocity.0.y, 0.0)), 0.0);
        Transform::new(translation, rotation, Vector3::new(1.0, 1.0, 1.0))
    };
    world
        .create_entity()
        .with(position)
        .with(velocity)
        .with(Closest(Vec::new()))
        .with(SeparationVector(Vec2::zeros()))
        .with(CohesionVector(Vec2::zeros()))
        .with(AlignmentVector(Vec2::zeros()))
        .with(transform)
        .with(handle)
        .build();
}
