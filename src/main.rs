extern crate amethyst;

use amethyst::{
    prelude::*,
    renderer::{DisplayConfig, DrawFlat, Pipeline, PosNormTex, RenderBundle, Stage},
    utils::application_root_dir,
    ecs::*,
};
use nalgebra::{Vector2, Point2};
use rayon::iter::ParallelIterator;

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

struct Pos(Point2<f32>);

impl Component for Pos {
    type Storage = VecStorage<Self>;
}


// The velocity also determines the heading!
struct Vel(Vec2);

impl Component for Vel {
    type Storage = VecStorage<Self>;
}

struct Closest(Vec<Entity>);

impl Component for Closest {
    type Storage = VecStorage<Self>;
}

struct SeparationVector(Vec2);

impl Component for SeparationVector {
    type Storage = VecStorage<Self>;
}

struct CohesionVector(Vec2);

impl Component for CohesionVector {
    type Storage = VecStorage<Self>;
}

struct AlignmentVector(Vec2);

impl Component for AlignmentVector {
    type Storage = VecStorage<Self>;
}

// Systems

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

                }
            }
        }
    }
}

struct CentreOfFlockValue(Point2<f32>);

/// Computes the centre of the flock, where flock is defined as all boids in the program.
struct CentreOfFlock;

impl <'a> System<'a> for CentreOfFlock {

    type SystemData = (ReadStorage<'a, Pos>, Write<'a, Option<CentreOfFlockValue>>);

    fn run(&mut self, (posdata, mut centre_of_mass): Self::SystemData) {
        let mut ongoing = (0, Vec2::new(0.0, 0.0));
        for pos in posdata.join() {
            ongoing.0 += 1;
            ongoing.1 += pos.0.coords;
        }
        let total = if ongoing.0 == 0 {ongoing.1} else {ongoing.1 / (ongoing.0 as f32)};
        *centre_of_mass = Some(CentreOfFlockValue(Point2::from(total)));
    }
}

struct Separation;

impl <'a> System<'a> for Separation {
    type SystemData =
        ( Entities<'a>
        , ReadStorage<'a, Closest>
        , WriteStorage<'a, SeparationVector>
    );

    fn run(&mut self, (entities, closedata, mut sepdata): Self::SystemData) {

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
                cohesion.0 = closest.0.iter().filter_map(|close| {
                    (posdata).join().get(*close, &entities).map(|a| a.0.coords + pos.0.coords)
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
        , WriteStorage<'a, AlignmentVector>
    );

    fn run(&mut self, (entities, closedata, mut alidata): Self::SystemData) {

    }
}

struct ParMovement;

impl <'a> System<'a> for ParMovement {
    type SystemData =
        ( ReadStorage<'a, Vel>
        , WriteStorage<'a, Pos>
    );

    fn run(&mut self, (veldata, mut posdata): Self::SystemData) {
        (&veldata, &mut posdata).par_join().for_each(|(vel, pos)| {
            pos.0.coords += vel.0
        })
    }
}

struct Movement;

impl <'a> System<'a> for Movement {
    type SystemData =
        ( ReadStorage<'a, Vel>
        , WriteStorage<'a, Pos>
    );

    fn run(&mut self, (veldata, mut posdata): Self::SystemData) {
        (&veldata, &mut posdata).join().for_each(|(vel, pos)| {
            pos.0.coords += vel.0
        })
    }
}
