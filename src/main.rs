extern crate amethyst;

use amethyst::{
    prelude::*,
    renderer::{DisplayConfig, DrawFlat, Pipeline, PosNormTex, RenderBundle, Stage},
    utils::application_root_dir,
    ecs::*,
};
use nalgebra::{Vector2, Point2};
use rayon::iter::ParallelIterator;
use amethyst::core::SystemBundle;

struct Example;

impl SimpleState for Example {
    fn on_start(&mut self, data: StateData<'_, GameData<'_, '_>>) {
        data.world.register::<Pos>();
        data.world.register::<Vel>();
        data.world.register::<Closest>();
        data.world.register::<CohesionVector>();
        data.world.register::<AlignmentVector>();
        data.world.register::<SeparationVector>();
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
            .clear_target([0.00196, 0.23726, 0.21765, 1.0], 1.0)
            .with_pass(DrawFlat::<PosNormTex>::new()),
    );

    let game_data =
        GameDataBuilder::default()
        //.with_bundle(RenderBundle::new(pipe, Some(config)))?
        .with_bundle(BoidsBundle)?;
    let mut game = Application::new("./", Example, game_data)?;

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
            pos.0.coords += vel.0
        });
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
        builder.add(Movement, "par_movement", &["apply_adjustments"]);
        Ok(())
    }
}

fn make_a_boid(world: &mut World, position: Pos, vel: Vel) {
    
}
