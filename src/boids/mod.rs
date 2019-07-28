use amethyst::{
    prelude::*,
    assets::{Handle, Asset},
    renderer::{camera::Camera, visibility::BoundingSphere},
    ecs::*,
    core::{transform::{Transform}, Float},
};
use nalgebra::{Vector2, Vector3, Point2, Point3, UnitQuaternion, Unit, Translation};
use rayon::iter::ParallelIterator;
use amethyst::core::SystemBundle;

pub type Vec2 = Vector2<f32>;

// components

// The very vanilla pos/vel components, as this doesn't hook into nphysics

#[derive(Debug)]
pub struct Pos(pub Point2<f32>);

impl Component for Pos {
    type Storage = VecStorage<Self>;
}


#[derive(Debug)]
// The velocity also determines the heading!
pub struct Vel(pub Vec2);

impl Component for Vel {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
pub struct Closest(Vec<Entity>);

impl Component for Closest {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
pub struct SeparationVector(pub Vec2);

impl Component for SeparationVector {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
pub struct CohesionVector(pub Vec2);

impl Component for CohesionVector {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
pub struct AlignmentVector(pub Vec2);

impl Component for AlignmentVector {
    type Storage = VecStorage<Self>;
}

#[derive(Debug)]
pub struct CollisionVector(Vec2);

impl Component for CollisionVector {
    type Storage = VecStorage<Self>;
}

// Systems

#[derive(Debug)]
pub struct ClosenessThreshold(pub f32);

impl Default for ClosenessThreshold {
    fn default() -> Self {
        ClosenessThreshold(10.0)
    }
}

/// System that computes all the entities within a threshold.
pub struct ComputeClose;

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
pub struct CentreOfFlockValue(pub Point2<f32>);

/// Computes the centre of the flock, where flock is defined as all boids in the program.
pub struct CentreOfFlock;

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
pub struct SeparationDistance(pub f32);

impl Default for SeparationDistance {
    fn default() -> Self {
        SeparationDistance(0.0)
    }
}

pub struct Separation;

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
                        // Some(a.0 - pos.0)
                    } else {
                        None
                    }
                })
            }).fold(Vec2::zeros(), |acc, a| acc + a);
        }
    }
}

pub struct Cohesion;

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
                cohesion.0 = (pos.0.coords - (closest.0.iter().filter_map(|close| {
                    posdata.join().get(*close, &entities).map(|a| a.0.coords + pos.0.coords)
                }).sum::<Vec2>() / closest.0.len() as f32)).normalize();
            } else {
                cohesion.0 *= 0.0;
            }
        }
    }
}

pub struct Alignment;

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

#[derive(Default)]
pub struct CollisionBounds {
    pub x_min: f32,
    pub x_max: f32,
    pub y_min: f32,
    pub y_max: f32,
    pub threshold: f32,
}

impl CollisionBounds {

    pub fn new_rect_origin(width: f32, height: f32, origin: (f32, f32), threshold: f32) -> CollisionBounds {
        CollisionBounds {
            x_min: (width / -2.0) + origin.0,
            x_max: (width / 2.0) + origin.0,
            y_min: (height / -2.0) + origin.0,
            y_max: (height / 2.0) + origin.0,
            threshold
        }
    }
    
    pub fn new_rect(width: f32, height: f32, threshold: f32) -> CollisionBounds {
        Self::new_rect_origin(width, height, (0.0, 0.0), threshold)
    }

    pub fn closest_boxpoint(&self, pos: &Vec2) -> Option<Vec2> {
        let mut boxpoint = pos.clone();
        if pos.x <= self.x_min {
            boxpoint.x = self.x_min;
        } else if pos.x >= self.x_max {
            boxpoint.x = self.x_max;
        }
        if pos.y <= self.y_min {
            boxpoint.y = self.y_min;
        } else if pos.y >= self.y_max {
            boxpoint.y = self.y_max;
        }
        if boxpoint != *pos { Some(boxpoint) } else { None }
    }

    pub fn compute_collision_vector(&self, pos: &Pos, vel: &Vel) -> Vec2 {
        let mut avoidance = Vec2::new(0.0, 0.0);
        let predicted: Point2<f32> = pos.0 + vel.0;
        if let Some(boxpoint) = self.closest_boxpoint(&predicted.coords) {
            // mathematics nabbed from https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
            avoidance = (boxpoint - predicted.coords);
        }
        avoidance
    }
}

pub struct CollisionAvoidance;

impl <'a> System<'a> for CollisionAvoidance {
    type SystemData =
        ( ReadStorage<'a, Pos>
        , ReadStorage<'a, Vel>
        , WriteStorage<'a, CollisionVector>
        , Read<'a, CollisionBounds>
    );

    fn run(&mut self, (posdata, veldata, mut collisiondata, bounds): Self::SystemData) {
        (&posdata, &veldata, &mut collisiondata).par_join().for_each(
            |(pos, vel, mut collision)| {
                collision.0 = bounds.compute_collision_vector(pos, vel)
            }
        )
    }
}

pub struct ApplyAdjustments;

impl <'a> System<'a> for ApplyAdjustments {
    type SystemData =
        ( WriteStorage<'a, Vel>
        , ReadStorage<'a, SeparationVector>
        , ReadStorage<'a, CohesionVector>
        , ReadStorage<'a, AlignmentVector>
        , ReadStorage<'a, CollisionVector>
    );

    fn run(&mut self, (mut veldata, sepdata, cohdata, alidata, collidedata): Self::SystemData) {
        (&mut veldata, &sepdata, &cohdata, &alidata, &collidedata).par_join().for_each(
            |(vel, separation, cohesion, alignment, collision_avoid)| {
                vel.0 += (separation.0 * 1.0) + (cohesion.0 * 0.025) + (alignment.0 * 0.5) + (collision_avoid.0 * 1.0);
                vel.0.normalize_mut();
            }
        );
    }
}

pub struct Movement;

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

pub struct ReportEndCycle;

impl <'a> System<'a> for ReportEndCycle {
    type SystemData = Read<'a, Option<CentreOfFlockValue>>;

    fn run(&mut self, centre: Self::SystemData) {
        println!("Flock centre: {:?}", *centre);
    }
}

pub struct SyncWithTransform;

impl <'a> System<'a> for SyncWithTransform {
    type SystemData =
        ( WriteStorage<'a, Transform>
        , ReadStorage<'a, Pos>
        , ReadStorage<'a, Vel>
    );

    fn run(&mut self, (mut transformdata, posdata, veldata): Self::SystemData) {
        for (transform, pos, vel) in (&mut transformdata, &posdata, &veldata).join() {
            let t_pos = Vector3::new(pos.0.x, pos.0.y,0.0);
            let rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(0.0, 1.0, 0.0)), (vel.0.x.powi(2) + vel.0.y.powi(2)).sqrt());
            // let rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(0.0, vel.0.x, vel.0.y)), (vel.0.x.powi(2) + vel.0.y.powi(2)).sqrt());
            transform.set_translation(t_pos);
            transform.set_rotation(rot);
        }
    }
}

pub struct SyncCameraWithCentre;

impl <'a> System<'a> for SyncCameraWithCentre {
    type SystemData =
        ( WriteStorage<'a, Transform>
        , ReadStorage<'a, Camera>
        , Read<'a, Option<CentreOfFlockValue>>
    );

    fn run(&mut self, (mut transdata, cameradata, flockval_opt): Self::SystemData) {
        if let Some(ref flockval) = *flockval_opt {
            for (transform, _) in (&mut transdata, &cameradata).join() {
                let translate_mut = transform.translation_mut();
                translate_mut.x = flockval.0.coords.x.into();
                translate_mut.y = flockval.0.coords.y.into();
                
            }
        }
    }
}

pub struct BoidsBundle;

impl <'a, 'b> SystemBundle<'a, 'b> for BoidsBundle {
    fn build(self, builder: &mut DispatcherBuilder<'a, 'b>) -> amethyst::Result<()> {
        builder.add(ComputeClose, "compute_close", &[]);
        builder.add(Separation, "separation", &["compute_close"]);
        builder.add(Alignment, "alignment", &["compute_close"]);
        builder.add(Cohesion, "cohesion", &["compute_close"]);
        builder.add(CollisionAvoidance, "avoidance", &["compute_close"]);
        builder.add(ApplyAdjustments, "apply_adjustments", &["separation", "alignment", "cohesion", "avoidance"]);
        builder.add(Movement, "movement", &["apply_adjustments"]);
        builder.add(SyncWithTransform, "sync_with_transform", &["movement"]);
        builder.add(CentreOfFlock, "centre_of_flock", &[]);
        // builder.add(SyncCameraWithCentre, "sync_camera_with_centre", &["centre_of_flock"]);
        // builder.add(ReportEndCycle, "report_end", &["movement", "centre_of_flock", "sync_with_transform", "sync_camera_with_centre"]);
        Ok(())
    }
}

pub fn make_a_boid<A: Asset>(world: &mut World, position: Pos, velocity: Vel, handle: Handle<A>) {
    let transform = {
        let translation = Translation::from(Vector3::new(position.0.x, position.0.y, -20.0));
        let rotation = UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(velocity.0.x, velocity.0.y, 0.0)), 0.0);
        Transform::new(translation, rotation, Vector3::new(0.05, 0.05, 0.05))
    };
    world
        .create_entity()
        .with(position)
        .with(velocity)
        .with(Closest(Vec::new()))
        .with(SeparationVector(Vec2::zeros()))
        .with(CohesionVector(Vec2::zeros()))
        .with(AlignmentVector(Vec2::zeros()))
        .with(CollisionVector(Vec2::zeros()))
        .with(transform)
        .with(BoundingSphere::new(Point3::new(0.0.into(), 0.0.into(), 0.0.into()), 10.0))
        .with(handle)
        .build();
}

pub fn make_an_asset_entity<A: Asset>(world: &mut World, transform: Transform, handle: Handle<A>) -> Entity {
    world.create_entity().with(transform).with(handle).build()
}