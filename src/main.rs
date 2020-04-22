#![feature(clamp)]

use ordered_float::OrderedFloat;
use std::collections::BTreeMap;
use std::f64::{self, consts::E};

const MACH_ONE: f64 = 343.0;

struct FuelOptimizationProblem {
    gravity: f64,
    rocket_mass: f64,
    cross_sectional_area: f64,
    fuel_efficiency: f64,
    max_flow_rate: f64,
    max_throttle_change_rate: f64, // % / s
    // Velocity -> coefficient
    drag_coefficient: BTreeMap<OrderedFloat<f64>, f64>,
}

impl FuelOptimizationProblem {
    fn drag_coefficient(&self, velocity: f64) -> f64 {
        if velocity < 1.7 * MACH_ONE {
            0.3
        } else if velocity < 5.0 * MACH_ONE {
            0.15
        } else {
            0.07
        }
    }
}

#[derive(Clone, Default, Debug)]
struct RocketState {
    position: f64,
    velocity: f64,
    fuel_mass: f64,
    throttle_opening: f64,
    total_time: f64,
}

impl RocketState {
    fn tick(&mut self, problem: &FuelOptimizationProblem, dt: f64, desired_throttle_opening: f64) {
        let max_throttle_change = problem.max_throttle_change_rate * dt;
        let throttle_opening_delta = (desired_throttle_opening - self.throttle_opening)
            .clamp(-max_throttle_change, max_throttle_change);
        self.throttle_opening += throttle_opening_delta;
        self.total_time += dt;

        self.velocity += self.acceleration(problem) * dt;
        self.position += self.velocity * dt;

        self.fuel_mass -= self.mass_flow_rate(problem) * dt;
        self.fuel_mass = self.fuel_mass.clamp(0.0, f64::MAX);

        self.position = self.position.clamp(0.0, f64::MAX);
        if self.velocity < 0.0 && self.position == 0.0 {
            self.velocity = 0.0;
        }
    }

    fn mass_flow_rate(&self, problem: &FuelOptimizationProblem) -> f64 {
        if self.fuel_mass <= 0.0 {
            0.0
        } else {
            problem.max_flow_rate * self.throttle_opening
        }
    }

    fn acceleration(&self, problem: &FuelOptimizationProblem) -> f64 {
        let total_mass = self.fuel_mass + problem.rocket_mass;
        let air_density = 1.46 * E.powf(-0.000134 * self.position);
        let thrust = (problem.fuel_efficiency * -problem.gravity * self.mass_flow_rate(problem));
        let gravitational_drag = (problem.gravity * total_mass);
        let air_drag = (problem.cross_sectional_area / 2.0
            * self.velocity
            * self.velocity
            * air_density
            * problem.drag_coefficient(self.velocity));
        (thrust + gravitational_drag - air_drag) / total_mass
    }
}

fn main() {
    let problem = FuelOptimizationProblem {
        gravity: -9.8,
        rocket_mass: 50.0,
        cross_sectional_area: 0.16,
        fuel_efficiency: 300.0,
        max_flow_rate: 20.0,
        max_throttle_change_rate: 0.2,
        drag_coefficient: BTreeMap::new(),
    };
    let initial_state = RocketState {
        fuel_mass: 950.0,
        ..RocketState::default()
    };

    simulate(
        &problem,
        initial_state,
        100.0,
        0.1,
        |_, _| 1.0,
        |state, current_time| {
            println!(
                "{}\t{}\t{}",
                current_time,
                state.acceleration(&problem) / -problem.gravity,
                state.fuel_mass
            );
        },
    );
}

fn simulate(
    problem: &FuelOptimizationProblem,
    initial_state: RocketState,
    total_time: f64,
    timestep: f64,
    desired_throttle: impl Fn(&FuelOptimizationProble, &RocketState) -> f64,
    on_step: impl Fn(&RocketState, f64),
) {
    let mut current_time = 0.0;
    let mut state = initial_state;
    while current_time <= total_time {
        on_step(&state, current_time);
        state.tick(&problem, timestep, desired_throttle(&state));

        current_time += timestep;
    }
}
