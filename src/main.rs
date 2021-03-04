#![feature(clamp)]

use interpolation::Lerp;
use ordered_float::OrderedFloat;
use std::collections::BTreeMap;
use std::f64::{self, consts::E};
use std::path::Path;

//ploter test lol not sure if I need it all
use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::{PointMarker, PointStyle};



type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

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
    max_drag_coefficient: f64,
}

impl FuelOptimizationProblem {
    fn drag_coefficient(&self, velocity: f64) -> f64 {
        let velocity = OrderedFloat(velocity);
        let first_below = self
            .drag_coefficient
            .range(..velocity)
            .next_back()
            .unwrap_or((&OrderedFloat(f64::MIN), &self.max_drag_coefficient));
        let next_above = self
            .drag_coefficient
            .range(velocity..)
            .next()
            .unwrap_or((&OrderedFloat(f64::MAX), &0.0));

        let percent = (velocity.0 - (first_below.0).0) / ((next_above.0).0 - (first_below.0).0);
        first_below.1.lerp(next_above.1, &percent)
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


    fn air_drag(&self, problem: &FuelOptimizationProblem) -> f64 {
        let air_density = 1.46 * E.powf(-0.000134 * self.position);
        (problem.cross_sectional_area / 2.0)
            * self.velocity
            * self.velocity
            * air_density
            * problem.drag_coefficient(self.velocity)
    }

    fn acceleration(&self, problem: &FuelOptimizationProblem) -> f64 {
        let total_mass = self.fuel_mass + problem.rocket_mass;
        let thrust = problem.fuel_efficiency * -problem.gravity * self.mass_flow_rate(problem);
        let gravitational_drag = problem.gravity * total_mass;
        (thrust + gravitational_drag - self.air_drag(problem)) / total_mass
    }
}

fn main() -> Result<()> {
    let map = load_drag_coefficient_map(Path::new("./drag_coefficient_map.txt"))?;
    let hard_coded = {
        let mut map = BTreeMap::new();
        map.insert(OrderedFloat(0.0), 0.3);
        map.insert(OrderedFloat(1.7 * MACH_ONE), 0.15);
        map.insert(OrderedFloat(5.0 * MACH_ONE), 0.07);
        map
    };
    let problem = FuelOptimizationProblem {
        gravity: -9.8,
        rocket_mass: 50.0,
        cross_sectional_area: 0.16,
        fuel_efficiency: 300.0, // ISP in seconds
        max_flow_rate: 20.0,
        max_throttle_change_rate: 2.0,
        drag_coefficient: map,
        max_drag_coefficient: 0.3,
    };
    let initial_state = RocketState {
        fuel_mass: 950.0,
        ..RocketState::default()
    };

    simulate(
        &problem,
        initial_state,
        900.0,
        100.0,
        |_, _| 1.0,
        |state, current_time| {
            println!(
                "{}\t{}\t{}\t{}\t{}\t{}",
                current_time,
                state.acceleration(&problem) / -problem.gravity,
                state.fuel_mass,
                state.position,
                state.velocity,
                state.air_drag(&problem)
            );
        },
    );


//ploter test
// Scatter plots expect a list of pairs
    let data1 = vec![
        (-3.0, 2.3),
        (-1.6, 5.3),
        (0.3, 0.7),
        (4.3, -1.4),
        (6.4, 4.3),
        (8.5, 3.7),
    ];

    // We create our scatter plot from the data
    let s1: Plot = Plot::new(data1).point_style(
        PointStyle::new()
            .marker(PointMarker::Square) // setting the marker to be a square
            .colour("#DD3355"),
    ); // and a custom colour

    // The 'view' describes what set of data is drawn
    let v = ContinuousView::new()
        .add(s1)
        .x_range(-5., 10.)
        .y_range(-2., 6.)
        .x_label("Some varying variable")
        .y_label("The response of something");

    // A page with a single view is then saved to an SVG file
    Page::single(&v).save("scatter.svg").unwrap();




    Ok(())

}


fn load_drag_coefficient_map(path: &Path) -> Result<BTreeMap<OrderedFloat<f64>, f64>> {
    let mut reader = csv::Reader::from_path(path)?;
    let map = reader
        .records()
        .map(|result| parse_floats(&result?[0]))
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .step_by(6)
        .map(|floats| (OrderedFloat(floats[0] * MACH_ONE), floats[2]))
        .collect::<BTreeMap<_, _>>();
    Ok(map)
}

fn parse_floats(record: &str) -> Result<Vec<f64>> {
    record
        .split_whitespace()
        .map(|s| {
            s.parse::<f64>()
                .map_err(|err| Box::new(err) as Box<dyn std::error::Error>)
        })
        .collect()
}

fn simulate(
    problem: &FuelOptimizationProblem,
    initial_state: RocketState,
    total_time: f64,
    timestep: f64,
    desired_throttle: impl Fn(&FuelOptimizationProblem, &RocketState) -> f64,
    on_step: impl Fn(&RocketState, f64),
) {
    let mut current_time = 0.0;
    let mut state = initial_state;
    while current_time <= total_time {
        on_step(&state, current_time);
        state.tick(&problem, timestep, desired_throttle(problem, &state));

        current_time += timestep;
    }
}
