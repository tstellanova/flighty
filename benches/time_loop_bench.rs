#[macro_use]
extern crate criterion;

use criterion::Criterion;

extern crate flighty;
use flighty::simulato::*;
use flighty::physical_types::*;
use flighty::models::ActuatorControls;
use rand::prelude::*;



fn get_test_reference_position() -> GlobalPosition {
    GlobalPosition {
        lat: 37.8716, //degrees
        lon: -122.2727, //degrees
        alt_wgs84: 10.0 //meters
    }
}


/// Run the core simulation loop:
/// - update the simulated time
/// - change the actuator settings
/// - repeat
fn core_simulation() {
    let mut simulato = Simulato::new(&get_test_reference_position());
    simulato.increment_simulated_time();
    let mut rng = rand::thread_rng();

    for _i in 1..100 {
        let rand_act_level = rng.gen::<f32>().abs();
        let actuators: ActuatorControls = [rand_act_level; 16];
        simulato.increment_simulated_time();
        simulato.update(&actuators);
    }
}

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("coresim", |b| b.iter(|| core_simulation()));
}


criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

