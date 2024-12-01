mod cr3bp;
mod gui;

use cr3bp::{Cr3bs, Vec3};

const EARTH_MOON_PI2: f64 = 0.01215359904;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Earth-Moon L4 with slight perturbation
    // let system = Cr3bs::new(
    //     EARTH_MOON_PI2,
    //     Vec3::new(0.5 - EARTH_MOON_PI2 + 0.005, 3f64.sqrt() / 2.0, 0.0),
    //     Vec3::new(0.0, 0.0, 0.0),
    // );

    // The following initial conditions come from
    // https://ssd.jpl.nasa.gov/tools/periodic_orbits.html

    // Earth-Moon distant retrograde orbit 840
    // let system = Cr3bs::new(
    //     EARTH_MOON_PI2,
    //     Vec3::new(7.842146212260807E-1, 0.0, 0.0),
    //     Vec3::new(0.0, 5.473049267844822E-1, 0.0),
    // );

    // Earth-Moon southern L2 halo orbit 77
    let system = Cr3bs::new(
        EARTH_MOON_PI2,
        Vec3::new(1.0895866679458164, 0.0, -2.016985733889109E-1),
        Vec3::new(0.0, -2.0747636286776489E-1, 0.0),
    );

    // Earth-Moon southern L2 halo orbit 560 (NRHO)
    // let system = Cr3bs::new(
    //     EARTH_MOON_PI2,
    //     Vec3::new(1.0286910409504162, 0.0, -1.8633782121335304E-1),
    //     Vec3::new(0.0, -1.1733440134433075E-1, 0.0),
    // );

    // Earth-Moon northern butterfly orbit 54
    // let system = Cr3bs::new(
    //     EARTH_MOON_PI2,
    //     Vec3::new(9.685265364255409E-1, 0.0, 4.6230010224106094E-1),
    //     Vec3::new(0.0, -1.6184486927197775E-1, 0.0),
    // );

    // Earth-Moon northern dragonfly orbit 1065
    // let system = Cr3bs::new(
    //     EARTH_MOON_PI2,
    //     Vec3::new(1.1115598114212446, 0.0, 1.8773782353883817E-1),
    //     Vec3::new(
    //         -7.424123344268251E-3,
    //         -2.2979620037171375E-1,
    //         7.084291638485501E-2,
    //     ),
    // );

    // Earth-Moon northern dragonfly orbit 71
    // let system = Cr3bs::new(
    //     EARTH_MOON_PI2,
    //     Vec3::new(1.1442729375808927, 0.0, 9.741181763409024E-2),
    //     Vec3::new(
    //         1.0072600603243715E-2,
    //         -3.439515629156075E-1,
    //         2.834590019383109E-1,
    //     ),
    // );

    // Earth-Moon northern butterfly orbit 810
    // let system = Cr3bs::new(
    //     EARTH_MOON_PI2,
    //     Vec3::new(9.18949042648061E-1, 0.0, 1.4641710057359364E-1),
    //     Vec3::new(0.0, -1.3308495654568678E-1, 0.0),
    // );

    // Earth-Moon distant retrograde orbit 504
    // let system = Cr3bs::new(
    //     EARTH_MOON_PI2,
    //     Vec3::new(2.3311424621341986E-1, 0.0, 0.0),
    //     Vec3::new(0.0, 2.418105116140247, 0.0),
    // );

    gui::App::new(system, 0.001, 30.0).run()?;

    Ok(())
}
