#![allow(dead_code)]
#![allow(unused_variables)]
mod cr3bp;
mod gui;

use cr3bp::{Cr3bs, Vec3};

#[allow(dead_code)]
// const EARTH_MOON_MU2: f64 = 1.215359904E-2;
const EARTH_MOON_MU2: f64 = 1.215058560962404E-2;
#[allow(dead_code)]
// const MARS_PHOBOS_MU2: f64 = 1.658841913E-8;
const MARS_PHOBOS_MU2: f64 = 1.611081404409632E-8;
#[allow(dead_code)]
// const SATURN_TITAN_MU2: f64 = 2.374941726E-4;
const SATURN_TITAN_MU2: f64 = 2.366393158331484E-4;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Earth-Moon L4 with slight perturbation
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(0.5 - EARTH_MOON_MU2 + 0.0005, 3f64.sqrt() / 2.0, 0.005),
    //     Vec3::new(0.0, 0.0, 0.0),
    // );

    // The following initial conditions come from
    // https://ssd.jpl.nasa.gov/tools/periodic_orbits.html

    // Earth-Moon distant retrograde orbit 840
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(7.842146212260807E-1, 0.0, 0.0),
    //     Vec3::new(0.0, 5.473049267844822E-1, 0.0),
    // );

    // Earth-Moon southern L2 halo orbit 77
    let system = Cr3bs::new(
        EARTH_MOON_MU2,
        Vec3::new(
            1.0895866679458164,
            -3.6612330039936E-27,
            -2.016985733889109E-1,
        ),
        Vec3::new(
            1.0612683677362947E-14,
            -2.0747636286776489E-1,
            3.917721566356704E-14,
        ),
    )
    .with_period(2.4829089190914457);

    // Earth-Moon southern L2 halo orbit 560 (NRHO)
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(
    //         1.0286910409504162,
    //         1.3946207617342125E-27,
    //         -1.8633782121335304E-1,
    //     ),
    //     Vec3::new(
    //         2.547548026681105E-14,
    //         -1.1733440134433075E-1,
    //         4.0454879893677124E-13,
    //     ),
    // )
    // .with_period(1.5991853351534902);

    // Earth-Moon northern butterfly orbit 54
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(9.685265364255409E-1, 0.0, 4.6230010224106094E-1),
    //     Vec3::new(0.0, -1.6184486927197775E-1, 0.0),
    // )
    // .with_period(1.1333928636951818E1);

    // Earth-Moon northern dragonfly orbit 1065
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(1.1115598114212446, 0.0, 1.8773782353883817E-1),
    //     Vec3::new(
    //         -7.424123344268251E-3,
    //         -2.2979620037171375E-1,
    //         7.084291638485501E-2,
    //     ),
    // );

    // Earth-Moon northern dragonfly orbit 71
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(1.1442729375808927, 0.0, 9.741181763409024E-2),
    //     Vec3::new(
    //         1.0072600603243715E-2,
    //         -3.439515629156075E-1,
    //         2.834590019383109E-1,
    //     ),
    // );

    // Earth-Moon northern butterfly orbit 810
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(
    //         9.18949042648061E-1,
    //         1.1933639486478275E-27,
    //         1.4641710057359364E-1,
    //     ),
    //     Vec3::new(
    //         7.205835296182384E-16,
    //         -1.3308495654568678E-1,
    //         -9.039324487239042E-15,
    //     ),
    // )
    // .with_period(4.612461961021733);

    // Earth-Moon distant retrograde orbit 504
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(2.3311424621341986E-1, 0.0, 0.0),
    //     Vec3::new(0.0, 2.418105116140247, 0.0),
    // );

    // Earth-Moon eastern low prograde orbit 1088
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(1.1037018564395995, 0.0, 0.0),
    //     Vec3::new(0.0, 1.5466801744690156E-1, 0.0),
    // );

    // Earth-Moon southern L3 halo orbit 728
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(-1.4910911328473246, 0.0, -9.190811179399557E-1),
    //     Vec3::new(0.0, 1.1226361377641865, 0.0),
    // );

    // Earth-Moon L4 axial orbit 357
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(4.8383615721820494E-1, 3.447318455507729E-1, 1E-1),
    //     Vec3::new(
    //         -5.08875059674026E-1,
    //         2.5546757482145843E-1,
    //         9.769305981522378E-1,
    //     ),
    // );

    // Earth-Moon L4 long period orbit 118
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(4.87849413449431E-1, 3.5688980531417824E-1, 0.0),
    //     Vec3::new(-8.550199344420721E-1, 3.0569819108334334E-1, 0.0),
    // );

    // Earth-Moon L4 short period orbit 118
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(4.87849413449431E-1, 1.4752174735141173, 0.0),
    //     Vec3::new(1.0286389945059438, -7.830350597801781E-1, 0.0),
    // );

    // Earth-Moon 2:3 resonant orbit 636
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(8.237755688613639E-1, 0.0, 0.0),
    //     Vec3::new(0.0, 6.169210577539296E-1, 0.0),
    // );

    // Earth-Moon southern L2 halo orbit 1386
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(1.1790623949937609, 0.0, -4.20474898439647E-2),
    //     Vec3::new(0.0, -1.6531977431864223E-1, 0.0),
    // );

    // Earth-Moon L1 Lyapunov orbit 468
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(
    //         6.819333350640226E-1,
    //         -5.368072145804689E-23,
    //         6.786899624599657E-26,
    //     ),
    //     Vec3::new(
    //         3.035814925676621E-13,
    //         6.785699388026654E-1,
    //         -1.3800761675868528E-24,
    //     ),
    // )
    // .with_period(6.111556928682104);

    // Earth-Moon L1 southern halo orbit 928
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(
    //         8.60129407766971E-1,
    //         1.2156094011891834E-27,
    //         -1.841529273318465E-1,
    //     ),
    //     Vec3::new(
    //         -8.010142504772853E-16,
    //         2.5380541457638645E-1,
    //         3.8099461975734E-14,
    //     ),
    // )
    // .with_period(2.3976941611197926);

    // Earth-Moon L3 southern halo orbit 520
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(
    //         -1.1498878138199098,
    //         1.2777122597454046E-23,
    //         -1.431709616456065,
    //     ),
    //     Vec3::new(
    //         2.284529721308476E-12,
    //         8.65249600694386E-1,
    //         3.749321548886891E-12,
    //     ),
    // )
    // .with_period(6.224058121581944);

    // Earth-Moon L2 southern halo orbit 1463
    // let system = Cr3bs::new(
    //     EARTH_MOON_MU2,
    //     Vec3::new(
    //         1.180506824843928,
    //         -3.3114436259161434E-27,
    //         -1.990687479633898E-2,
    //     ),
    //     Vec3::new(
    //         3.367239432965829E-15,
    //         -1.5811356683023692E-1,
    //         -2.2436379855191516E-15,
    //     ),
    // )
    // .with_period(3.4122877697103866);

    // let system = Cr3bs::new(
    //     SATURN_TITAN_MU2,
    //     Vec3::new(1.0146402930203535, 0.0, -5.197874717887378E-2),
    //     Vec3::new(0.0, -3.743802561661956E-2, 0.0),
    // );

    // gui::App::new(system, 0.001, 10, 30.0).run()?;
    // gui::App::new(system, 0.001, 10, 1000.0).run()?;
    gui::App::new(system, 0.001, 10, 100.0).run()?;

    Ok(())
}
