extern crate diffeq;
extern crate kiss3d;
use diffeq::ode::{problem::OdeProblem, Ode};
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Point3, Translation3, UnitQuaternion, Vector3};
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::window::Window;

fn dynamics(_: f64, v: &Vec<f64>) -> Vec<f64> {
    let theta_1 = v[0];
    let theta_2 = v[1];
    let _theta_3 = v[2];
    let d_theta_1 = v[3];
    let d_theta_2 = v[4];
    let d_theta_3 = v[5];

    let j_11 = 2500.0;
    let j_22 = 2300.0;
    let j_33 = 3000.0;

    let c_13 = -theta_2.sin();
    let c_23 = theta_1.sin() * theta_2.cos();
    let c_33 = theta_1.cos() * theta_2.sin();

    let n = 1.0 as f64;

    // Lorenz equations
    let dd_theta_1 = (-3.0 * n.powf(2.0) * (j_22 - j_33) * c_23 * c_33
        + (j_22 - j_33) * d_theta_2 * d_theta_3)
        / j_11;
    let dd_theta_2 = (-3.0 * n.powf(2.0) * (j_33 - j_11) * c_33 * c_13
        + (j_33 - j_11) * d_theta_3 * d_theta_1)
        / j_22;
    let dd_theta_3 = (-3.0 * n.powf(2.0) * (j_11 - j_22) * c_13 * c_23
        + (j_11 - j_22) * d_theta_1 * d_theta_2)
        / j_33;

    // derivatives as vec
    vec![
        d_theta_1, d_theta_2, d_theta_3, dd_theta_1, dd_theta_2, dd_theta_3,
    ]
}

fn main() {
    // Physics
    let x0 = vec![0.01, 0.01, 0.0, 0.0, 0.0, 0.0];
    let t0 = 0.0;
    let t_end = 100.0;
    let n_samples = 1000;

    let ode = Ode::Ode45;

    let problem = OdeProblem::builder()
        .tspan_linspace(t0, t_end, n_samples)
        .fun(dynamics)
        .init(x0)
        .build()
        .unwrap();

    let solution = problem.solve(ode, Default::default()).unwrap().zipped();

    // Setup
    let camera_pos = Point3::new(80.0, -80.0, -80.0);
    let mut camera = ArcBall::new(camera_pos, Point3::origin());
    camera.rebind_drag_button(None);
    camera.set_up_axis_dir(-Vector3::z_axis());

    let font = Font::default();

    let mut window = Window::new("AE4313");
    window.set_light(Light::StickToCamera);

    // Inertial frame
    let mut inertial_axes = window.add_group();
    create_inertial_axes(&mut inertial_axes);

    // Satellite
    let mut satellite = window.add_group();
    create_satellite_cube(&mut satellite);
    create_satellite_axes(&mut satellite);

    while !window.should_close() {
        for event in window.events().iter() {
            match event.value {
                _ => {}
            }
        }

        for step in &solution {
            let rot = UnitQuaternion::<f32>::from_euler_angles(
                step.1[0] as f32,
                step.1[1] as f32,
                step.1[2] as f32,
            );
            satellite.set_local_rotation(rot);
            //println!("{}", satellite.data().local_rotation());

            window.draw_text(
                &format!("Time: {:.2}s", step.0),
                &Point2::new(50.0, 50.0),
                60.0,
                &font,
                &Point3::new(1.0, 1.0, 1.0),
            );

            window.draw_text(
                &format!(
                    "Attitude:\nRoll:  {:.2}°\nPitch: {:.2}°\nYaw:   {:.2}°",
                    step.1[0] * 57.2958,
                    step.1[1] * 57.2958,
                    step.1[2] * 57.2958
                ),
                &Point2::new(50.0, 140.0),
                60.0,
                &font,
                &Point3::new(1.0, 1.0, 1.0),
            );

            window.draw_text(
                &format!(
                    "Rates:\nRoll:  {:.2}°/s\nPitch: {:.2}°/s\nYaw:   {:.2}°/s",
                    step.1[3] * 57.2958,
                    step.1[4] * 57.2958,
                    step.1[5] * 57.2958
                ),
                &Point2::new(50.0, 405.0),
                60.0,
                &font,
                &Point3::new(1.0, 1.0, 1.0),
            );

            window.render_with_camera(&mut camera);
        }
    }
}

fn create_satellite_axes(group: &mut SceneNode) {
    let axis_shaft_radius = 1.0;
    let axis_shaft_length = 10.0;
    let axis_cone_radius = 1.5;
    let axis_cone_height = 4.0;

    // Origin
    let mut origin = group.add_sphere(2.0);
    origin.set_color(1.0, 1.0, 1.0);

    // X axis
    let mut x_axis = group.add_group();
    let mut x_axis_shaft = x_axis.add_cylinder(axis_shaft_radius, axis_shaft_length);
    let mut x_axis_cone = x_axis.add_cone(axis_cone_radius, axis_cone_height);

    x_axis_shaft.set_color(1.0, 0.0, 0.0);
    x_axis_cone.set_color(1.0, 0.0, 0.0);

    let x_rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -1.5708);
    x_axis_shaft.append_rotation(&x_rot);
    x_axis_cone.append_rotation(&x_rot);

    x_axis_shaft.append_translation(&Translation3::new(axis_shaft_length / 2.0, 0.0, 0.0));
    x_axis_cone.append_translation(&Translation3::new(
        axis_shaft_length + (axis_cone_height / 2.0),
        0.0,
        0.0,
    ));

    // Y axis
    let mut y_axis = group.add_group();
    let mut y_axis_shaft = y_axis.add_cylinder(axis_shaft_radius, axis_shaft_length);
    let mut y_axis_cone = y_axis.add_cone(axis_cone_radius, axis_cone_height);

    y_axis_shaft.set_color(0.0, 1.0, 0.0);
    y_axis_cone.set_color(0.0, 1.0, 0.0);

    y_axis_shaft.append_translation(&Translation3::new(0.0, axis_shaft_length / 2.0, 0.0));
    y_axis_cone.append_translation(&Translation3::new(
        0.0,
        axis_shaft_length + (axis_cone_height / 2.0),
        0.0,
    ));

    // Z axis
    let mut z_axis = group.add_group();
    let mut z_axis_shaft = z_axis.add_cylinder(axis_shaft_radius, 10.0);
    let mut z_axis_cone = z_axis.add_cone(axis_cone_radius, axis_cone_height);

    z_axis_shaft.set_color(0.0, 0.0, 1.0);
    z_axis_cone.set_color(0.0, 0.0, 1.0);

    let z_rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 1.5708);
    z_axis_shaft.append_rotation(&z_rot);
    z_axis_cone.append_rotation(&z_rot);

    z_axis_shaft.append_translation(&Translation3::new(0.0, 0.0, axis_shaft_length / 2.0));
    z_axis_cone.append_translation(&Translation3::new(
        0.0,
        0.0,
        axis_shaft_length + (axis_cone_height / 2.0),
    ));
}

fn create_satellite_cube(group: &mut SceneNode) {
    let mut cube = group.add_cube(50.0, 50.0, 50.0);
    cube.set_color(1.0, 1.0, 1.0);
    cube.set_points_size(10.0);
    cube.set_lines_width(1.0);
    cube.set_surface_rendering_activation(false);
}

fn create_inertial_axes(group: &mut SceneNode) {
    let axis_length = 500.0;
    let axis_thickness = 0.5;

    // Origin
    let mut origin = group.add_sphere(2.0);
    origin.set_color(1.0, 1.0, 1.0);

    // X axis
    let mut x_axis = group.add_group();
    let mut x_axis_pos = x_axis.add_cylinder(axis_thickness, axis_length);
    let mut x_axis_neg = x_axis.add_cylinder(axis_thickness, axis_length);
    x_axis_pos.set_color(1.0, 0.0, 0.0);
    x_axis_neg.set_color(1.0, 1.0, 1.0);

    let x_rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -1.5708);
    x_axis_pos.append_rotation(&x_rot);
    x_axis_neg.append_rotation(&x_rot);

    x_axis_pos.append_translation(&Translation3::new(axis_length / 2.0, 0.0, 0.0));
    x_axis_neg.append_translation(&Translation3::new(-axis_length / 2.0, 0.0, 0.0));

    // Y axis
    let mut y_axis = group.add_group();
    let mut y_axis_pos = y_axis.add_cylinder(axis_thickness, axis_length);
    let mut y_axis_neg = y_axis.add_cylinder(axis_thickness, axis_length);

    y_axis_pos.set_color(0.0, 1.0, 0.0);
    y_axis_neg.set_color(1.0, 1.0, 1.0);

    y_axis_pos.append_translation(&Translation3::new(0.0, axis_length / 2.0, 0.0));
    y_axis_neg.append_translation(&Translation3::new(0.0, -axis_length / 2.0, 0.0));

    // Z axis
    let mut z_axis = group.add_group();
    let mut z_axis_pos = z_axis.add_cylinder(axis_thickness, axis_length);
    let mut z_axis_neg = z_axis.add_cylinder(axis_thickness, axis_length);

    z_axis_pos.set_color(0.0, 0.0, 1.0);
    z_axis_neg.set_color(1.0, 1.0, 1.0);

    let z_rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -1.5708);
    z_axis_pos.append_rotation(&z_rot);
    z_axis_neg.append_rotation(&z_rot);

    z_axis_pos.append_translation(&Translation3::new(0.0, 0.0, axis_length / 2.0));
    z_axis_neg.append_translation(&Translation3::new(0.0, 0.0, -axis_length / 2.0));
}
