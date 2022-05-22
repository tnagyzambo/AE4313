extern crate csv;
extern crate kiss3d;
extern crate serde;
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::window::Window;
use serde::Deserialize;

#[derive(Deserialize)]
struct Step {
    t: f64,
    omega1: f64,
    omega2: f64,
    omega3: f64,
    q0: f64,
    q1: f64,
    q2: f64,
    q3: f64,
}

fn main() -> Result<(), csv::Error> {
    // CSV
    let mut reader = csv::Reader::from_path("output.csv")?;
    let mut steps = Vec::<Step>::new();

    for record in reader.deserialize() {
        let step: Step = record?;
        steps.push(step);
    }

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

        for step in &steps {
            let rot_truth = UnitQuaternion::<f32>::from_quaternion(Quaternion::new(
                step.q0 as f32,
                step.q1 as f32,
                step.q2 as f32,
                step.q3 as f32,
            ));

            let rot_truth_euler = rot_truth.euler_angles();

            satellite.set_local_rotation(rot_truth);

            window.draw_text(
                &format!("Time: {:.2}s", step.t),
                &Point2::new(50.0, 50.0),
                60.0,
                &font,
                &Point3::new(1.0, 1.0, 1.0),
            );

            window.draw_text(
                &format!(
                    "Attitude:\nRoll:  {:.2}°\nPitch: {:.2}°\nYaw:   {:.2}°",
                    rot_truth_euler.0 * 57.2958,
                    rot_truth_euler.1 * 57.2958,
                    rot_truth_euler.2 * 57.2958
                ),
                &Point2::new(50.0, 140.0),
                60.0,
                &font,
                &Point3::new(1.0, 1.0, 1.0),
            );

            window.draw_text(
                &format!(
                    "Rates:\nRoll:  {:.4}°/s\nPitch: {:.4}°/s\nYaw:   {:.4}°/s",
                    step.omega1 * 57.2958,
                    step.omega2 * 57.2958,
                    step.omega3 * 57.2958
                ),
                &Point2::new(50.0, 405.0),
                60.0,
                &font,
                &Point3::new(1.0, 1.0, 1.0),
            );

            window.render_with_camera(&mut camera);
        }
    }

    Ok(())
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
