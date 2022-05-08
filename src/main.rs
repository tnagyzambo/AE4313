extern crate diffeq;
extern crate kiss3d;
use diffeq::ode::{problem::OdeProblem, Ode};
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::nalgebra::{
    Matrix3, Matrix4, Point2, Point3, Quaternion, Translation3, UnitQuaternion, Vector3, Vector4,
};
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::window::Window;

fn dynamics(_: f64, v: &Vec<f64>) -> Vec<f64> {
    let omega_1 = v[0];
    let omega_2 = v[1];
    let omega_3 = v[2];
    let omega = Vector3::new(omega_1, omega_2, omega_3);

    let q_0 = v[3];
    let q_1 = v[4];
    let q_2 = v[5];
    let q_3 = v[6];
    let q = Vector4::new(q_0, q_1, q_2, q_3);

    let inertia = Matrix3::new(2500.0, 0.0, 0.0, 0.0, 2300.0, 0.0, 0.0, 0.0, 3000.0);

    let omega_o = 2.0 * std::f64::consts::PI / 5926.0;

    let r_bo = Matrix3::new(
        1.0 - 2.0 * (q_2.powf(2.0) + q_3.powf(2.0)),
        2.0 * (q_1 * q_2 + q_3 * q_0),
        2.0 * (q_1 * q_3 - q_2 * q_0),
        2.0 * (q_2 * q_1 - q_3 * q_0),
        1.0 - 2.0 * (q_1.powf(2.0) + q_3.powf(2.0)),
        2.0 * (q_2 * q_3 - q_1 * q_0),
        2.0 * (q_3 * q_1 - q_2 * q_0),
        2.0 * (q_3 * q_2 - q_1 * q_0),
        1.0 - 2.0 * (q_1.powf(2.0) + q_2.powf(2.0)),
    );

    let omega_bo = omega - omega_o * r_bo.column(1);

    let m_gg = 3.0 * omega_o.powf(2.0) * (r_bo.column(2).cross(&(inertia * r_bo.column(2))));

    let m_d = Vector3::new(0.001, 0.001, 0.001);

    let omega_dot = inertia.try_inverse().unwrap() * (m_gg + m_d - omega.cross(&(inertia * omega)));

    let q_dot =
        0.5 * Matrix4::new(
            0.0,
            omega_bo[(2, 0)],
            -omega_bo[(1, 0)],
            omega_bo[(0, 0)],
            -omega_bo[(2, 0)],
            0.0,
            omega_bo[(0, 0)],
            omega_bo[(1, 0)],
            omega_bo[(1, 0)],
            -omega_bo[(0, 0)],
            0.0,
            omega_bo[(2, 0)],
            -omega_bo[(0, 0)],
            -omega_bo[(1, 0)],
            -omega_bo[(2, 0)],
            0.0,
        ) * q;

    // Derivatives as vec
    vec![
        omega_dot[(0, 0)],
        omega_dot[(1, 0)],
        omega_dot[(2, 0)],
        q_dot[(0, 0)],
        q_dot[(1, 0)],
        q_dot[(2, 0)],
        q_dot[(3, 0)],
    ]
}

fn main() {
    // Physics
    let x0 = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
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
            let rot = UnitQuaternion::from_quaternion(Quaternion::new(
                step.1[3] as f32,
                step.1[4] as f32,
                step.1[5] as f32,
                step.1[6] as f32,
            ));
            satellite.set_local_rotation(rot);
            //println!("{}", satellite.data().local_rotation());

            window.draw_text(
                &format!("Time: {:.2}s", step.0),
                &Point2::new(50.0, 50.0),
                60.0,
                &font,
                &Point3::new(1.0, 1.0, 1.0),
            );

            let euler = rot.euler_angles();
            window.draw_text(
                &format!(
                    "Attitude:\nRoll:  {:.2}°\nPitch: {:.2}°\nYaw:   {:.2}°",
                    euler.0 * 57.2958,
                    euler.1 * 57.2958,
                    euler.2 * 57.2958
                ),
                &Point2::new(50.0, 140.0),
                60.0,
                &font,
                &Point3::new(1.0, 1.0, 1.0),
            );

            window.draw_text(
                &format!(
                    "Rates:\nRoll:  {:.4}°/s\nPitch: {:.4}°/s\nYaw:   {:.4}°/s",
                    step.1[0] * 57.2958,
                    step.1[1] * 57.2958,
                    step.1[2] * 57.2958
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
