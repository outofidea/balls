use crossbeam::channel::*;
use rand::Rng;
use rapier2d::math::Vector;
use rapier2d::prelude::*;
use raylib::prelude::*;
use std::collections::VecDeque;
use std::time::Duration;
use std::{f32, thread};
fn main() {
    let balls_num = 10000;
    let buffering_length = 50;

    #[derive(Clone, Copy, Debug)]
    struct Data {
        x: f32,
        y: f32,
        vel: f32,
        ang: f32,
    }

    struct Frame {
        frame: Vec<Data>,
    }

    let mut frames_buffer: VecDeque<Frame> = VecDeque::new();

    let mut prev_frame_data: Vec<Data> = vec![];

    let (rapier_to_main, main_from_rapier) = bounded(100);
    // let (tx, rx) = mpsc::channel();
    // let (main_to_rapier, rapier_from_main) = (rapier_to_main.clone(), main_from_rapier.clone());
    let (mut rl, thread) = raylib::init()
        .size(1000, 1000)
        .title("ass")
        .msaa_4x()
        .build();
    rl.set_target_fps(60);

    let screen_dims: (i32, i32) = (rl.get_screen_width(), rl.get_screen_height());
    println!("screen dims: {:#?}", screen_dims);
    rl.set_window_size(screen_dims.0, screen_dims.1);

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider4 = ColliderBuilder::cuboid(100.0, 3.0).restitution(0.6).build();
    let collider2 = ColliderBuilder::cuboid(100.0, 3.0)
        .restitution(0.6)
        .translation(Vector::new(0.0, 100.0))
        .build();
    let collider3 = ColliderBuilder::cuboid(3.0, 100.0)
        .restitution(0.6)
        .translation(Vector::new(100.0, 0.0))
        .build();
    let collider1 = ColliderBuilder::cuboid(3.0, 100.0).restitution(0.6).build();

    let collider_center = ColliderBuilder::capsule_x(2.0, 1.0).translation(Vector::new(50.0, 50.0));
    collider_set.insert(collider1);
    collider_set.insert(collider2);
    collider_set.insert(collider3);
    collider_set.insert(collider4);
    // collider_set.insert(collider_center);

    /* Create the bouncing ball. */

    let mut ball_body_handles: Vec<RigidBodyHandle> = Vec::new();
    for _ in 0..balls_num {
        let x = rand::thread_rng().gen_range(10..100);
        let y = rand::thread_rng().gen_range(10..100);
        // let x = 45;
        // let y = 45;
        let velx = rand::thread_rng().gen_range(90..100);
        let vely = rand::thread_rng().gen_range(90..100);
        // let velx = 10;
        // let vely = -10;
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![x as f32, y as f32])
            .linvel(Vector::new(velx as f32, vely as f32))
            .build();
        let ball_body_handle = rigid_body_set.insert(rigid_body);
        let collider = ColliderBuilder::ball(0.5).restitution(0.4).build();
        ball_body_handles.push(ball_body_handle);
        collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);
    }

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, -50.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    /* Run the game loop, stepping the simulation once per frame. */

    let child_thread = thread::spawn(move || {
        println!("RAPIER: child thread created!");
        loop {
            physics_pipeline.step(
                &gravity,
                &integration_parameters,
                &mut island_manager,
                &mut broad_phase,
                &mut narrow_phase,
                &mut rigid_body_set,
                &mut collider_set,
                &mut impulse_joint_set,
                &mut multibody_joint_set,
                &mut ccd_solver,
                Some(&mut query_pipeline),
                &physics_hooks,
                &event_handler,
            );

            let mut results: Vec<Data> = Vec::new();

            for balls in &ball_body_handles {
                let currball = &rigid_body_set[*balls];
                results.push(Data {
                    x: (currball.translation().x * 10.0),
                    y: (currball.translation().y * 10.0),
                    vel: currball.linvel().abs().magnitude().clamp(0.0, 255.0),
                    ang: currball.rotation().re,
                });
            }

            let frame = Frame { frame: results };

            let send = rapier_to_main.send(frame);

            match send {
                Ok(_) => (),
                Err(err) => println!("RAPIER: error sending results {}", err),
            }

            // let recv = rapier_from_main.recv_timeout(Duration::from_micros(50));
            // match recv {
            //     Ok(message) => {
            //         println!("recv mousepos: {:#?}", &message.mouse_pos);
            //         let (mouse_x, mouse_y) = message.mouse_pos;
            //         let rigid_body = RigidBodyBuilder::dynamic()
            //             .translation()
            //             .build();
            //         let handle = rigid_body_set.insert(rigid_body);
            //         let collider = ColliderBuilder::ball(0.5).restitution(0.5).build();
            //         ball_body_handles.push(handle);
            //         collider_set.insert_with_parent(collider, handle, &mut rigid_body_set);
            //     }
            //     Err(err) => (),
            // }

            thread::sleep(Duration::from_millis(1));
        }
    });

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);
        // if d.is_mouse_button_pressed(MouseButton::MOUSE_LEFT_BUTTON) {
        //     // d.draw_text("coc", 1000, 30, 25, Color::YELLOW)
        //     let touch: (f32, f32) = (d.get_mouse_position().x, d.get_mouse_position().y);
        //     println!("sent mousepos: {:#?}", touch);
        //     let tx = main_to_rapier.send(Message {
        //         message_type: MessageType::MainMousePos,
        //         rapier_result: Vec::new(),
        //         mouse_pos: touch,
        //     });
        //     match tx {
        //         Ok(_) => (),
        //         Err(err) => println!("error send mousepos: {err}"),
        //     }
        // }
        match main_from_rapier.try_recv() {
            Ok(recv_msg) => {
                frames_buffer.push_back(recv_msg);
                if frames_buffer.len() >= buffering_length {
                    d.clear_background(Color::BLACK);
                    let frame_data = match frames_buffer.pop_front() {
                        None => panic!("wtff?"),
                        Some(curr_frame) => curr_frame.frame,
                    };
                    for data in frame_data {
                        d.draw_circle(
                            d.get_screen_width() - data.x as i32,
                            d.get_screen_height() - data.y as i32,
                            5.0,
                            Color::new((data.vel + 100.0) as u8, (255.0 - data.vel) as u8, 0, 255),
                        );
                        d.draw_fps(700, 30);
                        d.draw_text(
                            format!("buffer length: {}", frames_buffer.len()).as_str(),
                            700,
                            60,
                            25,
                            Color::RED,
                        );
                    }
                } else {
                    d.clear_background(Color::BLACK);
                    d.draw_text("Buffering (Startup?)", 100, 60, 25, Color::RED);
                    d.draw_fps(700, 30);
                    d.draw_text(
                        format!("buffer length: {}", frames_buffer.len()).as_str(),
                        700,
                        60,
                        25,
                        Color::RED,
                    );
                }
            }
            Err(_) => {
                if (frames_buffer.len() >= buffering_length - 1) & (frames_buffer.len() >= 1) {
                    //if theres data in buffer
                    d.clear_background(Color::BLACK);
                    let frame_data = match frames_buffer.pop_front() {
                        None => panic!("wtff?"),
                        Some(curr_frame) => curr_frame.frame,
                    };
                    prev_frame_data.clone_from(&frame_data);
                    for data in frame_data {
                        d.draw_circle_lines(
                            d.get_screen_width() - data.x as i32,
                            d.get_screen_height() - data.y as i32,
                            5.0,
                            Color::new((data.vel + 100.0) as u8, (255.0 - data.vel) as u8, 0, 255),
                        );
                        d.draw_fps(700, 30);
                        d.draw_text(
                            format!("buffer length: {}", frames_buffer.len()).as_str(),
                            700,
                            60,
                            25,
                            Color::RED,
                        );
                    } //END if data is in buffer
                } else {
                    //if theres no data in buffer
                    d.clear_background(Color::BLACK);
                    // println!("RAYLIB: waiting for rapier data: {}", err);
                    if !prev_frame_data.is_empty() {
                        for data in &prev_frame_data {
                            d.draw_circle_lines(
                                d.get_screen_width() - data.x as i32,
                                d.get_screen_height() - data.y as i32,
                                5.0,
                                Color::new(
                                    (data.vel + 100.0) as u8,
                                    (255.0 - data.vel) as u8,
                                    0,
                                    255,
                                ),
                            );
                        }
                    }
                    d.draw_fps(700, 30);
                    d.draw_text(
                        format!("buffer length: {}", frames_buffer.len()).as_str(),
                        700,
                        60,
                        25,
                        Color::RED,
                    );
                    d.draw_text("Buffering (Startup?)", 100, 60, 25, Color::RED);
                } //END if theres no data in buffer

                thread::sleep(Duration::from_millis(1));
            }
        }
    }
}
