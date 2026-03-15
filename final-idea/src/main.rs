mod solver;
mod verlet;

use solver::Solver;
use verlet::Verlet;

use macroquad::prelude::{clear_background, draw_circle, draw_circle_lines, draw_text, get_fps, next_frame, screen_height, screen_width, Color, BLACK, RED, WHITE};
use glam::vec2;

use std::time::Instant;

#[macroquad::main("Game")]
async fn main() {
    let screen_width = screen_width();
    let screen_height = screen_height();

    let constraint_radius = screen_height.min(screen_width) / 2.0 - 50.0;

    let ball_size = 2.0;
    let mut solver = Solver::new(
        &[],
        vec2(0.0, 0.0),
        constraint_radius,
        8,
        ball_size * 2.5
    );

    let start_time = Instant::now();

    let dt = 32;  // 1 / 60.0 = 16.6 ms
    let mut accumulator = 0;

    let ball_drop_per_frame = 1;
    let mut ball_drop_accumlator = 0;

    let mut last_time = start_time.elapsed().as_millis();
    let mut total_time: u128 = 0;

    let fps_threshold: i32 = 60;
    let measurement_frames: i32 = 30; // Number of frames to confirm slowdown
    let mut slow_frames_accumulator: i32 = 0;
    let mut balls_til_60_fps: usize = 0;

    let mut angle_degree = 0;


    // let mut ball = Verlet::new(vec2(0.0, 100.0));
    // ball.set_radius(ball_size);
    // ball.set_velocity(10.0 * vec2(0.0, -1.0), dt as f32 / 1000.0);
    // solver.add_position(ball);

    // let mut ball2 = Verlet::new(vec2(0.0, 0.0));
    // ball2.set_radius(ball_size * 10.0);
    // ball2.set_velocity(10.0 * vec2(0.0, -1.0), dt as f32 / 1000.0);
    // solver.add_position(ball2);

    loop {
        let current_time = start_time.elapsed().as_millis();
        let frame_time = current_time - last_time; // Maybe add a cap to stop death dpiral
        let fps = 1.0 / (frame_time as f32 / 1000.0); // Maybe implement smoothing FPS
        last_time = current_time;
        
        accumulator += frame_time;

        while accumulator >= dt {
            solver.update(dt as f32 / 1000.0);
            accumulator -= dt;
            total_time += dt;
            ball_drop_accumlator += 1;

            if ball_drop_accumlator >= ball_drop_per_frame && !solver.is_container_full() {
                for _ in 0..10 {
                    let angle = angle_degree as f32 / 180.0 * std::f32::consts::PI;
                    let angle_vec = vec2(angle.cos(), angle.sin());
                    let mut ball = Verlet::new(constraint_radius * 0.98 * angle_vec);
                    ball.set_radius(ball_size);
                    ball.set_velocity(-100.0 * angle_vec, dt as f32 / 1000.0);
                    solver.add_position(ball);
                    angle_degree = (angle_degree % 360) + 3;
                }

                ball_drop_accumlator = 0;
            }
        }

        clear_background(BLACK);
        draw_circle_lines(screen_width / 2.0, screen_height / 2.0, constraint_radius, 1.0, WHITE);

        let alpha = accumulator as f32 / dt as f32;
        for verlet in solver.get_verlets() {
            // This is since the solver imagines the ball at being shows at 0, 0
            let origin = vec2(screen_width / 2.0, screen_height / 2.0);
            let interpolated_pos = origin + verlet.get_interpolated_position(alpha)  * vec2(1.0, -1.0);
            let (x, y) = interpolated_pos.into();
            draw_circle(x, y, verlet.get_radius(), Color::from_rgba(
                verlet.get_color().x as u8,
                verlet.get_color().y as u8,
                verlet.get_color().z as u8,
                255
            ));
        }

        if get_fps() < fps_threshold && balls_til_60_fps == 0 {
            slow_frames_accumulator += 1;
            if slow_frames_accumulator >= measurement_frames {
                balls_til_60_fps = solver.get_verlets().len();
            }
        } else if balls_til_60_fps == 0 {
            slow_frames_accumulator = 0;
        }

        [
            &format!(
                "FPS: {fps:.0}",
            ),
            &format!(
                "time: {:.3}", total_time as f32 / 1000.0
            ),
            &format!(
                "Verlets: {}", solver.get_verlets().len()
            ),
            &format!(
                "60 fps ball count: {balls_til_60_fps}"
            ),
        ].iter().enumerate().for_each(|(i, text)| {
            draw_text(text, 20.0, 30.0 + 30.0 * i as f32, 20.0, RED);
        });

        next_frame().await;
    }
}