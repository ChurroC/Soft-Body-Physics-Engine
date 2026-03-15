use glam::{vec4, Vec2, Vec4};

#[derive(Clone, Debug)]
pub struct Verlet {
    position: Vec2,
    last_position: Vec2,
    acceleration: Vec2,
    last_acceleration: Vec2,
    radius: f32,
    density: f32,
    last_dt: f32,
    color: Vec4,
}

impl Verlet {
    pub fn new(position: Vec2) -> Self {
        Verlet {
            position,
            last_position: position,
            acceleration: Vec2::ZERO,
            last_acceleration: Vec2::ZERO,
            radius: 9.0,
            density: 1.0,
            last_dt: 0.0,
            color: vec4(255.0, 255.0, 255.0, 1.0),
        }
    }

    pub fn get_color(&self) -> Vec4 {
        self.color
    }

    pub fn set_color(&mut self, color: Vec4) {
        self.color = color;
    }

    pub fn get_radius(&self) -> f32 {
        self.radius
    }

    pub fn set_radius(&mut self, radius: f32) {
        self.radius = radius;
    }

    pub fn get_mass(&self) -> f32 {
        self.density * std::f32::consts::PI * self.radius * self.radius
    }

    pub fn add_acceleration(&mut self, acceleration: Vec2) {
        self.acceleration += acceleration;
    }

    pub fn get_position(&self) -> Vec2 {
        self.position
    }

    pub fn get_velocity(&self) -> Vec2 {
        if self.last_dt == 0.0 {
            Vec2::ZERO
        } else {
            (self.position - self.last_position) / self.last_dt
        }
    }

    pub fn set_velocity(&mut self, velocity: Vec2, dt: f32) {
        self.last_position = self.position - velocity * dt;
    }

    pub fn set_position(&mut self, position: Vec2) {
        self.position = position;
    }

    pub fn get_interpolated_position(&self, alpha: f32) -> Vec2 {
        self.last_position + (self.position - self.last_position) * alpha
        // self.position + (self.get_velocity() * (alpha * self.last_dt)) // Problem here is vel is implicit and we could go out contraint but why not try it
    }

    pub fn update_position(&mut self, dt: f32) {
        let displacement = self.position - self.last_position;
        self.last_position = self.position;

        self.position += self.acceleration * dt * dt;
        self.position += displacement;

        self.last_acceleration = self.acceleration;
        self.last_dt = dt;
        self.acceleration = Vec2::ZERO;
    }
}
