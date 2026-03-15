use super::verlet::Verlet;
use glam::Vec2;

pub struct Solver {
    verlets: Vec<Verlet>,
    gravity: Vec2,
    constraint_radius: f32,
    subdivision: usize,

    cell_size: f32,
    grid_size: usize,
    grid_heads: Vec<i32>,
    next_indices: Vec<i32>,

    collision_pairs: Vec<(usize, usize)>,
}

impl Solver {
    pub fn new(
        verlets: &[Verlet],
        gravity: Vec2,
        constraint_radius: f32,
        subdivision: usize,
        cell_size: f32,
    ) -> Self {
        let grid_size = (constraint_radius * 2.0 / cell_size) as usize;
        Solver {
            verlets: verlets.iter().cloned().collect(),
            gravity,
            constraint_radius,
            subdivision,
            cell_size,
            grid_size,
            grid_heads: vec![-1; grid_size * grid_size],
            next_indices: vec![-1; verlets.len()],
            collision_pairs: Vec::new(),
        }
    }

    pub fn update(&mut self, dt: f32) {
        let sub_dt = dt / self.subdivision as f32;
        for _ in 0..self.subdivision {
            for verlet in &mut self.verlets {
                verlet.add_acceleration(self.gravity);
            }

            self.apply_wall_constraints(sub_dt);

            self.find_collisions_space_partitioning();
            self.solve_collisions(sub_dt);

            for verlet in &mut self.verlets {
                verlet.update_position(sub_dt);
            }
        }
    }

    fn apply_wall_constraints(&mut self, dt: f32) {
        let coefficient_of_restitution = 1.0;

        for verlet in &mut self.verlets {
            let dist_to_cen = verlet.get_position();
            let dist = dist_to_cen.length();

            if dist > self.constraint_radius - verlet.get_radius() {
                let dist_norm = dist_to_cen.normalize();

                let vel = verlet.get_velocity();
                let v_norm = vel.project_onto(dist_norm);

                let correct_position = dist_norm * (self.constraint_radius - verlet.get_radius());
                verlet.set_position(correct_position);
                verlet.set_velocity((vel - 2.0 * v_norm) * coefficient_of_restitution, dt);
                // Just push the portion normal to the wall inverse
            }
        }
    }

    // 1322 balls - 6 rad - 8 subs - 16 ms
    fn find_collisions_space_partitioning(&mut self) {
        // When we had double Vec we had cache misses but here we almost have a linked list in order in memory so better cache??
        self.collision_pairs.clear();
        self.grid_heads.fill(-1);

        if self.next_indices.len() != self.verlets.len() {
            self.next_indices.resize(self.verlets.len(), -1);
        }
        for i in 0..self.verlets.len() {
            let pos = self.verlets[i].get_position();

            let cell_x = (pos.x / self.cell_size).floor() as i32;
            let cell_y = (pos.y / self.cell_size).floor() as i32;

            // 2. Clamp the values so they stay inside the grid array
            let cx = cell_x.clamp(0, (self.grid_size - 1) as i32) as usize;
            let cy = cell_y.clamp(0, (self.grid_size - 1) as i32) as usize;

            let cell_idx = (cy * self.grid_size) + cx;
            self.next_indices[i] = self.grid_heads[cell_idx];
            self.grid_heads[cell_idx] = i as i32;
        }

        let neighbor_offsets: [(i8, i8); 5] = [(0, 0), (1, 0), (0, 1), (1, 1), (-1, 1)];

        for y in 0..self.grid_size {
            for x in 0..self.grid_size {
                // This logic correctly selects every cell in a checkerboard
                let current_cell_idx = (y * self.grid_size + x) as usize;

                for (dx, dy) in neighbor_offsets {
                    let nx = x as i32 + dx as i32;
                    let ny = y as i32 + dy as i32;

                    if nx >= 0
                        && nx < (self.grid_size as i32)
                        && ny >= 0
                        && ny < (self.grid_size as i32)
                    {
                        let target_cell_idx = (ny * (self.grid_size as i32) + nx) as usize;

                        let mut i = self.grid_heads[current_cell_idx];
                        while i != -1 {
                            // If checking the SAME cell, start 'j' at the ball AFTER 'i'
                            // If checking a NEIGHBOR cell, start 'j' at the 'grid_head'
                            let mut j = if dx == 0 && dy == 0 {
                                self.next_indices[i as usize]
                            } else {
                                self.grid_heads[target_cell_idx]
                            };

                            while j != -1 {
                                // Now you don't even need 'if i < j' for (0,0)
                                // because j is guaranteed to be a different ball further down the list
                                self.collision_pairs.push((i as usize, j as usize));

                                j = self.next_indices[j as usize];
                            }
                            i = self.next_indices[i as usize];
                        }
                    }
                }
            }
        }
    }

    fn solve_collisions(&mut self, dt: f32) {
        let coefficient_of_restitution = 0.93;

        for (i, j) in self.collision_pairs.iter() {
            let (left, right) = self.verlets.split_at_mut(*j);
            let verlet1 = &mut left[*i];
            let verlet2 = &mut right[0];

            let collision_axis = verlet1.get_position() - verlet2.get_position(); // This is the distance vector between the two verlets which is also the collision_axis vector to the plane of collison
            let dist = collision_axis.length();
            let min_dist = verlet1.get_radius() + verlet2.get_radius();

            if dist < min_dist {
                let collision_normal = collision_axis / dist;
                let overlap = (min_dist - dist) * 1.1;

                let vel1 = verlet1.get_velocity().project_onto(collision_normal);
                let vel1_perp = verlet1.get_velocity() - vel1;
                let vel2 = verlet2.get_velocity().project_onto(collision_normal);
                let vel2_perp = verlet2.get_velocity() - vel2;

                let m1 = verlet1.get_mass();
                let m2 = verlet2.get_mass();
                let total_mass = m1 + m2;
                let mass_ratio1 = m2 / total_mass;
                let mass_ratio2 = m1 / total_mass;

                let vel1f = (vel1 * (m1 - m2) + 2.0 * m2 * vel2) / (m1 + m2);
                let vel2f = (vel2 * (m2 - m1) + 2.0 * m1 * vel1) / (m1 + m2);

                verlet1.set_position(
                    verlet1.get_position() + collision_normal * overlap * mass_ratio1,
                );
                verlet2.set_position(
                    verlet2.get_position() - collision_normal * overlap * mass_ratio2,
                );

                // keeping the perp vel same and changing the collision vel
                verlet1.set_velocity((vel1_perp + vel1f) * coefficient_of_restitution, dt);
                verlet2.set_velocity((vel2_perp + vel2f) * coefficient_of_restitution, dt);
            }
        }
    }

    pub fn is_container_full(&self) -> bool {
        // Calculate total area of particles
        let total_particle_area: f32 = self
            .verlets
            .iter()
            .map(|v| std::f32::consts::PI * v.get_radius() * v.get_radius())
            .sum();

        // Calculate container area
        let container_area = std::f32::consts::PI * self.constraint_radius * self.constraint_radius;

        // Consider it full if particles take up more than X% of space
        // Note: Perfect circle packing is ~90.7% efficient
        let density = total_particle_area / container_area;
        density > 0.9 // or whatever threshold makes sense
    }

    pub fn add_position(&mut self, mut verlet: Verlet) {
        self.verlets.push(verlet);
    }

    pub fn get_verlets(&self) -> &Vec<Verlet> {
        &self.verlets
    }
}
