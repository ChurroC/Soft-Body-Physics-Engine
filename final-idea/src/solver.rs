use super::verlet::Verlet;
use glam::Vec2;

pub struct Solver {
    verlets: Vec<Verlet>,
    gravity: Vec2,
    constraint_radius: f32,
    subdivision: usize,
    cell_size: f32,
    grid_size: usize,
    grid_heads: Vec<usize>,
    grid_next: Vec<usize>,
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
            grid_heads: vec![usize::MAX; grid_size * grid_size],
            grid_next: vec![usize::MAX; verlets.len()],
        }
    }

    pub fn update(&mut self, dt: f32) {
        let sub_dt = dt / self.subdivision as f32;
        for _ in 0..self.subdivision {
            for verlet in &mut self.verlets {
                verlet.add_acceleration(self.gravity);
            }

            self.apply_wall_constraints(sub_dt);

            let collisions: Vec<(usize, usize)> = self.find_collisions_space_partitioning();
            self.solve_collisions(collisions, sub_dt);

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
    fn find_collisions_space_partitioning(&mut self) -> Vec<(usize, usize)> {
        // When we had double Vec we had cache misses but here we almost have a linked list in order in memory so better cache??
        let mut collisions: Vec<(usize, usize)> = vec![];

        self.grid_heads.fill(usize::MAX);
        if self.grid_next.len() != self.verlets.len() {
            self.grid_next.resize(self.verlets.len(), usize::MAX);
        }

        for (i, verlet) in self.verlets.iter().enumerate() {
            let pos = verlet.get_position();

            let cell_x = ((pos.x + self.constraint_radius) / self.cell_size) as i32;
            let cell_y = ((pos.y + self.constraint_radius) / self.cell_size) as i32;

            if cell_x >= 0
                && cell_x < self.grid_size as i32
                && cell_y >= 0
                && cell_y < self.grid_size as i32
            {
                let cell_index = (cell_y as usize * self.grid_size) + cell_x as usize;

                // Link this verlet to the current head of the cell
                self.grid_next[i] = self.grid_heads[cell_index];
                // Make this verlet the new head
                self.grid_heads[cell_index] = i;
            }
        }

        let neighbors = [(1, 0), (1, 1), (0, 1), (-1, 1)];

        for y in 0..self.grid_size {
            for x in 0..self.grid_size {
                let cell_idx = y * self.grid_size + x;

                // Iterate through every verlet in THIS cell
                let mut i_ptr = self.grid_heads[cell_idx];
                while i_ptr != usize::MAX {
                    let i = i_ptr;

                    let mut j_ptr = self.grid_next[i]; // I'th
                    while j_ptr != usize::MAX {
                        let j = j_ptr;
                        collisions.push((i.min(j), i.max(j)));
                        j_ptr = self.grid_next[j];
                    }

                    for (dx, dy) in neighbors {
                        let nx = x as i32 + dx;
                        let ny = y as i32 + dy;

                        if nx >= 0 && nx < self.grid_size as i32 && ny < self.grid_size as i32 {
                            let neighbor_idx = (ny as usize * self.grid_size) + nx as usize;

                            let mut n_ptr = self.grid_heads[neighbor_idx];
                            while n_ptr != usize::MAX {
                                let j = n_ptr as usize;
                                collisions.push((i.min(j), i.max(j)));
                                n_ptr = self.grid_next[j];
                            }
                        }
                    }

                    i_ptr = self.grid_next[i];
                }
            }
        }

        collisions
    }

    fn solve_collisions(&mut self, collisions: Vec<(usize, usize)>, dt: f32) {
        let coefficient_of_restitution = 0.93;

        for (i, j) in collisions {
            let (left, right) = self.verlets.split_at_mut(j);
            let verlet1 = &mut left[i];
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

    pub fn add_position(&mut self, verlet: Verlet) {
        self.verlets.push(verlet);
    }

    pub fn get_verlets(&self) -> &Vec<Verlet> {
        &self.verlets
    }
}
