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
    active_grid: Vec<usize>,
    sort_grid: usize,
    grid_round: usize,
}

impl Solver {
    pub fn new(
        verlets: &[Verlet],
        gravity: Vec2,
        constraint_radius: f32,
        subdivision: usize,
        cell_size: f32,
    ) -> Self {
        // let grid_size = (constraint_radius * 2.0 / cell_size) as usize;
        let grid_size = 128;
        println!();
        println!("Grid size: {}", grid_size);
        println!(
            "Grid size best: {}",
            (constraint_radius * 2.0 / cell_size) as usize
        );
        Solver {
            verlets: verlets.iter().cloned().collect(),
            gravity,
            constraint_radius,
            subdivision,
            cell_size,
            grid_size,
            grid_heads: vec![usize::MAX; grid_size * grid_size],
            grid_next: vec![usize::MAX; verlets.len()],
            active_grid: Vec::new(),
            sort_grid: 100,
            grid_round: 0,
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

    fn part1by1(mut x: u32) -> u32 {
        x &= 0x0000ffff; // x = ---- ---- ---- ---- fedc ba98 7654 3210
        x = (x | (x << 8)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
        x = (x | (x << 4)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
        x = (x | (x << 2)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
        x = (x | (x << 1)) & 0x55555555; // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
        x
    }

    fn get_z_index(x: u32, y: u32) -> usize {
        // Interleave the bits of x and y
        ((Self::part1by1(y) << 1) | Self::part1by1(x)) as usize

        // Initally had
        /*
        fn get_z_index(x: u32, y: u32) -> usize {
            let mut z = 0;
            for i in 0..16 {
                z |= ((x & (1 << i)) << i) | ((y & (1 << i)) << (i + 1));
            }
            z as usize
        }
        */
    }

    fn compact1by1(mut x: u32) -> u32 {
        x &= 0x55555555; // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
        x = (x | (x >> 1)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
        x = (x | (x >> 2)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
        x = (x | (x >> 4)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
        x = (x | (x >> 8)) & 0x0000ffff; // x = ---- ---- ---- ---- fedc ba98 7654 3210
        x
    }

    fn decode_z_index(z: usize) -> (u32, u32) {
        let z = z as u32;
        (Self::compact1by1(z), Self::compact1by1(z >> 1))
    }

    // 1322 balls - 6 rad - 8 subs - 16 ms
    fn find_collisions_space_partitioning(&mut self) -> Vec<(usize, usize)> {
        // When we had double Vec we had cache misses but here we almost have a linked list in order in memory so better cache??
        // We now use Z order where up left right and down are close instead of left and right like in x and y order
        self.grid_round += 1;
        if self.grid_round % self.sort_grid == 0 {
            let radius = self.constraint_radius;
            let c_size = self.cell_size;
            let g_size_m1 = (self.grid_size as u32).saturating_sub(1);

            // 2. Use unstable sort for a ~20-30% speed boost
            self.verlets.sort_unstable_by_key(|verlet| {
                let pos = verlet.get_position();

                // 3. Use .clamp() or manual min/max with the local variables
                // This is faster than repeated self. accesses
                let cx = ((pos.x + radius) / c_size).max(0.0) as u32;
                let cy = ((pos.y + radius) / c_size).max(0.0) as u32;

                let cx = cx.min(g_size_m1);
                let cy = cy.min(g_size_m1);

                // 4. Call the associated function directly
                Solver::get_z_index(cx, cy)
            });
        }
        let mut collisions: Vec<(usize, usize)> = vec![];

        self.active_grid.clear();
        self.grid_heads.fill(usize::MAX);
        if self.grid_next.len() != self.verlets.len() {
            self.grid_next.resize(self.verlets.len(), usize::MAX);
        }
        self.grid_next.fill(usize::MAX);

        for (i, verlet) in self.verlets.iter().enumerate() {
            let pos = verlet.get_position();
            let cell_x = ((pos.x + self.constraint_radius) / self.cell_size) as i32;
            let cell_y = ((pos.y + self.constraint_radius) / self.cell_size) as i32;

            if cell_x >= 0
                && cell_x < self.grid_size as i32
                && cell_y >= 0
                && cell_y < self.grid_size as i32
            {
                let cell_index = Self::get_z_index(cell_x as u32, cell_y as u32);

                if self.grid_heads[cell_index] == usize::MAX {
                    self.active_grid.push(cell_index); // Only track cells that actually HAVE balls
                }
                // Link this verlet to the current head of the cell
                self.grid_next[i] = self.grid_heads[cell_index];
                // Make this verlet the new head
                self.grid_heads[cell_index] = i;
            }
        }

        let neighbors = [(1, 0), (0, 1), (1, 1), (-1, 1)];

        for &cell_idx in &self.active_grid {
            let mut i_ptr = self.grid_heads[cell_idx];
            if i_ptr == usize::MAX {
                continue;
            }

            // Iterate through every verlet in THIS cell
            let (x, y) = Self::decode_z_index(cell_idx);
            while i_ptr != usize::MAX {
                let i = i_ptr;

                let mut j_ptr = self.grid_next[i]; // I'th on chain and check till the end
                while j_ptr != usize::MAX {
                    let j = j_ptr;
                    collisions.push((i.min(j), i.max(j)));
                    j_ptr = self.grid_next[j];
                }

                for (dx, dy) in neighbors {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;

                    if nx >= 0 && nx < self.grid_size as i32 && ny < self.grid_size as i32 {
                        let neighbor_idx = Self::get_z_index(nx as u32, ny as u32);

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

        collisions
    }

    fn solve_collisions(&mut self, collisions: Vec<(usize, usize)>, dt: f32) {
        let coefficient_of_restitution = 0.93;

        for (idx1, idx2) in collisions {
            let i = idx1.min(idx2);
            let j = idx2.max(idx1);

            // 2. Check bounds to be safe
            if j >= self.verlets.len() {
                continue;
            }
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
