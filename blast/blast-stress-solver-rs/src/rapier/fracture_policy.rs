/// Controls how fractures are processed per frame.
///
/// All limits default to -1 (unlimited). This mirrors the JS `FracturePolicy`.
#[derive(Clone, Copy, Debug)]
pub struct FracturePolicy {
    /// Max bonds to break per frame. -1 = unlimited.
    pub max_fractures_per_frame: i32,
    /// Max new rigid bodies to create per frame. -1 = unlimited.
    pub max_new_bodies_per_frame: i32,
    /// Global cap on dynamic rigid bodies. -1 = unlimited.
    pub max_dynamic_bodies: i32,
    /// Minimum node count for a split child to get a body. Default: 1.
    pub min_child_node_count: u32,
    /// Skip solver on idle frames (no forces, no recent fractures). Default: true.
    pub idle_skip: bool,
}

impl Default for FracturePolicy {
    fn default() -> Self {
        Self {
            max_fractures_per_frame: -1,
            max_new_bodies_per_frame: -1,
            max_dynamic_bodies: -1,
            min_child_node_count: 1,
            idle_skip: true,
        }
    }
}

impl FracturePolicy {
    /// Whether fracture generation should be suppressed this frame.
    pub fn should_suppress(&self, current_dynamic_bodies: usize) -> bool {
        if self.max_dynamic_bodies > 0
            && current_dynamic_bodies >= self.max_dynamic_bodies as usize
        {
            return true;
        }
        false
    }

    /// Clamp the number of fractures to the per-frame budget.
    pub fn clamp_fractures(&self, count: usize) -> usize {
        if self.max_fractures_per_frame < 0 {
            count
        } else {
            count.min(self.max_fractures_per_frame as usize)
        }
    }

    /// Clamp the number of new bodies to the per-frame budget.
    pub fn clamp_new_bodies(&self, count: usize) -> usize {
        if self.max_new_bodies_per_frame < 0 {
            count
        } else {
            count.min(self.max_new_bodies_per_frame as usize)
        }
    }

    /// Whether a child with the given node count should receive a body.
    pub fn child_qualifies(&self, node_count: u32) -> bool {
        node_count >= self.min_child_node_count
    }
}
