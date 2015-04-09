use libc::{c_float, c_int, c_char};

#[repr(C)]
pub struct rbDynamicsWorld;

#[repr(C)]
pub struct rbRigidBody;

#[repr(C)]
pub struct rbVert;

#[repr(C)]
pub struct rbTri;

#[repr(C)]
pub struct rbMeshData;

#[repr(C)]
pub struct rbCollisionShape;

#[repr(C)]
pub struct rbFilterCallback;

#[repr(C)]
pub struct rbConstraint;

pub const RB_LIMIT_LIN_X: c_int = 0;
pub const RB_LIMIT_LIN_Y:c_int = 1;
pub const RB_LIMIT_LIN_Z:c_int = 2;
pub const RB_LIMIT_ANG_X:c_int = 3;
pub const RB_LIMIT_ANG_Y:c_int = 4;
pub const RB_LIMIT_ANG_Z:c_int = 5;

#[link(name = "rb_bullet", kind = "static")]
extern {

	pub fn RB_dworld_new(gravity: *const c_float) -> *const rbDynamicsWorld;

	// Delete the given dynamics world, and free any extra data it may require
	pub fn RB_dworld_delete(world: *const rbDynamicsWorld);

	// Settings -------------------------

	// Gravity
	pub fn RB_dworld_get_gravity(world: *const rbDynamicsWorld, g_out: *const c_float);
	pub fn RB_dworld_set_gravity(world: *const rbDynamicsWorld, g_in: *const c_float);

	// Constraint Solver
	pub fn RB_dworld_set_solver_iterations(world: *const rbDynamicsWorld, num_solver_iterations: c_int);
	// Split Impulse
	pub fn RB_dworld_set_split_impulse(world: *const rbDynamicsWorld, split_impulse: c_int);

	// Simulation -----------------------

	// Step the simulation by the desired amount (in seconds) with extra controls on substep sizes and maximum substeps
	pub fn RB_dworld_step_simulation(world: *const rbDynamicsWorld, timeStep: c_float, maxSubSteps: c_int, timeSubStep: c_float);

	// Export

	// Exports the dynamics world to physics simulator's serialisation format
	pub fn RB_dworld_export(world: *const rbDynamicsWorld, filename: *const c_char);

	// **********************************
	// Rigid Body Methods

	// Setup ----------------------------

	// Add RigidBody to dynamics world
	pub fn RB_dworld_add_body(world: *const rbDynamicsWorld, body: *const rbRigidBody, col_groups: c_int);

	// Remove RigidBody from dynamics world
	pub fn RB_dworld_remove_body(world: *const rbDynamicsWorld, body: *const rbRigidBody);

	// Collision detection

	pub fn RB_world_convex_sweep_test(
        world: *const rbDynamicsWorld, object: *const rbRigidBody,
        loc_start: *const c_float, loc_end: *const c_float,
        v_location: *const c_float, v_hitpoint: *const c_float, v_normal: *const c_float, 
        r_hit: *const c_int);

	// ............

	// Create new RigidBody instance
	pub fn RB_body_new(shape: *const rbCollisionShape, loc: *const c_float, rot: *const c_float) -> *const rbRigidBody;

	// Delete the given RigidBody instance
	pub fn RB_body_delete(body: *const rbRigidBody);

	// Settings -------------------------

	// 'Type'
	pub fn RB_body_set_type(body: *const rbRigidBody, body_type: c_int, mass: c_float);

	// ............

	// Collision Shape
	pub fn RB_body_set_collision_shape(body: *const rbRigidBody, shape: *const rbCollisionShape);

	// ............

	// Mass
	pub fn RB_body_get_mass(body: *const rbRigidBody) -> c_float;
	pub fn RB_body_set_mass(body: *const rbRigidBody, value: c_float);

	// Friction
	pub fn RB_body_get_friction(body: *const rbRigidBody) -> c_float;
	pub fn RB_body_set_friction(body: *const rbRigidBody, value: c_float);

	// Restitution
	pub fn RB_body_get_restitution(body: *const rbRigidBody) -> c_float;
	pub fn RB_body_set_restitution(body: *const rbRigidBody, value: c_float);

	// Damping
	pub fn RB_body_get_linear_damping(body: *const rbRigidBody) -> c_float;
	pub fn RB_body_set_linear_damping(body: *const rbRigidBody, value: c_float);

	pub fn RB_body_get_angular_damping(body: *const rbRigidBody) -> c_float;
	pub fn RB_body_set_angular_damping(body: *const rbRigidBody, value: c_float);

	pub fn RB_body_set_damping(object: *const rbRigidBody, liner: c_float, angular: c_float);

	// Sleeping Thresholds
	pub fn RB_body_get_linear_sleep_thresh(body: *const rbRigidBody) -> c_float;
	pub fn RB_body_set_linear_sleep_thresh(body: *const rbRigidBody, value: c_float);

	pub fn RB_body_get_angular_sleep_thresh(body: *const rbRigidBody) -> c_float;
	pub fn RB_body_set_angular_sleep_thresh(body: *const rbRigidBody, value: c_float);

	pub fn RB_body_set_sleep_thresh(body: rbRigidBody, liner: c_float, angular: c_float);

	// Linear Velocity
	pub fn RB_body_get_linear_velocity(body: *const rbRigidBody, v_out: *const c_float);
	pub fn RB_body_set_linear_velocity(body: *const rbRigidBody, v_in: *const c_float);

	// Angular Velocity
	pub fn RB_body_get_angular_velocity(body: *const rbRigidBody, v_out: c_float);
	pub fn RB_body_set_angular_velocity(body: *const rbRigidBody, v_in: *const c_float);

	// Linear/Angular Factor, used to lock translation/roation axes
	pub fn RB_body_set_linear_factor(object: *const rbRigidBody, x: c_float, y: c_float, z: c_float);
	pub fn RB_body_set_angular_factor(object: *const rbRigidBody, x: c_float, y: c_float, z: c_float);

	// Kinematic State
	pub fn RB_body_set_kinematic_state(body: *const rbRigidBody, kinematic: c_int);

	// RigidBody Interface - Rigid Body Activation States
	pub fn RB_body_get_activation_state(body: *const rbRigidBody);
	pub fn RB_body_set_activation_state(body: *const rbRigidBody, use_deactivation: c_int);
	pub fn RB_body_activate(body: *const rbRigidBody);
	pub fn RB_body_deactivate(body: *const rbRigidBody);


	// Simulation -----------------------

	// Get current transform matrix of RigidBody to use in Blender (OpenGL format)
	pub fn RB_body_get_transform_matrix(body: *const rbRigidBody, m_out: *const c_float);

	// Set RigidBody's location and rotation
	pub fn  RB_body_set_loc_rot(body: *const rbRigidBody, loc: *const c_float, rot: *const c_float);
	// Set RigidBody's local scaling
	pub fn RB_body_set_scale(body: *const rbRigidBody, scale: *const c_float);

	// ............

	// Get RigidBody's position as vector
	pub fn RB_body_get_position(body: *const rbRigidBody, v_out: *const c_float);
	// Get RigidBody's orientation as quaternion
	pub fn RB_body_get_orientation(body: *const rbRigidBody, v_out: *const c_float);

	// ............

	pub fn RB_body_apply_central_force(body: *const rbRigidBody, v_in: *const c_float);

	// **********************************
	// Collision Shape Methods

	// Setup (Standard Shapes) -----------

	pub fn RB_shape_new_box(x: c_float, y: c_float, z: c_float) -> *const rbCollisionShape;
	pub fn RB_shape_new_sphere(radius: c_float) -> *const rbCollisionShape;
	pub fn RB_shape_new_capsule(radius: c_float, height: c_float) -> *const rbCollisionShape;
	pub fn RB_shape_new_cone(radius: c_float, height: c_float) -> *const rbCollisionShape;
	pub fn RB_shape_new_cylinder(radius: c_float, height: c_float) -> *const rbCollisionShape;

	// Setup (Convex Hull) ------------

	pub fn RB_shape_new_convex_hull(verts: *const c_float, stride: c_int, count: c_int, margin: c_float, can_embed: *const c_char) -> *const rbCollisionShape;

	// Setup (Triangle Mesh) ----------

	// 1
	pub fn RB_trimesh_data_new(num_tris: c_int, num_verts: c_int) -> *const rbMeshData;
	pub fn RB_trimesh_add_vertices(mesh: *const rbMeshData, vertices: *const c_float, num_verts: c_int, vert_stride: c_int);
	pub fn RB_trimesh_add_triangle_indices(mesh: *const rbMeshData, num: c_int, index0: c_int, index1: c_int, index2: c_int);
	pub fn RB_trimesh_finish(mesh: *const rbMeshData);

	// 2a - Triangle Meshes
	pub fn RB_shape_new_trimesh(mesh: *const rbMeshData) -> *const rbCollisionShape;
	// 2b - GImpact Meshes
	pub fn RB_shape_new_gimpact_mesh(mesh: *const rbMeshData);


	// Cleanup ---------------------------

	pub fn RB_shape_delete(shape: *const rbCollisionShape);

	// Settings ---------------------------

	// Collision Margin
	pub fn RB_shape_get_margin(shape: *const rbCollisionShape) -> c_float;
	pub fn RB_shape_set_margin(shape: *const rbCollisionShape, value: c_float);

	pub fn RB_shape_trimesh_update(shape: *const rbCollisionShape, vertices: *const c_float, num_verts: c_int, 
		vert_stride: c_int, min: *const c_float, max: *const c_float);

	// **********************************
	// Constraints

	// Setup -----------------------------

	// Add Rigid Body Constraint to simulation world
	pub fn RB_dworld_add_constraint(world: *const rbDynamicsWorld, con: *const rbConstraint, disable_collisions: c_int);

	// Remove Rigid Body Constraint from simulation world
	pub fn RB_dworld_remove_constraint(world: *const rbDynamicsWorld, con: *const rbConstraint);

	pub fn RB_constraint_new_point(pivot: *const c_float, rb1: *const rbRigidBody, rb2: *const rbRigidBody) -> *const rbConstraint;
	pub fn RB_constraint_new_fixed(pivot: *const c_float, orn: *const c_float, rb1: *const rbRigidBody, rb2: *const rbRigidBody) -> *const rbConstraint;
	pub fn RB_constraint_new_hinge(pivot: *const c_float, orn: *const c_float, rb1: *const rbRigidBody, rb2: *const rbRigidBody) -> *const rbConstraint;
	pub fn RB_constraint_new_slider(pivot: *const c_float, orn: *const c_float, rb1: *const rbRigidBody, rb2: *const rbRigidBody) -> *const rbConstraint;
	pub fn RB_constraint_new_piston(pivot: *const c_float, orn: *const c_float, rb1: *const rbRigidBody, rb2: *const rbRigidBody) -> *const rbConstraint;
	pub fn RB_constraint_new_6dof(pivot: *const c_float, orn: *const c_float, rb1: *const rbRigidBody, rb2: *const rbRigidBody) -> *const rbConstraint;
	pub fn RB_constraint_new_6dof_spring(pivot: *const c_float, orn: *const c_float, rb1: *const rbRigidBody, rb2: *const rbRigidBody) -> *const rbConstraint;
	pub fn RB_constraint_new_motor(pivot: *const c_float, orn: *const c_float, rb1: *const rbRigidBody, rb2: *const rbRigidBody) -> *const rbConstraint;

	// ............

	// Cleanup ---------------------------

	pub fn RB_constraint_delete(con: *const rbConstraint);

	// Settings ---------------------------

	// Enable or disable constraint
	pub fn RB_constraint_set_enabled(con: *const rbConstraint, enabled: c_int);

	// Limits
	/* Bullet uses the following convention:
	 * - lower limit == upper limit -> axis is locked
	 * - lower limit > upper limit -> axis is free
	 * - lower limit < upper limit -> axis is limited in given range
	 */
	pub fn RB_constraint_set_limits_hinge(con: *const rbConstraint, lower: c_float, upper: c_float);
	pub fn RB_constraint_set_limits_slider(con: *const rbConstraint, lower: c_float, upper: c_float);
	pub fn RB_constraint_set_limits_piston(con: *const rbConstraint, lin_lower: c_float, lin_upper: c_float, ang_lower: c_float, ang_upper: c_float);
	pub fn RB_constraint_set_limits_6dof(con: *const rbConstraint, axis: c_int, lower: c_float, upper: c_float);

	/* 6dof spring specific */
	pub fn RB_constraint_set_stiffness_6dof_spring(con: *const rbConstraint, axis: c_int, stiffness: c_float);
	pub fn RB_constraint_set_damping_6dof_spring(con: *const rbConstraint, axis: c_int, damping: c_float);
	pub fn RB_constraint_set_spring_6dof_spring(con: *const rbConstraint, axis: c_int, enable: c_int);
	pub fn RB_constraint_set_equilibrium_6dof_spring(con: *const rbConstraint);

	/* motors */
	pub fn RB_constraint_set_enable_motor(con: *const rbConstraint, enable_lin: c_int, enable_ang: c_int);
	pub fn RB_constraint_set_max_impulse_motor(con: *const rbConstraint, max_impulse_lin: c_float, max_impulse_ang: c_float);
	pub fn RB_constraint_set_target_velocity_motor(con: *const rbConstraint, velocity_lin: c_float, velocity_ang: c_float);

	/* Set number of constraint solver iterations made per step, this overrided world setting
	 * To use default set it to -1 */
	pub fn RB_constraint_set_solver_iterations(con: *const rbConstraint, num_solver_iterations: c_int);

	/* Set breaking impulse threshold, if constraint shouldn't break it can be set to FLT_MAX */
	pub fn RB_constraint_set_breaking_threshold(con: *const rbConstraint, threshold: c_float);

}
