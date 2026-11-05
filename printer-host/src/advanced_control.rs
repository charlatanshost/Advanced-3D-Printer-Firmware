use nalgebra::{DMatrix, DVector, SMatrix, Matrix4, Vector4, Matrix2x4, Vector2, Matrix2};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

type Matrix4x2 = SMatrix<f64, 4, 2>;

#[derive(Debug, Clone)]
pub struct StateSpaceModel {
    pub a_matrix: Matrix4<f64>,      // State transition matrix
    pub b_matrix: Matrix4x2,    // Input matrix
    pub c_matrix: Matrix4<f64>,      // Output matrix
    pub d_matrix: Matrix4x2,    // Feedthrough matrix
    pub q_matrix: Matrix4<f64>,      // Process noise covariance
    pub r_matrix: Matrix4<f64>,      // Measurement noise covariance
}

#[derive(Debug, Clone)]
pub struct StateSpaceController {
    model: StateSpaceModel,
    state_estimate: Vector4<f64>,    // [position, velocity, acceleration, jerk]
    control_gain: Matrix2x4<f64>,    // LQR gain matrix
    observer_gain: Matrix4<f64>,     // Kalman filter gain
    reference: Vector4<f64>,         // Reference trajectory
    integral_state: Vector4<f64>,    // Integral state for steady-state error elimination
    previous_error: Vector4<f64>,    // For derivative term
    last_control: Vector2<f64>,      // Output of the previous update
    dt: f64,                    // Sample time
}

#[derive(Debug, Clone)]
pub struct MPCController {
    prediction_horizon: usize,  // N - prediction horizon
    control_horizon: usize,     // Nu - control horizon
    state_weights: Vector4<f64>,     // Q - state penalty weights
    input_weights: Vector2<f64>,     // R - input penalty weights
    terminal_weights: Vector4<f64>,  // P - terminal state weights
    
    // Constraints
    input_min: Vector2<f64>,         // Minimum control inputs
    input_max: Vector2<f64>,         // Maximum control inputs
    state_min: Vector4<f64>,         // Minimum states
    state_max: Vector4<f64>,         // Maximum states
    
    // Internal matrices for optimization
    prediction_matrix: DMatrix<f64>,  // Prediction matrix Phi
    control_matrix: DMatrix<f64>,     // Control matrix Gamma
    
    current_state: Vector4<f64>,
    reference_trajectory: VecDeque<Vector4<f64>>,
    dt: f64,
}

#[derive(Debug, Clone)]
pub struct AdvancedMotionController {
    state_space: StateSpaceController,
    mpc: MPCController,
    control_mode: ControlMode,
    axis_name: String,
    last_update: Instant,
    performance_metrics: ControlMetrics,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ControlMode {
    PID,           // Traditional PID control
    StateSpace,    // State-space control (LQR + observer)
    MPC,           // Model Predictive Control
    Hybrid,        // Switching between modes based on conditions
}

#[derive(Debug, Clone)]
pub struct ControlMetrics {
    pub steady_state_error: f64,
    pub overshoot_percent: f64,
    pub settling_time: Duration,
    pub rise_time: Duration,
    pub control_effort: f64,
    pub tracking_error_rms: f64,
}

impl StateSpaceModel {
    pub fn new_motion_model(dt: f64) -> Self {
        // State vector: [position, velocity, acceleration, jerk]
        // Input vector: [force, force_derivative]
        
        // Continuous-time state-space model for a mechanical system
        // x_dot = A*x + B*u
        // y = C*x + D*u
        
        let a_continuous = Matrix4::new(
            0.0, 1.0, 0.0, 0.0,  // position derivative = velocity
            0.0, 0.0, 1.0, 0.0,  // velocity derivative = acceleration  
            0.0, 0.0, 0.0, 1.0,  // acceleration derivative = jerk
            0.0, 0.0, 0.0, -1.0, // jerk derivative with damping
        );
        
        let b_continuous = Matrix4x2::new(
            0.0, 0.0,
            0.0, 0.0,
            1.0, 0.0,
            0.0, 1.0,
        );
        
        // Convert to discrete-time using zero-order hold
        // A_d = e^(A*dt)
        // B_d = A^(-1) * (A_d - I) * B
        
        let a_matrix = (a_continuous * dt).exp();

        // A motion model has integrators, so A is singular and A^(-1) does not exist. (Falling
        // back to an identity gave a wrong input matrix.) The series form needs no inverse:
        // B_d = (I*dt + A*dt²/2! + A²*dt³/3! + ...) * B
        let mut term = Matrix4::identity() * dt;
        let mut sum = term;
        for k in 2..16 {
            term = term * a_continuous * (dt / k as f64);
            sum += term;
        }
        let b_matrix = sum * b_continuous;
        
        // Output matrix - we can measure position and velocity
        let c_matrix = Matrix4::new(
            1.0, 0.0, 0.0, 0.0,  // position measurement
            0.0, 1.0, 0.0, 0.0,  // velocity measurement
            0.0, 0.0, 1.0, 0.0,  // acceleration measurement (if available)
            0.0, 0.0, 0.0, 1.0,  // jerk measurement (if available)
        );
        
        let d_matrix = Matrix4x2::zeros();
        
        // Process and measurement noise covariance matrices
        let q_matrix = Matrix4::identity() * 0.01; // Process noise
        let r_matrix = Matrix4::new(
            0.001, 0.0,   0.0,   0.0,    // Position measurement noise
            0.0,   0.01,  0.0,   0.0,    // Velocity measurement noise  
            0.0,   0.0,   0.1,   0.0,    // Acceleration measurement noise
            0.0,   0.0,   0.0,   1.0,    // Jerk measurement noise
        );
        
        Self {
            a_matrix,
            b_matrix,
            c_matrix,
            d_matrix,
            q_matrix,
            r_matrix,
        }
    }
}

impl StateSpaceController {
    pub fn new(model: StateSpaceModel, dt: f64) -> Self {
        // Design LQR controller
        let q_lqr = Matrix4::new(
            100.0, 0.0,  0.0, 0.0,   // Position weight
            0.0,   10.0, 0.0, 0.0,   // Velocity weight
            0.0,   0.0,  1.0, 0.0,   // Acceleration weight
            0.0,   0.0,  0.0, 0.1,   // Jerk weight
        );
        
        let r_lqr = Matrix2::<f64>::new(
            1.0, 0.0,
            0.0, 1.0,
        );
        
        // Solve Discrete Algebraic Riccati Equation (DARE)
        // This is a simplified implementation - in practice use a proper DARE solver
        let control_gain = Self::solve_lqr(&model.a_matrix, &model.b_matrix, &q_lqr, &r_lqr);
        
        // Design Kalman filter (observer)
        let observer_gain = Self::design_kalman_filter(&model);
        
        Self {
            model,
            state_estimate: Vector4::zeros(),
            control_gain,
            observer_gain,
            reference: Vector4::zeros(),
            integral_state: Vector4::zeros(),
            previous_error: Vector4::zeros(),
            last_control: Vector2::zeros(),
            dt,
        }
    }

    fn solve_lqr(a: &Matrix4<f64>, b: &Matrix4x2, q: &Matrix4<f64>, r: &Matrix2<f64>) -> Matrix2x4<f64> {
        // Simplified LQR solution using iterative method
        // In practice, use a proper DARE solver like from the `control` crate
        
        let mut p = q.clone();
        
        // Iterate to solve Riccati equation
        for _ in 0..100 {
            let temp = b.transpose() * &p * a;
            let s = r + b.transpose() * &p * b;
            let s_inv = s.try_inverse().unwrap_or_else(Matrix2::<f64>::identity);
            let k = s_inv * temp;
            
            let new_p = q + a.transpose() * &p * a - a.transpose() * &p * b * s_inv * b.transpose() * &p * a;
            
            if (new_p - &p).norm() < 1e-6 {
                break;
            }
            p = new_p;
        }
        
        let s = r + b.transpose() * &p * b;
        let s_inv = s.try_inverse().unwrap_or_else(Matrix2::<f64>::identity);
        s_inv * b.transpose() * &p * a
    }
    
    fn design_kalman_filter(model: &StateSpaceModel) -> Matrix4<f64> {
        // Simplified Kalman filter gain design
        // In practice, solve the dual Riccati equation
        
        let mut p = model.q_matrix.clone();
        
        // Iterate to solve filter Riccati equation
        for _ in 0..100 {
            let temp = &p * model.c_matrix.transpose();
            let s = &model.c_matrix * &p * model.c_matrix.transpose() + &model.r_matrix;
            let s_inv = s.try_inverse().unwrap_or(Matrix4::identity());
            let k = temp * s_inv;
            
            let new_p = &model.a_matrix * &p * model.a_matrix.transpose() + &model.q_matrix
                - &model.a_matrix * &p * model.c_matrix.transpose() * s_inv * &model.c_matrix * &p * model.a_matrix.transpose();
            
            if (new_p - &p).norm() < 1e-6 {
                break;
            }
            p = new_p;
        }
        
        let s = &model.c_matrix * &p * model.c_matrix.transpose() + &model.r_matrix;
        let s_inv = s.try_inverse().unwrap_or(Matrix4::identity());
        &p * model.c_matrix.transpose() * s_inv
    }
    
    pub fn update(&mut self, measurement: Vector4<f64>, reference: Vector4<f64>) -> Vector2<f64> {
        // Update reference
        self.reference = reference;

        // Kalman filter update (observer)
        let innovation = measurement - &self.model.c_matrix * &self.state_estimate;
        self.state_estimate += &self.observer_gain * innovation;

        // Predict next state, including the effect of the last control output. Leaving that
        // out means the observer never expects the motion it is itself commanding.
        self.state_estimate = &self.model.a_matrix * &self.state_estimate + &self.model.b_matrix * self.last_control;

        // Calculate control input using LQR
        let error = self.reference - self.state_estimate;
        let control_input = &self.control_gain * error;
        
        // Update integral state for steady-state error elimination, limited against windup
        const INTEGRAL_LIMIT: f64 = 100.0;
        self.integral_state += error * self.dt;
        self.integral_state = self.integral_state.map(|v| v.clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT));
        let integral_gain = Matrix2x4::new(0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0);
        let integral_control = integral_gain * self.integral_state;
        
        // Add derivative term
        let derivative_error = (error - self.previous_error) / self.dt;
        let derivative_gain = Matrix2x4::new(0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0);
        let derivative_control = derivative_gain * derivative_error;
        
        self.previous_error = error;

        let output = control_input + integral_control + derivative_control;
        self.last_control = output;
        output
    }
}

impl MPCController {
    pub fn new(prediction_horizon: usize, control_horizon: usize, dt: f64) -> Self {
        let state_weights = Vector4::new(100.0, 10.0, 1.0, 0.1);  // Position, velocity, acceleration, jerk
        let input_weights = Vector2::new(1.0, 1.0);                // Force, force_derivative
        let terminal_weights = state_weights * 10.0;               // Terminal cost
        
        // Constraints
        let input_min = Vector2::new(-1000.0, -10000.0);  // Force limits
        let input_max = Vector2::new(1000.0, 10000.0);
        let state_min = Vector4::new(-1000.0, -1000.0, -1000.0, -1000.0);
        let state_max = Vector4::new(1000.0, 1000.0, 1000.0, 1000.0);
        
        let mut controller = Self {
            prediction_horizon,
            control_horizon,
            state_weights,
            input_weights,
            terminal_weights,
            input_min,
            input_max,
            state_min,
            state_max,
            prediction_matrix: DMatrix::zeros(0, 0),
            control_matrix: DMatrix::zeros(0, 0),
            current_state: Vector4::zeros(),
            reference_trajectory: VecDeque::new(),
            dt,
        };
        
        controller.build_prediction_matrices();
        controller
    }
    
    fn build_prediction_matrices(&mut self) {
        let n_states = 4;
        let n_inputs = 2;
        let n = self.prediction_horizon;
        
        // Build prediction matrix Phi and control matrix Gamma
        self.prediction_matrix = DMatrix::zeros(n_states * n, n_states);
        self.control_matrix = DMatrix::zeros(n_states * n, n_inputs * self.control_horizon);
        
        let model = StateSpaceModel::new_motion_model(self.dt);
        
        // Build Phi matrix
        let mut a_power = Matrix4::identity();
        for i in 0..n {
            self.prediction_matrix.view_mut((i * n_states, 0), (n_states, n_states))
                .copy_from(&a_power);
            a_power = &model.a_matrix * a_power;
        }
        
        // Build Gamma matrix
        for i in 0..n {
            for j in 0..self.control_horizon.min(i + 1) {
                let row_start = i * n_states;
                let col_start = j * n_inputs;
                
                let mut a_power = Matrix4::identity();
                for _ in 0..(i - j) {
                    a_power = &model.a_matrix * a_power;
                }
                
                let gamma_block = a_power * &model.b_matrix;
                self.control_matrix.view_mut((row_start, col_start), (n_states, n_inputs))
                    .copy_from(&gamma_block);
            }
        }
    }
    
    pub fn update(&mut self, current_state: Vector4<f64>, reference_trajectory: &[Vector4<f64>]) -> Vector2<f64> {
        self.current_state = current_state;
        
        // Update reference trajectory
        self.reference_trajectory.clear();
        for &ref_point in reference_trajectory.iter().take(self.prediction_horizon) {
            self.reference_trajectory.push_back(ref_point);
        }
        
        // Pad with final reference if needed
        while self.reference_trajectory.len() < self.prediction_horizon {
            if let Some(&last_ref) = reference_trajectory.last() {
                self.reference_trajectory.push_back(last_ref);
            } else {
                self.reference_trajectory.push_back(Vector4::zeros());
            }
        }
        
        // Solve MPC optimization problem
        self.solve_mpc_optimization()
    }
    
    fn solve_mpc_optimization(&self) -> Vector2<f64> {
        // This is a simplified QP solver implementation
        // In practice, use a proper QP solver like OSQP or similar
        
        let n_states = 4;
        let n_inputs = 2;
        let n = self.prediction_horizon;
        let nu = self.control_horizon;
        
        // Stack reference trajectory
        let mut ref_stack = DVector::zeros(n_states * n);
        for (i, ref_point) in self.reference_trajectory.iter().enumerate() {
            for j in 0..n_states {
                ref_stack[i * n_states + j] = ref_point[j];
            }
        }
        
        // Predicted states without control: x_pred = Phi * x0
        let x_pred = &self.prediction_matrix * &self.current_state;
        
        // Error without control
        let error_free = &ref_stack - &x_pred;
        
        // Build cost matrices
        let mut q_big = DMatrix::zeros(n_states * n, n_states * n);
        for i in 0..n {
            let weight = if i == n - 1 { &self.terminal_weights } else { &self.state_weights };
            for j in 0..n_states {
                q_big[(i * n_states + j, i * n_states + j)] = weight[j];
            }
        }
        
        let mut r_big = DMatrix::zeros(n_inputs * nu, n_inputs * nu);
        for i in 0..nu {
            for j in 0..n_inputs {
                r_big[(i * n_inputs + j, i * n_inputs + j)] = self.input_weights[j];
            }
        }
        
        // Quadratic programming matrices
        // min 0.5 * u^T * H * u + f^T * u
        let h = self.control_matrix.transpose() * &q_big * &self.control_matrix + r_big;
        let f = -self.control_matrix.transpose() * &q_big * &error_free;
        
        // Solve using simplified method (in practice use proper QP solver)
        let h_inv = h.clone().try_inverse().unwrap_or_else(|| {
            // Regularization if matrix is singular
            (&h + DMatrix::identity(h.nrows(), h.ncols()) * 1e-6).try_inverse().unwrap()
        });
        
        let optimal_control = -h_inv * f;
        
        // Extract first control input
        let u0 = Vector2::new(optimal_control[0], optimal_control[1]);
        
        // Apply constraints
        Vector2::new(
            u0.x.max(self.input_min.x).min(self.input_max.x),
            u0.y.max(self.input_min.y).min(self.input_max.y),
        )
    }
}

impl AdvancedMotionController {
    pub fn new(axis_name: String, dt: f64) -> Self {
        let model = StateSpaceModel::new_motion_model(dt);
        let state_space = StateSpaceController::new(model, dt);
        let mpc = MPCController::new(10, 5, dt); // 10 step prediction, 5 step control horizon
        
        Self {
            state_space,
            mpc,
            control_mode: ControlMode::StateSpace,
            axis_name,
            last_update: Instant::now(),
            performance_metrics: ControlMetrics {
                steady_state_error: 0.0,
                overshoot_percent: 0.0,
                settling_time: Duration::from_millis(0),
                rise_time: Duration::from_millis(0),
                control_effort: 0.0,
                tracking_error_rms: 0.0,
            },
        }
    }
    
    pub fn set_control_mode(&mut self, mode: ControlMode) {
        self.control_mode = mode;
    }
    
    pub fn update_control(&mut self, 
                         position: f64, 
                         velocity: f64, 
                         acceleration: f64,
                         target_position: f64,
                         target_velocity: f64) -> f64 {
        
        let now = Instant::now();
        let dt = now.duration_since(self.last_update).as_secs_f64();
        self.last_update = now;
        
        let current_state = Vector4::new(position, velocity, acceleration, 0.0);
        let reference = Vector4::new(target_position, target_velocity, 0.0, 0.0);
        
        let control_output = match self.control_mode {
            ControlMode::StateSpace => {
                let control_vector = self.state_space.update(current_state, reference);
                control_vector.x // Use force output
            },
            
            ControlMode::MPC => {
                // For MPC, we need a reference trajectory
                let ref_trajectory = vec![reference; 10]; // Simple constant reference
                let control_vector = self.mpc.update(current_state, &ref_trajectory);
                control_vector.x // Use force output
            },
            
            ControlMode::PID => {
                // Fallback to simple PID
                let error = target_position - position;
                let kp = 100.0;
                let kd = 10.0;
                kp * error - kd * velocity
            },
            
            ControlMode::Hybrid => {
                // Switch between controllers based on conditions
                if (target_position - position).abs() > 10.0 {
                    // Use MPC for large moves
                    let ref_trajectory = vec![reference; 10];
                    let control_vector = self.mpc.update(current_state, &ref_trajectory);
                    control_vector.x
                } else {
                    // Use state-space for fine positioning
                    let control_vector = self.state_space.update(current_state, reference);
                    control_vector.x
                }
            }
        };
        
        // Update performance metrics
        self.update_metrics(position, target_position, control_output, dt);
        
        control_output
    }
    
    fn update_metrics(&mut self, position: f64, target: f64, control_effort: f64, dt: f64) {
        let error = target - position;
        self.performance_metrics.steady_state_error = error.abs();
        self.performance_metrics.control_effort += control_effort.abs() * dt;
        
        // Update RMS tracking error (simplified)
        let current_rms = self.performance_metrics.tracking_error_rms;
        self.performance_metrics.tracking_error_rms = 
            (current_rms * 0.99 + error * error * 0.01).sqrt();
    }
    
    pub fn get_metrics(&self) -> &ControlMetrics {
        &self.performance_metrics
    }
    
    pub fn tune_parameters(&mut self, aggressive: bool) {
        // Automatically tune controller parameters based on performance
        match self.control_mode {
            ControlMode::StateSpace => {
                if aggressive {
                    // Increase state weights for faster response
                    self.state_space.control_gain *= 1.2;
                } else {
                    // Decrease for smoother response
                    self.state_space.control_gain *= 0.8;
                }
            },
            ControlMode::MPC => {
                if aggressive {
                    self.mpc.state_weights *= 1.2;
                } else {
                    self.mpc.input_weights *= 1.2;
                }
            },
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_state_space_controller() {
        let dt = 0.001; // 1ms sample time
        let model = StateSpaceModel::new_motion_model(dt);
        let mut controller = StateSpaceController::new(model, dt);
        
        // Test step response
        let reference = Vector4::new(10.0, 0.0, 0.0, 0.0); // 10mm step
        let measurement = Vector4::zeros();
        
        let control_output = controller.update(measurement, reference);
        assert!(control_output.x.abs() > 0.0); // Should produce non-zero control
    }
    
    #[test]
    fn test_mpc_controller() {
        let dt = 0.001;
        let mut controller = MPCController::new(10, 5, dt);
        
        let current_state = Vector4::zeros();
        let reference = Vector4::new(10.0, 0.0, 0.0, 0.0);
        let ref_trajectory = vec![reference; 10];
        
        let control_output = controller.update(current_state, &ref_trajectory);
        assert!(control_output.x.abs() > 0.0); // Should produce non-zero control
    }
}