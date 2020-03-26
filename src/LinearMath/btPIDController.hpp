
#ifndef BT_PID_Controller_H
#define BT_PID_Controller_H

class btPIDController
{
private:
	Inputs m_inputs;
	Outputs m_outputs;

	btPIDController(Inputs _inputs) 
		: m_inputs(_inputs), m_outputs(_outputs){}

public:
	struct Inputs
	{
		float kp;
		float ki;
		float kd;
		float current_val;
		float target_val;
		float previous_error;
		float previous_integrator_val;
		float integral_term_min;
		float integral_term_max;
		float dt;

		Inputs(float _kp, float _ki, float _kd,
			float _current_val, float _target_val,
			float _previous_error, float _previous_integrator_val,
			float _integral_term_min, float _integral_term_max,
			float _dt ) 
			: kp(_kp), ki(_ki), kd(_kd),
			  current_val(_current_val), target_val(_target_val),
			  previous_error(_previous_error), previous_integrator_val(_previous_integrator_val),
			  integral_term_min(_integral_term_min), integral_term_max(_integral_term_max),
			  dt(_dt) {}
			
	};

	struct Outputs
	{
		float output;
		float current_integrator_val;
		float current_error;

		Outputs(float output, 
			float current_integrator_val, 
			float current_error ) 
			: output(_output), 
			  current_integrator_val(_current_integrator_val), 
			  current_error(_current_error) {}
	};


	inline static Outputs compute(Inputs ins)
	{
		Outputs outs;

		// Declare variables.
		float error = ins.target_val - ins.current_val;
		float integral_term = ins.previous_integrator_val + error * ins.dt;

		// Apply integrator limits.
		if (integral_term < ins.integral_term_min)
		{
			integral_term = ins.integral_term_min;
		}
		if (integral_term > ins.integral_term_max)
		{
			integral_term = ins.integral_term_max;
		}

		// Compute the output.
		outs.output = ins.kp * error + ins.ki * integral_term + ins.kd * (error - ins.previous_error) / ins.dt;
		outs.current_integrator_val = integral_term;
		outs.current_error = error;

		return outs;
	}

	inline float compute(float _dt)
	{
		m_inputs.dt = _dt;

		m_outputs = compute(m_inputs);

		m_inputs.previous_error = m_outputs.current_error;
		m_inputs.previous_integrator_val = m_outputs.current_integrator_val;

		return m_outputs.output;
	}

	inline void setPGain(float _kp) { m_inputs.kp = _kp; }
	inline void setIGain(float _ki) { m_inputs.ki = _ki; }
	inline void setDGain(float _kd) { m_inputs.kd = _kd; }

	inline void setITermMin(float _integral_term_min) { m_inputs.integral_term_min = _integral_term_min; }
	inline void setITermMax(float _integral_term_max) { m_inputs.integral_term_max = _integral_term_max; }

};

#endif