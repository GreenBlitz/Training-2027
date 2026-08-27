package frc.robot.hardware;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Frequency;
import org.littletonrobotics.junction.Logger;


public class Tome_motor {

	private final TalonFX motor;
	private final SoftwareLimitSwitchConfigs spin_limit = new SoftwareLimitSwitchConfigs();
	private final CurrentLimitsConfigs current_limit = new CurrentLimitsConfigs();
	private InvertedValue Clockwise_Positive = InvertedValue.valueOf(1);
	private InvertedValue Counter_Clockwise_Positive = null;
	private PositionVoltage check = new PositionVoltage(1);
	public PIDController pidController = new PIDController(1,1,1);
	public Tome_motor(int id) {
		this.motor = new TalonFX(id);
		motor_limit();
		setCurrent_limit();
	}


	public void motor_limit() {
		spin_limit.ForwardSoftLimitEnable = true;
		spin_limit.ForwardSoftLimitThreshold = 5.0;
		spin_limit.ReverseSoftLimitEnable = true;
		spin_limit.ReverseSoftLimitThreshold = -3.0;

		motor.getConfigurator().apply(spin_limit);
	}

	public void setCurrent_limit() {
		current_limit.StatorCurrentLimitEnable = true;
		current_limit.withStatorCurrentLimit(40);
		motor.getConfigurator().apply(current_limit);
	}






	public void move_half() {
		motor.set(0.5);
	}

	public void move_tenth() {
		motor.set(-0.1);
	}

	public void stop() {
		motor.stopMotor();
	}

	public String path = "/users/downloads/";

	public double getacl() {

		return  motor.getAcceleration().getValueAsDouble();

	}

	public double get_vel() {
		return  motor.getVelocity().getValueAsDouble();

	}

	public double get_pos() {
		return  motor.getPosition().getValueAsDouble();


	}

	public double get_vol() {
		return  motor.getMotorVoltage().getValueAsDouble();

	}

	public double get_cur() {
		return  motor.getStatorCurrent().getValueAsDouble();
	}
	public void getthem(){
		StatusSignal current =motor.getStatorCurrent();
		StatusSignal vel =motor.getVelocity();
		StatusSignal voltage =motor.getMotorVoltage();
		StatusSignal position =motor.getPosition();
		check.withUpdateFreqHz((Frequency) position).withUpdateFreqHz(50);
		check.withUpdateFreqHz((Frequency) voltage).withUpdateFreqHz(50);
		check.withUpdateFreqHz((Frequency) vel).withUpdateFreqHz(50);
		check.withUpdateFreqHz((Frequency) current).withUpdateFreqHz(50);


	}





	public void  logger() {
		Logger.recordOutput(path + "/current", get_cur());
		Logger.recordOutput(path + "/VOL", get_vol());
		Logger.recordOutput(path + "/VEL", get_vel());
		Logger.recordOutput(path + "/POSITION", get_pos());
		Logger.recordOutput(path + "/ACL", getacl());
		connected();
	}


	public void SwitchDierction() {
		if (Clockwise_Positive == null) {
			Counter_Clockwise_Positive = null;
			Clockwise_Positive = InvertedValue.Clockwise_Positive;
			MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
			motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
			motor.getConfigurator().apply(motorConfigs);
		} else if (Counter_Clockwise_Positive == null) {
			Clockwise_Positive = null;
			Counter_Clockwise_Positive = InvertedValue.CounterClockwise_Positive;
			MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
			motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
			motor.getConfigurator().apply(motorConfigs);
		}
	}

	public void connected() {
		Logger.recordOutput(path + "/current", motor.isConnected());
	}


	public void mode_switcher(NeutralModeValue mode) {
		MotorOutputConfigs motorConfig = new MotorOutputConfigs();
		motorConfig.NeutralMode = mode;
		motor.getConfigurator().apply(motorConfig);
	}

	public void set_pos(double pos) {
		motor.setPosition(pos);
	}


	public void pid_misson(double target){
		double current = get_pos();
		double Diffrence = target-current;
		if (Diffrence>0) {
			motor.setVoltage(11-11/(Diffrence+1));
			check.withPosition(Diffrence);
		}
		if(Diffrence<0){
			motor.setVoltage(11+11/(Diffrence-1));
			check.withPosition(Diffrence);

		}}
	public void pid_misson2(double target) {
			pidController.setPID(target,1,1);
			check.withPosition(target);


	}










		}











