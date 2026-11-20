package frc.robot.hardware;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

import static com.ctre.phoenix6.BaseStatusSignal.getLatencyCompensatedValueAsDouble;

public class Tome_motor {

	private final TalonFX motor;
	private InvertedValue Clockwise_Positive = InvertedValue.Clockwise_Positive;
	private InvertedValue Counter_Clockwise_Positive = null;
	private PositionVoltage pid_thing = new PositionVoltage(0);
	TalonFXConfiguration config = new TalonFXConfiguration();
	public Double KP = 1.5;

	public Tome_motor(int id,TalonFXConfiguration config) {
		this.motor = new TalonFX(id, CANBus.roboRIO());
		motor.getConfigurator().apply(config);
		motor.optimizeBusUtilization(RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);


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

	public String path = "motor1";

	public double getacl() {
		StatusSignal acl = motor.getAcceleration();
		return acl.getValueAsDouble();
	}

	public Rotation2d get_vel() {
		StatusSignal vel = motor.getVelocity();
		double double_vel = (vel.getValueAsDouble() * 360);
		return Rotation2d.fromDegrees(double_vel);
	}

	public Rotation2d get_pos() {
		double rotations = getLatencyCompensatedValueAsDouble(motor.getPosition(), motor.getVelocity());
		return Rotation2d.fromRotations(rotations);
	}

	public double get_vol() {
		StatusSignal vol = motor.getMotorVoltage();
		return vol.getValueAsDouble();
	}

	public double get_cur() {
		StatusSignal current = motor.getStatorCurrent();
		return current.getValueAsDouble();
	}

	public void logger() {
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
			config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
			motor.getConfigurator().apply(config);
		} else if (Counter_Clockwise_Positive == null) {
			Clockwise_Positive = null;
			Counter_Clockwise_Positive = InvertedValue.CounterClockwise_Positive;
			config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
			motor.getConfigurator().apply(config);
		}
	}

	public void connected() {
		Logger.recordOutput(path + "/connected", motor.isConnected());
	}

	public void mode_switcher(NeutralModeValue mode) {
		config.MotorOutput.NeutralMode = mode;
		motor.getConfigurator().apply(config);
	}

	public void set_pos(double pos) {
		motor.setPosition(pos);
	}

	public void pid_misson2(double target) {
		motor.setControl(pid_thing.withPosition(target));

	}

	public void setvoltage(double vol) {
		motor.setVoltage(vol);
	}

	public void move(double power) {
		motor.set(power);
	}

	public TalonFX getMotor1() {
		return motor;
	}

	public Double isconnected() {
		if (motor.isConnected()) {
			return (1.0);
		} else {
			return (0.0);
		}
	}

	public void moveToPosition(Rotation2d rotation) {
		motor.setControl(pid_thing.withPosition(rotation.getRotations()));
	}

	public void moveRotations(Rotation2d rotations){
		double curent = get_pos().getRotations();
		motor.setControl(pid_thing.withPosition(curent+rotations.getRotations()));


	}

}
