package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

import static com.ctre.phoenix6.BaseStatusSignal.getLatencyCompensatedValueAsDouble;

public class TomerMotor {

	private final TalonFX motor;
	private InvertedValue clockwisePositive = InvertedValue.Clockwise_Positive;
	private InvertedValue counterClockwise_Positive = null;
	private PositionVoltage pid_thing = new PositionVoltage(0);
	private TalonFXConfiguration config = new TalonFXConfiguration();
	public Double KP = 1.5;
	public String path = "motor1";


	public TomerMotor(int id, TalonFXConfiguration config,CANBus canBus) {
		this.motor = new TalonFX(id, canBus.roboRIO());
		motor.getConfigurator().apply(config);
		motor.optimizeBusUtilization(RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
	}

	public void motorMoveHalfSpeed() {
		motor.set(0.5);
	}

	public void move_tenth() {
		motor.set(-0.1);
	}

	public void stop() {
		motor.stopMotor();
	}

	public double getAcelertion() {
		StatusSignal acl = motor.getAcceleration();
		return acl.getValueAsDouble();
	}

	public Rotation2d get_velocity() {
		StatusSignal vel = motor.getVelocity();
		double double_vel = (vel.getValueAsDouble() * 360);
		return Rotation2d.fromDegrees(double_vel);
	}

	public Rotation2d getPosition() {
		double rotations = getLatencyCompensatedValueAsDouble(motor.getPosition(), motor.getVelocity());
		return Rotation2d.fromRotations(rotations);
	}

	public double getVoltage() {
		StatusSignal vol = motor.getMotorVoltage();
		return vol.getValueAsDouble();
	}

	public double getCurrent() {
		StatusSignal current = motor.getStatorCurrent();
		return current.getValueAsDouble();
	}

	public void logger() {
		Logger.recordOutput(path + "/current", getCurrent());
		Logger.recordOutput(path + "/VOL", getVoltage());
		Logger.recordOutput(path + "/VEL", get_velocity());
		Logger.recordOutput(path + "/POSITION", getPosition());
		Logger.recordOutput(path + "/ACL", getAcelertion());
		connected();
	}

	public void SwitchDierction() {
		if (clockwisePositive == null) {
			counterClockwise_Positive = null;
			clockwisePositive = InvertedValue.Clockwise_Positive;
			config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
			motor.getConfigurator().apply(config);
		} else if (counterClockwise_Positive == null) {
			clockwisePositive = null;
			counterClockwise_Positive = InvertedValue.CounterClockwise_Positive;
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

	public void MoveRotations(Rotation2d rotations){
		double curent = getPosition().getRotations();
		motor.setControl(pid_thing.withPosition(curent+rotations.getRotations()));


	}
	public void movedegrees(Rotation2d degrees){
		double curent = getPosition().getDegrees();
		motor.setControl(pid_thing.withPosition(curent+degrees.getRotations()));


	}


}
