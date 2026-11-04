package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.hardware.Tome_motor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import static com.ctre.phoenix6.BaseStatusSignal.getLatencyCompensatedValueAsDouble;


public class SwerveModduleTomer extends GBSubsystem {

	public static Tome_motor drive;
	public static Tome_motor steer;
	private InvertedValue Clockwise_Positive = InvertedValue.valueOf(1);
	private InvertedValue Counter_Clockwise_Positive = null;
	private PositionVoltage pid_thing = new PositionVoltage(1);
	TalonFXConfiguration config = new TalonFXConfiguration();
	public Double targetvoltage = 11.0;
	public Double target1 = 0.0;
	CANcoderConfiguration canfig = new CANcoderConfiguration();
	public static final Rotation2d STEER_POSITION_TOLERANCE = Rotation2d.fromDegrees(2);
	public SwerveModduleTomer(int id1, int id2, double value) {
		super();
		drive = new Tome_motor(id1);
		steer = new Tome_motor(id2);
		motor_limit();
		setCurrent_limit();
		config.Feedback.SensorToMechanismRatio = value;
	}

	public void SwitchDierctionswerve() {
		if (Clockwise_Positive == null) {
			Counter_Clockwise_Positive = null;
			Clockwise_Positive = InvertedValue.Clockwise_Positive;
			config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
			steer.SwitchDierction();
		} else if (Counter_Clockwise_Positive == null) {
			Clockwise_Positive = null;
			Counter_Clockwise_Positive = InvertedValue.CounterClockwise_Positive;
			config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
			steer.SwitchDierction();
		}
	}

	public void motor_limit() {
		config.HardwareLimitSwitch.ForwardLimitEnable = true;
		config.HardwareLimitSwitch.ReverseLimitEnable = true;
		config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 5.0;
		config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = -3.0;
	}

	public void setCurrent_limit() {
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 40.0;
	}

	public boolean ceheckVelocity(Rotation2d vel) {
		if (drive.get_vel() == vel) {
			return true;
		} else {
			return false;
		}
	}


	public boolean checkPosition(Tome_motor motor ,Rotation2d pos) {
		if (motor.get_pos() == pos) {
			return true;
		} else {
			return false;
		}
	}

	public String path = "motor2";

	public void LogVarubles() {
		Logger.recordOutput(path + "/velocity",steer.get_vel());
		Logger.recordOutput(path + "/driveVoltage", drive.get_vol());
		Logger.recordOutput(path + "/position", steer.get_pos());
		Logger.recordOutput(path + "/targerpos", target1);
		Logger.recordOutput(path + "/targetvol", targetvoltage);
		drive.logger();
	}

	public void pid_misson2(double target) {
		target1 = target;
		config.withSlot0(new Slot0Configs().withKP(1.5));
		pid_thing.withPosition(target);
	}

	public void setvoltage(double vol) {
		steer.setvoltage(vol);
	}

	public void mode_switcher(NeutralModeValue mode) {
		steer.mode_switcher(mode);

	}

	public void stopmodula() {
		steer.stop();
		drive.stop();
	}

	public void move() {
		drive.move_half();
	}

	public static void steer() {
		steer.move_half();
	}

	public boolean check(Rotation2d deegres,Tome_motor motor) {
		return MathUtil.isNear(deegres.getDegrees(), motor.get_pos().getDegrees(), STEER_POSITION_TOLERANCE.getDegrees());
	}
	public boolean check2(double rotations,Tome_motor motor) {
		return MathUtil.isNear(rotations, motor.get_pos().getRotations(), STEER_POSITION_TOLERANCE.getRotations());
	}

	public void larp(double x, double y) {
		double rads = Math.atan2(x, y);
		RunCommand command2 = new RunCommand(this::move);
		command2.until(() -> check2(rads,steer));
		double power = Math.sqrt(Math.abs((x * x) + (y * y)));
		new InstantCommand(() -> setvoltage(11 / power));
	}

	public void set2nmodes(NeutralModeValue mode) {
		new InstantCommand(() -> mode_switcher(mode));
		new InstantCommand(() -> drive.mode_switcher(mode));
	}

	public void stop() {
		InstantCommand command1;
		command1 = new InstantCommand(this::stopmodula);
		setDefaultCommand(command1);
	}

	public void movepower(double power) {
		drive.move(power);
	}

	public boolean checkrotations(Rotation2d rotation,Tome_motor motor) {
		return motor.get_pos() == rotation;
	}

	public void pid_steer(Rotation2d rotation) {
		steer.pid_rotations(rotation);
	}

	public void pid_Degrees_Steer(Rotation2d degrees) {
		steer.pid_angle(degrees);
	}

	public void pid_drive(Rotation2d rotation) {
		drive.pid_rotations(rotation);

	}

	public void pid_degrees_drive(Rotation2d angle) {
		drive.pid_angle(angle);
	}

	public static Tome_motor getdrive() {
		return drive;
	}

	public static TalonFX getdrive2() {
		return drive.getMotor1();
	}

	public static Tome_motor getSteer() {
		return steer;
	}


}
