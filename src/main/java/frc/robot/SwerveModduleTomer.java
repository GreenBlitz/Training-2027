package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.hardware.Tome_motor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;


public class SwerveModduleTomer extends GBSubsystem {

	public  Tome_motor drive;
	public  Tome_motor steer;
	TalonFXConfiguration config = new TalonFXConfiguration();
	public static final Rotation2d STEER_POSITION_TOLERANCE = Rotation2d.fromDegrees(2);
	public String Path1;
	public String Path2;
	public SwerveModduleTomer(int id1, int id2, double value) {
		super();
		drive = new Tome_motor(id1);
		steer = new Tome_motor(id2);
		Path1 = ("drive1");
		Path2 = ("steer1");
		motor_limit();
		setCurrent_limit();
		config.Feedback.SensorToMechanismRatio = value;
	}

	public void SwitchDierctionswerve(Tome_motor motor) {
		motor.SwitchDierction();

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

	public boolean checkVelocity(Rotation2d vel, Tome_motor motor) {
		return MathUtil.isNear(vel.getDegrees(), motor.get_vel().getDegrees(), STEER_POSITION_TOLERANCE.getDegrees());
	}

	public boolean checkPosition(Tome_motor motor, Rotation2d pos) {
		return MathUtil.isNear(pos.getDegrees(), motor.get_pos().getDegrees(), STEER_POSITION_TOLERANCE.getDegrees());
	}



	public void setvoltage(double vol) {
		steer.setvoltage(vol);
	}

	public void mode_switcher(NeutralModeValue mode) {
		steer.mode_switcher(mode);
	}

	public SequentialCommandGroup stopmodula() {
		return new SequentialCommandGroup(
		new InstantCommand(drive::stop,this),
		new InstantCommand(steer::stop,this));

	}

	public void move() {
		drive.move_half();
	}

	public boolean check(Rotation2d degrees, Tome_motor motor) {
		return MathUtil.isNear(degrees.getDegrees(), motor.get_pos().getDegrees(), STEER_POSITION_TOLERANCE.getDegrees());
	}

	public boolean check2(double rotations, Tome_motor motor) {
		return MathUtil.isNear(rotations, motor.get_pos().getRotations(), STEER_POSITION_TOLERANCE.getRotations());
	}

	public void manualDriving(double x, double y) {
		double rads = Math.atan2(x, y);
		RunCommand command2 = new RunCommand(this::move);
		command2.until(() -> check2(rads, steer));
		double power = Math.sqrt(Math.abs((x * x) + (y * y)));
		new InstantCommand(() -> setvoltage(11 / power));
	}

	public void stop() {
		InstantCommand command1;
		command1 = new InstantCommand(this::stopmodula);
		setDefaultCommand(command1);
	}
	public void movepower(double power) {
		drive.move(power);
	}

		@Override
		public void subsystemPeriodic() {
			Logger.recordOutput(Path1 + "/velocity", drive.get_vel());
			Logger.recordOutput(Path1 + "/driveVoltage", drive.get_vol());
			Logger.recordOutput(Path1 + "/position", drive.get_pos());
			Logger.recordOutput(Path1 + "/isconnected", drive.isconnected());
			Logger.recordOutput(Path2 + "/velocity", steer.get_vel());
			Logger.recordOutput(Path2 + "/driveVoltage", steer.get_vol());
			Logger.recordOutput(Path2 + "/position", steer.get_pos());
			Logger.recordOutput(Path2 + "/isconnected", steer.isconnected());
		}

		public SequentialCommandGroup DriveDistanceCommand(Rotation2d degrees, Rotation2d rotations) {
			SequentialCommandGroup commandGroup = new SequentialCommandGroup();
			commandGroup.addRequirements(this);
			commandGroup.addCommands(
					new InstantCommand(this::stopmodula),
					steerToAngleCommand(degrees),
					rotations_pid(rotations),
					new InstantCommand(this::stopmodula)
			);
			return commandGroup;
		}

		public Command brake(boolean brake) {
			if (brake)
				return new InstantCommand(() -> this.mode_switcher(NeutralModeValue.Brake));
			else {
				return new InstantCommand(() -> this.mode_switcher(NeutralModeValue.Coast));
			}
		}

		public Command manualDriveCommand(Double power) {
			Command runCommand = new RunCommand(() -> this.movepower(power));
			runCommand.addRequirements(this);
			return runCommand;
		}

		public Command steerToAngleCommand(Rotation2d degrees) {
			return this.asSubsystemCommand(new InstantCommand(() -> steer.moveToPosition(degrees), this),"steerToAngle" + degrees.getDegrees());
		}

		public Command rotations_pid(Rotation2d rotations) {
			return this.asSubsystemCommand(new InstantCommand(() -> drive.moveToPosition(rotations), this),"steer rotations" + rotations.getRotations());
		}

		public SequentialCommandGroup fullCircleDrive() {
			SequentialCommandGroup sequentialCommandGroup = new SequentialCommandGroup(
					steerToAngleCommand(Rotation2d.fromDegrees(0)),
					rotations_pid(Rotation2d.fromRotations(2.0)),
					steerToAngleCommand(Rotation2d.fromDegrees(90)),
					rotations_pid(Rotation2d.fromRotations(2.0)),
					steerToAngleCommand(Rotation2d.fromDegrees(180)),
					rotations_pid(Rotation2d.fromRotations(2.0)),
					steerToAngleCommand(Rotation2d.fromDegrees(270)),
					rotations_pid(Rotation2d.fromRotations(2.0)),
					new InstantCommand(this::stopmodula));
					sequentialCommandGroup.addRequirements(this);
					return sequentialCommandGroup;
		}

		@Override
		public String getLogPath(){
			return "ModulacurrentCommand";
		}
		public Tome_motor getDrive(){
		return drive;
		}
		public Tome_motor getSteer(){
		return steer;
		}
	}
