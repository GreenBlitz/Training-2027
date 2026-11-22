package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.hardware.CommandBuilder;
import frc.robot.hardware.Tome_motor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;


public class SwerveModduleTomer extends GBSubsystem {
	public Tome_motor drive;
	public Tome_motor steer;
	TalonFXConfiguration driveConfig = new TalonFXConfiguration();
	TalonFXConfiguration steerConfig = new TalonFXConfiguration();

	public static final Rotation2d STEER_POSITION_TOLERANCE = Rotation2d.fromDegrees(2);
	public String Path1;
	public String Path2;

	public SwerveModduleTomer(int id1, int id2, double steergearRation,double drivegearRatio) {
		super();
		steerConfig.Slot0.kP = 11;
		steerConfig.Slot0.kD = 0.22;
		steerConfig.Slot0.kS = 0.28;
		steerConfig.Feedback.SensorToMechanismRatio = steergearRation;
		driveConfig.Slot0.kP = 11;
		driveConfig.Slot0.kD = 0.22;
		driveConfig.Slot0.kS = 0.28;
		driveConfig.Feedback.SensorToMechanismRatio = drivegearRatio;
		drive = new Tome_motor(id1,driveConfig);
		steer = new Tome_motor(id2,steerConfig);
		Path1 = ("drive1");
		Path2 = ("steer1");
	}

	CommandBuilder commandBuilder = new CommandBuilder(this);

	public void SwitchDierctionswerve() {
		steer.SwitchDierction();
		drive.SwitchDierction();
	}

	public void setvoltage(double vol) {
		steer.setvoltage(vol);
		drive.setvoltage(vol);
	}

	public void mode_switcher(NeutralModeValue mode) {
		steer.mode_switcher(mode);
		drive.mode_switcher(mode);
	}

	public void stopmodula() {
		steer.stop();
		drive.stop();
	}

	public void move() {
		drive.move_half();
	}

	public boolean check2(double rotations, Tome_motor motor) {
		return MathUtil.isNear(rotations, motor.get_pos().getRotations(), STEER_POSITION_TOLERANCE.getRotations());
	}

	public void manualDriving(double x, double y) {
		double rads = Math.atan2(x, y);
		RunCommand command2 = new RunCommand(this::move, this);
		command2.until(() -> check2(rads, steer));
		double power = Math.sqrt(Math.abs((x * x) + (y * y)));
		new InstantCommand(() -> setvoltage(11 / power), this);
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

	public void DriveDistanceCommand(Rotation2d degrees, Rotation2d rotations) {
			steerToAngle(degrees);
			drive.MoveRotations(rotations);
	}

	public void brake(NeutralModeValue brake) {
		steer.mode_switcher(brake);
		drive.mode_switcher(brake);
	}

	public void powerDriveCommand(Double power) {
		movepower(power);
	}

	public void steerToAngle(Rotation2d degrees) {
		steer.moveToPosition(degrees);
	}

	public void rotations_pid(Rotation2d rotations) {
		drive.moveToPosition(rotations);
	}
	public void testing1(){
		drive.MoveRotations(Rotation2d.fromRotations(2.0));
	}

	public void fullCircleDrive() {
		steer.movedegrees(Rotation2d.fromDegrees(90));
		drive.MoveRotations(Rotation2d.fromRotations(2.0));
		steer.movedegrees(Rotation2d.fromDegrees(90));
		drive.MoveRotations(Rotation2d.fromRotations(2.0));
		steer.movedegrees(Rotation2d.fromDegrees(90));
		drive.MoveRotations(Rotation2d.fromRotations(2.0));
		steer.movedegrees(Rotation2d.fromDegrees(90));
		drive.MoveRotations(Rotation2d.fromRotations(2.0));
	}

	@Override
	public String getLogPath() {
		return "ModulacurrentCommand";
	}

	public Tome_motor getDrive() {
		return drive;
	}

	public Tome_motor getSteer() {
		return steer;
	}

	public CommandBuilder getCommandBuilder() {
		return commandBuilder;
	}


}
