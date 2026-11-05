package frc.robot;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.Tome_motor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Modulesubsystem extends GBSubsystem {

	private final PositionVoltage pid_thing = new PositionVoltage(1);
	public String Path1;
	public String Path2;
	Tome_motor steer;
	Tome_motor drive;
	public int check = 0;
	private final SwerveModduleTomer modula = new SwerveModduleTomer(10, 11, 1);

	Modulesubsystem() {
		Path1 = ("drive1");
		Path2 = ("steer1");
		Slot0Configs pidcofing = new Slot0Configs().withKP(0.5);
		drive = SwerveModduleTomer.getdrive();
		steer = SwerveModduleTomer.getSteer();
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

	public Command DriveDistanceCommand(Rotation2d degrees, Rotation2d rotations) {
		SequentialCommandGroup commandGroup = new SequentialCommandGroup();
		commandGroup.addRequirements(modula);
		commandGroup.addCommands(
				new InstantCommand(modula::stopmodula),
				steerToAngleCommand(degrees, steer),
				rotations_pid(rotations, drive),
				new InstantCommand(modula::stopmodula)
		);
		return commandGroup;
	}

	public Command brake(boolean brake) {
		if (brake)
			return new InstantCommand(() -> modula.mode_switcher(NeutralModeValue.Brake));
		else {
			return new InstantCommand(() -> modula.mode_switcher(NeutralModeValue.Coast));
		}
	}

	public Command manualDriveCommand(Double power) {
		Command runCommand = new RunCommand(() -> modula.movepower(power));
		runCommand.addRequirements(modula);
		return runCommand;
	}

	public Command steerToAngleCommand(Rotation2d degrees, Tome_motor motor) {
		check++;
		return new RunCommand(() -> modula.pid_Degrees_Steer(degrees), modula).until(() -> modula.check(degrees, motor));
	}

	public Command rotations_pid(Rotation2d rotations, Tome_motor motor) {
		check++;
		return new RunCommand(() -> modula.pid_drive(rotations), modula).until(() -> modula.checkrotations(rotations, motor));
	}

	public Command longnamecommand() {
		SequentialCommandGroup command = new SequentialCommandGroup(
				steerToAngleCommand(Rotation2d.fromDegrees(0), steer),
				rotations_pid(Rotation2d.fromRotations(2.0), drive),
				steerToAngleCommand(Rotation2d.fromDegrees(90), steer),
				rotations_pid(Rotation2d.fromRotations(2.0), drive),
				steerToAngleCommand(Rotation2d.fromDegrees(180), steer),
				rotations_pid(Rotation2d.fromRotations(2.0), drive),
				steerToAngleCommand(Rotation2d.fromDegrees(270), steer),
				rotations_pid(Rotation2d.fromRotations(2.0), drive),
				new InstantCommand(modula::stopmodula));
		command.addRequirements(modula);
		return command;
	}
}