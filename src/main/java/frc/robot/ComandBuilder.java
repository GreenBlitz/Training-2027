package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ComandBuilder {

	Modulesubsystem subsystem;

	ComandBuilder(Modulesubsystem modulesubsystem) {
		subsystem = modulesubsystem;
	}

	public Command getLongNameCommnad() {
		return subsystem.longnamecommand();
	}

	public Command getManualDriveCommand(double power) {
		return subsystem.manualDriveCommand(power);
	}

	public Command getDriveDistanceCommand(Rotation2d degrees, Rotation2d rotations) {
		return subsystem.DriveDistanceCommand(degrees, rotations);
	}

}
