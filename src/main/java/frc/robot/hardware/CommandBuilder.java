package frc.robot.hardware;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SwerveModduleTomer;

public class CommandBuilder {

	SwerveModduleTomer module;

	public CommandBuilder(SwerveModduleTomer modula) {
		module = modula;
	}

	public Command getFullCircleDrive() {
		return module.asSubsystemCommand( new SequentialCommandGroup(

				new InstantCommand(module::fullCircleDrive,module).until(()->module.checkdegrees(Rotation2d.fromDegrees(90),module.getSteer())),
				new InstantCommand(module::fullCircleDrive,module).until(()->module.checkdegrees(Rotation2d.fromDegrees(180),module.getSteer())),
				new InstantCommand(module::fullCircleDrive,module).until(()->module.checkdegrees(Rotation2d.fromDegrees(270),module.getSteer())),
				new InstantCommand(module::fullCircleDrive,module).until(()->module.checkdegrees(Rotation2d.fromDegrees(360),module.getSteer()))),"FulleCircleDrive");
	}

	public Command getdriveDistance(Rotation2d angle, Rotation2d spins) {
		return module.asSubsystemCommand(new InstantCommand(()->module.DriveDistanceCommand(angle,spins),module),"DriveAtCommand");
	}

	public Command getPowerDrive(double power) {
		return module.asSubsystemCommand(new RunCommand(()->module.movepower(power),module),"PowerDrive");
	}

	public Command getRotationspid(Rotation2d rotation) {
		return module.asSubsystemCommand(new InstantCommand(()->module.rotations_pid(rotation),module),"Rotations pid");
	}

	public Command getSteerToAngleCommand(Rotation2d angle) {
		return module.asSubsystemCommand( new InstantCommand(()->module.steerToAngle(angle),module),"steerToAngleCommand");
	}

	public Command getStopModule() {
		module.setDefaultCommand(new InstantCommand(module::stopmodula , module));
		return module.asSubsystemCommand(new InstantCommand(module::stopmodula,module),"stopModula");

	}
	public Command getPowerdrivecommand(double power){
		return module.asSubsystemCommand(new RunCommand(()->module.powerDriveCommand(power),module),"drivePower");
	}
	public Command getModeSwitcher(NeutralModeValue value){
		return module.asSubsystemCommand(new InstantCommand(()->module.mode_switcher(value),module),"ModeSwitch");
	}
	public Command getTest(){
		return module.asSubsystemCommand(new InstantCommand(module::testing1,module),"testing");
	}
	public Command getPosToZero(){
		return module.asSubsystemCommand(new InstantCommand(module::posToZero,module),"posToZero");
	}

}
