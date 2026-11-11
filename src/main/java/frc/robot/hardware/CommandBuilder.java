package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SwerveModduleTomer;

public class CommandBuilder
{
    SwerveModduleTomer module = new SwerveModduleTomer(10,11,1);
    public CommandBuilder() {

    }
    public SequentialCommandGroup getFullCircleDrive(){
        return module.fullCircleDrive();
    }
    public SequentialCommandGroup getManualDriveCommand(Rotation2d angle,Rotation2d spins){
        return module.DriveDistanceCommand(angle,spins);
    }
    public Command getPowerDrive(double power){
        return module.manualDriveCommand(power);
    }
    public Command getRotationspid(Rotation2d rotation){
        return module.rotations_pid(rotation);
    }
    public Command getAnglePid(Rotation2d angle){
        return module.steerToAngleCommand(angle);
    }
    public Command getStopModule(){
        return module.stopmodula();
    }

}
