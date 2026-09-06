package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveDistanceCommand extends InstantCommand {
    public DriveDistanceCommand(Rotation2d drive, Rotation2d angle, ModuleAlon subsystem){
        super(()->{

        });
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        super.execute();
    }
}
