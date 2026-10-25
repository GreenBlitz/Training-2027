package frc.robot;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.Tome_motor;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.utilcommands.LoggedDashboardCommand;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;


public class Modulesubsystem extends GBSubsystem {


    private final PositionVoltage pid_thing = new PositionVoltage(1);
    public String Path1;
    public String Path2;
    TalonFX steer;
    Tome_motor drive;
    public int check = 0;
    private final SwerveModduleTomer modula = new SwerveModduleTomer(10,11,1);
    Modulesubsystem(){

    Path1 = ("drive1");
    Path2 = ("steer1");
    Slot0Configs pidcofing = new Slot0Configs().withKP(0.5);
    steer = SwerveModduleTomer.getSteer();
    drive = SwerveModduleTomer.getdrive();
    steer.getConfigurator().apply(pidcofing);


}
    @Override
    public void subsystemPeriodic() {
        Logger.recordOutput(Path1 + "/velocity", Rotation2d.fromDegrees(drive.get_vel().getRotations()));
        Logger.recordOutput(Path1 + "/driveVoltage", drive.get_vol());
        Logger.recordOutput(Path1 + "/position", Rotation2d.fromDegrees(drive.get_pos().getRotations()));
        Logger.recordOutput(Path1 + "/isconnected",drive.isconnected1());
        Logger.recordOutput(Path2 + "/velocity", Rotation2d.fromDegrees(steer.getVelocity().getValueAsDouble()));
        Logger.recordOutput(Path2 + "/driveVoltage", steer.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(Path2 + "/position", Rotation2d.fromDegrees(steer.getVelocity().getValueAsDouble()));
        Logger.recordOutput(Path2 + "/isconnected",isconnected1(steer));
        Logger.recordOutput(Path2 + "test2",check);

}
public Double isconnected1(TalonFX motor){
    if(motor.isConnected()){
        return (1.0);
    }else {
        return (0.0);
    }
}
public double getacl(TalonFX motor) {

    StatusSignal acl = motor.getAcceleration();
    acl.setUpdateFrequency(50);
    StatusSignal.refreshAll();
    return acl.getValueAsDouble();

}

public Rotation2d get_vel(TalonFX motor) {
    StatusSignal vel = motor.getVelocity();
    vel.setUpdateFrequency(50);
    StatusSignal.refreshAll();
    double double_vel = (vel.getValueAsDouble() * 360);
    return Rotation2d.fromDegrees(double_vel);


}

public Rotation2d get_pos(TalonFX motor) {
    StatusSignal late = (StatusSignal) BaseStatusSignal.getLatencyCompensatedValue(motor.getPosition(), motor.getVelocity());
    late.setUpdateFrequency(50);
    Rotation2d rotatoin = Rotation2d.fromDegrees(late.getValueAsDouble());
    return rotatoin;

}


public double get_vol(TalonFX motor) {
    StatusSignal vol = motor.getMotorVoltage();
    vol.setUpdateFrequency(50);
    StatusSignal.refreshAll();
    return vol.getValueAsDouble();


}

public double get_cur(TalonFX motor) {

    StatusSignal current = motor.getStatorCurrent();
    current.setUpdateFrequency(50);
    StatusSignal.refreshAll();
    return current.getValueAsDouble();

}
public void setvoltage(TalonFX motor,Double volts){
    motor.setVoltage(volts);

}



public Command DriveDistanceCommand(Rotation2d degrees, Rotation2d rotations){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addRequirements(modula);
     commandGroup.addCommands(
    new InstantCommand(modula::stopmodula),
    steerToAngleCommand(degrees.getDegrees()),
    rotations_pid(rotations.getRotations()),
    new InstantCommand(modula::stopmodula)
    );
    return commandGroup;
}
public Command brake(boolean brake){
    if (brake)
    return new InstantCommand(()->modula.mode_switcher(NeutralModeValue.Brake));
    else {
        return new InstantCommand(()->modula.mode_switcher(NeutralModeValue.Coast));
    }

}
public Command manualDriveCommand(Double power){
     Command runCommand = new RunCommand(()->modula.movepower(power));
     runCommand.addRequirements(modula);
    return runCommand;
}

    public Command steerToAngleCommand(double degrees) {
        check++;
        return new RunCommand(() -> modula.pid_Degrees_Steer(degrees), modula)
                .until(() -> modula.check(degrees))
    ;}

    public Command rotations_pid(double rotations) {
        check++;
        return new RunCommand(() -> modula.pid_drive(rotations), modula)
                .until(() -> modula.checkrotations(rotations));
    }

    public Command  longnamecommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addRequirements(modula);
        command.addCommands(
                new InstantCommand(modula::stopmodula),
                steerToAngleCommand(0),
                rotations_pid(2),
                steerToAngleCommand(90),
                rotations_pid(2),
                new InstantCommand(modula::stopmodula)
        );
        return command;}


}



