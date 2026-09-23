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
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.utils.brakestate.BrakeMode;
import org.littletonrobotics.junction.Logger;

public class Modulesubsystem{


    private static SwerveModduleTomer modula;
    private PositionVoltage pid_thing = new PositionVoltage(1);
    public String Path1;
    public String Path2;
    private TalonFX drive;
    private TalonFX steer;
private Modulesubsystem(int id,double value){
    steer = new TalonFX(12);
    modula =new SwerveModduleTomer(id, value);
    drive = new TalonFX(21);
    Path1 = new String("drive1");
    Path2 = new String("steer1");
    Slot0Configs pidcofing = new Slot0Configs().withKP(0.5);
    steer.getConfigurator().apply(pidcofing);
}

public void LogVarubles() {
    Logger.recordOutput(Path1 + "/velocity", Rotation2d.fromDegrees(drive.getVelocity().getValueAsDouble()));
    Logger.recordOutput(Path1 + "/driveVoltage", drive.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(Path1 + "/position", Rotation2d.fromDegrees(drive.getVelocity().getValueAsDouble()));
    Logger.recordOutput(Path1 + "isconnected",isconnected1(drive));
    Logger.recordOutput(Path2 + "/velocity", Rotation2d.fromDegrees(steer.getVelocity().getValueAsDouble()));
    Logger.recordOutput(Path2 + "/driveVoltage", steer.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(Path2 + "/position", Rotation2d.fromDegrees(steer.getVelocity().getValueAsDouble()));
    Logger.recordOutput(Path2 + "isconnected",isconnected1(steer));




}
public Double isconnected1(TalonFX motor){
    if(motor.isConnected()){
        return (0.1);
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

public void movetoposition(Rotation2d postion){
    pid_thing.withPosition(postion.getDegrees());
}
public void setposition(Rotation2d position){
    steer.setPosition(position.getDegrees());
}

public Command DriveDistanceCommand(Rotation2d degrees, Rotation2d rotations){
    return Commands.sequence(
    new InstantCommand(modula::stopmodula),
    new InstantCommand(()->modula.pointy_pointy(degrees.getDegrees())),
            new InstantCommand(modula::move)
    .until(()-> modula.check(degrees.getDegrees())),
    new InstantCommand(modula::stopmodula)
    );
}
public Command brake(boolean brake){
    if (brake = true)
    return new InstantCommand(()->modula.mode_switcher(NeutralModeValue.Brake));
    else {
        return new InstantCommand(()->modula.mode_switcher(NeutralModeValue.Coast));

    }
}
public Command manualDriveCommand(Double power){
    return new RunCommand(()->modula.movepower(power));
}
public Command longnamecommand(){
    return Commands.sequence(
            new InstantCommand(modula::stopmodula),
            new InstantCommand(()->modula.pointy_pointy(0)),
            new InstantCommand(modula::move)
                    .until(()-> modula.checkrotations2(2.0)),
                            new InstantCommand(()->modula.pointy_pointy(90)),
                            new InstantCommand(modula::move)
                                    .until(()-> modula.checkrotations2(2.0)),
                                            new InstantCommand(()->modula.pointy_pointy(180)),
                                            new InstantCommand(modula::move)
                                                    .until(()-> modula.checkrotations2(2.0)),
                                                            new InstantCommand(()->modula.pointy_pointy(-90)),
                                                            new InstantCommand(modula::move)
                                                                    .until(()-> modula.checkrotations2(2.0))
    );





}

}

