package frc;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleAlon extends GBSubsystem {
    private final TalonFXTraining linear;
    private final TalonFXTraining steer;
    private final String LOGPATH;
    public ModuleAlon(TalonFXTraining linear, TalonFXTraining steer, String LOGPATH){
        super(LOGPATH);
        this.linear=linear;
        this.steer=steer;
        this.LOGPATH=LOGPATH;
    }
    public void invertLinear(){
        linear.invertMotor();
    }
    public void invertSteer(){
        steer.invertMotor();
    }
    public void steerToPosition(double angleRadians){
        steer.driveToPosition(angleRadians);
    }
    public void linearToPosition(double angleRadians){
        linear.driveToPosition(angleRadians);
    }
    public void linearSetPower(double power){
        linear.setPower(power);
    }
    public Rotation2d getSteerAngle(){
        return steer.getPosition();
    }
    public Rotation2d getLinearVelocity(){
        return linear.getVelocity();
    }
    public void setLinearNeutral(NeutralModeValue neutralMode){
        linear.setNeutralMode(neutralMode);
    }
    public void setSteerNeutral(NeutralModeValue neutralMode) {
        steer.setNeutralMode(neutralMode);
    }
    public void logAll(){
        Logger.recordOutput(LOGPATH+"/linearMotorVelocity",getLinearVelocity());
        Logger.recordOutput(LOGPATH+"/steerAngle",getSteerAngle());
        Logger.recordOutput(LOGPATH+"/steerTarget",steer.getPIDTarget());
        Logger.recordOutput(LOGPATH+"/linearTarget",linear.getPIDTarget());

    }


}
