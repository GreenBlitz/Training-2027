package frc;

public class ModuleAlon {
    private final TalonFXTraining linear;
    private final TalonFXTraining steer;
    public ModuleAlon(TalonFXTraining linear, TalonFXTraining steer){
        this.linear=linear;
        this.steer=steer;
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


}
