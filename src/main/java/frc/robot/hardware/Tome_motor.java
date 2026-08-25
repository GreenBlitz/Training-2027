package frc.robot.hardware;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

public class Tome_motor {
    private final TalonFX motor;

    public Tome_motor(TalonFX motor) {
        this.motor = new TalonFX(1);
    }
    public void move_half()
    {
        motor.set(0.5);

    }
    public void move_tenth(){
        motor.set(0.1);
    }
    public void stop(){
        motor.stopMotor();
    }
    public String path = "users/downloads/";
    public void getstuff(){
        StatusSignal acl = motor.getAcceleration();
        StatusSignal vol = motor.getMotorVoltage();
        StatusSignal pos= motor.getPosition();
        StatusSignal cur= motor.getStatorCurrent();
        Logger.recordOutput(path+"/vol", vol.hasUpdated());
        Logger.recordOutput();
    }public StatusSignal getacl(){
        StatusSignal acl = motor.getAcceleration();
        return(acl);

    }public StatusSignal get_vel(){
        StatusSignal vel = motor.getVelocity();
        return (vel);

    }


    }












