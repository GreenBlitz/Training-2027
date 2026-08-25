package frc.robot.hardware;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
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
    public String path = "/users/downloads/";

    public StatusSignal<Double> getacl(){
        StatusSignal acl = motor.getAcceleration();
        return(acl);

    }public StatusSignal<Double> get_vel(){
        StatusSignal vel = motor.getVelocity();
        return (vel);

    }public StatusSignal<Double> aura() {
        StatusSignal pos = motor.getPosition();
        return (pos);
    }
    public StatusSignal<Double> get_vol() {
        StatusSignal vol = motor.getMotorVoltage();
        return (vol);

    }
    public StatusSignal<Double> get_cur() {
        StatusSignal cur= motor.getStatorCurrent();
        return (cur);

    }
    public void aurasigma() {


        Logger.recordOutput(path + "/current", get_cur().getValue());
        Logger.recordOutput(path + "/current", get_vol().getValue());
        Logger.recordOutput(path + "/current", get_vel().getValue());
        Logger.recordOutput(path + "/current", aura().getValue());
        Logger.recordOutput(path + "/current", getacl().getValue());


    }

    }

























