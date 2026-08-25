package frc.robot.hardware;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;

public class Motor_class {
    void TalonFX(int deviceId, CANBus canbus) {



    }
    public Motor_class(){
        TalonFX motor = new TalonFX(1);
    }

    motor.get(1,0);
}
