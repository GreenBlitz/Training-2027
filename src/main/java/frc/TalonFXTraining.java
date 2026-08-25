package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.Logger;


public class TalonFXTraining {

	private final TalonFX motor;

	public TalonFXTraining(int deviceId, CANBus canBus) {
		this.motor = new TalonFX(deviceId, canBus);
        SoftwareLimitSwitchConfigs slsc = new SoftwareLimitSwitchConfigs();
        slsc.ForwardSoftLimitEnable=true;
        slsc.ForwardSoftLimitThreshold=360*5;
        motor.getConfigurator().apply(slsc);
	}

	private void setPower(double amount) {
		motor.set(amount);
	}

	public void moveAtHalfPower() {
		setPower(.5);
	}

	public void moveReverseTenthSpeed() {
		setPower(-.1);
	}

	public void stopMotor() {
		motor.stopMotor();
	}

	public StatusSignal<Angle> getPosition() {
		return motor.getPosition();
	}

	public StatusSignal<AngularVelocity> getVelocity() {
		return motor.getVelocity();
	}

	public StatusSignal<Voltage> getVoltage() {
		return motor.getMotorVoltage();
	}

	public StatusSignal<Current> getCurrent() {
		return motor.getStatorCurrent();
	}

    private static final String logPath="/home/linus/roboticsLog";

    public void logAll(){
        Logger.recordOutput(logPath+"/position",getPosition().getValue());
        Logger.recordOutput(logPath+"/velocity",getVelocity().getValue());
        Logger.recordOutput(logPath+"/voltage",getVoltage().getValue());
        Logger.recordOutput(logPath+"/current",getCurrent().getValue());
    }

}
