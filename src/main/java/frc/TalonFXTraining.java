package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;


public class TalonFXTraining {

	private final TalonFX motor;

	public TalonFXTraining(int deviceId, CANBus canBus) {
		this.motor = new TalonFX(deviceId, canBus);
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

}
