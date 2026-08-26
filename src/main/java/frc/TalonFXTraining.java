package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.InvertedValue;


public class TalonFXTraining {

	private final TalonFX motor;
	private InvertedValue direction;
	private final String logPath;

	public TalonFXTraining(int deviceId, CANBus canBus, String logPath) {
		this.logPath = logPath;
		this.motor = new TalonFX(deviceId, canBus);
		direction = InvertedValue.CounterClockwise_Positive;

		SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
		/* tasks 5 and 7 */
		softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
		softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = 5;
		softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
		softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = -3;
		motor.getConfigurator().apply(softwareLimitSwitchConfigs);
		/* task 9 */
		CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
		currentLimitsConfigs.StatorCurrentLimitEnable = true;
		currentLimitsConfigs.SupplyCurrentLowerLimit = 5;
		currentLimitsConfigs.StatorCurrentLimit = 40;
		motor.getConfigurator().apply(currentLimitsConfigs);
		/* task 8 */
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withInverted(direction);
		motor.getConfigurator().apply(motorOutputConfigs);
	}

	private boolean isMotorConnected() {
		return motor.isConnected();
	}

	public void logMotorConnection() {
		Logger.recordOutput(logPath + "/isMotorConnected", isMotorConnected());
	}

	public void stopMotor() {
		motor.stopMotor();
	}


	private void setPower(double amount) {
		motor.set(amount);
	}

	public void moveAtHalfPower() {
		setPower(0.5);
	}

	public void moveReverseTenthSpeed() {
		setPower(-0.1);
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

	public void invertMotor() {
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		motorOutputConfigs.withInverted(getMotorInvertedDirection());
		motor.getConfigurator().apply(motorOutputConfigs);
		direction = getMotorInvertedDirection();
	}

	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	public void setNeutralMode(NeutralModeValue neutralMode) {
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		motorOutputConfigs.withNeutralMode(neutralMode);
		motor.getConfigurator().apply(motorOutputConfigs);
	}

	public void setPosition(double angle) {
		motor.setPosition(angle);
	}

	public void setPosition(Angle angle) {
		motor.setPosition(angle);
	}

	private InvertedValue getMotorInvertedDirection() {
		return direction == InvertedValue.Clockwise_Positive ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
	}


	public void logAll() {
		Logger.recordOutput(logPath + "/position", getPosition().getValue());
		Logger.recordOutput(logPath + "/velocity", getVelocity().getValue());
		Logger.recordOutput(logPath + "/voltage", getVoltage().getValue());
		Logger.recordOutput(logPath + "/current", getCurrent().getValue());
		logMotorConnection();
	}

}
