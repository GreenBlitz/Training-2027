package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
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
		this.logPath=logPath;
		this.motor = new TalonFX(deviceId, canBus);
		direction = InvertedValue.CounterClockwise_Positive;

		SoftwareLimitSwitchConfigs slsc = new SoftwareLimitSwitchConfigs();
		/* tasks 5 and 7 */
		slsc.ForwardSoftLimitEnable = true;
		slsc.ForwardSoftLimitThreshold = 360 * 5;
		slsc.ReverseSoftLimitEnable = true;
		slsc.ReverseSoftLimitThreshold = 360 * 3;
		motor.getConfigurator().apply(slsc);
		/* task 9 */
		CurrentLimitsConfigs clc = new CurrentLimitsConfigs();
		//clc.StatorCurrentLimitEnable = true;
		//clc.StatorCurrentLimit = 40;
		//motor.getConfigurator().apply(clc);
		/* task 8 */
		MotorOutputConfigs moc = new MotorOutputConfigs().withInverted(direction);
		motor.getConfigurator().apply(moc);
	}

	private boolean isMotorConnected() {
		return motor.isConnected();
	}

	public void logMotorConnection() {
		Logger.recordOutput(logPath + "/isMotorConnected", isMotorConnected());
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

	public void stopMotor() {
		motor.stopMotor();
        motor.set(0);
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

	public void reverseMotor() {
		MotorOutputConfigs moc = new MotorOutputConfigs();
		moc.withInverted(getMotorInvertedDirection());
		motor.getConfigurator().apply(moc);
		direction = getMotorInvertedDirection();
	}

    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

	public void setNeutralMode(NeutralModeValue neutralMode) {
		MotorOutputConfigs moc = new MotorOutputConfigs();
		moc.withNeutralMode(neutralMode);
		motor.getConfigurator().apply(moc);
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
