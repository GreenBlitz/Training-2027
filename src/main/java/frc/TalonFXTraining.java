package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
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

	StatusSignal<AngularVelocity> velocity;
	StatusSignal<Voltage> voltage;
	StatusSignal<Current> current;
	StatusSignal<Angle> position;

	public TalonFXTraining(int deviceId, CANBus canBus, String logPath) {
		this.logPath = logPath;
		this.motor = new TalonFX(deviceId, canBus);
		direction = InvertedValue.CounterClockwise_Positive;
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();

		softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
		softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = 5;
		softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
		softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = -3;
		configuration.SoftwareLimitSwitch = softwareLimitSwitchConfigs;

		CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
		currentLimitsConfigs.StatorCurrentLimitEnable = true;
		currentLimitsConfigs.SupplyCurrentLowerLimit = 5;
		currentLimitsConfigs.StatorCurrentLimit = 40;
		configuration.CurrentLimits = currentLimitsConfigs;

		Slot0Configs slot0Configs = new Slot0Configs();
		slot0Configs.kP =1.7197265625;
		slot0Configs.kD =0.0001;
		slot0Configs.kI=0;
		configuration.Slot0 = slot0Configs;
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withInverted(direction);
		configuration.MotorOutput = motorOutputConfigs;

		motor.getConfigurator().apply(configuration);
		motor.optimizeBusUtilization(50);
		motor.getConfigurator().refresh(configuration);

		velocity = motor.getVelocity();
		voltage = motor.getMotorVoltage();
		current = motor.getStatorCurrent();
		position = motor.getPosition();
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

	public void setPower(double amount) {
		motor.set(amount);
	}

	private double angleInRadians(Angle angle){
		return angle.baseUnitMagnitude();
	}

    private double clamp(double val, double min, double max){
        if (val>min&&val<max){
            return val;
        } else if (val>=max){
            return max;
        } else {
            return min;
        }
    }

	public void driveToPositionTick(double angleRadians){
		double difference = angleRadians- getPosition().getRadians();
        setVoltage(difference/(2*Math.PI));
        Logger.recordOutput(logPath+"/target",angleRadians);
        Logger.recordOutput(logPath+"/positionInRadians",getPosition().getRadians());
	}

	public void driveToPosition(double positionRadians){
		PositionVoltage positionVoltage = new PositionVoltage(positionRadians/(2*Math.PI));
		motor.setControl(positionVoltage);
	}


	public static double angleDifferenceRadians(double angle1, double angle2){
		double baseAngleDiff = (angle1-angle2)%(2*Math.PI);
		if (baseAngleDiff>Math.PI){
			return baseAngleDiff-2*Math.PI;
		} else if (baseAngleDiff<-Math.PI){
			return baseAngleDiff+2*Math.PI;
		} else {
			return baseAngleDiff;
		}
	}

	public void moveAtHalfPower() {
		setPower(0.5);
	}

	public void moveReverseTenthSpeed() {
		setPower(-0.1);
	}

	public Rotation2d getPosition() {
        position.refresh();
		return Rotation2d.fromRadians(StatusSignal.getLatencyCompensatedValue(position,velocity).baseUnitMagnitude());
	}

	public Rotation2d getVelocity() {
        velocity.refresh();
		return Rotation2d.fromRotations(velocity.getValueAsDouble());
	}
	public double getVoltage() {
        voltage.refresh();
		return voltage.getValueAsDouble();
	}
	public double getCurrent() {
        current.refresh();
		return current.getValueAsDouble();
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
		Logger.recordOutput(logPath + "/position", position.refresh().getValue());
		Logger.recordOutput(logPath + "/velocity", velocity.refresh().getValue());
		Logger.recordOutput(logPath + "/voltage", voltage.refresh().getValue());
		Logger.recordOutput(logPath + "/current", current.refresh().getValue());
		logMotorConnection();
	}

}
