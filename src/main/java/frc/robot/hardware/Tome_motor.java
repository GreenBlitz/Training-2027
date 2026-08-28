package frc.robot.hardware;



import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.Logger;


public class Tome_motor {

	private final TalonFX motor;
	private InvertedValue Clockwise_Positive = InvertedValue.valueOf(1);
	private InvertedValue Counter_Clockwise_Positive = null;
	private PositionVoltage pid_thing = new PositionVoltage(1);
	public ParentDevice parentDevice = new ParentDevice(21, parentDevice.toString(),new CANBus()) {
	};
	TalonFXConfiguration config = new TalonFXConfiguration();

	public Tome_motor(int id) {
		this.motor = new TalonFX(id,CANBus.roboRIO());
		motor_limit();
		setCurrent_limit();
		frequncy_optimaztion();
		parentDevice.optimizeBusUtilization(50);
		motor.getConfigurator().apply(config);
	}

	public void motor_limit() {
		config.HardwareLimitSwitch.ForwardLimitEnable=true;
		config.HardwareLimitSwitch.ReverseLimitEnable = true;
		config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 5.0;
		config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = -3.0;
	}

	public void setCurrent_limit() {
		config.CurrentLimits.StatorCurrentLimitEnable =true;
		config.CurrentLimits.StatorCurrentLimit = 40.0;
	}

	public void move_half() {
		motor.set(0.5);
	}

	public void move_tenth() {
		motor.set(-0.1);
	}

	public void stop() {
		motor.stopMotor();
	}

	public String path = "motor1";

	public double getacl() {

		StatusSignal acl = motor.getAcceleration();
		acl.setUpdateFrequency(50);
		StatusSignal.refreshAll();
		return acl.getValueAsDouble();

	}

	public Rotation2d get_vel() {
		StatusSignal vel = motor.getVelocity();
		vel.setUpdateFrequency(50);
		StatusSignal.refreshAll();
		double double_vel = (vel.getValueAsDouble()*360);
		return Rotation2d.fromDegrees(double_vel);


	}

	public Rotation2d get_pos() {
		StatusSignal late = (StatusSignal) BaseStatusSignal.getLatencyCompensatedValue(motor.getPosition(),motor.getVelocity());
		late.setUpdateFrequency(50);
		double check = motor.getPosition().getValueAsDouble();
		double check2 = (check*360);
		Rotation2d rotatoin = Rotation2d.fromDegrees(check2);
		return rotatoin;

	}


	public double get_vol() {
		StatusSignal vol = motor.getMotorVoltage();
		vol.setUpdateFrequency(50);
		StatusSignal.refreshAll();
		return vol.getValueAsDouble();


	}
	public double get_cur() {

		StatusSignal current = motor.getStatorCurrent();
		current.setUpdateFrequency(50);
		StatusSignal.refreshAll();
		return current.getValueAsDouble();

	}
	public void frequncy_optimaztion(){
		StatusSignal current =motor.getStatorCurrent();
		StatusSignal vel =motor.getVelocity();
		StatusSignal voltage =motor.getMotorVoltage();
		StatusSignal position =motor.getPosition();
		position.setUpdateFrequency(50);
		vel.setUpdateFrequency(50);
		voltage.setUpdateFrequency(50);
		current.setUpdateFrequency(50);
		StatusSignal.refreshAll();

	}

	public void  logger() {
		Logger.recordOutput(path + "/current", get_cur());
		Logger.recordOutput(path + "/VOL", get_vol());
		Logger.recordOutput(path + "/VEL", get_vel());
		Logger.recordOutput(path + "/POSITION", get_pos());
		Logger.recordOutput(path + "/ACL", getacl());
		connected();
	}

	public void SwitchDierction() {
		if (Clockwise_Positive == null) {
			Counter_Clockwise_Positive = null;
			Clockwise_Positive = InvertedValue.Clockwise_Positive;
			config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
			motor.getConfigurator().apply(config);
		} else if (Counter_Clockwise_Positive == null) {
			Clockwise_Positive = null;
			Counter_Clockwise_Positive = InvertedValue.CounterClockwise_Positive;
			config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
			motor.getConfigurator().apply(config);

		}
	}

	public void connected() {
		Logger.recordOutput(path + "/connected", motor.isConnected());
	}

	public void mode_switcher(NeutralModeValue mode) {
		config.MotorOutput.NeutralMode = mode;
		motor.getConfigurator().apply(config);

	}

	public void set_pos(double pos) {
		motor.setPosition(pos);
	}

	public void pid_misson2(double target) {
		config.withSlot0(new Slot0Configs().withKP(-2).withKD(2).withKI(2));
		pid_thing.withPosition(target);
	}
		}






