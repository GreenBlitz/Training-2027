package frc.robot;


import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.mechanisms.wpilib.WPILibMechanismSimulation;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;

import static com.ctre.phoenix6.hardware.TalonFX.*;

public class Simulation implements WPILibMechanismSimulation {

	MechanismSimulation mechanismSimulation;
	TalonFXSimState motorSimState;


	private final DCMotorSim motorSimulation;

	public Simulation(DCMotorSim motorSimulation) {
		this.motorSimulation = motorSimulation;
	}

	@Override
	public Rotation2d getMechanismPosition() {
		return Rotation2d.fromRadians(motorSimulation.getAngularPositionRad());
	}

	@Override
	public Rotation2d getMechanismVelocityRPS() {
		return Rotation2d.fromRadians(motorSimulation.getAngularVelocityRadPerSec());
	}

	@Override
	public void setInputVoltage(double voltage) {
		motorSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		motorSimulation.update(TimeUtil.getLatestCycleTimeSeconds());
	}

	@Override
	public double getGearRatio() {
		return motorSimulation.getGearing();
	}

	public void motorsim() {
		motorSimState.setSupplyVoltage(BatteryUtil.DEFAULT_VOLTAGE);
	}

	public void InizlizeMotor() {
		mechanismSimulation.setInputVoltage(motorSimState.getMotorVoltage());
		mechanismSimulation.updateMotor();
		motorSimState.setRawRotorPosition(mechanismSimulation.getRotorPosition().getRotations());
		motorSimState.setRotorVelocity(mechanismSimulation.getRotorVelocityRPS().getRotations());
	}


}

