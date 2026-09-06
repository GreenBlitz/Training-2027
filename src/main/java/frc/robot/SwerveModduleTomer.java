`package frc.robot;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.hardware.Tome_motor;
import org.littletonrobotics.junction.Logger;


public class SwerveModduleTomer {

    TalonFX swerve;
    private InvertedValue Clockwise_Positive = InvertedValue.valueOf(1);
    private InvertedValue Counter_Clockwise_Positive = null;
    private PositionVoltage pid_thing = new PositionVoltage(1);
    public ParentDevice parentDevice = new ParentDevice(21, parentDevice.toString(),new CANBus()) {
    };
    private final Tome_motor drive = new Tome_motor(21);
    TalonFXConfiguration config = new TalonFXConfiguration();

    public SwerveModduleTomer(int id) {
        this.swerve = new TalonFX(id,CANBus.roboRIO());
        motor_limit();
        setCurrent_limit();
        parentDevice.optimizeBusUtilization(50);
        swerve.getConfigurator().apply(config);
    }
    public void SwitchDierctionswerve() {
        if (Clockwise_Positive == null) {
            Counter_Clockwise_Positive = null;
            Clockwise_Positive = InvertedValue.Clockwise_Positive;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            swerve.getConfigurator().apply(config);
        } else if (Counter_Clockwise_Positive == null) {
            Clockwise_Positive = null;
            Counter_Clockwise_Positive = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            swerve.getConfigurator().apply(config);

        }
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
    public void setposition(double pos){
        swerve.setPosition(pos);

    }
    public void drivewheelVoltage(double vol){
        drive.setvoltage(vol);
    }
    public void getPosition(){
        swerve.getPosition();

    }
    public boolean ceheckVelocity(Rotation2d vel) {
        if (drive.get_vel() == vel) {
            return true;

        }else{
            return false;
        }

    }
    public Rotation2d get_pos() {
        StatusSignal late = (StatusSignal) BaseStatusSignal.getLatencyCompensatedValue(swerve.getPosition(),swerve.getVelocity());
        late.setUpdateFrequency(50);
        Rotation2d rotatoin = Rotation2d.fromDegrees(late.getValueAsDouble());
        return rotatoin;

    }
    public boolean checkPosition(Rotation2d pos){
        if (get_pos() == pos){
            return true;
        }else {
            return false;
        }

    }
    public String path = "motor2";
    public void LogVarubles(){
        Logger.recordOutput(path + "/velocity", Rotation2d.fromDegrees(swerve.getVelocity().getValueAsDouble()));
        Logger.recordOutput(path + "/driveVoltage", drive.get_vol());
        Logger.recordOutput(path + "/position", get_pos());
        Logger.recordOutput(path+"/");


    }
    public void pid_misson2(double target) {
        config.withSlot0(new Slot0Configs().withKP(1.5));
        pid_thing.withPosition(target);
    }






}






`