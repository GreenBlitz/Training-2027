package frc.robot;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.Tome_motor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;


public class SwerveModduleTomer extends GBSubsystem {

    static TalonFX swerve;
    private InvertedValue Clockwise_Positive = InvertedValue.valueOf(1);
    private InvertedValue Counter_Clockwise_Positive = null;
    private PositionVoltage pid_thing = new PositionVoltage(1);
    public ParentDevice parentDevice = new ParentDevice(21, parentDevice.toString(),new CANBus()) {
    };
    private static final Tome_motor drive = new Tome_motor(21);
    TalonFXConfiguration config = new TalonFXConfiguration();
    public Double targetvoltage = null;
    public Double target1 = null;
    CANcoderConfiguration canfig = new CANcoderConfiguration();

    public SwerveModduleTomer(int id,double value) {
        super();
        this.swerve = new TalonFX(id,CANBus.roboRIO());
        motor_limit();
        setCurrent_limit();
        config.Feedback.SensorToMechanismRatio = value;
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
    public void setposition(Rotation2d pos){
        swerve.setPosition(pos.getDegrees());

    }
    public void drivewheelVoltage(double vol){
        targetvoltage = vol;
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
        Logger.recordOutput(path+"/targerpos", target1);
        Logger.recordOutput(path+"/targetvol",targetvoltage);


    }
    public void pid_misson2(double target) {
        target1 = target;
        config.withSlot0(new Slot0Configs().withKP(1.5));
        pid_thing.withPosition(target);
    }
    public void setvoltage(double vol){
        swerve.setVoltage(vol);
    }
    public void mode_switcher(NeutralModeValue mode) {
        config.MotorOutput.NeutralMode = mode;
        swerve.getConfigurator().apply(config);

    }
    public void stopmodula(){
        swerve.stopMotor();
        drive.stop();
    }
    public void move(){
        drive.move_half();
    }
    public boolean check(double deegres){
        if (deegres == get_pos().getDegrees()){
            return true;

        }else {
            return false;
        }
    }
    public boolean check2(double rads){
        if(rads == get_pos().getRadians()){
            return true;
        }else{
            return false;
        }
    }

    public void larp(double x,double y){
        double rads = Math.atan2(x,y);
        InstantCommand command2 = new InstantCommand(this::move);
        command2.until(() -> check2(rads));
        double power = Math.sqrt(Math.abs((x*x)+(y*y)));
        new InstantCommand(()->setvoltage(11/power));

    }
    public void set2nmodes (NeutralModeValue mode){
        new InstantCommand(()->mode_switcher(mode));
        new InstantCommand(()->drive
                .mode_switcher(mode));
    }
    public void stop(){
        InstantCommand command1;
        command1 = new InstantCommand(this::stopmodula);
        setDefaultCommand(command1);
    }
    public void pointy_pointy(double deegrees){
        InstantCommand command1 = new InstantCommand(this::move);
        command1.until(() -> check(deegrees));
    }
    public void trigger(GenericHID genericHID,int button){
        Trigger combo = new JoystickButton(genericHID,button);
        InstantCommand command2 = new InstantCommand(drive::move_half);
        combo.onTrue(command2);


    }
    public void movepower(double power){
        drive.move(power);
    }
    public boolean checkrotations(double rotation){
        if(drive.get_pos().getRotations()==rotation){
            return true;
        }else return false;

    }
    public boolean checkrotations2(double rotation){
        if(swerve.getPosition().getValueAsDouble()==rotation){
            return true;
        }else return false;

    }








}






