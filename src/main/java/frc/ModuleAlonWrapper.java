package frc;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;

import java.util.function.Supplier;

public class ModuleAlonWrapper {
    private ModuleAlon moduleAlon;
    private boolean comboButton1=false;
    private boolean comboButton2=false;
    public ModuleAlonWrapper(int steerID, int linearID,double steerGearRatio,double linearGearRatio, CANBus canBus,String logPath){
        TalonFXTraining steer = new TalonFXTraining(steerID,canBus,logPath+"/steer",steerGearRatio);
        TalonFXTraining drive = new TalonFXTraining(linearID,canBus,logPath+"/drive",linearGearRatio);
        moduleAlon = new ModuleAlon(drive,steer,logPath);
    }
    public RunCommand driveWithStick(Supplier<Double> xAxis, Supplier<Double> yAxis){
        return new RunCommand(()->{
            double x = xAxis.get();
            double y = yAxis.get();
            moduleAlon.steerToPosition(Math.atan2(y,x));
            moduleAlon.linearSetPower(Math.sqrt(x*x+y*y));
        });
    }
    public RunCommand driveWithLeftStick(SmartJoystick joystick){
        return driveWithStick(()-> joystick.getAxisValue(Axis.LEFT_X),()-> joystick.getAxisValue(Axis.LEFT_Y));
    }
    public RunCommand driveWithRightStick(SmartJoystick joystick){
        return driveWithStick(()-> joystick.getAxisValue(Axis.RIGHT_X),()-> joystick.getAxisValue(Axis.RIGHT_Y));
    }
    public void checkComboAndExecute(){
        if (comboButton1&&comboButton2){
            moduleAlon.linearSetPower(0.5);
        }
    }
    public void bindButtons(SmartJoystick joystick){
        joystick.A.onTrue(new InstantCommand(()->{comboButton1=true;checkComboAndExecute();}));
        joystick.A.onFalse(new InstantCommand(()->{comboButton1=false;}));
        joystick.B.onTrue(new InstantCommand(()->{comboButton2=true;checkComboAndExecute();}));
        joystick.B.onFalse(new InstantCommand(()->{comboButton2=false;}));
    }

}
