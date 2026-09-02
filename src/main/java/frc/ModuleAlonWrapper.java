package frc;

import com.ctre.phoenix6.CANBus;

public class ModuleAlonWrapper {
    private ModuleAlon moduleAlon;
    public ModuleAlonWrapper(int steerID, int linearID,double steerGearRatio,double linearGearRatio, CANBus canBus,String logPath){
        TalonFXTraining steer = new TalonFXTraining(steerID,canBus,logPath+"/steer",steerGearRatio);
        TalonFXTraining drive = new TalonFXTraining(linearID,canBus,logPath+"/drive",linearGearRatio);
        moduleAlon = new ModuleAlon(drive,steer,logPath);
    }
}
