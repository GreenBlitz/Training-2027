package frc.robot;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

public class jbsubclass {
    public final SwerveModduleTomer tuff= new SwerveModduleTomer(12,1);

    public void larp(double x,double y){
        double rads = Math.atan2(x,y);
        Rotation2d rotation2d = Rotation2d.fromRadians(rads);
        tuff.setposition(rotation2d);
    }
    public void larp2 (){

    }


    

}
