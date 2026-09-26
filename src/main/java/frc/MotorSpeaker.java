package frc;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.GBSubsystem;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

public class MotorSpeaker extends GBSubsystem {
    Queue<AlonFX> inactiveMotors;
    Queue<Byte> currentNotes;
    Byte lastDumpedNote;
    AlonFX dumpedMotor;
    Byte getLastDumpedNote(){
        return lastDumpedNote;
    }
    public MotorSpeaker(AlonFX... motors){
        super("motorSpeaker");
        inactiveMotors = new LinkedList<>();
        inactiveMotors.addAll(Arrays.asList(motors));
    }
    public static void playNote(AlonFX motor, byte note){
        motor.playFreq(440.0*Math.pow(2,1.0*note/12));
    }
    public FunctionalCommand playNoteCommand(byte note){
        AlonFX[] motor = new AlonFX[1];
        return new FunctionalCommand(
                ()->{motor[0]=inactiveMotors.poll();},
                ()->{playNote(motor[0],note);},
                (i)->{
                    motor[0].stopMotor();
                    inactiveMotors.offer(motor[0]);
                    if (i){
                        currentNotes.remove(note);
                    }
                },
                ()->{return getLastDumpedNote()==note;}
        );
    }
}
