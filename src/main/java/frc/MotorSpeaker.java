package frc;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.GBSubsystem;

import java.util.*;

public class MotorSpeaker extends GBSubsystem {
    private AlonFX[] motors;
    Queue<Integer> unusedMotors;
    Map<Byte,Integer> noteToMotor;
    Queue<Byte> noteQueue;
    public MotorSpeaker(AlonFX... motors){
        super("motorSpeaker");
        this.motors=motors;
        noteToMotor=new HashMap<>();
        unusedMotors=new LinkedList<>();
        for (int i=0; i<motors.length; i++){
            unusedMotors.add(i);
        }
    }
    public MotorSpeaker(CANBus canBus, int... ids){
        this(generateMotorFromIDs(canBus, ids));
    }
    private static AlonFX[] generateMotorFromIDs(CANBus canBus, int... ids){
        AlonFX[] motors1 = new AlonFX[ids.length];
        for (int i=0; i<ids.length; i++){
            motors1[i]=new AlonFX(ids[i],canBus,"motorSpeaker/motor"+ids[i],1,2);
        }
        return motors1;
    }
    private boolean isFull(){
        return unusedMotors.isEmpty();
    }
    public void playNote(byte note){
        if (noteToMotor.containsKey(note)){
            return;
        }
        int motorIndex;
        if (isFull()){
            Byte outNote = noteQueue.poll();
            motorIndex=noteToMotor.get(note);
            noteToMotor.remove(outNote);
        } else {
            motorIndex= unusedMotors.poll();
        }
        unusedMotors.remove(motorIndex);
        noteToMotor.put(note,motorIndex);
    }

    public void stopNote(byte note){
        unusedMotors.add(noteToMotor.get(note));
        noteToMotor.remove(note);
        noteQueue.remove(note);
    }

    public void playAllNotesOnMotors(){
        noteToMotor.forEach((note,motor)->{
            playNoteOnMotor(motors[motor],note);
        });
    }
    public static void playNoteOnMotor(AlonFX motor, byte note){
        motor.playFreq(440.0*Math.pow(2,1.0*note/12));
    }
}
