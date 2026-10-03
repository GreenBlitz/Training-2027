package frc;

import edu.wpi.first.wpilibj2.command.RunCommand;

public class MotorSpeakerCommandBuilder {
    private MotorSpeaker motorSpeaker;
    public MotorSpeakerCommandBuilder(MotorSpeaker motorSpeaker){
        this.motorSpeaker=motorSpeaker;
    }
    public RunCommand playNotes(){
        return new RunCommand(
                ()->{motorSpeaker.playAllNotesOnMotors();},
                motorSpeaker
        );
    }
    public void bindMidi(Midi midi){
        midi.withPressNote(
                (n)->{
                    motorSpeaker.playNote(n);
                }
        );
        midi.withReleaseNote(
                (n)->{
                    motorSpeaker.stopNote(n);
                }
        );
    }
}
