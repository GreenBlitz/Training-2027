package frc;

import javax.sound.midi.*;
import java.util.HashSet;
import java.util.function.Consumer;

public class Midi{
    private HashSet<Byte> pressedKeys;
    private Consumer<Byte> pressNote;
    private Consumer<Byte> releaseNote;
    private static Transmitter getFirstMidiTransmitter() {
        Transmitter trans = null;
        try {
            MidiDevice device;
            for (MidiDevice.Info info : MidiSystem.getMidiDeviceInfo()) {
                device = MidiSystem.getMidiDevice(info);
                if (!(device instanceof Sequencer || device instanceof Synthesizer) && device.getMaxTransmitters() != 0) {
                    device.open();
                    trans = device.getTransmitter();
                }
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return trans;
    }
    public Midi(Transmitter trans){
        Receiver receiver = new Receiver() {
            @Override
            public void send(MidiMessage message, long timeStamp) {
                if (message.getMessage()[0]==-112){
                    pressedKeys.add(message.getMessage()[1]);
                    pressNote.accept(message.getMessage()[1]);
                } else if (message.getMessage()[0]==-128){
                    pressedKeys.remove(message.getMessage()[1]);
                    releaseNote.accept(message.getMessage()[1]);
                }
            }

            @Override
            public void close() {

            }
        };
        trans.setReceiver(receiver);
        this.pressedKeys=new HashSet<>();
    }

    public void withPressNote(Consumer<Byte> pressNote){
        this.pressNote=pressNote;
    }

    public void withReleaseNote(Consumer<Byte> releaseNote){
        this.releaseNote=releaseNote;
    }

    public static Midi firstMidi(){
        System.out.println("firstMidi");
        return new Midi(getFirstMidiTransmitter());

    }
}
