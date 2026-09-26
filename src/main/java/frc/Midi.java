package frc;

import javax.sound.midi.*;
import java.util.HashSet;

public class Midi{
    private HashSet<Byte> pressedKeys;
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
                if (message.getMessage()[0]==-122){
                    pressedKeys.add(message.getMessage()[1]);
                } else if (message.getMessage()[0]==-128){
                    pressedKeys.remove(message.getMessage()[1]);
                }
            }

            @Override
            public void close() {

            }
        };
    }

    public Midi(){
        this(getFirstMidiTransmitter());
    }
}
