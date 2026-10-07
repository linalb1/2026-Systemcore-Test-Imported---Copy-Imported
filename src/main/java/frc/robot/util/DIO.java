package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class DIO {


    private static DigitalInput port0 = new DigitalInput(0);
    private static DigitalInput port1 = new DigitalInput(1);
    private static DigitalInput port2 = new DigitalInput(2);
    private static DigitalInput port3 = new DigitalInput(3);
    private static DigitalInput port4 = new DigitalInput(4);
    private static DigitalInput port5 = new DigitalInput(5);
    private static DigitalInput port6 = new DigitalInput(6);
    private static DigitalInput port7 = new DigitalInput(7);
    private static DigitalInput port8 = new DigitalInput(8);
    private static DigitalInput port9 = new DigitalInput(9);
 
    public DIO() {}

    public static boolean[] getInputs() 
	{
        return new boolean[] {
            port0.get(),
            port1.get(),
            port2.get(),
            port3.get(),
            port4.get(),
            port5.get(),
            port6.get(),
            port7.get(),
            port8.get(),
            port9.get()
        };
    }
}