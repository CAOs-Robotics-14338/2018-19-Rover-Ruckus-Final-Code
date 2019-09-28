package org.firstinspires.ftc.teamcode;

public class EncodersANNOTATED {
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     CIRCUMFERENCE  = WHEEL_DIAMETER_INCHES*3.1415;
    static final double     TICKS_PER_ROTATION_DRIVE = 723.24;
    static final double     TICKS_PER_ROTATION_CENTER = 1120;

    public EncodersANNOTATED(){
        //CONSTRUCTOR
    }


    public int encoderTicksDrive(double inches){
        //
        double rotations = inches/CIRCUMFERENCE;//gets the number of rotations needed by wheel based on the number of inches
        double encoderTicks = rotations * TICKS_PER_ROTATION_DRIVE; //finds out the number of ticks needed for the encoders in order to make the necesary number of wheel rotations
        return (int)encoderTicks; //returns an integer of encoderTicks since there can not be a partial tick
    }
    public int encoderTicksCenter(double inches){
        double rotations = inches/CIRCUMFERENCE;//gets the number of rotations needed by wheel based on the number of inches
        double encoderTicks = rotations * TICKS_PER_ROTATION_CENTER;//finds out the number of ticks needed for the encoders in order to make the necesary number of wheel rotations
        return (int)encoderTicks;//returns an integer of encoderTicks since there can not be a partial tick
    }
    public int encoderTicksTurn(double degrees){
        //circumference = 2 * pi * r
        double turn = (degrees/360)*((2)*(3.1415)*(9)); //determines the ticks needed in order to turn the approximate number of degrees
        return (int)turn;                               // proportion of the degrees to 360 and the circumference around the circle
    }

}
