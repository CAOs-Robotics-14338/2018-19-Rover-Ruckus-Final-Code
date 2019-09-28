package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



public class Encoders {

    //TEst for turning later--> encoder ticks = (360 / circumference) * Distance to travel
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     CIRCUMFERENCE  = WHEEL_DIAMETER_INCHES*3.1415;
    static final double     TICKS_PER_ROTATION_DRIVE = 723.24;
    static final double     TICKS_PER_ROTATION_CENTER = 1120;


    // Rotations = totalDistance/Circumference
    //REVISE MATH


    public Encoders(){
        //CONSTRUCTOR
    }
//for center wheel motor
    public void driveEncoder(DcMotor motor, double distanceToTravel){
       // double encoderTicks = (360 / CIRCUMFERENCE)* distanceToTravel;double rotations = distanceToTravel/CIRCUMFERENCE;double encoderTicks = rotations / TICKS_PER_ROTATION;

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(encoderTicksCenter(distanceToTravel));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//new


    }
//for drive motors
    public void driveEncoder(DcMotor motor, DcMotor motor2, double distanceToTravel){

       // motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int ticks = encoderTicksDrive(distanceToTravel);
       // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(ticks);
        motor2.setTargetPosition(ticks);
        motor.setPower(.8);
        motor2.setPower(.8);

       // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//new
        //motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//new


    }

    public int encoderTicksDrive(double distanceToTravel){
        double rotations = distanceToTravel/CIRCUMFERENCE;
        double encoderTicks = rotations * TICKS_PER_ROTATION_DRIVE;
        return (int)encoderTicks;
    }
    public int encoderTicksCenter(double distanceToTravel){
        double rotations = distanceToTravel/CIRCUMFERENCE;
        double encoderTicks = rotations * TICKS_PER_ROTATION_CENTER;
        return (int)encoderTicks;
    }
    public int encoderTicksTurn(double degrees){
        //double turn = TICKS_PER_ROTATION_DRIVE/(degrees/360);
        double turn = (degrees/360)*((2)*(3.1415)*(9));
        return (int)turn;
    }
/*
    public void driveForward(double distance){//WORKS
        int ticks = encoderTicksDrive(distance);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);
        leftMotor.setPower(0.8);
        rightMotor.setPower(0.8);
    }*/
}
