/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//Goal is to decrease the time for sampling to be able to go to the crater

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Crater Auto 9", group="Pushbot")//CHANGED ORDER OF SAMPLING MINERALS
@Disabled
public class AutoBeginningsCRATER9 extends LinearOpMode {
    DigitalChannel digitalTouch;  // Hardware Device Object

    private DcMotor centerDrive = null;
    private DcMotor climbMotor = null;

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    double climbPower;

    MarkerDeployment deploy = new MarkerDeployment();
    Servo marker;
    Encoders encoders = new Encoders();

    private GoldAlignDetector detector;

    @Override
    public void runOpMode() {
        //SET UP VISION
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());//, 1, false);
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

       // marker.setMarkerPosition(0.75);
        leftMotor  = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
//Limit Switch
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        marker  = hardwareMap.servo.get("deploy_marker");

        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");
        climbMotor = hardwareMap.get(DcMotor.class, "climb_motor");

        climbMotor.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.REVERSE);

        climbPower = -0.75;//-0.5

      //  centerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        String pathTaken = "";

        while (opModeIsActive()) {
            /* AUTO STEPS:
            --Start in collapsed state with climb collapsed
            --Auto mode starts and robot descends and goes sideways.
            --Samples Minerals **NOT AVAILABLE
            -- Goes to depot and deploys marker
             */
            boolean descend = true;
            // descend climber

                while(descend) {//digitalTouch.getState() == false
                    telemetry.addData("Digital Touch", "Is Not Pressed");
                    climbMotor.setPower(climbPower);
                    telemetry.update();
                    if (digitalTouch.getState()){
                        climbMotor.setPower(0);//stop
                        descend = false;
                    }
                }

            sleep(250);
            //slide sideways to get unattached
            slideLeft(2.5);
            sleep(500);//1000
            boolean rescend = true;
            while(rescend){
                telemetry.addData("Digital Touch", "Is Pressed");
                climbMotor.setPower(-climbPower);
                telemetry.update();
                if(digitalTouch.getState() == false){
                    climbMotor.setPower(0);
                    rescend = false;
                }
            }

            //raise climber 2 inches
            // gos forward to sample
            driveForward(15);
            sleep(1000);
            //
            //\\\SAMPLING ///\\\///////////////////////////////////////////////////////////////////////////////////////////////////
            if (detector.isFound()) {
                //CAN SEE GOLD
                telemetry.addData("Sees the mineral", " ");
                telemetry.update();
                sleep(250);
                if (detector.getAligned()) {//is robot aligned
                    telemetry.addData("Gold Mineral is in center --> move accordingly", detector.getXPosition());
                    telemetry.addData("Move forward", ' ');
                    pathTaken = "CENTER";
                    driveForward(12);//8
                    sleep(1000);//3000
                } else {
                    while (!detector.getAligned()) {
                        telemetry.addData("Not Aligned", " ");
                        telemetry.update();
                        align();
                    }
                    driveForward(8);
                    sleep(800);//3000
                }
            } else {
////See if gold is on the RIGHT
                telemetry.addData("Not Found", " ");
                telemetry.update();
                //sleep(2000);
                slideRight(15);//12.5
                sleep(2000);
                if (detector.isFound()) {
                    telemetry.addData("Found on Right", " ");
                    pathTaken = "RIGHT";
                    telemetry.update();
                    while (!detector.getAligned()){
                        align();}
                    sleep(2000);
                    driveForward(12);
                    sleep(1000);//3
                } else {
                    telemetry.addData("Not found on RIGHT", " going LEFT");
                    telemetry.update();
                    //sleep(2000);
                    slideLeft(30);//28
                    sleep(4450);//4500
                    pathTaken="LEFT";
                    if (detector.isFound()) {
                        telemetry.addData("Found on LEFT", " ");
                        telemetry.update();
                        while (!detector.getAligned()){
                            align();}
                        sleep(1000);//1700
                        driveForward(14);//20
                        sleep(1500);//1700
                    }
                    else{
                        driveForward(8);//20
                        sleep(850);
                    }
                }
            }
           // sleep(5000);
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //Sampling Over
            //Get position taken and get to depot
            //drive toward depot framework and then head to crater

            telemetry.addData("Path Taken: ", pathTaken);
            telemetry.update();
            sleep(100);
            if (pathTaken.equals("RIGHT")){
                driveBackward(15);//13
                sleep(1400);//1500
                //issues begin
                turnLeft(115);// 97
                sleep(3000);//2100
               /* slideLeft(2.5);//new
                sleep(500);*///see if the crazy right slide is the problem created by this left slide. //evidently this crazy slide was caused by this command.
                //Distance forward depends on position and path taken
                driveForward(44);
                sleep(2600);//3600
                turnLeft(60);//45
                sleep(1000);//1300
                slideRight(24);//18
                sleep(2000);
                driveForward(54, .8);//.7//48
                sleep(1800);//2800
                deploy.deployMarker(marker);
                sleep(500);
                driveBackward(66);//4
                sleep(6000);
                /*
                slideLeft(4);
                sleep(500);
                turnLeft(180);
                sleep(2000);
                slideLeft(4);
                sleep(500);
                driveForward(51);
                sleep(6000);
                //inch forward to insure touching crater
                driveForward(4,.30);
                sleep(1000);*/
            }
            else if (pathTaken.equals("LEFT")) {
                driveBackward(13);//13
                sleep(1200);//1500
                //issues begin
                turnLeft(115);// 97
                sleep(2400);//2500
               /* slideLeft(2.5);//new
                sleep(500);*///see if the crazy right slide is the problem created by this left slide. //evidently this crazy slide was caused by this command.
                //Distance forward depends on position and path taken
                driveForward(20);
                sleep(2000);//2600
                turnLeft(60);//45
                sleep(1300);//1300
                slideRight(24);//18
                sleep(2000);
                driveForward(50, .8);//.7//48
                sleep(1600);//2800
                deploy.deployMarker(marker);
                sleep(500);
                driveBackward(66);//4
                sleep(6000);
            }

            else{// == "CENTER"
                driveBackward(15);//15
                sleep(1700);//1500
                //issues begin
                turnLeft(110);// 97
                sleep(3000);//2100
               /* slideLeft(2.5);//new
                sleep(500);*///see if the crazy right slide is the problem created by this left slide. //evidently this crazy slide was caused by this command.
                //Distance forward depends on position and path taken
                driveForward(35);
                sleep(2600);//3600
                turnLeft(60);//45
                sleep(1000);//1300
                slideRight(24);//18
                sleep(2000);
                driveForward(50, .8);//.7//48
                sleep(1600);//1800
                deploy.deployMarker(marker);
                sleep(500);
                driveBackward(70);//4
                sleep(6500);/*
                slideLeft(4);
                sleep(500);
                turnLeft(180);
                sleep(2000);
                slideLeft(4);
                sleep(500);
                driveForward(51);
                sleep(6000);
                //inch forward to insure touching crater
                driveForward(4,.30);
                sleep(1000);*/
            }
            sleep(500);
            stop();
        }



    }

    public void sample() {
        if (detector.isFound()) {
            //CAN SEE GOLD
            telemetry.addData("Sees the mineral", " ");
            telemetry.update();
            sleep(2000);
            if (detector.getAligned()) {//is robot aligned
                telemetry.addData("Gold Mineral is in center --> move accordingly", detector.getXPosition());
                telemetry.addData("Move forward", ' ');
                driveForward(36);
                sleep(3000);
            } else {
                while (!detector.getAligned()) {
                    telemetry.addData("Not Aligned", " ");
                    telemetry.update();
                    align();
                }
                driveForward(36);
                sleep(3000);
            }
        } else {
////See if gold is on the left
            telemetry.addData("Not Found", " ");
            telemetry.update();
            sleep(2000);
            slideLeft(12.5);
            sleep(2000);
            if (detector.isFound()) {
                telemetry.addData("Found on Left", " ");
                telemetry.update();
                while (!detector.getAligned()){
                    align();}
                    sleep(2000);
                driveForward(25);
                sleep(3000);
            } else {
                telemetry.addData("Not found on left", " going right");
                telemetry.update();
                sleep(2000);
                slideRight(25);
                sleep(4000);
                if (detector.isFound()) {
                    telemetry.addData("Found on Right", " ");
                    telemetry.update();
                    while (!detector.getAligned()){
                    align();}
                    sleep(2000);
                    driveForward(25);
                    sleep(5000);
                }
                else{
                    driveForward(20);
                    sleep(5000);
                }
            }
        }
        sleep(5000);
        sleep(10);
    }
    public void align() {
        if ((detector.getXPosition() > 0) && (detector.getXPosition() < detector.getAlignXMin())) {
            telemetry.addData("Gold Mineral is on the left --> move accordingly", detector.getXPosition());
            //new additions
            telemetry.addData("Move to the left", 300 - detector.getXPosition());
            slideLeft(1.0);
            sleep(250);//500
        } else if (detector.getXPosition() > detector.getAlignXMax()) {
            telemetry.addData("Gold Mineral is on the right --> move accordingly", detector.getXPosition());
            //new addition
            telemetry.addData("Move to the right", detector.getXPosition() - 300);
            slideRight(1.0);//1.0
            sleep(250);//500
        }
    }


    public void turnRight(double degrees){
        int inches_needed = encoders.encoderTicksTurn(degrees);
        int ticks = encoders.encoderTicksDrive(inches_needed);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ///left forward, right backward
        leftMotor.setTargetPosition(-ticks);
        rightMotor.setTargetPosition(ticks);
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
    }
    public void turnLeft(double degrees){
        int inches_needed = encoders.encoderTicksTurn(degrees);
        int ticks = encoders.encoderTicksDrive(inches_needed);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ///left backward, right forward
        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(-ticks);
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        telemetry.addData("Ticks for TURN LEFT: ", ticks);

    }

        public void driveForward(double distance){//WORKS
        int ticks = encoders.encoderTicksDrive(distance);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(-ticks);
        rightMotor.setTargetPosition(-ticks);
        leftMotor.setPower(0.6);//.5
        rightMotor.setPower(0.6);//.5
        telemetry.addData("Ticks for DRIVE FORWARD: ", ticks);

        }
    public void driveForward(double distance, double power){//WORKS
        int ticks = encoders.encoderTicksDrive(distance);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(-ticks);
        rightMotor.setTargetPosition(-ticks);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void driveBackward(double distance){
        int ticks = encoders.encoderTicksDrive(distance);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);
        leftMotor.setPower(0.6);//.5
        rightMotor.setPower(0.6);
        telemetry.addData("Ticks for DRIVE BACKWARD: ", ticks);

    }
    public void slideLeft(double distance){
        int ticks = encoders.encoderTicksCenter(distance);
        centerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        centerDrive.setTargetPosition(ticks);
        centerDrive.setPower(0.2);//.3
        telemetry.addData("Ticks for SLIDE LEFT: ", ticks);
    }

    public void slideRight(double distance){
        int ticks = encoders.encoderTicksCenter(distance);
        centerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        centerDrive.setTargetPosition(-ticks);
        centerDrive.setPower(0.2);//.3
        telemetry.addData("Ticks for SLIDE RIGHT: ", ticks);
    }
    public void rotateRight(double distance){//WORKS
        int ticks = encoders.encoderTicksDrive(distance);
        telemetry.addData("Rotate", ticks);
        telemetry.update();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(ticks);
        leftMotor.setPower(0.5);
        sleep(2000);
    }
    public void rotateLeft(double distance){//WORKS
        int ticks = encoders.encoderTicksDrive(distance);
        telemetry.addData("Rotate", ticks);
        telemetry.update();
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setTargetPosition(ticks);
        rightMotor.setPower(0.5);
        sleep(2000);
    }
        }