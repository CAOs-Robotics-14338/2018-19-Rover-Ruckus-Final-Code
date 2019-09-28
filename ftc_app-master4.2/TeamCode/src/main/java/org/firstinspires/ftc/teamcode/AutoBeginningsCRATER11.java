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

@Autonomous(name="Crater Auto 11", group="Pushbot")//CHANGED ORDER OF SAMPLING MINERALS
//@Disabled
public class AutoBeginningsCRATER11 extends LinearOpMode {
    //Declaring objects to be able to control the different systems later

    //limit switch object
    DigitalChannel digitalTouch;  // Hardware Device Object

    //drive system motor objects
    private DcMotor centerDrive = null;
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    //climb system objects
    private DcMotor climbMotor = null;
    double climbPower;

    //Marker deployment system objects from MarkerDeployment class
    MarkerDeployment deploy = new MarkerDeployment();
    // servo object for marker servo
    Servo marker;


    //Encoder object from encoder class to be able to use encoder math
    Encoders encoders = new Encoders();

    //Gold mineral detector object from the Vision Class
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

        detector.enable(); //enable the detector based on the parameters that were given above

        //Gets the value of the motor port of drive motors connected to Expansion Hub
        leftMotor  = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");

        //sets the motor direction so the motors run the same way.
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.REVERSE);

//Limit Switch
        //gets location of the limit switch from expansion hub
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT); //sets the limit switch to receive input

        marker  = hardwareMap.servo.get("deploy_marker");//gets location of the servo on expansion hub
        //locking marker into place
        deploy.setMarkerPosition(marker, 0.75);

        //gets location of climb motor on expansion hub, sets direction, and sets the power of the motor.
        climbMotor = hardwareMap.get(DcMotor.class, "climb_motor");
        climbMotor.setDirection(DcMotor.Direction.REVERSE);
        climbPower = -0.75;//-0.5

        waitForStart(); //wait for the start button to be pressed to signal the beginning of autonomous.

        String pathTaken = ""; //path taken string variable for remembering the sampling path that was taken.

        while (opModeIsActive()) {
            /* AUTO STEPS:
            --Start in collapsed state with climb collapsed
            --Auto mode starts and robot descends and goes sideways to detach from lander.
            --Samples Minerals
            -- Goes to depot and deploys marker
            -- Makes a beeline to the crater.
             */
            boolean descend = true;
            // descend from lander

                while(descend) {//while the robot should still descend
                telemetry.addData("Digital Touch", "Is Not Pressed"); //print to drive station
                climbMotor.setPower(climbPower);//set climb motor to climb power
                telemetry.update();//update the telemetry so the message will show up
                if (digitalTouch.getState()){//see if the limit switch is pressed
                    climbMotor.setPower(0);//if limit switch pressed stop climb motor
                    descend = false;//set descend to false to stop loop
                }
            }

            sleep(250);
            //slide sideways to get unattached
            slideLeft(2.5);
            sleep(500);//1000
            //move the climber hook down to avoid the limit switch issue
            /* We were experiencing issues that if the limit switch was pressed then expansion hub had trouble with power
             * To fix this issue I coded autonomous to lower the climber hook so the limit switch wouldn't be press that way there
             * would be no problems with it being pressed.
             * Basically, I just coded it to do the opposite of what it did for descending
             * */
            boolean rescend = true; //rescend the climber hook. Bring climber hook down so the limit switch is not pressed.
            while(rescend){//while rescending climber hook
                telemetry.addData("Digital Touch", "Is Pressed");//add data to drive station
                climbMotor.setPower(-climbPower);//set the motor to reversed power
                telemetry.update(); //update telemetry to add data
                if(digitalTouch.getState() == false){ //if the limit switch is not pressed
                    climbMotor.setPower(0); //stop the motor
                    rescend = false; //set rescending to false to exit the loop
                }
            }

            //lower climber hook 2 inches
            // gos forward to sample
            driveForward(15);
            sleep(1000);
            //
            //\\\SAMPLING ///\\\///////////////////////////////////////////////////////////////////////////////////////////////////
            /*
            For the sampling for crater side, I changed the order of sampling is different from that of the Depot side.
            I changed this because it would be more effective and the robot would have a chance to do all the tasks.
            Before I change the order of the sampling, there would not be enough time for the robot to everything and it would barely
            make it to the crater.
            I decided that if I made the sampling go to the right first instead of the left then the robot would have a better chance
            to make it the farther distance. With the more time from testing the farther distance first, the robot was able to then get
            the marker in the depot. 
             */
            if (detector.isFound()) { //if the detector can see a gold mineral
                //CAN SEE GOLD
                telemetry.addData("Sees the mineral", " ");
                telemetry.update();
                sleep(250);
                if (detector.getAligned()) { //is robot aligned with gold mineral?
                    telemetry.addData("Gold Mineral is in center --> move accordingly", detector.getXPosition());
                    telemetry.addData("Move forward", ' ');
                    pathTaken = "CENTER"; //set path taken so it remembers what sampling path it took
                    driveForward(12);//drive forward to knock mineral off of mark
                    sleep(1000);//3000
                } else { //the robot is not aligned with gold mineral
                    while (!detector.getAligned()) {//while robot is not aligned with mineral
                        telemetry.addData("Not Aligned", " ");//print not aligned
                        telemetry.update();
                        align();//align with gold mineral
                    }
                    driveForward(8);//drive forward to knock mineral off of mark
                    sleep(800);
                }
            } else { //Since the detector can not see a gold mineral, == See if gold is on the right
////See if gold is on the RIGHT
                telemetry.addData("Not Found", " ");
                telemetry.update();
                //sleep(2000);
                slideRight(19);//21  //15//slide to the right to see if there is a gold mineral
                sleep(2500);//2000
                if (detector.isFound()) {//if it sees a gold mineral
                    telemetry.addData("Found on Right", " ");
                    pathTaken = "RIGHT";//set path taken to right so it remembers what path it took
                    telemetry.update(); //update telemetry
                    while (!detector.getAligned()){//while the robot is not aligned with gold
                        align();} //align with gold mineral
                    sleep(2000);
                    driveForward(8);//12 //drive forward to knock mineral off of tape
                    sleep(1000);//3
                } else {//The detector could not find gold mineral on right so it goes to the left
                    telemetry.addData("Not found on RIGHT", " going LEFT");
                    telemetry.update();
                    //sleep(2000);
                    slideLeft(34); //30 //slides left
                    sleep(4450);//4500
                    pathTaken="LEFT";//set path taken to remember the path
                    if (detector.isFound()) { //if the gold mineral is seen
                        telemetry.addData("Found on LEFT", " ");
                        telemetry.update();
                       /* while (!detector.getAligned()){//while robot not aligned with gold mineral
                            align();} //align with gold mineral*/
                        sleep(1000);//
                        driveForward(10);//14////drive forward to knock mineral off of place
                        sleep(1500);//
                    }
                    else{//in the apocalyptic case that the mineral is not found
                        driveForward(8);////drive forward
                        sleep(850);
                    }
                }
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //Sampling Over
            //Get position taken and get to depot
            //drive toward depot framework, deploy marker and then head to crater

            telemetry.addData("Path Taken: ", pathTaken);//add the pathTaken data to see what the value of pathTaken is
            telemetry.update();
            sleep(100);

            //Frame work for second half of autonomous
            //Based on what the pathTaken value is the program with run the corresponding
            /* The methods like driveForward move the robot like stated for that number of inches
             */

            if (pathTaken.equals("RIGHT")){
                driveBackward(11);//13  //15 //13
                sleep(1000);// 1400  //1500
                turnLeft(95);//93 // 98 //95  //115 // 97
                sleep(3000);//3000 //2100
                //Distance forward depends on position and path taken
                driveForward(44);
                sleep(2600);//3600
                turnLeft(60);//45
                sleep(1000);//1300
                slideRight(24);//18
                sleep(2000);
                driveForward(54, .8);//.7//48
                sleep(1800);//2800
                deploy.deployMarker(marker);//using deploy marker method from the MarkerDeployment Class to deploy marker
                sleep(500);
                driveBackward(66);//4
                sleep(6000);

            }
            else if (pathTaken.equals("LEFT")) {
                driveBackward(13);//13
                sleep(1200);//1500
                //issues begin
                turnLeft(95);// 97
                sleep(2400);//2500
               //Distance forward depends on position and path taken
                driveForward(20);
                sleep(2000);//2600
                turnLeft(60);//45
                sleep(1300);//1300
                slideRight(24);//18
                sleep(2000);
                driveForward(50, .8);//.7//48
                sleep(1600);//2800
                deploy.deployMarker(marker);//using deploy marker method from the MarkerDeployment Class to deploy marker
                sleep(500);
                driveBackward(66);//4
                sleep(6000);
            }

            else{// == "CENTER"
                driveBackward(15);//15
                sleep(1700);//1500
                turnLeft(95);// //110 97
                sleep(3000);//2100
              //Distance forward depends on position and path taken
                driveForward(35);
                sleep(2600);//3600
                turnLeft(60);//45
                sleep(1000);//1300
                slideRight(24);//18
                sleep(2000);
                driveForward(50, .8);//.7//48
                sleep(1600);//1800
                deploy.deployMarker(marker);//using deploy marker method from the MarkerDeployment Class to deploy marker
                sleep(500);
                driveBackward(70);//4
                sleep(6500);
            }
            sleep(500);
            stop();//stops the opmode so it ends the autonomous
        }



    }
    public void align() {
        if ((detector.getXPosition() > 0) && (detector.getXPosition() < detector.getAlignXMin())) {// if the gold mineral is on the left side of the alignment move left
            telemetry.addData("Gold Mineral is on the left --> move accordingly", detector.getXPosition());
            //new additions
            telemetry.addData("Move to the left", 300 - detector.getXPosition());
            slideLeft(1.0);
            sleep(250);//500
        } else if (detector.getXPosition() > detector.getAlignXMax()) { //if the gold mineral is on the right side of the alignment move right
            telemetry.addData("Gold Mineral is on the right --> move accordingly", detector.getXPosition());
            //new addition
            telemetry.addData("Move to the right", detector.getXPosition() - 300);
            slideRight(1.0);//1.0
            sleep(250);//500
        }
    }

    ///FUNCTIONS FOR DRIVING THE ROBOT AROUND WITH ENCODERS
    /*
    These methods use the encoder methods from the Encoder Class.
    See Encoder class for more descriptions of encoder math

    In order to get the encoders to work, you need to:

    -- Stop and Reset the Encoders
    -- Set them to run to a position
    -- Give them the target number of ticks the encoder needs to make
    -- Set the Power of the motor.
     */

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