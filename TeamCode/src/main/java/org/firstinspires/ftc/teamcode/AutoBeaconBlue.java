/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "AutoBeaconRed", group = "2017")

public class AutoBeaconBlue extends LinearOpMode {

    /* Declare OpMode members. */
    Harderware robot = new Harderware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final int DRIVE_LEFT = 0;
    static final int DRIVE_RIGHT = 1;
    static final int REVERSE = -1;
    static final int FORWARD = 1;

    static final int COLOR_THRESHOLD = 1000;
    String desiredColor = "BLUE";

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //MAIN TASK////////////



        shootCatapult();                        //Drive Forward, turn 180, shoot
        elevatorCycle();
        shootCatapult();

        encoderDriveStraight(.6, 12, 5);
        strafeDiagonalUSS(.8, 8);

        moveToLine(-0.3);
        colorPick(desiredColor);
        checkBeacon("BLUE");


        encoderDriveStraight(.6, 6, 3);
        moveToLine(0.3);
        colorPick(desiredColor);
        checkBeacon("BLUE");

        /**
         * ################################## MOVEMENT END #########################################
         */

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void elevatorCycle() {
        robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elevatorMotor.setPower(-0.5);
        while (!robot.bottomLimit.isPressed() && robot.elevatorMotor.getCurrentPosition() <= 2450);
        robot.elevatorMotor.setPower(0);
        sleep(1000);
        robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elevatorMotor.setPower(0.5);
        while (!robot.bottomLimit.isPressed() && robot.elevatorMotor.getCurrentPosition() <= 2450);
        robot.elevatorMotor.setPower(0);
    }

    public void pushButton(double speed, int timeout){


        // reset the timeout time and start motion.
        robot.leftFrontMotor.setPower(-speed); //these signs may change depending on the robot
        robot.leftBackMotor.setPower(speed);
        robot.rightFrontMotor.setPower(speed);
        robot.rightBackMotor.setPower(-speed);
        sleep(500);

        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (robot.leftFrontMotor.isBusy() || robot.leftBackMotor.isBusy() || robot.rightFrontMotor.isBusy() || robot.rightBackMotor.isBusy())) {
        telemetry.addData("Status", "Pressing Button");
        }
        endMotion();

        robot.leftFrontMotor.setPower(speed);
        robot.leftBackMotor.setPower(-speed);
        robot.rightFrontMotor.setPower(-speed);
        robot.rightBackMotor.setPower(speed);

        while (robot.ultraSonic.getUltrasonicLevel() < 8);
        endMotion();
    }

    public void checkBeacon(String targetColor){
       if(targetColor == "BLUE"){
            if(isColorBlue(robot.colorLeft) == isColorBlue(robot.colorRight)){
                if(isColorRed(robot.colorLeft)){
                    pushButton(0.5, 2);
                }
                }
           else{
                colorPick("BLUE");
            }
            }



        else if(targetColor == "RED"){
           if(isColorRed(robot.colorLeft) == isColorRed(robot.colorRight)){
               if(isColorBlue(robot.colorLeft)){
                   pushButton(0.5, 2);
               }
           }
           else{
               colorPick("RED");
           }
        }
    }


    public boolean isColorRed(ColorSensor colorSide){
        return (colorSide.red() > colorSide.blue() && colorSide.red() >= COLOR_THRESHOLD);
    }
    public boolean isColorBlue(ColorSensor colorSide){
        return (colorSide.blue() > colorSide.red() && colorSide.blue() >= COLOR_THRESHOLD);
    }

    public void colorPick(String targetColor){

        if (targetColor == "RED") {

            if(isColorBlue(robot.colorLeft)){
                    encoderDriveStraight(0.3, 1.5, 3);
                    pushButton(0.5, 2);
                    encoderDriveStraight(-0.3, 1.5,3 );
            }
            else {
                encoderDriveStraight(-0.3, 1.5, 3);
                pushButton(0.5, 2);
                encoderDriveStraight(0.3, 1.5,3 );

            }


        if (targetColor == "BLUE") {

            if (isColorRed(robot.colorLeft)) {
                encoderDriveStraight(0.3, 1.5, 3);
                pushButton(0.5, 2);
                encoderDriveStraight(-0.3, 1.5,3 );
            } else {
                encoderDriveStraight(-0.3, 1.5, 3);
                pushButton(0.5, 2);
                encoderDriveStraight(0.3, 1.5,3 );

            }
        }

    }
    }

    public void strafeDiagonalUSS(double speed, int distance){



        // reset the timeout time and start motion.
        runtime.reset();
        robot.leftFrontMotor.setPower(Math.abs(speed));
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(Math.abs(speed));
        while(robot.ultraSonic.getUltrasonicLevel() > distance);
        endMotion();

    }
    public void moveToLine(double speed){

        while(robot.leftODS.getLightDetected() < 2){
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));
        }
        endMotion();
    }
    public void startMotion(double speed){
        // Turn On RUN_TO_POSITION
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.leftFrontMotor.setPower(Math.abs(speed));
        robot.leftBackMotor.setPower(Math.abs(speed));
        robot.rightFrontMotor.setPower(Math.abs(speed));
        robot.rightBackMotor.setPower(Math.abs(speed));
    }

    public void endMotion(){
        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

        // Turn off RUN_TO_POSITION mode
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);
    }


    public void rotateTime(double speed, double time, int direction) {

        double currentRuntime = runtime.seconds();

        if (direction == 0){
            while (runtime.seconds() <= currentRuntime + time) {
                robot.leftFrontMotor.setPower(-speed);
                robot.leftBackMotor.setPower(-speed);
                robot.rightFrontMotor.setPower(speed);
                robot.rightBackMotor.setPower(speed);
            }
        } else if (direction == 1){
            while (runtime.seconds() <= currentRuntime + time) {
                robot.leftFrontMotor.setPower(speed);
                robot.leftBackMotor.setPower(speed);
                robot.rightFrontMotor.setPower(-speed);
                robot.rightBackMotor.setPower(-speed);
            }
        }


    }

    /*
     *  FORWARD Drive POSITIVE distance / REVERSE Drive NEGATIVE dsitance
     */
    public void encoderDriveStraight(double speed, double distance, double timeoutS) {

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // change mode and start motion
            startMotion(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Run to L(%7d) | R(%7d)", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at L(%7d) | R(%7d)", robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop motion and reset motor mode
            endMotion();
        }
    }

    /*
     *  Strafe Direction: 0 = left, 1 = right
     */

    public void encoderDriveStrafe(int direction, double speed, double distance, double timeoutS) {

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            if (direction == 0) {
                newLeftFrontTarget *= REVERSE;
                newLeftBackTarget *= FORWARD;
                newRightFrontTarget *= FORWARD;
                newRightBackTarget *= REVERSE;
            } else if (direction == 1) {
                newLeftFrontTarget *= FORWARD;
                newLeftBackTarget *= REVERSE;
                newRightFrontTarget *= REVERSE;
                newRightBackTarget *= FORWARD;
            }

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // change mode and start motion
            startMotion(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Run to L(%7d) | R(%7d)", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at L(%7d) | R(%7d)", robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop motion and reset motor mode
            endMotion();
        }
    }

    public void gyroTurn ( double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void shootCatapult() {
        robot.catapultMotor.setPower(1);
        sleep(4000);
        robot.catapultMotor.setPower(0);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftFrontSpeed;
        double leftBackSpeed;
        double rightFrontSpeed;
        double rightBackSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftFrontSpeed  = 0.0;
            rightFrontSpeed = 0.0;
            leftBackSpeed = 0.0;
            rightBackSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightFrontSpeed  = speed * steer;
            rightBackSpeed= speed * steer;
            leftFrontSpeed = -rightFrontSpeed;
            leftBackSpeed = -rightBackSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontMotor.setPower(leftFrontSpeed);
        robot.rightFrontMotor.setPower(rightFrontSpeed);
        robot.leftBackMotor.setPower(leftBackSpeed);
        robot.rightBackMotor.setPower(rightBackSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f:%5.2f:%5.2f", leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



}


