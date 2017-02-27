
/*
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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * <p>
 * <p>
 * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the robot is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 * <p>
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Auto4", group = "Pushbot")
//@Disabled
public class Auto4 extends LinearOpMode {

    /* Declare OpMode members. */
    Harderware robot = new Harderware();   // Use a Pushbot's hardware
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark NeverRest Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.4;     // Nominal half speed for better accuracy.

    static final int REVERSE = -1;
    static final int FORWARD = 1;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.5;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    double initialHeading;
    double currentHeading;
    Orientation initAngles;
    Orientation currentAngles;


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration2.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Wait until we're told to go
        waitForStart();

        initialHeading = getInitialHeading();


        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        //set current encoder position 0
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        //Run at target velocity
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        telemetry.addData(">", "Robot Ready.");    //
//        telemetry.addData("getHead", getCurrentHeading());
        telemetry.addData("initHead", getInitialHeading());
        telemetry.update();

//MAIN TASK
        gyroDrive(1.0, 48, initialHeading);
        sleep(500);
        rotateTime(.8, 5, 1);
        endMotion();

    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed  Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param inches Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double inches,
                          double angle) {

        double maxFront;
        double maxBack;
        double combinedMax;
        double error;
        double steer;
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftBackSpeed;
        double rightBackSpeed;
        int newLeftBackTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            int avgCurrent = (robot.leftBackMotor.getCurrentPosition() + robot.leftFrontMotor.getCurrentPosition() + robot.rightBackMotor.getCurrentPosition() + robot.rightFrontMotor.getCurrentPosition()) / 4;

            // Turn On RUN_TO_POSITION
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftBackMotor.isBusy() && robot.leftFrontMotor.isBusy() && robot.rightBackMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (inches < 0)
                    steer *= -1.0;

                leftFrontSpeed = speed - steer;
                leftBackSpeed = speed - steer;
                rightFrontSpeed = speed + steer;
                rightBackSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                maxFront = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
                maxBack = Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed));
                combinedMax = Math.max(maxFront, maxBack);
                if (combinedMax > 1.0) {
                    leftFrontSpeed /= combinedMax;
                    leftBackSpeed /= combinedMax;
                    rightFrontSpeed /= combinedMax;
                    rightBackSpeed /= combinedMax;

                }

                robot.rightFrontMotor.setPower(Math.abs(rightFrontSpeed));
                robot.leftFrontMotor.setPower(Math.abs(leftFrontSpeed));
                robot.leftBackMotor.setPower(Math.abs(leftBackSpeed));
                robot.rightBackMotor.setPower(Math.abs(rightBackSpeed));

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d:%7d:7d", newLeftBackTarget, newRightBackTarget, newRightFrontTarget, newLeftFrontTarget);
                telemetry.addData("Actual", "%7d:%7d:%7d:%7d", robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f:%5.2f:%5.2f", leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        double startHeading = getCurrentHeading();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, startHeading, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        telemetry.addData("startHeading", startHeading);
        telemetry.update();
        sleep(5000);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
////    public void gyroHold(double speed, double angle, double holdTime) {
////
////        ElapsedTime holdTimer = new ElapsedTime();
////
////        // keep looping while we have time remaining.
////        holdTimer.reset();
////        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
////            // Update telemetry & Allow time for other processes to run.
////            onHeading(speed, angle, P_TURN_COEFF);
////            telemetry.update();
////        }
//
//        // Stop all motion;
//        robot.leftBackMotor.setPower(0);
//        robot.leftFrontMotor.setPower(0);
//        robot.rightFrontMotor.setPower(0);
//        robot.rightBackMotor.setPower(0);
//    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double startHeading, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftFrontSpeed;
        double leftBackSpeed;
        double rightFrontSpeed;
        double rightBackSpeed;

        double offsetAngle = angle + startHeading;
        offsetAngle = Double.parseDouble(formatAngle(currentAngles.angleUnit, offsetAngle));

        telemetry.addData("offsetAngle", offsetAngle);


        // determine turn power based on +/- error
        error = getError(offsetAngle);


        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftFrontSpeed = 0.0;
            rightFrontSpeed = 0.0;
            leftBackSpeed = 0.0;
            rightBackSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightFrontSpeed = speed * steer;
            rightBackSpeed = speed * steer;
            leftFrontSpeed = -rightFrontSpeed;
            leftBackSpeed = -rightBackSpeed;
        }

//        // Send desired speeds to motors.
//        robot.leftFrontMotor.setPower(leftFrontSpeed);
//        robot.rightFrontMotor.setPower(rightFrontSpeed);
//        robot.leftBackMotor.setPower(leftBackSpeed);
//        robot.rightBackMotor.setPower(rightBackSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", offsetAngle);
        telemetry.addData("Current", "%5.2f", currentHeading);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f:%5.2f:%5.2f", leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        currentHeading = getCurrentHeading();

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - currentHeading;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getInitialHeading() {
        initAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return Double.parseDouble(formatAngle(initAngles.angleUnit, initAngles.firstAngle));

    }

    public double getCurrentHeading() {
        currentAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return Double.parseDouble(formatAngle(currentAngles.angleUnit, currentAngles.firstAngle));

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public void startMotion(double speed) {
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

    public void endMotion() {
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

}