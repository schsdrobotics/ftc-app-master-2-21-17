//package org.firstinspires.ftc.teamcode;
//
//import android.app.Activity;
//import android.view.View;
//
//import com.qualcomm.hardware.adafruit.BNO055IMU;
//import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
//import com.qualcomm.robotcore.hardware.DigitalChannelController;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.UltrasonicSensor;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//import java.util.Locale;
//
//import static org.firstinspires.ftc.teamcode.AutonoLine.P_DRIVE_COEFF;
//import static org.firstinspires.ftc.teamcode.ColorServo.LED_CHANNEL;
//
//
//@Autonomous(name = "Autonomous2017", group = "2017")
//public class Auto2017 extends LinearOpMode {
//
//        /* Declare OpMode members. */
//        Harderware robot = new Harderware();   // Use a Pushbot's hardware
//        private ElapsedTime runtime = new ElapsedTime();
//        static final double COUNTS_PER_MOTOR_REV = 1120;
//        static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
//        static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
//        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
//        static final double DRIVE_SPEED = 0.6;
//        static final int DRIVE_LEFT = 0;
//        static final int DRIVE_RIGHT = 1;
//        static final int REVERSE = -1;
//        static final int FORWARD = 1;
//         static final double MAX_POS = 1.0;     // Maximum rotational position
//        static final double MIN_POS = 0.0;     // Minimum rotational position
//        static final double STRAFE_SPEED = 0.8;
//        static final double SENSITIVITY = 2.3;
//    // Define class members
//    Servo servo;
//    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
//
//    ColorSensor sensorRGB;
//    DeviceInterfaceModule cdim;
//    OpticalDistanceSensor ODS;
//    BNO055IMU imu;
//    UltrasonicSensor USS;
//    public void gyroDrive ( double speed,
//
//                                 double inches,
//
//                                 double angle) {
//
//
//
//
//        double  maxFront;
//
//        double maxBack;
//
//        double combinedMax;
//
//        double  error;
//
//        double  steer;
//
//        double  leftFrontSpeed;
//
//        double  rightFrontSpeed;
//
//        double leftBackSpeed;
//
//        double rightBackSpeed;
//
//        int newLeftBackTarget;
//
//        int newLeftFrontTarget;
//
//        int newRightBackTarget;
//
//        int newRightFrontTarget;
//
//
//
//
//
//
//
//        // Ensure that the opmode is still active
//
//        if (opModeIsActive()) {
//
//
//
//
//            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//
//            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//
//            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//
//            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//
//
//
//
//            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
//
//            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
//
//            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
//
//            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
//
//            int avgCurrent = (robot.leftBackMotor.getCurrentPosition() + robot.leftFrontMotor.getCurrentPosition() + robot.rightBackMotor.getCurrentPosition() + robot.rightFrontMotor.getCurrentPosition()) / 4;
//
//
//
//
//            // Turn On RUN_TO_POSITION
//
//            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//
//
//
//
//
//            // reset the timeout time and start motion.
//
//            runtime.reset();
//
//            robot.rightFrontMotor.setPower(Math.abs(speed));
//
//            robot.leftFrontMotor.setPower(Math.abs(speed));
//
//            robot.leftBackMotor.setPower(Math.abs(speed));
//
//            robot.rightBackMotor.setPower(Math.abs(speed));
//
//
//
//
//            // keep looping while we are still active, and BOTH motors are running.
//
//            while (opModeIsActive() &&
//
//                    (robot.leftBackMotor.isBusy() && robot.leftFrontMotor.isBusy() && robot.rightBackMotor.isBusy() && robot.rightFrontMotor.isBusy())) {
//
//
//
//
//                // adjust relative speed based on heading error.
//
//                error = getError(angle);
//
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//
//
//
//                // if driving in reverse, the motor correction also needs to be reversed
//
//                if (inches < 0)
//
//                    steer *= -1.0;
//
//
//
//
//                leftFrontSpeed = speed - steer;
//
//                leftBackSpeed = speed - steer;
//
//                rightFrontSpeed = speed + steer;
//
//                rightBackSpeed = speed + steer;
//
//
//
//
//                // Normalize speeds if any one exceeds +/- 1.0;
//
//                maxFront = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
//
//                maxBack = Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed));
//
//                combinedMax = Math.max(maxFront, maxBack);
//
//                if (combinedMax > 1.0)
//
//                {
//
//                    leftFrontSpeed /= combinedMax;
//
//                    leftBackSpeed /= combinedMax;
//
//                    rightFrontSpeed /= combinedMax;
//
//                    rightBackSpeed /= combinedMax;
//
//
//
//
//                }
//
//
//
//
//                robot.rightFrontMotor.setPower(Math.abs(rightFrontSpeed));
//
//                robot.leftFrontMotor.setPower(Math.abs(leftFrontSpeed));
//
//                robot.leftBackMotor.setPower(Math.abs(leftBackSpeed));
//
//                robot.rightBackMotor.setPower(Math.abs(rightBackSpeed));
//
//
//
//
//                // Display drive status for the driver.
//
//                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//
//                telemetry.addData("Target",  "%7d:%7d:%7d:7d",      newLeftBackTarget,  newRightBackTarget, newRightFrontTarget, newLeftFrontTarget);
//
//                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.leftFrontMotor.getCurrentPosition(),
//
//                        robot.rightFrontMotor.getCurrentPosition(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
//
//                telemetry.addData("Speed",   "%5.2f:%5.2f:%5.2f:%5.2f",  leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);
//
//                telemetry.update();
//
//            }
//
//
//
//
//            // Stop all motion;
//
//            robot.leftBackMotor.setPower(0);
//
//            robot.leftFrontMotor.setPower(0);
//
//            robot.rightFrontMotor.setPower(0);
//
//            robot.rightBackMotor.setPower(0);
//
//
//
//
//            robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//
//
//            // Turn off RUN_TO_POSITION
//
//            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//
//    }
//    @Override
//    public void runOpMode() {
//
//        /*
//         * Initialize the drive system variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
//
//
//        // Set up the parameters with which we will use our IMU. Note that integration
//
//        // algorithm here just reports accelerations to the logcat log; it doesn't actually
//
//        // provide positional information.
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//
//        parameters.calibrationDataFile = "AdafruitIMUCalibration2.json"; // see the calibration sample opmode
//
//        parameters.loggingEnabled = true;
//
//        parameters.loggingTag = "IMU";
//
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//
//
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//
//        // and named "imu".
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters);
//
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Resetting Encoders");
//        telemetry.update();
//
//        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        idle();
//
//        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Path0", "L(%7d) | R(%7d)", robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
//        telemetry.update();
//        // Connect to servo (Assume PushBot Left Hand)
//        // Change the text in quotes to match any servo name on your robot.
//        servo = hardwareMap.servo.get("servo");
//        USS = hardwareMap.ultrasonicSensor.get(("uss"));
//        // hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F, 0F, 0F};
//
//        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//
//        // get a reference to the RelativeLayout so we can change the background
//        // color of the Robot Controller app to match the hue detected by the RGB sensor.
//        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
//
//        // bPrevState and bCurrState represent the previous and current state of the button.
//        boolean bPrevState = false;
//        boolean bCurrState = false;
//        // bLedOn represents the state of the LED.
//        boolean bLedOn = false;
//
//        // get a reference to our DeviceInterfaceModule object.
//        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
//
//        // set the digital channel to output mode.
//        // remember, the Adafruit sensor is actually two devices.
//        // It's an I2C sensor and it's also an LED that can be turned on or off.
//        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
//
//        // get a reference to our ColorSensor object.
//        sensorRGB = hardwareMap.colorSensor.get("color");
//
//        // turn the LED on in the beginning, just so user will know that the sensor is active.
//       // cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//
//            /*
//            ##########  PLAN ##########
//
//            1. shoot 2 balls
//
//            2. Drive Forward ___ inches
//
//
//            3. Rotate to initial heading
//
//
//            4. gyro drive up until line
//
//            5. Strafe close to beacons
//
//            6. Detect color + plus position arm
//
//            7. Strafe in to button and strafe back out
//
//            8.  Reverse into other line
//
//            9. Repeat steps #4-#6
//
//            10. strafe into cap ball/park (??)
//
//            */
//
//
//
//
//        //####### MAIN TASK #########
//
//        shootParticle();
//        gyroDrive(0.5, 36, getCurrentHeading());
//        strafeLeft();
//        for(int i = 0; i<2; i++){
//            gyroDriveODS(0.5, 50, getCurrentHeading());
//            strafeLeft();
//            colorPick("RED");
//            strafeRight();
//        }
//
//
//
//
//
//
//
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//    }
//
//
//
//
//    public double getCurrentHeading(){
//        Orientation currentAngles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//        return  Double.parseDouble(formatAngle(currentAngles.angleUnit, currentAngles.firstAngle));
//
//    }
//    public double getError(double targetAngle) {
//
//
//
//
//        double robotError;
//
//        double currentHeading = getCurrentHeading();
//
//
//
//
//        // calculate error in -179 to +180 range  (
//
//        robotError = targetAngle - currentHeading;
//
//        while (robotError > 180) robotError -= 360;
//
//        while (robotError <= -180) robotError += 360;
//
//        return robotError;
//
//    }
//
//
//
//
//    /**
//
//     * returns desired steering force.  +/- 1 range.  +ve = steer left
//
//     *
//
//     * @param error  Error angle in robot relative degrees
//
//     * @param PCoeff Proportional Gain Coefficient
//
//     * @return
//
//     */
//
//    public double getSteer(double error, double PCoeff) {
//
//        return Range.clip(error * PCoeff, -1, 1);
//
//    }
//    public void gyroDriveODS(double speed,
//
//                             double inches,
//
//                             double angle) {
//
//
//
//
//        double maxFront;
//
//        double maxBack;
//
//        double combinedMax;
//
//        double error;
//
//        double steer;
//
//        double leftFrontSpeed;
//
//        double rightFrontSpeed;
//
//        double leftBackSpeed;
//
//        double rightBackSpeed;
//
//        int newLeftBackTarget;
//
//        int newLeftFrontTarget;
//
//        int newRightBackTarget;
//
//        int newRightFrontTarget;
//
//
//
//
//
//
//
//        // Ensure that the opmode is still active
//
//        if (opModeIsActive()) {
//
//
//
//
//            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//
//            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//
//            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//
//            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//
//
//
//
//            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
//
//            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
//
//            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
//
//            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
//
//            int avgCurrent = (robot.leftBackMotor.getCurrentPosition() + robot.leftFrontMotor.getCurrentPosition() + robot.rightBackMotor.getCurrentPosition() + robot.rightFrontMotor.getCurrentPosition()) / 4;
//
//
//
//
//            // Turn On RUN_TO_POSITION
//
//            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//
//
//
//
//
//            // reset the timeout time and start motion.
//
//            runtime.reset();
//
//            robot.rightFrontMotor.setPower(Math.abs(speed));
//
//            robot.leftFrontMotor.setPower(Math.abs(speed));
//
//            robot.leftBackMotor.setPower(Math.abs(speed));
//
//            robot.rightBackMotor.setPower(Math.abs(speed));
//
//
//
//
//            // keep looping while we are still active, and BOTH motors are running.
//
//            while (opModeIsActive() &&
//
//                    (robot.leftBackMotor.isBusy() && robot.leftFrontMotor.isBusy() && robot.rightBackMotor.isBusy() && robot.rightFrontMotor.isBusy())) {
//
//
//
//
//                // adjust relative speed based on heading error.
//
//                error = getError(angle);
//
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//
//
//
//                // if driving in reverse, the motor correction also needs to be reversed
//
//                if (inches < 0)
//
//                    steer *= -1.0;
//
//
//
//
//                leftFrontSpeed = speed - steer;
//
//                leftBackSpeed = speed - steer;
//
//                rightFrontSpeed = speed + steer;
//
//                rightBackSpeed = speed + steer;
//
//
//
//
//                // Normalize speeds if any one exceeds +/- 1.0;
//
//                maxFront = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
//
//                maxBack = Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed));
//
//                combinedMax = Math.max(maxFront, maxBack);
//
//                if (combinedMax > 1.0) {
//
//                    leftFrontSpeed /= combinedMax;
//
//                    leftBackSpeed /= combinedMax;
//
//                    rightFrontSpeed /= combinedMax;
//
//                    rightBackSpeed /= combinedMax;
//
//
//
//
//                }
//
//
//
//
//                robot.rightFrontMotor.setPower(Math.abs(rightFrontSpeed));
//
//                robot.leftFrontMotor.setPower(Math.abs(leftFrontSpeed));
//
//                robot.leftBackMotor.setPower(Math.abs(leftBackSpeed));
//
//                robot.rightBackMotor.setPower(Math.abs(rightBackSpeed));
//
//
//
//
//                // Display drive status for the driver.
//                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
//                telemetry.addData("Target", "%7d:%7d:%7d:7d", newLeftBackTarget, newRightBackTarget, newRightFrontTarget, newLeftFrontTarget);
//                telemetry.addData("Actual", "%7d:%7d:%7d:%7d", robot.leftFrontMotor.getCurrentPosition(),
//                        robot.rightFrontMotor.getCurrentPosition(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
//                telemetry.addData("Speed", "%5.2f:%5.2f:%5.2f:%5.2f", leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);
//                if (ODS.getLightDetected() < 2.3*SENSITIVITY) {
//                    telemetry.addData("MAT DETECTED", ODS.getLightDetected());
//                }
//                else if(ODS.getLightDetected() > 2.3*SENSITIVITY){
//                    telemetry.addData("TAPE DETECTED", ODS.getLightDetected());
//                    break;
//                }
//                telemetry.update();
//            }
//            // Stop all motion;
//            robot.leftBackMotor.setPower(0);
//
//            robot.leftFrontMotor.setPower(0);
//
//            robot.rightFrontMotor.setPower(0);
//
//            robot.rightBackMotor.setPower(0);
//
//
//
//
//            robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//
//
//            // Turn off RUN_TO_POSITION
//
//            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//        sleep(200);
//
//    }
//        public void shootParticle(){
//
//        }
//
//        public void strafeLeft(){
//            while(USS.getUltrasonicLevel() > 20){
//                robot.leftFrontMotor.setPower(-STRAFE_SPEED);
//                robot.leftBackMotor.setPower(STRAFE_SPEED);
//                robot.rightBackMotor.setPower(-STRAFE_SPEED);
//                robot.rightFrontMotor.setPower(STRAFE_SPEED);
//            }
//            robot.leftFrontMotor.setPower(0);
//            robot.leftBackMotor.setPower(0);
//            robot.rightBackMotor.setPower(0);
//            robot.rightFrontMotor.setPower(0);
//            sleep(200);
//        }
//
//        public void strafeRight(){
//            while(USS.getUltrasonicLevel() < 8){
//                robot.leftFrontMotor.setPower(STRAFE_SPEED);
//                robot.leftBackMotor.setPower(-STRAFE_SPEED);
//                robot.rightBackMotor.setPower(STRAFE_SPEED);
//                robot.rightFrontMotor.setPower(-STRAFE_SPEED);
//            }
//            robot.leftFrontMotor.setPower(0);
//            robot.leftBackMotor.setPower(0);
//            robot.rightBackMotor.setPower(0);
//            robot.rightFrontMotor.setPower(0);
//            sleep(200);
//        }
//
//        public void colorPick(String color){
//
//
//            if (color == "RED") {
//
//                if(sensorRGB.red() > sensorRGB.blue() && sensorRGB.red() >= 1000){
//                    position = 0.75;
//                } else if(sensorRGB.red() < sensorRGB.blue() && sensorRGB.red() >= 1000){
//                    position = 0.25;
//                }
//            } else if (color == "BLUE") {
//
//                if (sensorRGB.red() < sensorRGB.blue() && sensorRGB.red() >= 1000) {
//                    position = 0.75;
//                } else if (sensorRGB.red() > sensorRGB.blue() && sensorRGB.red() >= 1000) {
//                    position = 0.25;
//                }
//            }
//
//
//
//
//            servo.setPosition(position);
//            sleep(500);
//        }
//
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
//
//    String formatDegrees(double degrees){
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }
//    }
//
