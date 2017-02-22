package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Holodrive Record", group="2017")
public class holoRecord extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    Harderware robot = new Harderware();    // gets hardware property configuration
                                            // from hardware class


    static final double VERTICAL_COEFF = 1.0;   // assign constants for altering values for
    static final double HORIZONTAL_COEFF = 1.0; // degrees of motion
    static  final double TURN_COEFF = 1.0;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark NeverRest Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP

    double leftFrontPower;
    double leftBackPower;          //declare power value variables outside of main loop
    double rightFrontPower;
    double rightBackPower;

    int[] posArray = new int[3];

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);        //get hardware configuration from phones

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        int controlToggle = -1;    // either 1 or -1, used to reverse "front" orientation
                                    // of robot




        // loop until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            float j1x = gamepad1.left_stick_x;
            float j1y = -gamepad1.left_stick_y;     //get values from joysticks
            float j2x = gamepad1.right_stick_x;


            //multiplies each joystick value by corresponding matrix value for each degree of motion
            //
            //
            //   horizontal:              |     vertical:                     |     rotation:
            //      -1/√2  ,    -1/√2     |          -1/√2     ,     1/√2     |         1   ,    1
            //                            |                                   |
            //      1/√2   ,     1/√2     |          -1/√2      ,    1/√2     |         1   ,    1
            //                            |
            //  uses rounded value of 1/√2 as calculating it takes too much processing power

            if (controlToggle == -1){
                leftFrontPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
                rightFrontPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
                leftBackPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
                rightBackPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
            } else if (controlToggle == 1) {
                leftFrontPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
                rightFrontPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
                leftBackPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
                rightBackPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
            }




            Range.clip(leftFrontPower, -1, 1);
            Range.clip(rightFrontPower, -1, 1);  // clips values as motor power cannot
            Range.clip(leftBackPower, -1, 1);    // go above 1 or below -1c v
            Range.clip(rightBackPower, -1, 1);

            robot.leftFrontMotor.setPower(leftFrontPower);
            robot.rightFrontMotor.setPower(rightFrontPower);  //assigns finalized values to motors
            robot.leftBackMotor.setPower(leftBackPower);
            robot.rightBackMotor.setPower(rightBackPower);


//            /////////////Single Button Controls/////////////
//
//            if (gamepad2.right_bumper){
//                robot.catapultMotor.setPower(1);
//            } else {
//                robot.catapultMotor.setPower(0);
//            }
//
//
//            if (gamepad2.dpad_up){
//                robot.liftMotor.setPower(1);
//            } else if (gamepad2.dpad_down){
//                robot.liftMotor.setPower(-1);
//            } else {
//                robot.liftMotor.setPower(0);
//            }
//
//            if (gamepad2.a){
//                robot.holdMotor.setPower(1.0);
//            } else if (gamepad2.b) {
//                robot.holdMotor.setPower(-1.0);
//            } else {
//                robot.holdMotor.setPower(0);
//            }
//
//            if (gamepad2.left_bumper) {
//                robot.basketServo.setPosition(1);
//            }

        //RECORDING BUTTONS
            if(gamepad1.x) {
                startStep();
                sleep(250);
            }

            if (gamepad1.y) {
                endStep();
                sleep(250);
            }

            //TELEMETRY
            telemetry.addData("Control Direction", controlToggle);  //writes value of controlToggle
            idle();                                                 // to telemetry

            telemetry.addLine("Joystick Data")
                    .addData("Joy1 X: ", j1x)
                    .addData("Joy1 Y: ", j1y)
                    .addData("Joy2 X: ", j2x);

            telemetry.addLine("leftFront")
                    .addData("Power: ", leftFrontPower)
                    .addData("Ticks: ", posArray[0])
                    .addData("Inches: ", posArray[0] / COUNTS_PER_INCH);

            telemetry.addLine("rightFront")
                    .addData("Power: ", rightFrontPower)
                    .addData("Ticks: ", posArray[1])
                    .addData("Inches: ", posArray[1] / COUNTS_PER_INCH);

            telemetry.addLine("leftBack")
                    .addData("Power: ", leftBackPower)
                    .addData("Ticks: ", posArray[2])
                    .addData("Inches: ", posArray[2] / COUNTS_PER_INCH);

            telemetry.addLine("rightBack")
                    .addData("Power: ", rightBackPower)
                    .addData("Ticks: ", posArray[3])
                    .addData("Inches: ", posArray[3] / COUNTS_PER_INCH);

            telemetry.update();
        }

    }

        public void startStep(){

            //reset encoders on all motors
            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        public void endStep() {

            posArray[0] = robot.leftFrontMotor.getCurrentPosition();
            posArray[1] = robot.rightFrontMotor.getCurrentPosition();
            posArray[2] = robot.leftBackMotor.getCurrentPosition();
            posArray[3] = robot.rightBackMotor.getCurrentPosition();

        }

}
