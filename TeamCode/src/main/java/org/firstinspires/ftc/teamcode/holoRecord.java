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


    private static final double VERTICAL_COEFF = 1.0;   // assign constants for altering values for
    private static final double HORIZONTAL_COEFF = 1.0; // degrees of motion
    private static  final double TURN_COEFF = 1.0;

    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark NeverRest Motor Encoder
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    private double leftFrontPower;
    private double leftBackPower;          //declare power value variables outside of main loop
    private double rightFrontPower;
    private double rightBackPower;

    private int leftFrontPosition;
    private int rightFrontPosition;
    private int leftBackPosition;
    private int rightBackPosition;

    int controlToggle = -1;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);        //get hardware configuration from phones

        robot.leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);  //set all to forward to
        robot.rightBackMotor.setDirection(DcMotor.Direction.FORWARD);   //allow for correct power
        robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);        //assignments
        robot.holdMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.catapultMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
            // either 1 or -1, used to reverse "front" orientation
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


            /////////////Single Button Controls/////////////

            if (gamepad2.right_bumper){
                robot.catapultMotor.setPower(1);
            } else {
                robot.catapultMotor.setPower(0);
            }




            if (gamepad2.dpad_up){
                robot.liftMotor.setPower(1);
            } else if (gamepad2.dpad_down){
                robot.liftMotor.setPower(-1);
            } else {
                robot.liftMotor.setPower(0);
            }

            if (gamepad2.a){
                robot.holdMotor.setPower(1.0);
            } else if (gamepad2.b) {
                robot.holdMotor.setPower(-1.0);
            } else {
                robot.holdMotor.setPower(0);
            }


            if(gamepad1.start){
                controlToggle = -1*controlToggle;
                sleep(250);
            }

        //RECORDING BUTTONS
            if(gamepad1.x) {
                startStep();
                sleep(250);
            }

            if (gamepad1.y) {
                endStep();
                sleep(250);
            }


        }

    }


        public void startStep(){

            //reset encoders on all motors
            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            leftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            rightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            leftBackPosition = robot.leftBackMotor.getCurrentPosition();
            rightBackPosition = robot.rightBackMotor.getCurrentPosition();

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        public void endStep() {
            telemetry.addLine("Joystick Data")
                    .addData("Joy1 X: ", gamepad1.left_stick_x)
                    .addData("Joy1 Y: ", -gamepad1.left_stick_y)
                    .addData("Joy2 X: ", gamepad1.right_stick_x);

            telemetry.addLine("leftFront")
                    .addData("Ticks: ", leftFrontPosition)
                    .addData("Inches: ", leftFrontPosition / COUNTS_PER_INCH);

            telemetry.addLine("rightFront")
                    .addData("Ticks: ", rightFrontPosition)
                    .addData("Inches: ", rightFrontPosition / COUNTS_PER_INCH);

            telemetry.addLine("leftBack")
                    .addData("Ticks: ", leftBackPosition)
                    .addData("Inches: ", leftBackPosition / COUNTS_PER_INCH);

            telemetry.addLine("rightBack")
                    .addData("Ticks: ", rightBackPosition)
                    .addData("Inches: ", rightBackPosition / COUNTS_PER_INCH);





            telemetry.addData("Control Direction", controlToggle);  //writes value of controlToggle
            idle();                                                 // to telemetry

            telemetry.update();


        }

}
