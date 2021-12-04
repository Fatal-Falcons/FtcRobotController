package org.firstinspires.ftc.teamcode;
//didnt really work out need to work on this one
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Meow TestAutonomous2", group="Exercises")
//@Disabled
public class MeowTestAutonomous2 extends LinearOpMode {
    // Sets the runtime variable to the elapsed time within autonomous
    private ElapsedTime runtime = new ElapsedTime();

    // Creates variables for all the motors and servos
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private Servo clawOne = null;
    private DcMotor flywheel = null;
    private Servo clawTwo = null;


    // Sets the motor specifications as variables
    static final double COUNTS_PER_MOTOR_REV = 960;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 2.95276;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED =.6;
    static final double TURN_SPEED = .5;


    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException {

        // Defines the names of all the motors and servos
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        clawOne = hardwareMap.get(Servo.class, "claw_one");
        clawTwo = hardwareMap.get(Servo.class, "claw_two");

        // sets the direction of the motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
            armMotor.setDirection(DcMotor.Direction.REVERSE);
        clawOne.setDirection(Servo.Direction.REVERSE);
        clawTwo.setDirection(Servo.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d", leftDrive.getCurrentPosition()
                , rightDrive.getCurrentPosition());
        telemetry.update();


        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        encoderDrive(DRIVE_SPEED, 20, 20, 0.5);

        encoderDrive(TURN_SPEED, -45, 45, .5);

        encoderDrive(DRIVE_SPEED, 60, 60, 3);

    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTargetOne;
        int newRightTargetOne;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetOne = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTargetOne = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftDrive.setTargetPosition(newLeftTargetOne);
            rightDrive.setTargetPosition(newRightTargetOne);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTargetOne, newRightTargetOne);
                telemetry.addData("Path2", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}