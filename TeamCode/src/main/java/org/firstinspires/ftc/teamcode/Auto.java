package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

    /**
     * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
     * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
     * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
     * It includes all the skeletal structure that all linear OpModes contain.
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @TeleOp(name="Semi Auto", group="Linear Opmode")
//@Disabled
    public class Auto extends LinearOpMode{

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
        private DcMotor armMotor = null;
        private Servo clawOne = null;
        private DcMotor flywheel = null;
        private Servo clawTwo = null;
        double clawPosition;

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
            flywheel = hardwareMap.get(DcMotor.class, "flywheel");
            clawOne = hardwareMap.get(Servo.class, "claw_one");
            clawTwo = hardwareMap.get(Servo.class, "claw_two");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            armMotor.setDirection(DcMotor.Direction.REVERSE);
            clawOne.setDirection(Servo.Direction.FORWARD);
            clawTwo.setDirection(Servo.Direction.REVERSE);
            flywheel.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            armMotor.setPower(.23);
            sleep(250);
            armMotor.setPower(0);
            sleep(500);
            clawOne.setPosition(0);
            clawTwo.setPosition(0.5);


            // run until the end of the match (driver presses STOP)

        }}
