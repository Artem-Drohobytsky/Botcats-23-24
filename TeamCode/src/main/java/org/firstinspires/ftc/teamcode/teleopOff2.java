package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//TELEOP WITH TWO CONTROLLERS GP1 IS DRIVE GP2 IS OTHER
@TeleOp(name = "Tele w/ 2")
public class teleopOff2 extends LinearOpMode {
    public void runOpMode() {

        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");

        DcMotor left_slide_motor = hardwareMap.dcMotor.get("left_slide_motor");
        DcMotor right_slide_motor = hardwareMap.dcMotor.get("right_slide_motor");

        double ENCODERTICKS = 537.7;

        CRServo left_intake = hardwareMap.crservo.get("left_intake");
        CRServo right_intake = hardwareMap.crservo.get("right_intake");

        Servo left_claw_servo = hardwareMap.servo.get("left_claw_servo");
        Servo right_claw_servo = hardwareMap.servo.get("right_claw_servo");

        Servo left_slide_servo = hardwareMap.servo.get("left_slide_servo");
        Servo right_slide_servo = hardwareMap.servo.get("right_slide_servo");

        Servo tilt = hardwareMap.servo.get("tilt");
        Servo launch = hardwareMap.servo.get("launch");

        // fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        // bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        left_slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double driveSpeed = 0.4;
        final int liftHome = 0;
        boolean fieldCentric = false;
        double finalAngle;
        double robotAngle;
        boolean speedToggle = true;
        boolean launching = false;
        boolean launchRED = true;
        boolean aButton = true;
        boolean yButton = true;
        boolean xButton = true;
        boolean bButton = true;

        double slowRelease = 0.3;

        double liftEncoder = 0;

        double enc = .5;

        Orientation angles;
        IMU imu;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        /* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters); */
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            //speed control gamepad1

            if (gamepad1.right_trigger > 0 ) {
                driveSpeed = 0.7;
            }
            else {
                driveSpeed = 0.4;
            }

            if (gamepad1.y && yButton){
                yButton = false;
                fieldCentric = !fieldCentric;
            }
            if (!gamepad1.y && !yButton) {
                yButton = true;
            }

            angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
            robotAngle = angles.firstAngle;

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y); //finds hypotenuse (power of each motor)
            double gpAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //finds angle of robot subtracted by pi/4 bc
            //it "shifts" the powers to each motor CW
            double rightX = (-gamepad1.right_stick_x) * .8; //for rotating w/ right stick

            if(fieldCentric){
                finalAngle = gpAngle - robotAngle;
            }
            else{
                finalAngle = gpAngle;
            }

            final double v1 = driveSpeed * (r * Math.cos(finalAngle) + rightX);
            final double v2 = driveSpeed * (r * Math.sin(finalAngle) - rightX);
            final double v3 = driveSpeed * (r * Math.sin(finalAngle) + rightX);
            final double v4 = driveSpeed * (r * Math.cos(finalAngle) - rightX);
            /*
                r is multiplier for power
                math.cos is used for fl and br bc fl&br are used to go diagonal top right, if you want to go faster to the right apply more power to
                those motors so closer joystick is to x axis faster robot go to that direction
                math.sin is used for same reason as ^ but to go faster forward/backwards
             */

            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);

            // DRIVER 2

            // ! TEST THESE INDIVIDUAL TO KEEP OUTTAKE ALIGNMENT
            if(gamepad2.left_stick_y != 0){
                left_slide_motor.setPower(gamepad2.left_stick_x/2);
                //right_slide_motor.setPower(gamepad2.left_stick_x/2);
            }
            else {
                left_slide_motor.setPower(0);
                //right_slide_motor.setPower(0);
            }

            /* if (gamepad2.a && aButton){
                aButton = false;
                if(slowRelease == 1){
                    slowRelease = 0.3;
                }
                if(slowRelease == 0.3){
                    slowRelease = 1;
                }
            }
            if (!gamepad2.a && !aButton) {
                aButton = true;
            } */

            if(gamepad2.right_bumper) {
                left_intake.setPower(-1);
                right_intake.setPower(1);
            }
            if(gamepad2.left_bumper) {
                left_intake.setPower(1);
                right_intake.setPower(-1);
            }
            if(!gamepad2.right_bumper && !gamepad2.left_bumper) {
                left_intake.setPower(0);
                right_intake.setPower(0);
            }

            // ! TEST THESE INDIVIDUALLY
            if (gamepad2.dpad_down) {
                left_slide_servo.setPosition(0);
                // right_slide_servo.setPosition(0);
            }
            if (gamepad2.dpad_up) {
                left_slide_servo.setPosition(0.5);
                // right_slide_servo.setPosition(0.5);
            }
            if(gamepad2.dpad_left) {
                left_claw_servo.setPosition(0);
            }
            if(gamepad2.dpad_right) {
                right_claw_servo.setPosition(0);
            }
            if(gamepad2.x) {
                left_claw_servo.setPosition(0.5);
            }
            if(gamepad2.b) {
                right_claw_servo.setPosition(0.5);
            }

            if(gamepad2.y && launchRED) {
                launchRED = false;
                //launching = !launching;
                if(launching){
                    launching = false;
                }
                if(!launching){
                    launching = true;
                }
            }
            else if(!gamepad2.y && !launchRED) {launchRED = true;}

            if(!launching) {
                tilt.setPosition(0.55);
                launch.setPosition(0.85);

            }
            if(launching) {
                tilt.setPosition(0.45);
            }

            if(gamepad2.a) {
                launch.setPosition(0.5);
            }

            // if(gamepad2.dpad_up) {enc += 0.01;}
            // if(gamepad2.dpad_down) {enc -= 0.01;}


            telemetry.addData("Drive Speed", driveSpeed);
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("flEncoder", fl.getCurrentPosition());
            telemetry.addData("frEncoder", fr.getCurrentPosition());
            telemetry.addData("blEncoder", bl.getCurrentPosition());
            telemetry.addData("brEncoder", br.getCurrentPosition());

            telemetry.addData("increments", enc);
            // telemetry.addData("lift1 enc ticks", lift1.getCurrentPosition());
            // telemetry.addData("lift2 enc ticks", lift2.getCurrentPosition());
            telemetry.update();
        }
    }
}