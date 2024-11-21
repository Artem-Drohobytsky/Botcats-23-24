package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//TELEOP WITH 1 CONTROLLER
@TeleOp(name = "Tele Test")
public class teleTest extends LinearOpMode {
    public void runOpMode() {



        DcMotor fourBar = hardwareMap.dcMotor.get("4bar_motor");

        double driveSpeed = .5;
        boolean aButton = true;
        double finalAngle;

        boolean lift1run = true;


        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

           if(gamepad1.right_trigger >= 0) {
               fourBar.setPower(gamepad1.right_trigger);
           }
           if(gamepad1.left_trigger >= 0) {
                fourBar.setPower(-gamepad1.left_trigger);
            }
           if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
               fourBar.setPower(0);
           }

           telemetry.addData("4Bar Encoder Ticks: ", fourBar.getCurrentPosition());
           telemetry.addData("LT: ", gamepad1.left_trigger);
           telemetry.addData("RT: ", gamepad1.right_trigger);
           telemetry.update();
        }
    }
}
