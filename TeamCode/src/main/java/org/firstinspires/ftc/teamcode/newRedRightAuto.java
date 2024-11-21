package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RED_RIGHT_AUTO 2", group="cvAutos")
//RED TOP STARTING PLACE
public class newRedRightAuto extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    //Robot robot = new Robot();

    public void runOpMode() {

        //robot.init(hardwareMap,telemetry);
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera pipeline;

        pipeline = new camera();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        DcMotor fourBar = hardwareMap.dcMotor.get("4bar_motor");
        Servo outtake = hardwareMap.servo.get("4bar_servo");

        CRServo back_left_servo = hardwareMap.crservo.get("back_left_servo");
        CRServo back_right_servo = hardwareMap.crservo.get("back_right_servo");
        CRServo front_left_servo = hardwareMap.crservo.get("front_left_servo");
        CRServo front_right_servo = hardwareMap.crservo.get("front_right_servo");

        Servo tilt = hardwareMap.servo.get("tilt");
        Servo launch = hardwareMap.servo.get("launch");

        fourBar.setTargetPosition(-10);
        fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int beacon = 0;

        Pose2d startPos = new Pose2d(65, 12, Math.toRadians(0));
        Pose2d depositRight = new Pose2d(30, 12, Math.toRadians(270));
        Pose2d depositLeft = new Pose2d(30, 12, Math.toRadians(90));
        Pose2d depositCenter = new Pose2d(36, 12, Math.toRadians(0));
        Pose2d placeLeft = new Pose2d(28, 54, Math.toRadians(90));
        Pose2d placeCenter = new Pose2d(36, 56, Math.toRadians(90));
        Pose2d placeRight = new Pose2d(44, 54, Math.toRadians(90));
        Pose2d parking = new Pose2d(60, 50, Math.toRadians(180));

        drive.setPoseEstimate(startPos);

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(startPos)
                .back(10)
                .strafeLeft(10)
                .lineToLinearHeading(depositLeft)
                //.forward(10)
                //.turn(Math.toRadians(90))
                .build();

        TrajectorySequence purpleMid = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(depositCenter)
                //.turn(180)
                //.forward(15)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(depositRight)
                //.turn(Math.toRadians(-90))
                //.forward(10)
                //.turn(Math.toRadians(-90))
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(depositLeft)
                .lineToLinearHeading(placeLeft)
                .build();

        TrajectorySequence yellowMid = drive.trajectorySequenceBuilder(depositCenter)
                .forward(4)
                .lineToLinearHeading(placeCenter)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(depositRight)
                .strafeLeft(24)
                .lineToLinearHeading(placeRight)
                .build();

        TrajectorySequence parkFromMid = drive.trajectorySequenceBuilder(placeCenter)
                .back(5)
                .lineToLinearHeading(parking)
                .build();

        TrajectorySequence parkFromLeft = drive.trajectorySequenceBuilder(placeLeft)
                .back(5)
                .lineToLinearHeading(parking)
                .build();

        TrajectorySequence parkFromRight = drive.trajectorySequenceBuilder(placeRight)
                .back(5)
                .lineToLinearHeading(parking)
                .build();



        DcMotor fl = null;
        DcMotor fr = null;
        DcMotor bl = null;
        DcMotor br = null;

        fl = hardwareMap.dcMotor.get("front_left_motor");
        fr = hardwareMap.dcMotor.get("back_right_motor");
        bl = hardwareMap.dcMotor.get("back_left_motor");
        br = hardwareMap.dcMotor.get("front_right_motor");

        // fr.setDirection(DcMotorSimple.Direction.REVERSE);
        // bl.setDirection(DcMotorSimple.Direction.REVERSE);

        while(!opModeIsActive()) {
            if (pipeline.getRedAnalysis() == camera.SkystonePosition.RIGHT) {
                telemetry.addData("right", "found item in right box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleRight);

                beacon = 2;
            } else if (pipeline.getRedAnalysis() == camera.SkystonePosition.LEFT) {
                telemetry.addData("left", "found item in left box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleLeft);

                beacon = 0;
            } else if (pipeline.getRedAnalysis() == camera.SkystonePosition.CENTER) {
                telemetry.addData("center", "found item in center box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleMid);

                beacon = 1;
            }
        }
        waitForStart();
        if (opModeIsActive()) {

            outtake.setPosition(0.5);

            telemetry.addData("img analysis", beacon);
            telemetry.update();

            /*if (pipeline.getRedAnalysis() == camera.SkystonePosition.RIGHT) {
                telemetry.addData("right", "found item in right box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleRight);

                beacon = 2;
            }
            else if (pipeline.getRedAnalysis() == camera.SkystonePosition.LEFT) {
                telemetry.addData("left", "found item in left box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleLeft);

                beacon = 0;
            }
            else if (pipeline.getRedAnalysis() == camera.SkystonePosition.CENTER) {
                telemetry.addData("center", "found item in center box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleMid);

                beacon = 1;
            }*/

            if(beacon == 1) {drive.followTrajectorySequence(purpleMid);}
            if(beacon == 0) {drive.followTrajectorySequence(purpleLeft);}
            if(beacon == 2) {drive.followTrajectorySequence(purpleRight);}
            // drive.followTrajectorySequence(test);

            front_left_servo.setPower(0.1);
            front_right_servo.setPower(-0.1);
            back_left_servo.setPower(0.1);
            back_right_servo.setPower(-0.1);
            sleep(4000);
            front_left_servo.setPower(0);
            front_right_servo.setPower(0);
            back_left_servo.setPower(0);
            back_right_servo.setPower(0);

            if(beacon == 1) {drive.followTrajectorySequence(yellowMid);}
            if(beacon == 0) {drive.followTrajectorySequence(yellowLeft);}
            if(beacon == 2) {drive.followTrajectorySequence(yellowRight);}

            while(fourBar.isBusy()) {
                fourBar.setTargetPosition(-1000);
                fourBar.setPower(1);
            }

            outtake.setPosition(0.05);
        }
    }
}