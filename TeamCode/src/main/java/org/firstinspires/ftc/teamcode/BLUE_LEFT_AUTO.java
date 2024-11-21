package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Timer;

@Autonomous(name="BLUE_LEFT_AUTO", group="cvAutos")
//RED TOP STARTING PLACE
public class BLUE_LEFT_AUTO extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Robot robot = new Robot();

    public void runOpMode() {

        robot.init(hardwareMap,telemetry);
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(-65, 12, Math.toRadians(0));
        Pose2d parking = new Pose2d(-60, 48, Math.toRadians(90));

        TrajectorySequence placementDemo = drive.trajectorySequenceBuilder(startPos)
                .forward(5)
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

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        if (opModeIsActive()) {

            telemetry.addData("img analysis", pipeline.getBlueAnalysis());
            telemetry.update();

            if (pipeline.getBlueAnalysis() == camera.SkystonePosition.RIGHT) {
                telemetry.addData("right", "found item in right box");
                telemetry.update();
                drive.followTrajectorySequence(placementDemo);
            }
        }
    }
}