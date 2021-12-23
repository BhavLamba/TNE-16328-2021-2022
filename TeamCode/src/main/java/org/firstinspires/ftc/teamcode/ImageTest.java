package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;

import java.io.IOException;
import java.util.List;


@Autonomous
public class ImageTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

//
//        TensorImageClassifier tfic = null;
//        try {
//            tfic = new TFICBuilder(hardwareMap, "tape.tflite", "tape", "no tape").build();
//            List<TensorImageClassifier.Recognition> data = tfic.recognize();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }


    }
}
