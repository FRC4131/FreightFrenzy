package org.firstinspires.ftc.teamcode.subsystem.visionsystem;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCvVisionSystem implements VisionSystem{
    OpenCvWebcam webcam;
    BarCodeDetectPipeline barCodeDetectPipeline = new BarCodeDetectPipeline();

    public OpenCvVisionSystem(OpenCvWebcam inputWebcam){
        this.webcam = inputWebcam;
        this.webcam.setPipeline(this.barCodeDetectPipeline);
        this.webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //See long comment about potential resolutions and frame rate
                // tradeoffs in OpenCV sample codes
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened -- needs something here
                 */
            }
        });
    }

    public int scanBarCode(){
        return this.barCodeDetectPipeline.FindObjectPosition();
    };

    public void stopVisionStream(){
        this.webcam.stopStreaming();
        this.webcam.closeCameraDevice(); //double check that closing the webcam doesn't destroy the estimated object position
    }
}
