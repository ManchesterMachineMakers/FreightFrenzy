package org.firstinspires.ftc.teamcode.util

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Mat
import java.util.concurrent.atomic.AtomicReference

/**
 * To use DashOpMode, simply add it in your implementation clause
 *
 * #### TO USE VISION:*
 * After start, but before the loop, add "FtcDashboard.getInstance().startCameraStream(vision.dash, 0.0)"
 *
 * You can now view your camera stream on the dashboard
 *
 * #### TO USE TELEMETRY:
 * **TELEMETRY WILL AUTOMATICALLY BE REROUTED TO THE DASHBOARD BY SUBASSEMBLY.KT**
 *
 * **You must inherit Subassembly.kt in your subassemblies.**
 * If you'd like to use it in your OpMode, then create your own instance of
 * MultipleTelemetry via "MultipleTelemetry(this.telemetry, dashboard.telemetry)"
 */
interface DashOpMode {

    //Talk Volatile vs AtomicReference, final to make sure it is only initialized once, etc.
    class CameraStreamProcessor : VisionProcessor, CameraStreamSource {
        private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

        //private volatile Bitmap lastFrame =  Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565);
        override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>?>) {
            continuation.dispatch { bitmapConsumer: Consumer<Bitmap>? ->
                bitmapConsumer!!.accept(
                    lastFrame.get()
                )
            }
        }

        override fun init(width: Int, height: Int, cameraCalibration: CameraCalibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565))
        }

        override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
            val b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
            Utils.matToBitmap(frame, b)
            lastFrame.set(b)
            return null
        }

        override fun onDrawFrame(canvas: Canvas, i: Int, i1: Int, v: Float, v1: Float, o: Any?) {
        }
    }

    object Static {
        @JvmStatic val dashboard = FtcDashboard.getInstance()
        @JvmStatic val telemetry = dashboard.telemetry
    }
}