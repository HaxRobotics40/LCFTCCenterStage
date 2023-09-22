import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "ColorStopAutonomous", group = "Autonomous")
public class ColorStopAutonomous extends LinearOpMode {
    private ColorSensor colorSensor;
    
    private int stopThreshold = 100; // Adjust this value according to your requirements

    private void initializeHardware(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor"); // Use the correct sensor name
    }

    @Override
    public void runOpMode() {
        initializeHardware(hardwareMap);

        waitForStart();

        // Move the robot forward until the red value exceeds the stop threshold
        while (opModeIsActive()) {
            // Read the red color value from the color sensor
            int redValue = colorSensor.red();

            // Check if the red value is higher than the stop threshold
            if (redValue > stopThreshold) {
                // Stop the robot (you should replace this with your actual motor control logic)
                // For example, if you're using motors with encoders:
                // motor.setPower(0);
                // Or, if you're using a drivetrain object:
                // drivetrain.stop();
                break; // Exit the loop
            }

            // Move the robot forward (replace with your motor control logic)
            // For example, if you're using motors with encoders:
            // motor.setPower(0.5); // Adjust the power level as needed
            // Or, if you're using a drivetrain object:
            // drivetrain.moveForward(0.5); // Adjust the power level as needed

            // Add any other necessary logic for your autonomous routine
        }
    }
}
