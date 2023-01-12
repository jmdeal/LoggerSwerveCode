package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.util.Units;

public class Pigeon2IO implements GyroIO {
    private final Pigeon2 gyro;
    private final double[] xyzDps = new double[3];
  
    public Pigeon2IO() {
          gyro = new Pigeon2(0);
    }
    
  
    public void updateInputs(GyroIOInputs inputs) {
      // Again, the 2022 code base has good examples for reading this data. We generally prefer to use
      // "getAngle" instead of "getYaw" (what's the difference?)
      //
      // Remember to pay attention to the UNITS.
      gyro.getRawGyro(xyzDps);
      inputs.connected = gyro.getLastError().equals(ErrorCode.OK);
      inputs.positionRad = Units.degreesToRadians(gyro.getYaw());
      inputs.velocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
  
    }
  }
