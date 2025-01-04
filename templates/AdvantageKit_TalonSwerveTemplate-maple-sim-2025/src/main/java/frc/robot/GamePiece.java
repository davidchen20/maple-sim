package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class GamePiece extends GamePieceOnFieldSimulation {
    /* https://www.andymark.com/products/frc-2024-am-4999 */
    public static final double NOTE_HEIGHT = Units.inchesToMeters(2),
            NOTE_DIAMETER = Units.inchesToMeters(14),
            NOTE_WEIGHT_KG = 0.2;

    public GamePiece(Translation2d initialPosition) {
        super("Note", Geometry.createCircle(NOTE_DIAMETER / 2), NOTE_HEIGHT, NOTE_WEIGHT_KG, initialPosition);
    }
} 
