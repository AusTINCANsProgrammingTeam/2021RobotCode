/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class determines which game the user has entered in the smart dashboard
 */
public class Game {
  public enum GameType {
    PowerPort,
    InterstellarAccuracy,

    // Do not remove
    Unknown
  }
  

  public static GameType getGame() {
    GameType currentGame;
    String game = SmartDashboard.getString("Current Game", "").toLowerCase();
    switch(game) {
      case "powerport":
        currentGame = GameType.PowerPort;
        break;
      case "interstellaraccuracy":
        currentGame = GameType.InterstellarAccuracy;
        break;
      default:
        currentGame = GameType.Unknown;
    }
    return currentGame;
  }
}
