// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.BuildConstants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

/** Determines whether to burn SparkMax configs to flash. */
public class SparkoidBurnManager {
    public static final int kConfigCANTimeout = 500;
    public static final int kConfigAttempts = 4; // How many times to set the config on init
    private static final String kBuildDateFile = "/home/lvuser/build-date.txt";
    private static boolean shouldBurn = false;

    private SparkoidBurnManager() {
    }

    public static boolean shouldBurn() {
        return shouldBurn;
    }

    public static void update() {
        if (RobotBase.isSimulation()) {
            shouldBurn = false;
            return;
        }

        File file = new File(kBuildDateFile);
        if (!file.exists()) {
            // No build date file, burn flash
            shouldBurn = true;
        } else {
            // Read previous build date
            String previousBuildDate = "";
            try {
                previousBuildDate = new String(Files.readAllBytes(Paths.get(kBuildDateFile)), StandardCharsets.UTF_8);
            } catch (IOException e) {
                e.printStackTrace();
            }

            shouldBurn = !previousBuildDate.equals(BuildConstants.BUILD_DATE);
        }

        try {
            FileWriter fileWriter = new FileWriter(kBuildDateFile);
            fileWriter.write(BuildConstants.BUILD_DATE);
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        if (shouldBurn) {
            System.out.println("[SparkoidBurnManager] Build date changed, burning Sparkoid's flash");
        } else {
            System.out.println(
                    "[SparkoidBurnManager] Build date unchanged, will not burn Sparkoid's flash");
        }
    }
}
