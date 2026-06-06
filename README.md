# MiniFRC Kiwi Swerve

This repository contains code for controlling a 3-pod coaxial swerve drive built using MiniFRC hardware. The project uses PlatformIO and compiles to the Alfredo NoU3. Each pod runs with two motors and an MT6701 set to analog mode to measure pod rotation. The project is configured to use clangd; to generate `compile_commands.json`, run `pio run -t compiledb`.