import os
Import("env")

# Force PlatformIO to include toolchain paths (e.g., Arduino/ESP32 framework headers)
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# Override compilation DB path to output straight into the project root
env.Replace(COMPILATIONDB_PATH=os.path.join(env.get("PROJECT_DIR"), "compile_commands.json"))
