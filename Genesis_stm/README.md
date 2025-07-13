Current implementation uses only the cortex M7 core while the M4 was disabled by the st cube programer. Code meant to be used by both cores is located in the _Common_ folder, while the core specific user application (none generated code) is under _CM7/Core/Application_.

MPC controller for lateral vehicle control is contained in the submodule under _CM7/Core/Application/LateralControl/lateralVehicleController_

## Building and Debugging
Project was initialized with cube MX as a makefile project with building and debugging directly from VS Code.
Build can be invoked by a task (Ctrl+Shift+B) but requires Make to be installed.
Flashing and debugging with Cortex-Debug extension (requires ST Cube IDE to be installed since ST-Link can not be installed separately)

