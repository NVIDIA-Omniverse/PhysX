# Omni.PhysX Fabric Output  [omni.physx.fabric]
   Output from PhysX simulation to fabric

## Usage

   Enable omni.physx.fabric extension before loading a stage with simulation. New physics fabric settings are 
   added to the Edit->Preferences window. Its possible to toggle the fabric output there.

## Know Issues

   Point instancer rigid bodies are now not updated through fabric extension. Runtime changes to the scene composition
   might not be correctly reflected in fabric and maybe lead into issues.
