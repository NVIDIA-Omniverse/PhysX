# Omni PhysX Visual Debugger
   PhysX visual debugger extension - OmniPvd

## Documentation
   Omniverse physics visual debugger (OmniPvd), an early preview version that allows for recording of transformations and attributes of Omniverse Physics objects and to transform them into USD.
   
   OmniPvd consist of a shared library and a Kit extension. The shared library is used by both the PhysX SDK simulation engine for the recording or writing of OmniPvd binary files (.OVD) and by the omni.physx.pvd Kit extension to read the recorded OmniPvd (.OVD) files.
   
   The OmniPvd Kit extension (omni.physx.pvd) allows firstly the transformation of PhysX recordings into USD or USDA format and to inspect them. It also allows secondly for the transforming of a single snapshot of such a PhysX recording into the Physics USD format. And thirdly allows for the baking or cooking of PhysX transforms onto Omniverse Physics objects in a separate Edit Layer.

   To be able to record a physics scene, go into the Physics Debug window, under the PVD section and check "OmniPvd Enabled" output. This will allow for a binary serialization of your Omniverse physics scene into an OmniPvd file. It primes the underlying PhysX simulation engine for OVD output.
   
   Once you have the OmniPvd file ready you can transform it into USD or USDA format by chosing the input (.OVD) file, the up axis and the output format (USDA/USD).

   A feature that is deprecated and is currently not functional is the overlaying of an OmniPvd simulation recording onto a USD Stage, thereby baking the physics simulation as an animation track.
            
   Below is a screenshot from an OmniPvd recording of the PhysX vehicle sample
