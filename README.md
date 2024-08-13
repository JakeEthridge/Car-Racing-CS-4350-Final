** Third Party Libraries **

Sound Audio Provided by irrKlang (located in mm/libraries)

PhysX 5 Engine (https://github.com/NVIDIA-Omniverse/PhysX.git) -> Place this in lib64 folder

** Building the Engine ** 
1. Execute the RUN CMAKE to generate a cwin64 folder with the SpeedRacer.sln
2. Copy and Paste all the files in the DLL Folder into cwin64 folder (Note: PhysXGpu_64.dll is missing due to file size restriction)
3. Within cwin64/Debug create a new folder and copy and paste the same dlls as before, then copy and paste SpeedRacer.exe and the aftr.conf file as well
4. Edit the aftr.conf file with the following changes
     uncomment sharedmultimediapath and set it to =../../../../../shared/mm/
     uncomment localmultimediapath and set it to =../../../mm/
     change NetServerListenPort=12684
     change NetServerTransmitPort=12683
     change clientside=1

** Game Description ** 
