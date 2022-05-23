# MIRAO_DeviceAdapter
Device adapter for MIRAO52E for Micro-Manager This device adapter allows for low level control of the deformable mirror MIRAO52E by Micro-Manager using the SDK provided by Imagine Optic. This device adapter is not a GUI to control the mirror, but the Micro-Manager user interface or script panel can be used to control the mirror.
For any questions please contact Marijn Siemons, marijnsiemons@gmail.com.

# Installation
- Get Micro-Manager version 1.4.23 (From: https://micro-manager.org/wiki/Micro-Manager_Nightly_Builds )
- Copy the MIRAO folder in Micro-Manager installation folder
-Copy all .dll-files from MIRAO/lib folder to the Micro-Manager installation folder
-Add the MIRAO in a hardware configuration as usual (https://micro-manager.org/wiki/Micro-Manager_Configuration_Guide)
The MIRAO52E device should appear under IODeformableMirror  add “MIRAO52E | Mirao52-e”. No further details are required.
For testing one can use “MIRAO52E_FAKE | Fake Mirao52-e”, which is a fake mirror.
MIRAO can now be used by Micro-Manager.

# Citing
If you use this device adapter, please cite our paper

Siemons, M.E., Hanemaaijer, N.A.K., Kole, M.H.P. et al. Robust adaptive optics for localization microscopy deep in complex tissue. Nat Commun 12, 3407 (2021). https://doi.org/10.1038/s41467-021-23647-2
