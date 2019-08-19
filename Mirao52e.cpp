///////////////////////////////////////////////////////////////////////////////
// FILE:          Mirao52e.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Device adapter for MICAO-52E deformable mirror
//
// AUTHOR:        Marijn Siemons 27-02-2018

#include "Mirao52e.h"
#include "windows.h"
#include<cstdlib>
#include<cstring>
#include<string>
#include <sstream>
#include <sys/stat.h>
#include "../../MMDevice/ModuleInterface.h"
#include "../../MMDevice/MMDevice.h"
#include "../../MMDevice/DeviceBase.h"
#include "Mirror.hpp"
#include "Model.h"
#include "PhaseDiversity.h"
#include "3NAlgorithm.h"
#include "merit_functions.hpp"
#include "conversion.hpp"

#define IMPORT_IMOP_WAVEKITBIO_FROM_LIBRARY
#define NOMINMAX

const char* g_DMname  = "MIRAO52E";
const char* g_DMfakename  = "MIRAO52E_FAKE";

const char* g_SetCalibration  = "Set calibration path";
const char* g_SetCalibrationParams  = "Set calibration params path";
const char* g_SetDiversityPref = "Set diversity preferences path";
const char* g_LoadWavefront = "Load wavefront";
const char* g_SaveCurrentPosition = "Save current position [input filename]";

const char* g_fakemirrorinit_path  = "MIRAO/init/Fake_Mirao52-e_0219.dat";
//const char* g_mirrorinit_path  = "MIRAO/init/WaveFrontCorrector_Mirao52-e_0235.dat";
const char* g_mirrorinit_path  = "MIRAO/init/MIRAO_initialization.dat";
const char* g_calib_initpath  = "MIRAO/init/MIRAO_calibration.aomi";
const char* g_calibparams_initpath  = "MIRAO/init/Diversity_calibration.xml";
const char* g_divpref_initpath  = "MIRAO/init/Diversity_prefs.xml";
const char* g_wfc_initpath  = "MIRAO/init/WavefrontCorrection.wcs";
const char* g_savepath  = "MIRAO/WavefrontCorrection_save.wcs";

const char* g_ApplyZernmodes  = "ApplyZernikes";
const char* g_SetZernMode_Tip  = "Z11";
const char* g_SetZernMode_Tilt  = "Z1-1";
const char* g_SetZernMode_Defocus  = "Z20";
const char* g_SetZernMode_Astig0deg  = "Z22";
const char* g_SetZernMode_Astig45deg  = "Z2-2";
const char* g_SetZernMode_Coma0deg  = "Z31";
const char* g_SetZernMode_Coma90deg  = "Z3-1";
const char* g_SetZernMode_PrimSpherical  = "Z40";
const char* g_SetZernMode_Trefoil0deg  = "Z33";
const char* g_SetZernMode_Trefoil90deg  = "Z3-3";
const char* g_SetZernMode_SecondAstig0deg  = "Z42";
const char* g_SetZernMode_SecondAstig45deg  = "Z4-2";
const char* g_SetZernMode_Quadrafoil0deg  = "Z44";
const char* g_SetZernMode_Quadrafoil45deg  = "Z4-4";
const char* g_SetZernMode_SecondComa0deg  = "Z51";
const char* g_SetZernMode_SecondComa90deg  = "Z5-1";
const char* g_SetZernMode_SecondTrefoil0deg  = "Z53";
const char* g_SetZernMode_SecondTrefoil90deg  = "Z5-3";
const char* g_SetZernMode_SecondSpherical  = "Z60";


inline bool fileexists (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}


MODULE_API void InitializeModuleData()
{
	RegisterDevice(g_DMname, MM::GenericDevice, "Mirao-52e");
	RegisterDevice(g_DMfakename, MM::GenericDevice, "Fake Mirao-52e");
}


MODULE_API MM::Device* CreateDevice(const char* deviceName)                  
{
   if (deviceName == 0) return 0;
   if (strcmp(deviceName, g_DMname)  == 0) return new Mirao52e();
   if (strcmp(deviceName, g_DMfakename)  == 0) return new Mirao52e_FAKE();
   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   delete pDevice;
}

Mirao52e::Mirao52e() :
   port_("Undefined"),
   initialized_(false),
   mirrorinitpath_(g_mirrorinit_path),
   calibpath_(g_calib_initpath),
   calibparamspath_(g_calibparams_initpath),
   divprefpath_(g_divpref_initpath),
   wfcpath_(g_wfc_initpath),
   savepath_(g_savepath)
{
   InitializeDefaultErrorMessages();
   // add custom messages
   std::string error_mirrorinit_file = "Mirror initialization file does not exist. Looking for: ";	error_mirrorinit_file.append(mirrorinitpath_.c_str());
   SetErrorText(ERR_MIRRORINIT_FILE_NONEXIST, error_mirrorinit_file.c_str());

   std::string error_divinit_file = "Diversity initialization file does not exist. Looking for: ";	error_divinit_file.append(calibpath_.c_str());
   SetErrorText(ERR_DIVINIT_FILE_NONEXIST, error_divinit_file.c_str());

   std::string error_cal_file = "Calibration parameter file does not exist. Looking for: ";	error_cal_file.append(calibparamspath_.c_str());
   SetErrorText(ERR_CAL_FILE_NONEXIST, error_cal_file.c_str());

   std::string error_divpref_file = "Diversity preferences file does not exist. Looking for: ";	error_divpref_file.append(divprefpath_.c_str());
   SetErrorText(ERR_DIVPREF_FILE_NONEXIST, error_divpref_file.c_str());

   SetErrorText(ERR_FILE_NONEXIST, "File does not exist");

   // create pre-initialization properties
   // ------------------------------------
   // Name
   CreateProperty(MM::g_Keyword_Name, g_DMname, MM::String, true);
   // Description
   CreateProperty(MM::g_Keyword_Description, "MIRAO-52E device adapter", MM::String, true);
   // Port
   CPropertyAction* pAct = new CPropertyAction (this, &Mirao52e::OnPort);
   CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);  
}

Mirao52e::~Mirao52e()
{
   if (initialized_)
      Shutdown();
}

bool Mirao52e::Busy()
{
      return false;
}

void Mirao52e::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, g_DMname);
}

// General utility function:
int ClearPort(MM::Device& device, MM::Core& core, std::string port)
{
   // Clear contents of serial port 
   const int bufSize = 255;
   unsigned char clear[bufSize];                      
   unsigned long read = bufSize;
   int ret;                                                                   
   while (read == (unsigned) bufSize) 
   {                                                                     
      ret = core.ReadFromSerial(&device, port.c_str(), clear, bufSize, read);
      if (ret != DEVICE_OK)                               
         return ret;                                               
   }
   return DEVICE_OK;                                                           
} 

// initialize function
int Mirao52e::Initialize()
{
	if (initialized_)
    return DEVICE_OK;

	//Check if mirror initialization files exist
	if (!fileexists(mirrorinitpath_))
	{
		return ERR_MIRRORINIT_FILE_NONEXIST;
	}
	else if(!fileexists(calibpath_))
	{
		return ERR_DIVINIT_FILE_NONEXIST;
	}
	else if(!fileexists(calibparamspath_))
	{
		return ERR_CAL_FILE_NONEXIST;
	}
	else if(!fileexists(divprefpath_))
	{
		return ERR_DIVPREF_FILE_NONEXIST;
	}

	//init Mirror HW driver
    mirrorhandle = new imop::microscopy::Mirror(g_mirrorinit_path);
	mirrorhandle->init_hardware();

	//Load calibration file
	diversityhandle = new imop::microscopy::Diversity(calibpath_, *mirrorhandle );

    calibparamshandle = new imop::microscopy::CalibrationParams;
	calibparamshandle->Load(calibparamspath_);

    divprefshandle = new imop::microscopy::DiversityPreferences;
    divprefshandle->Load(divprefpath_);

    diversityhandle->Init_Diversity(*calibparamshandle,*divprefshandle);

	//Apply initial wavefront correction if WFC file exists
	if (fileexists(g_wfc_initpath))
	{
		Mirao52e::LoadWavefront(g_wfc_initpath);
	}

	// Create action properties
	CPropertyAction* pAct = new CPropertyAction(this, &Mirao52e::OnSetCalibration);
	int ret = CreateProperty(g_SetCalibration, g_calib_initpath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	pAct = new CPropertyAction(this, &Mirao52e::OnSetCalibrationParams);
	ret = CreateProperty(g_SetCalibrationParams, g_calibparams_initpath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	pAct = new CPropertyAction(this, &Mirao52e::OnSetDiversityPref);
	ret = CreateProperty(g_SetDiversityPref, g_divpref_initpath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	pAct = new CPropertyAction(this, &Mirao52e::OnLoadWavefront);
	ret = CreateProperty(g_LoadWavefront, g_wfc_initpath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	pAct = new CPropertyAction(this, &Mirao52e::OnSaveCurrentPosition);
	ret = CreateProperty(g_SaveCurrentPosition, g_savepath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	// Zernike Modes
    pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Tip);
    ret = CreateProperty(g_SetZernMode_Tip, "0", MM::Float, false, pAct);
    if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Tip, -1, 1);


    pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Tilt);
    ret = CreateProperty(g_SetZernMode_Tilt, "0", MM::Float, false, pAct);
    if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Tilt, -1, 1);


	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Defocus);
	ret = CreateProperty(g_SetZernMode_Defocus, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Defocus, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Astig0deg);
	ret = CreateProperty(g_SetZernMode_Astig0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Astig0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Astig45deg);
	ret = CreateProperty(g_SetZernMode_Astig45deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Astig45deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Coma0deg);
	ret = CreateProperty(g_SetZernMode_Coma0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Coma0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Coma90deg);
	ret = CreateProperty(g_SetZernMode_Coma90deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Coma90deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_PrimSpherical);
	ret = CreateProperty(g_SetZernMode_PrimSpherical, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_PrimSpherical, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Trefoil0deg);
	ret = CreateProperty(g_SetZernMode_Trefoil0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Trefoil0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Trefoil90deg);
	ret = CreateProperty(g_SetZernMode_Trefoil90deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Trefoil90deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_SecondAstig0deg);
	ret = CreateProperty(g_SetZernMode_SecondAstig0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondAstig0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_SecondAstig45deg);
	ret = CreateProperty(g_SetZernMode_SecondAstig45deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondAstig45deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Quadrafoil0deg);
	ret = CreateProperty(g_SetZernMode_Quadrafoil0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Quadrafoil0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_Quadrafoil45deg);
	ret = CreateProperty(g_SetZernMode_Quadrafoil45deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Quadrafoil45deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_SecondComa0deg);
	ret = CreateProperty(g_SetZernMode_SecondComa0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondComa0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_SecondComa90deg);
	ret = CreateProperty(g_SetZernMode_SecondComa90deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondComa90deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_SecondTrefoil0deg);
	ret = CreateProperty(g_SetZernMode_SecondTrefoil0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondTrefoil0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_SecondTrefoil90deg);
	ret = CreateProperty(g_SetZernMode_SecondTrefoil90deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondTrefoil90deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnSetZernMode_SecondSpherical);
	ret = CreateProperty(g_SetZernMode_SecondSpherical, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondSpherical, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e::OnApplyZernmodes);
	ret = CreateProperty(g_ApplyZernmodes, "0", MM::Integer, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	initialized_ = true;

	return DEVICE_OK;
}

// Shut down function
int Mirao52e::Shutdown()
{
   initialized_    = false;
   return DEVICE_OK;
}

int Mirao52e::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
	{
      pProp->Set(port_.c_str());
	}
   else if (eAct == MM::AfterSet)
   {
      if (initialized_)
      {
         // revert
         pProp->Set(port_.c_str());
         return ERR_PORT_CHANGE_FORBIDDEN;
      }
      pProp->Get(port_);
   }
   return DEVICE_OK;
}


// load new calibration files
int Mirao52e::SetCalibration(const std::string   path)
{
	if (fileexists(path))
	{
		calibpath_ = path;
		diversityhandle = new imop::microscopy::Diversity(calibpath_, *mirrorhandle);
		diversityhandle->Init_Diversity(*calibparamshandle,*divprefshandle);
	}
	else
	{
		return ERR_FILE_NONEXIST;
	}
	return DEVICE_OK;
}

int Mirao52e::SetDiversityPref(const std::string   path)
{
	if (fileexists(path))
	{
		divprefpath_ = path;
		divprefshandle->Load(divprefpath_);
		diversityhandle->Init_Diversity(*calibparamshandle,*divprefshandle);
	}
		else
	{
		return ERR_FILE_NONEXIST;
	}
	return DEVICE_OK;
}

int Mirao52e::SetCalibrationParams(const std::string  path)
{
	if (fileexists(path))
	{
		calibparamspath_ = path;
		calibparamshandle->Load(calibparamspath_);
		diversityhandle->Init_Diversity(*calibparamshandle,*divprefshandle);
	}
		else
	{
		return ERR_FILE_NONEXIST;
	}
	return DEVICE_OK;
}

int Mirao52e::LoadWavefront(std::basic_string<char>  path)
{
	if (fileexists(path))
	{
		wfcpath_ = path;
		diversityhandle->Apply_Absolute_Commands_From_File(wfcpath_);
		zer_store.zernike_coefficients[1] = 0;
		zer_store.zernike_coefficients[2] = 0;
		zer_store.zernike_coefficients[3] = 0;
		zer_store.zernike_coefficients[4] = 0;
		zer_store.zernike_coefficients[5] = 0;
		zer_store.zernike_coefficients[6] = 0;
		zer_store.zernike_coefficients[7] = 0;
		zer_store.zernike_coefficients[8] = 0;
		zer_store.zernike_coefficients[9] = 0;
		zer_store.zernike_coefficients[10] = 0;
		zer_store.zernike_coefficients[11] = 0;
		zer_store.zernike_coefficients[12] = 0;
		zer_store.zernike_coefficients[13] = 0;
		zer_store.zernike_coefficients[14] = 0;
		zer_store.zernike_coefficients[15] = 0;
		zer_store.zernike_coefficients[16] = 0;
		zer_store.zernike_coefficients[17] = 0;
		zer_store.zernike_coefficients[18] = 0;
		zer_store.zernike_coefficients[19] = 0;

		zer_rel.zernike_coefficients[1] = 0;
		zer_rel.zernike_coefficients[2] = 0;
		zer_rel.zernike_coefficients[3] = 0;
		zer_rel.zernike_coefficients[4] = 0;
		zer_rel.zernike_coefficients[5] = 0;
		zer_rel.zernike_coefficients[6] = 0;
		zer_rel.zernike_coefficients[7] = 0;
		zer_rel.zernike_coefficients[8] = 0;
		zer_rel.zernike_coefficients[9] = 0;
		zer_rel.zernike_coefficients[10] = 0;
		zer_rel.zernike_coefficients[11] = 0;
		zer_rel.zernike_coefficients[12] = 0;
		zer_rel.zernike_coefficients[13] = 0;
		zer_rel.zernike_coefficients[14] = 0;
		zer_rel.zernike_coefficients[15] = 0;
		zer_rel.zernike_coefficients[16] = 0;
		zer_rel.zernike_coefficients[17] = 0;
		zer_rel.zernike_coefficients[18] = 0;
		zer_rel.zernike_coefficients[19] = 0;
		Sleep(10);
	}
	else
	{
		return ERR_FILE_NONEXIST;
	}
	return DEVICE_OK;
}

int Mirao52e::SaveCurrentPosition(std::basic_string<char> path)
{
	savepath_ = path;
	diversityhandle->Save_Current_Positions_ToFile(savepath_);
	return DEVICE_OK;
}

/*
// Get Actuator positions //
int Mirao52e::GetActuatorPos(std::vector<float> pos)
{
	pos = mirrorhandle->Get_Current_Position();
	return DEVICE_OK;
}
*/

// Set Zernike modes
int Mirao52e::ApplyZernmodes()
{
	diversityhandle->Apply_Relative_Commands(zer_rel);

	zer_store.zernike_coefficients[1] = zer_store.zernike_coefficients[1] + zer_rel.zernike_coefficients[1];
	zer_store.zernike_coefficients[2] = zer_store.zernike_coefficients[2] + zer_rel.zernike_coefficients[2];
	zer_store.zernike_coefficients[3] = zer_store.zernike_coefficients[3] + zer_rel.zernike_coefficients[3];
	zer_store.zernike_coefficients[4] = zer_store.zernike_coefficients[4] + zer_rel.zernike_coefficients[4];
	zer_store.zernike_coefficients[5] = zer_store.zernike_coefficients[5] + zer_rel.zernike_coefficients[5];
	zer_store.zernike_coefficients[6] = zer_store.zernike_coefficients[6] + zer_rel.zernike_coefficients[6];
	zer_store.zernike_coefficients[7] = zer_store.zernike_coefficients[7] + zer_rel.zernike_coefficients[7];
	zer_store.zernike_coefficients[8] = zer_store.zernike_coefficients[8] + zer_rel.zernike_coefficients[8];
	zer_store.zernike_coefficients[9] = zer_store.zernike_coefficients[9] + zer_rel.zernike_coefficients[9];
	zer_store.zernike_coefficients[10] = zer_store.zernike_coefficients[10] + zer_rel.zernike_coefficients[10];
	zer_store.zernike_coefficients[11] = zer_store.zernike_coefficients[11] + zer_rel.zernike_coefficients[11];
	zer_store.zernike_coefficients[12] = zer_store.zernike_coefficients[12] + zer_rel.zernike_coefficients[12];
	zer_store.zernike_coefficients[13] = zer_store.zernike_coefficients[13] + zer_rel.zernike_coefficients[13];
	zer_store.zernike_coefficients[14] = zer_store.zernike_coefficients[14] + zer_rel.zernike_coefficients[14];
	zer_store.zernike_coefficients[15] = zer_store.zernike_coefficients[15] + zer_rel.zernike_coefficients[15];
	zer_store.zernike_coefficients[16] = zer_store.zernike_coefficients[16] + zer_rel.zernike_coefficients[16];
	zer_store.zernike_coefficients[17] = zer_store.zernike_coefficients[17] + zer_rel.zernike_coefficients[17];
	zer_store.zernike_coefficients[18] = zer_store.zernike_coefficients[18] + zer_rel.zernike_coefficients[18];
	zer_store.zernike_coefficients[19] = zer_store.zernike_coefficients[19] + zer_rel.zernike_coefficients[19];

	zer_rel.zernike_coefficients[1] = 0;
	zer_rel.zernike_coefficients[2] = 0;
	zer_rel.zernike_coefficients[3] = 0;
	zer_rel.zernike_coefficients[4] = 0;
	zer_rel.zernike_coefficients[5] = 0;
	zer_rel.zernike_coefficients[6] = 0;
	zer_rel.zernike_coefficients[7] = 0;
	zer_rel.zernike_coefficients[8] = 0;
	zer_rel.zernike_coefficients[9] = 0;
	zer_rel.zernike_coefficients[10] = 0;
	zer_rel.zernike_coefficients[11] = 0;
	zer_rel.zernike_coefficients[12] = 0;
	zer_rel.zernike_coefficients[13] = 0;
	zer_rel.zernike_coefficients[14] = 0;
	zer_rel.zernike_coefficients[15] = 0;
	zer_rel.zernike_coefficients[16] = 0;
	zer_rel.zernike_coefficients[17] = 0;
	zer_rel.zernike_coefficients[18] = 0;
	zer_rel.zernike_coefficients[19] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Tip(float Acoef)
{
	zer_rel.zernike_coefficients[1] = Acoef - zer_store.zernike_coefficients[1];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Tilt(float Acoef)
{
	zer_rel.zernike_coefficients[2] = Acoef - zer_store.zernike_coefficients[2];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Defocus(float Acoef)
{
	zer_rel.zernike_coefficients[3] = Acoef - zer_store.zernike_coefficients[3];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Astig0deg(float Acoef)
{
	zer_rel.zernike_coefficients[4] = Acoef - zer_store.zernike_coefficients[4];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Astig45deg(float Acoef)
{
	zer_rel.zernike_coefficients[5] = Acoef - zer_store.zernike_coefficients[5];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Coma0deg(float Acoef)
{
	zer_rel.zernike_coefficients[6] = Acoef - zer_store.zernike_coefficients[6];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Coma90deg(float Acoef)
{
	zer_rel.zernike_coefficients[7] = Acoef - zer_store.zernike_coefficients[7];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_PrimSpherical(float Acoef)
{
	zer_rel.zernike_coefficients[8] = Acoef - zer_store.zernike_coefficients[8];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Trefoil0deg(float Acoef)
{
	zer_rel.zernike_coefficients[9] = Acoef - zer_store.zernike_coefficients[9];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Trefoil90deg(float Acoef)
{
	zer_rel.zernike_coefficients[10] = Acoef - zer_store.zernike_coefficients[10];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_SecondAstig0deg(float Acoef)
{
	zer_rel.zernike_coefficients[11] = Acoef - zer_store.zernike_coefficients[11];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_SecondAstig45deg(float Acoef)
{
	zer_rel.zernike_coefficients[12] = Acoef - zer_store.zernike_coefficients[12];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_SecondComa0deg(float Acoef)
{
	zer_rel.zernike_coefficients[13] = Acoef - zer_store.zernike_coefficients[13];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_SecondComa90deg(float Acoef)
{
	zer_rel.zernike_coefficients[14] = Acoef - zer_store.zernike_coefficients[14];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_SecondSpherical(float Acoef)
{
	zer_rel.zernike_coefficients[15] = Acoef - zer_store.zernike_coefficients[15];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Quadrafoil0deg(float Acoef)
{
	zer_rel.zernike_coefficients[16] = Acoef - zer_store.zernike_coefficients[16];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_Quadrafoil45deg(float Acoef)
{
	zer_rel.zernike_coefficients[17] = Acoef - zer_store.zernike_coefficients[17];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_SecondTrefoil0deg(float Acoef)
{
	zer_rel.zernike_coefficients[18] = Acoef - zer_store.zernike_coefficients[18];
	return DEVICE_OK;
}

int Mirao52e::SetZernMode_SecondTrefoil90deg(float Acoef)
{
	zer_rel.zernike_coefficients[19] = Acoef - zer_store.zernike_coefficients[19];
	return DEVICE_OK;
}



///////////////////////////////////////////////////////////////////////////////
// Action handlers
// Handle changes and updates to property values.
///////////////////////////////////////////////////////////////////////////////


int Mirao52e::OnSetCalibration(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
		pProp->Set(calibpath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return SetCalibration(path);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetCalibrationParams(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	   pProp->Set(calibparamspath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return SetCalibrationParams(path);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetDiversityPref(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	// string val;
    pProp->Set(divprefpath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return SetDiversityPref(path);
   }
   return DEVICE_OK;
}

int Mirao52e::OnLoadWavefront(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
    pProp->Set(wfcpath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return LoadWavefront(path);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSaveCurrentPosition(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
   pProp->Set(savepath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return SaveCurrentPosition(path);
   }
   return DEVICE_OK;
}

// Zernike modes
int Mirao52e::OnApplyZernmodes(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
 //     pProp->Set(false); 
   }
   else if (eAct == MM::AfterSet)
   {
      return ApplyZernmodes();
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Tip(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[1] + zer_rel.zernike_coefficients[1] ;
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Tip(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Tilt(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[2] + zer_rel.zernike_coefficients[2];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Tilt(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Defocus(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[3] + zer_rel.zernike_coefficients[3];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Defocus(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Astig0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[4] + zer_rel.zernike_coefficients[4];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Astig0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Astig45deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[5] + zer_rel.zernike_coefficients[5];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Astig45deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Coma0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[6] + zer_rel.zernike_coefficients[6];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Coma0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Coma90deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[7] +  zer_rel.zernike_coefficients[7];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Coma90deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_PrimSpherical(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[8] + zer_rel.zernike_coefficients[8];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_PrimSpherical(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Trefoil0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[9] + zer_rel.zernike_coefficients[9];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Trefoil0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Trefoil90deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[10] + zer_rel.zernike_coefficients[10];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Trefoil90deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_SecondAstig0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[11] + zer_rel.zernike_coefficients[11];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondAstig0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_SecondAstig45deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[12] + zer_rel.zernike_coefficients[12];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondAstig45deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_SecondComa0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[13] + zer_rel.zernike_coefficients[13];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondComa0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_SecondComa90deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[14] +  zer_rel.zernike_coefficients[14];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondComa90deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_SecondSpherical(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[15] + zer_rel.zernike_coefficients[15];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondSpherical(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Quadrafoil0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[16] + zer_rel.zernike_coefficients[16];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Quadrafoil0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_Quadrafoil45deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[17] + zer_rel.zernike_coefficients[17];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Quadrafoil45deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_SecondTrefoil0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[18] + zer_rel.zernike_coefficients[18];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondTrefoil0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e::OnSetZernMode_SecondTrefoil90deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[19] + zer_rel.zernike_coefficients[19];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondTrefoil90deg(Acoef);
   }
   return DEVICE_OK;
}

// FAKE MIRROR class

Mirao52e_FAKE::Mirao52e_FAKE() :
   port_("Undefined"),
   initialized_(false),
   mirrorinitpath_(g_mirrorinit_path),
   calibpath_(g_calib_initpath),
   calibparamspath_(g_calibparams_initpath),
   divprefpath_(g_divpref_initpath),
   wfcpath_(g_wfc_initpath),
   savepath_(g_savepath)
{
   InitializeDefaultErrorMessages();
   // add custom messages
   std::string error_mirrorinit_file = "Mirror initialization file does not exist. Looking for: ";	error_mirrorinit_file.append(mirrorinitpath_.c_str());
   SetErrorText(ERR_MIRRORINIT_FILE_NONEXIST, error_mirrorinit_file.c_str());

   std::string error_divinit_file = "Diversity initialization file does not exist. Looking for: ";	error_divinit_file.append(calibpath_.c_str());
   SetErrorText(ERR_DIVINIT_FILE_NONEXIST, error_divinit_file.c_str());

   std::string error_cal_file = "Calibration parameter file does not exist. Looking for: ";	error_cal_file.append(calibparamspath_.c_str());
   SetErrorText(ERR_CAL_FILE_NONEXIST, error_cal_file.c_str());

   std::string error_divpref_file = "Diversity preferences file does not exist. Looking for: ";	error_divpref_file.append(divprefpath_.c_str());
   SetErrorText(ERR_DIVPREF_FILE_NONEXIST, error_divpref_file.c_str());

   SetErrorText(ERR_FILE_NONEXIST, "File does not exist");

   // create pre-initialization properties
   // ------------------------------------
   // Name
   CreateProperty(MM::g_Keyword_Name, g_DMname, MM::String, true);
   // Description
   CreateProperty(MM::g_Keyword_Description, "MIRAO-52E fake mirror", MM::String, true);
   // Port
   CPropertyAction* pAct = new CPropertyAction (this, &Mirao52e_FAKE::OnPort);
   CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);  
}

Mirao52e_FAKE::~Mirao52e_FAKE()
{
   if (initialized_)
      Shutdown();
}

bool Mirao52e_FAKE::Busy()
{
      return false;
}

void Mirao52e_FAKE::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, g_DMname);
}

// initialize function
int Mirao52e_FAKE::Initialize()
{
	if (initialized_)
    return DEVICE_OK;

	//Check if mirror initialization files exist
	if (!fileexists(mirrorinitpath_))
	{
		return ERR_MIRRORINIT_FILE_NONEXIST;
	}
	else if(!fileexists(calibpath_))
	{
		return ERR_DIVINIT_FILE_NONEXIST;
	}
	else if(!fileexists(calibparamspath_))
	{
		return ERR_CAL_FILE_NONEXIST;
	}
	else if(!fileexists(divprefpath_))
	{
		return ERR_DIVPREF_FILE_NONEXIST;
	}

	//init Mirror HW driver
    mirrorhandle = new imop::microscopy::Mirror(g_fakemirrorinit_path);
	mirrorhandle->init_hardware();

	//Load calibration file
	diversityhandle = new imop::microscopy::Diversity(calibpath_, *mirrorhandle );

    calibparamshandle = new imop::microscopy::CalibrationParams;
	calibparamshandle->Load(calibparamspath_);

    divprefshandle = new imop::microscopy::DiversityPreferences;
    divprefshandle->Load(divprefpath_);

    diversityhandle->Init_Diversity(*calibparamshandle,*divprefshandle);

	//Apply initial wavefront correction if WFC file exists
	if (fileexists(g_wfc_initpath))
	{
		Mirao52e_FAKE::LoadWavefront(g_wfc_initpath);
	}

	// Create action properties
	CPropertyAction* pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetCalibration);
	int ret = CreateProperty(g_SetCalibration, g_calib_initpath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetCalibrationParams);
	ret = CreateProperty(g_SetCalibrationParams, g_calibparams_initpath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetDiversityPref);
	ret = CreateProperty(g_SetDiversityPref, g_divpref_initpath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnLoadWavefront);
	ret = CreateProperty(g_LoadWavefront, g_wfc_initpath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSaveCurrentPosition);
	ret = CreateProperty(g_SaveCurrentPosition, g_savepath, MM::String, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;

	// Zernike Modes
    pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Tip);
    ret = CreateProperty(g_SetZernMode_Tip, "0", MM::Float, false, pAct);
    if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Tip, -1, 1);


    pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Tilt);
    ret = CreateProperty(g_SetZernMode_Tilt, "0", MM::Float, false, pAct);
    if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Tilt, -1, 1);


	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Defocus);
	ret = CreateProperty(g_SetZernMode_Defocus, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Defocus, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Astig0deg);
	ret = CreateProperty(g_SetZernMode_Astig0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Astig0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Astig45deg);
	ret = CreateProperty(g_SetZernMode_Astig45deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Astig45deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Coma0deg);
	ret = CreateProperty(g_SetZernMode_Coma0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Coma0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Coma90deg);
	ret = CreateProperty(g_SetZernMode_Coma90deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Coma90deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_PrimSpherical);
	ret = CreateProperty(g_SetZernMode_PrimSpherical, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_PrimSpherical, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Trefoil0deg);
	ret = CreateProperty(g_SetZernMode_Trefoil0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Trefoil0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Trefoil90deg);
	ret = CreateProperty(g_SetZernMode_Trefoil90deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Trefoil90deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_SecondAstig0deg);
	ret = CreateProperty(g_SetZernMode_SecondAstig0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondAstig0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_SecondAstig45deg);
	ret = CreateProperty(g_SetZernMode_SecondAstig45deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_SecondAstig45deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Quadrafoil0deg);
	ret = CreateProperty(g_SetZernMode_Quadrafoil0deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Quadrafoil0deg, -1, 1);

	pAct = new CPropertyAction(this, &Mirao52e_FAKE::OnSetZernMode_Quadrafoil45deg);
	ret = CreateProperty(g_SetZernMode_Quadrafoil45deg, "0", MM::Float, false, pAct);
	if (ret!=DEVICE_OK)
	   return ret;
	SetPropertyLimits(g_SetZernMode_Quadrafoil45deg, -1, 1);

	initialized_ = true;

	return DEVICE_OK;
}

// Shut down function
int Mirao52e_FAKE::Shutdown()
{
   initialized_    = false;
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
	{
      pProp->Set(port_.c_str());
	}
   else if (eAct == MM::AfterSet)
   {
      if (initialized_)
      {
         // revert
         pProp->Set(port_.c_str());
         return ERR_PORT_CHANGE_FORBIDDEN;
      }
      pProp->Get(port_);
   }
   return DEVICE_OK;
}


// load new calibration files
int Mirao52e_FAKE::SetCalibration(const std::string   path)
{
	if (fileexists(path))
	{
		calibpath_ = path;
		diversityhandle = new imop::microscopy::Diversity(calibpath_, *mirrorhandle);
		diversityhandle->Init_Diversity(*calibparamshandle,*divprefshandle);
	}
	else
	{
		return ERR_FILE_NONEXIST;
	}
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetDiversityPref(const std::string   path)
{
	if (fileexists(path))
	{
		divprefpath_ = path;
		divprefshandle->Load(divprefpath_);
		diversityhandle->Init_Diversity(*calibparamshandle,*divprefshandle);
	}
		else
	{
		return ERR_FILE_NONEXIST;
	}
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetCalibrationParams(const std::string  path)
{
	if (fileexists(path))
	{
		calibparamspath_ = path;
		calibparamshandle->Load(calibparamspath_);
		diversityhandle->Init_Diversity(*calibparamshandle,*divprefshandle);
	}
		else
	{
		return ERR_FILE_NONEXIST;
	}
	return DEVICE_OK;
}

int Mirao52e_FAKE::LoadWavefront(std::basic_string<char>  path)
{
	if (fileexists(path))
	{
		wfcpath_ = path;
		diversityhandle->Apply_Absolute_Commands_From_File(wfcpath_);
		zer_store.zernike_coefficients[1] = 0;
		zer_store.zernike_coefficients[2] = 0;
		zer_store.zernike_coefficients[3] = 0;
		zer_store.zernike_coefficients[4] = 0;
		zer_store.zernike_coefficients[5] = 0;
		zer_store.zernike_coefficients[6] = 0;
		zer_store.zernike_coefficients[7] = 0;
		zer_store.zernike_coefficients[8] = 0;
		zer_store.zernike_coefficients[9] = 0;
		zer_store.zernike_coefficients[10] = 0;
		zer_store.zernike_coefficients[11] = 0;
		zer_store.zernike_coefficients[12] = 0;
		zer_store.zernike_coefficients[13] = 0;
		zer_store.zernike_coefficients[14] = 0;
		zer_store.zernike_coefficients[15] = 0;
		zer_store.zernike_coefficients[16] = 0;
		zer_store.zernike_coefficients[17] = 0;
		zer_store.zernike_coefficients[18] = 0;
		zer_store.zernike_coefficients[19] = 0;

		zer_rel.zernike_coefficients[1] = 0;
		zer_rel.zernike_coefficients[2] = 0;
		zer_rel.zernike_coefficients[3] = 0;
		zer_rel.zernike_coefficients[4] = 0;
		zer_rel.zernike_coefficients[5] = 0;
		zer_rel.zernike_coefficients[6] = 0;
		zer_rel.zernike_coefficients[7] = 0;
		zer_rel.zernike_coefficients[8] = 0;
		zer_rel.zernike_coefficients[9] = 0;
		zer_rel.zernike_coefficients[10] = 0;
		zer_rel.zernike_coefficients[11] = 0;
		zer_rel.zernike_coefficients[12] = 0;
		zer_rel.zernike_coefficients[13] = 0;
		zer_rel.zernike_coefficients[14] = 0;
		zer_rel.zernike_coefficients[15] = 0;
		zer_rel.zernike_coefficients[16] = 0;
		zer_rel.zernike_coefficients[17] = 0;
		zer_rel.zernike_coefficients[18] = 0;
		zer_rel.zernike_coefficients[19] = 0;
		Sleep(10);
	}
	else
	{
		return ERR_FILE_NONEXIST;
	}
	return DEVICE_OK;
}

int Mirao52e_FAKE::SaveCurrentPosition(std::basic_string<char> path)
{
	savepath_ = path;
	diversityhandle->Save_Current_Positions_ToFile(savepath_);
	return DEVICE_OK;
}

/*
// Get Actuator positions //
int Mirao52e_FAKE::GetActuatorPos(std::vector<float> pos)
{
	pos = mirrorhandle->Get_Current_Position();
	return DEVICE_OK;
}
*/

// Set Zernike modes
int Mirao52e_FAKE::SetZernMode_Tip(float Acoef)
{
	zer_rel.zernike_coefficients[1] = Acoef - zer_store.zernike_coefficients[1];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[1] = Acoef;
	zer_rel.zernike_coefficients[1] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Tilt(float Acoef)
{
	zer_rel.zernike_coefficients[2] = Acoef - zer_store.zernike_coefficients[2];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[2] = Acoef;
	zer_rel.zernike_coefficients[2] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Defocus(float Acoef)
{
	zer_rel.zernike_coefficients[3] = Acoef - zer_store.zernike_coefficients[3];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[3] = Acoef;
	zer_rel.zernike_coefficients[3] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Astig0deg(float Acoef)
{
	zer_rel.zernike_coefficients[4] = Acoef - zer_store.zernike_coefficients[4];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[4] = Acoef;
	zer_rel.zernike_coefficients[4] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Astig45deg(float Acoef)
{
	zer_rel.zernike_coefficients[5] = Acoef - zer_store.zernike_coefficients[5];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[5] = Acoef;
	zer_rel.zernike_coefficients[5] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Coma0deg(float Acoef)
{
	zer_rel.zernike_coefficients[6] = Acoef - zer_store.zernike_coefficients[6];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[6] = Acoef;
	zer_rel.zernike_coefficients[6] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Coma90deg(float Acoef)
{
	zer_rel.zernike_coefficients[7] = Acoef - zer_store.zernike_coefficients[7];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[7] = Acoef;
	zer_rel.zernike_coefficients[7] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_PrimSpherical(float Acoef)
{
	zer_rel.zernike_coefficients[8] = Acoef - zer_store.zernike_coefficients[8];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[8] = Acoef;
	zer_rel.zernike_coefficients[8] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Trefoil0deg(float Acoef)
{
	zer_rel.zernike_coefficients[9] = Acoef - zer_store.zernike_coefficients[9];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[9] = Acoef;
	zer_rel.zernike_coefficients[9] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Trefoil90deg(float Acoef)
{
	zer_rel.zernike_coefficients[10] = Acoef - zer_store.zernike_coefficients[10];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[10] = Acoef;
	zer_rel.zernike_coefficients[10] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_SecondAstig0deg(float Acoef)
{
	zer_rel.zernike_coefficients[11] = Acoef - zer_store.zernike_coefficients[11];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[11] = Acoef;
	zer_rel.zernike_coefficients[11] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_SecondAstig45deg(float Acoef)
{
	zer_rel.zernike_coefficients[12] = Acoef - zer_store.zernike_coefficients[12];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[12] = Acoef;
	zer_rel.zernike_coefficients[12] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Quadrafoil0deg(float Acoef)
{
	zer_rel.zernike_coefficients[16] = Acoef - zer_store.zernike_coefficients[16];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[16] = Acoef;
	zer_rel.zernike_coefficients[16] = 0;
	Sleep(10);
	return DEVICE_OK;
}

int Mirao52e_FAKE::SetZernMode_Quadrafoil45deg(float Acoef)
{
	zer_rel.zernike_coefficients[17] = Acoef - zer_store.zernike_coefficients[17];
	diversityhandle->Apply_Relative_Commands(zer_rel);
	zer_store.zernike_coefficients[17] = Acoef;
	zer_rel.zernike_coefficients[17] = 0;
	Sleep(10);
	return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// Action handlers
// Handle changes and updates to property values.
///////////////////////////////////////////////////////////////////////////////


int Mirao52e_FAKE::OnSetCalibration(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
		pProp->Set(calibpath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return SetCalibration(path);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetCalibrationParams(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	   pProp->Set(calibparamspath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return SetCalibrationParams(path);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetDiversityPref(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	// string val;
    pProp->Set(divprefpath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return SetDiversityPref(path);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnLoadWavefront(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
    pProp->Set(wfcpath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return LoadWavefront(path);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSaveCurrentPosition(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
   pProp->Set(savepath_.c_str()); 
   }
   else if (eAct == MM::AfterSet)
   {
      std::basic_string<char> path;;
      pProp->Get(path);
      return SaveCurrentPosition(path);
   }
   return DEVICE_OK;
}

// Zernike modes
int Mirao52e_FAKE::OnSetZernMode_Tip(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[1];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Tip(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Tilt(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[2];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Tilt(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Defocus(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[3];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Defocus(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Astig0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[4];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Astig0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Astig45deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[5];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Astig45deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Coma0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[6];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Coma0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Coma90deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[7];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Coma90deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_PrimSpherical(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[8];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_PrimSpherical(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Trefoil0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[9];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Trefoil0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Trefoil90deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[10];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Trefoil90deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_SecondAstig0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[11];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondAstig0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_SecondAstig45deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[12];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_SecondAstig45deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Quadrafoil0deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[16];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Quadrafoil0deg(Acoef);
   }
   return DEVICE_OK;
}

int Mirao52e_FAKE::OnSetZernMode_Quadrafoil45deg(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	  double Acoef;
	  Acoef = zer_store.zernike_coefficients[17];
      pProp->Set(Acoef); 
   }
   else if (eAct == MM::AfterSet)
   {
      double Acoef;
      pProp->Get(Acoef);
      return SetZernMode_Quadrafoil45deg(Acoef);
   }
   return DEVICE_OK;
}
