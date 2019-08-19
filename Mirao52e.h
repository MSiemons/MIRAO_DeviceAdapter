///////////////////////////////////////////////////////////////////////////////
// FILE:          Mirao52e.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Device adapter for MICAO-52E deformable mirror
//
// AUTHOR:        Marijn Siemons. m.e.siemons@uu.nl, 27-02-2018

#pragma once

#define NOMINMAX
#define IMPORT_IMOP_WAVEKITBIO_FROM_LIBRARY

#include "../../MMDevice/MMDevice.h"
#include "../../MMDevice/DeviceBase.h"
#include <string>
#include <sstream>
#include "Mirror.hpp"
#include "Model.h"
#include "PhaseDiversity.h"
#include "3NAlgorithm.h"
#include "merit_functions.hpp"
#include "conversion.hpp"

//////////////////////////////////////////////////////////////////////////////
// Error codes
//
#define ERR_PORT_CHANGE_FORBIDDEN		10001
#define ERR_MIRRORINIT_FILE_NONEXIST	10201
#define ERR_DIVINIT_FILE_NONEXIST		10202
#define ERR_CAL_FILE_NONEXIST			10203
#define ERR_DIVPREF_FILE_NONEXIST		10204
#define ERR_FILE_NONEXIST				10205

class Mirao52e : public	CGenericBase<Mirao52e>
{
public:
	Mirao52e(void);
	~Mirao52e(void);

   // Device API
   // ----------
   int Initialize();
   int Shutdown();
   void GetName(char* name) const; 
   bool Busy();
   
   // Deformable mirror API
   // ---------
   imop::microscopy::Mirror * mirrorhandle;
   imop::microscopy::Diversity * diversityhandle;
   imop::microscopy::CalibrationParams * calibparamshandle;
   imop::microscopy::DiversityPreferences * divprefshandle;
   imop::microscopy::Zernikes zer_store;
   imop::microscopy::Zernikes zer_rel;

 //  int GetActuatorPos(std::vector<float> pos);
   int SetCalibration(std::basic_string<char> path);
   int SetDiversityPref(std::basic_string<char> path);
   int SetCalibrationParams(std::basic_string<char> path);
   int LoadWavefront(std::basic_string<char> path);
   int SaveCurrentPosition(std::basic_string<char> path);
   int ApplyZernmodes();

   int SetZernMode_Tip(float Acoef);
   int SetZernMode_Tilt(float Acoef);
   int SetZernMode_Defocus(float Acoef);
   int SetZernMode_Astig0deg(float Acoef);
   int SetZernMode_Astig45deg(float Acoef);
   int SetZernMode_Coma0deg(float Acoef);
   int SetZernMode_Coma90deg(float Acoef);
   int SetZernMode_PrimSpherical(float Acoef);
   int SetZernMode_Trefoil0deg(float Acoef);
   int SetZernMode_Trefoil90deg(float Acoef);
   int SetZernMode_SecondAstig0deg(float Acoef);
   int SetZernMode_SecondAstig45deg(float Acoef);
   int SetZernMode_Quadrafoil0deg(float Acoef);
   int SetZernMode_Quadrafoil45deg(float Acoef);
   int SetZernMode_SecondComa0deg(float Acoef);
   int SetZernMode_SecondComa90deg(float Acoef);
   int SetZernMode_SecondTrefoil0deg(float Acoef);
   int SetZernMode_SecondTrefoil90deg(float Acoef);
   int SetZernMode_SecondSpherical(float Acoef);

   // action interface
   // ----------------
   int OnPort     (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetCalibration    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetCalibrationParams    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetDiversityPref    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnLoadWavefront    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSaveCurrentPosition    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnApplyZernmodes (MM::PropertyBase* pProp, MM::ActionType eAct);

   int OnSetZernMode_Tip    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Tilt    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Defocus    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Astig0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Astig45deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Coma0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Coma90deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_PrimSpherical    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Trefoil0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Trefoil90deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondAstig0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondAstig45deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Quadrafoil0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Quadrafoil45deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondComa0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondComa90deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondTrefoil0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondTrefoil90deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondSpherical    (MM::PropertyBase* pProp, MM::ActionType eAct);

   std::string mirrorinitpath_;
   std::string calibpath_;
   std::string calibparamspath_;
   std::string divprefpath_;
   std::string wfcpath_;
   std::string savepath_;

protected:
   bool initialized_;
   std::string port_;
   MM::Device *device_;
   MM::Core *core_;
};


class Mirao52e_FAKE : public	CGenericBase<Mirao52e_FAKE>
{
public:
	Mirao52e_FAKE(void);
	~Mirao52e_FAKE(void);

   // Device API
   // ----------
   int Initialize();
   int Shutdown();
   void GetName(char* name) const; 
   bool Busy();
   
   // Deformable mirror API
   // ---------
   imop::microscopy::Mirror * mirrorhandle;
   imop::microscopy::Diversity * diversityhandle;
   imop::microscopy::CalibrationParams * calibparamshandle;
   imop::microscopy::DiversityPreferences * divprefshandle;
   imop::microscopy::Zernikes zer_store;
   imop::microscopy::Zernikes zer_rel;

 //  int GetActuatorPos(std::vector<float> pos);
   int SetCalibration(std::basic_string<char> path);
   int SetDiversityPref(std::basic_string<char> path);
   int SetCalibrationParams(std::basic_string<char> path);
   int LoadWavefront(std::basic_string<char> path);
   int SaveCurrentPosition(std::basic_string<char> path);

   int SetZernMode_Tip(float Acoef);
   int SetZernMode_Tilt(float Acoef);
   int SetZernMode_Defocus(float Acoef);
   int SetZernMode_Astig0deg(float Acoef);
   int SetZernMode_Astig45deg(float Acoef);
   int SetZernMode_Coma0deg(float Acoef);
   int SetZernMode_Coma90deg(float Acoef);
   int SetZernMode_PrimSpherical(float Acoef);
   int SetZernMode_Trefoil0deg(float Acoef);
   int SetZernMode_Trefoil90deg(float Acoef);
   int SetZernMode_SecondAstig0deg(float Acoef);
   int SetZernMode_SecondAstig45deg(float Acoef);
   int SetZernMode_Quadrafoil0deg(float Acoef);
   int SetZernMode_Quadrafoil45deg(float Acoef);

   // action interface
   // ----------------
   int OnPort     (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetCalibration    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetCalibrationParams    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetDiversityPref    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnLoadWavefront    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSaveCurrentPosition    (MM::PropertyBase* pProp, MM::ActionType eAct);

   int OnSetZernMode_Tip    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Tilt    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Defocus    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Astig0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Astig45deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Coma0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Coma90deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_PrimSpherical    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Trefoil0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Trefoil90deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondAstig0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_SecondAstig45deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Quadrafoil0deg    (MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSetZernMode_Quadrafoil45deg    (MM::PropertyBase* pProp, MM::ActionType eAct);

   std::string mirrorinitpath_;
   std::string calibpath_;
   std::string calibparamspath_;
   std::string divprefpath_;
   std::string wfcpath_;
   std::string savepath_;

protected:
   bool initialized_;
   std::string port_;
   MM::Device *device_;
   MM::Core *core_;
};

