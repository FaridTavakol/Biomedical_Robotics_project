#include "NeuroKinematics/NeuroKinematics.hpp"
#include <iostream>
#include <string>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// Initializing the joint variables for use in the FK function
double AxialHeadTranslation{0.0};
double AxialFeetTranslation{0.0};
double LateralTranslation{0.0};
double PitchRotation{0.0};
double YawRotation{0.0};
double ProbeInsertion{0.0};
double ProbeRotation{0.0};

int main()
{

  // Creating an object called Forward for FK
  double _cannulaToTreatment{0.0};
  double _treatmentToTip{0.0};
  double _robotToEntry{0.0};
  double _robotToTreatmentAtHome{0.0};
  Probe probe_init = {
      _cannulaToTreatment,
      _treatmentToTip,
      _robotToEntry,
      _robotToTreatmentAtHome};

  std::cout << "The probe cannula to treatment is:" << probe_init._cannulaToTreatment << std::endl;
  Probe *probe1 = &probe_init;

  NeuroKinematics Forward(probe1);

  std::cout << Forward._lengthOfAxialTrapezoidSideLink << std::endl;
  std::cout << Forward._widthTrapezoidTop << std::endl;
  std::cout << Forward._initialAxialSeperation << std::endl;
  std::cout << Forward._xInitialRCM << std::endl;
  std::cout << Forward._yInitialRCM << std::endl;
  std::cout << Forward._zInitialRCM << std::endl;
  std::cout << Forward._robotToRCMOffset << std::endl;
  std::cout << Forward._probe->_treatmentToTip << std::endl;
  std::cout << "the code built successfully" << std::endl;
  std::cout << ProbeRotation << std::endl;

  Neuro_FK_outputs FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                  LateralTranslation, ProbeInsertion,
                                                  ProbeRotation, PitchRotation, YawRotation);
  std::cout << FK.zFrameToTreatment(3, 3) << std::endl;

  return 0;
}