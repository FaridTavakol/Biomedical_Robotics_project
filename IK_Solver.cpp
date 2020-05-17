#include "NeuroKinematics/NeuroKinematics.hpp"
#include <iostream>
#include <string>
#include <math.h> /* isnan, sqrt */
#include <cstdlib>
#include <fstream>
using std::endl;
using std::ofstream;

// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;

// Initializing the joint variables for use in the FK function
double AxialHeadTranslation{0.0};
double AxialFeetTranslation{0.0};
double LateralTranslation{0.0};
double PitchRotation{0.0};
double YawRotation{0.0};
double ProbeInsertion{0.0};
double ProbeRotation{0.0};
// A is treatment to tip, where treatment is the piezoelectric element,// A = 10mm
// B is robot to entry, this allows us to specify how close to the patient the physical robot can be,// B = 5mm
// C is cannula to treatment, we define this so the robot can compute the cannula length,// C = 5mm
// D is the robot to treatment distance,// D = 41mm
// Creating an object called Forward for FK
// In the neuroRobot.cpp the specs for the  probe are: 0,0,5,41
double _cannulaToTreatment{5.0};
double _treatmentToTip{10.0};
double _robotToEntry{5.0};
double _robotToTreatmentAtHome{41.0};
Probe probe_init = {
    _cannulaToTreatment,
    _treatmentToTip,
    _robotToEntry,
    _robotToTreatmentAtHome};
// Yaw rotation range : -1.54 - +0.01
// Probe Rotation range : -6.28 - +6.28
// Pitch Rotation range : -0.46 - 0.65
// Probe Insertion range : 0.00 - 34.93
// Lateral Translation range : -49.47 - 0.00
// Axial Head Translation range : -145.01 - 0.00
// Axial Feet Translation range : -70.00 - 75.01 -> Experimental range from -7 to 233 inclusive
double i{}, j{}, k{}; //counter initialization

int nan_checker_row{};
int nan_checker_col{};
// Function to search for nan values in the FK output
int nan_ckecker(Neuro_FK_outputs FK)
{
    for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
    {
        for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
        {

            if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
            {
                std::cout << "row :" << nan_checker_row << "cloumn :"
                          << "is nan!\n";
                return 1;
                break;
            }
        }
    }
    return 0;
};
// Y range in the robot frame should be between 158.5 mm - 218
// This script calculates the desired joint values for joints 1-3 to place the RCM in the entry point
int main()
{
    while (true)
    {
        NeuroKinematics Forward(&probe_init);

        Eigen::Vector4d entryPointScanner{};
        std::cout << "Enter Desired X value :";
        std::cin >> entryPointScanner(0);
        std::cout << "\nEnter Desired Y value :";
        std::cin >> entryPointScanner(1);
        std::cout << "\nEnter Desired Z value :";
        std::cin >> entryPointScanner(2);
        entryPointScanner(3) = 1;

        // An arbitrary Registration matrix is selected. This matrix is dependant of the Imager
        Eigen::Matrix4d _registration;
        _registration = (Eigen::Matrix4d() << 1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, 0,
                         0, 0, 0, 1)
                            .finished();
        Eigen::Vector4d zFrameToEntry = _registration.inverse() * entryPointScanner;
        if (zFrameToEntry(1) >= 158.5 && zFrameToEntry(1) <= 218)
        {
            // Start of the IK_solver code
            IK_Solver_outputs IK = Forward.IK_solver(zFrameToEntry);
            std::cout << "IK values for AxialHeadTranslation :" << IK.AxialHeadTranslation << std::endl;
            std::cout << "IK values for AxialFeetTranslation :" << IK.AxialFeetTranslation << std::endl;
            std::cout << "IK values for LateralTranslationition :" << IK.LateralTranslation << std::endl;
            // =================================Desired point checker =============================================================================
            // NeuroKinematics Forward(&probe_init);
            Eigen::Vector4d Input_robot;
            ProbeInsertion = 31.5;
            Input_robot << IK.AxialHeadTranslation, IK.AxialFeetTranslation, IK.LateralTranslation, ProbeInsertion;
            Neuro_FK_outputs FK{};

            AxialFeetTranslation = 0;
            AxialHeadTranslation = 0;
            LateralTranslation = 0;
            ProbeInsertion = 31.5;
            FK = Forward.ForwardKinematics(Input_robot(0), Input_robot(1),
                                           Input_robot(2), Input_robot(3),
                                           ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);

            std::cout << "zFrameToTreatment :\n"
                      << FK.zFrameToTreatment << std::endl;

            std::cout << "X Position :" << FK.zFrameToTreatment(0, 3) << std::endl;
            std::cout << "Y Position :" << FK.zFrameToTreatment(1, 3) << std::endl;
            std::cout << "Z Position :" << FK.zFrameToTreatment(2, 3) << std::endl;
            // i = false;
            break;
        }
        else
        {
            std::cout << "Your entry point is out of reach! Please choose another point and try again!" << std::endl;
        }
    }

    return 0;
}