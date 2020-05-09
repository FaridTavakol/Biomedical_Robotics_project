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

int main()
{
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
    //----------------------------------FK computation --------------------------------------------------------
    NeuroKinematics Forward(&probe_init);
    Neuro_FK_outputs FK{};

    // Initializing the counters for nan output
    // Min allowed seperation 75mm
    // Max allowed seperation  146mm
    const double Diff{71}; // Is the max allowed movement while one block is stationary 146-75 = 71 mm
    int nan_checker_row{};
    int nan_checker_col{};
    double pi{3.141};
    double Ry{};                    // Initializing the PitchRotation counter
    double RyF_max{-37 * pi / 180}; // in paper is 37.2
    double RyB_max{+30 * pi / 180}; // in paper is  30.6
    double Rx{};                    // Initializing the YawRotation counter
    double Rx_max{-90 * pi / 180};

    ofstream myout("test.xyz");

    // loop for visualizing the bottom
    double i{}, j{}, k{};
    for (i = 0, j = 0; i < 87; ++i, ++j) //75
    {
        AxialFeetTranslation = i;
        AxialHeadTranslation = j;
        for (k = 0; k <= 37.5; k += 0.5)
        {
            LateralTranslation = k;

            for (Rx = 0; Rx >= -90; --Rx)
            {
                YawRotation = Rx * pi / 180;
                FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                               LateralTranslation, ProbeInsertion,
                                               ProbeRotation, PitchRotation, YawRotation);
                for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
                {
                    for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                    {

                        if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                        {
                            std::cout << "row :" << nan_checker_row << "cloumn :"
                                      << "is nan!\n";
                            break;
                        }
                    }
                }
                myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;
            }
            YawRotation = 0;
        }
    }

    // Loop for visualizing the top
    for (i = 0, j = -71; i < 157; ++i, ++j) // 201 to 157 it should be 201 based on the paper
    {
        AxialFeetTranslation = i;
        AxialHeadTranslation = j;
        for (k = 0; k <= 37.5; k += 0.5)
        {
            LateralTranslation = k;
            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                           LateralTranslation, ProbeInsertion,
                                           ProbeRotation, PitchRotation, YawRotation);
            for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
            {
                for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                {

                    if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                    {
                        std::cout << "row :" << nan_checker_row << "cloumn :" << nan_checker_col
                                  << "is nan!\n";
                        break;
                    }
                }
            }

            myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;

            // }
        }
    }

    AxialFeetTranslation = 0;
    AxialHeadTranslation = 0;

    i = 0;
    j = -1;
    k = 0;
    // Loop for creating the feet face
    for (j = -1; Diff > abs(AxialHeadTranslation - AxialFeetTranslation); --j)
    {
        AxialHeadTranslation = j;
        for (k = 0; k <= 37.5; k += 0.5)
        {
            LateralTranslation = k;
            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                           LateralTranslation, ProbeInsertion,
                                           ProbeRotation, PitchRotation, YawRotation);
            for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
            {
                for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                {

                    if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                    {
                        std::cout << "row :" << nan_checker_row << "cloumn :" << nan_checker_col
                                  << "is nan!\n";
                        break;
                    }
                }
            }

            myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;
        }
    }

    // Loop for creating the Head face
    // j = 200 i = 200 i++ 71 = 271 ;
    AxialHeadTranslation = 86;
    AxialFeetTranslation = 86;
    nan_checker_row = 0;
    nan_checker_col = 0;
    i = 86;
    j = 86;
    k = 0;
    for (i = 87; Diff > abs(AxialHeadTranslation - AxialFeetTranslation); ++i)
    {
        AxialFeetTranslation = i;

        for (k = 0; k <= 37.5; k += 0.5)
        {
            LateralTranslation = k;

            if (abs(AxialHeadTranslation - AxialFeetTranslation) == Diff - 1)
            {
                for (Rx = 0; Rx >= -90; --Rx)
                {
                    YawRotation = Rx * pi / 180;
                    FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                   LateralTranslation, ProbeInsertion,
                                                   ProbeRotation, PitchRotation, YawRotation);
                    for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
                    {
                        for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                        {

                            if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                            {
                                std::cout << "row :" << nan_checker_row << "cloumn :"
                                          << "is nan!\n";
                                break;
                            }
                        }
                    }
                    // std::cout << "X Position :" << FK.zFrameToTreatment(0, 3) << std::endl;
                    // std::cout << "Y Position :" << FK.zFrameToTreatment(1, 3) << std::endl;
                    // std::cout << "Z Position :" << FK.zFrameToTreatment(2, 3) << std::endl;
                    myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;
                }
            }
            else
            {
                YawRotation = Rx_max;
                FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                               LateralTranslation, ProbeInsertion,
                                               ProbeRotation, PitchRotation, YawRotation);
                for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
                {
                    for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                    {

                        if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                        {
                            std::cout << "row :" << nan_checker_row << "cloumn :"
                                      << "is nan!\n";
                            break;
                        }
                    }
                }
                // std::cout << "X Position :" << FK.zFrameToTreatment(0, 3) << std::endl;
                // std::cout << "Y Position :" << FK.zFrameToTreatment(1, 3) << std::endl;
                // std::cout << "Z Position :" << FK.zFrameToTreatment(2, 3) << std::endl;
                // std::cout << "Feet Position :" << AxialFeetTranslation << std::endl;
                myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;
            }
        }
    }
    YawRotation = 0; //resetting the Yaw value to 0

    //loop for creating the sides
    AxialFeetTranslation = 0;
    AxialHeadTranslation = 0;
    LateralTranslation = 0;
    nan_checker_row = 0;
    nan_checker_col = 0;
    i = 0;
    j = -1;
    k = 0;
    double ii{};
    // double jj{};
    double min_travel{86};
    double max_travel{200};
    for (j = -1; Diff > abs(j); --j)
    {
        AxialHeadTranslation = j;
        ++min_travel;

        for (ii = 0; ii <= min_travel && min_travel <= max_travel; ++ii)
        {
            ++AxialHeadTranslation;
            ++AxialFeetTranslation;

            for (k = 0; k <= 37.5; k += 37.5)
            {
                LateralTranslation = k;

                for (Rx = 0; Rx >= -90; --Rx)
                {
                    YawRotation = Rx * pi / 180;

                    if (k == 0 && abs(j) == Diff - 1) // towards face and top of workspace
                    {

                        for (Ry = 0; Ry >= -37; --Ry)
                        {
                            PitchRotation = Ry * pi / 180;
                            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                           LateralTranslation, ProbeInsertion,
                                                           ProbeRotation, PitchRotation, YawRotation);
                            for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
                            {
                                for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                                {

                                    if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                                    {
                                        std::cout << "row :" << nan_checker_row << "cloumn :"
                                                  << "is nan!\n";
                                        break;
                                    }
                                }
                            }
                            myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;
                        }
                    }
                    else if (k == 0 && abs(j) != Diff - 1) // towards face from bottom to a point before max heigth
                    {
                        PitchRotation = RyF_max;
                        FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                       LateralTranslation, ProbeInsertion,
                                                       ProbeRotation, PitchRotation, YawRotation);
                        for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
                        {
                            for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                            {

                                if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                                {
                                    std::cout << "row :" << nan_checker_row << "cloumn :"
                                              << "is nan!\n";
                                    break;
                                }
                            }
                        }
                        myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;
                    }
                    else if (k == 37.5 && abs(j) == Diff - 1) // towards bore and top of workspace
                    {
                        for (Ry = 0; Ry <= 30; ++Ry)
                        {
                            PitchRotation = Ry * pi / 180;
                            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                           LateralTranslation, ProbeInsertion,
                                                           ProbeRotation, PitchRotation, YawRotation);
                            for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
                            {
                                for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                                {

                                    if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                                    {
                                        std::cout << "row :" << nan_checker_row << "cloumn :"
                                                  << "is nan!\n";
                                        break;
                                    }
                                }
                            }
                            myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;
                        }
                    }
                    else if (k == 37.5 && abs(j) != Diff - 1) // towards bore from bottom to a point before max heigth
                    {
                        PitchRotation = RyB_max;
                        FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                       LateralTranslation, ProbeInsertion,
                                                       ProbeRotation, PitchRotation, YawRotation);
                        for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
                        {
                            for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
                            {

                                if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
                                {
                                    std::cout << "row :" << nan_checker_row << "cloumn :"
                                              << "is nan!\n";
                                    break;
                                }
                            }
                        }
                        myout << FK.zFrameToTreatment(0, 3) << " " << FK.zFrameToTreatment(1, 3) << " " << FK.zFrameToTreatment(2, 3) << " 0.00 0.00 0.00" << endl;
                    }
                }
            }
            YawRotation = 0;
        }
        AxialHeadTranslation = 0;
        AxialFeetTranslation = 0;
        LateralTranslation = 0;
    }

    // =================================Desired point checker =============================================================================
    // AxialFeetTranslation = 13.5909;
    // AxialHeadTranslation = 20.5828;
    // LateralTranslation = 9.68;
    // FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
    //                                LateralTranslation, ProbeInsertion,
    //                                ProbeRotation, PitchRotation, YawRotation);
    // std::cout << "zFrameToTreatment :" << FK.zFrameToTreatment << std::endl;

    // std::cout << "X Position :" << FK.zFrameToTreatment(0, 3) << std::endl;
    // std::cout << "Y Position :" << FK.zFrameToTreatment(1, 3) << std::endl;
    // std::cout << "Z Position :" << FK.zFrameToTreatment(2, 3) << std::endl;
    myout.close();

    return 0;
}