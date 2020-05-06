#include "NeuroKinematics/NeuroKinematics.hpp"
#include <iostream>
#include <string>
#include <math.h> /* isnan, sqrt */
#include <cstdlib>
#include <eigen3/Eigen/Dense>
using namespace std;

//=================== Neuro Robot Poses ======================
// Transformation from scanner to robot zFrame
//_registration = Eigen::Matrix4d::Identity();
double _cannulaToTreatment{5.0};
double _treatmentToTip{10.0};
double _robotToEntry{5.0};
double _robotToTreatmentAtHome{41.0};
double AxialHeadTranslation{0.0};
double AxialFeetTranslation{0.0};
double LateralTranslation{0.0};
double PitchRotation{0.0};
double YawRotation{0.0};
double ProbeInsertion{0.0};
double ProbeRotation{0.0};

int main()
{ // begin test -----------------------------------
    Probe probe1 = {_cannulaToTreatment, _treatmentToTip, _robotToEntry, _robotToTreatmentAtHome};
    Probe *_probe{&probe1}; // Creating a pointer Probe1 of type Probe that points to the address of probe_init
    NeuroKinematics Inverse(_probe);
    //end test------------------------------------
    // Probe _probe = {_cannulaToTreatment, _treatmentToTip, _robotToEntry, _robotToTreatmentAtHome};
    // Probe *probe1{&_probe}; // Creating a pointer Probe1 of type Probe that points to the address of probe_init
    // NeuroKinematics Inverse(&_probe);
    // Current and Target pose are defaulted to identity
    // These transformations are with respect to the Imager Coordinate Frame
    Eigen::Matrix4d _currentPose;
    Eigen::Matrix4d _targetPose;
    _currentPose = Eigen::Matrix4d::Identity();
    _targetPose = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d _registration;
    _registration = (Eigen::Matrix4d() << 1, 0, 0, 5,
                     0, 1, 0, 5,
                     0, 0, 1, 5,
                     0, 0, 0, 1)
                        .finished();

    // Desired values for the entry and target points
    Eigen::Vector3d e1(33, 230, 95);
    Eigen::Vector3d t1(33, 225, 95);

    Eigen::Vector3d _entryPoint;  // Entry point for the desired pass-through location of the treatment zone
    Eigen::Vector3d _targetPoint; // Target point for the final location of the treatment zone
    _entryPoint = e1;
    _targetPoint = t1;
    // Entry and Target Points are received in scanner coordinates
    // Multiply them by the registration matrix to obtain them in zFrame coordinates
    Eigen::Vector4d entryPointScannerCoordinates(_entryPoint(0), _entryPoint(1), _entryPoint(2), 1);
    Eigen::Vector4d targetPointScannerCoordinates(_targetPoint(0), _targetPoint(1), _targetPoint(2), 1);
    // Calculate zFrameToEntry
    Eigen::Vector4d zFrameToEntry = _registration.inverse() * entryPointScannerCoordinates;

    // Calculate zFrameToTarget
    Eigen::Vector4d zFrameToTarget = _registration.inverse() * targetPointScannerCoordinates;

    // Perform Inverse Kinematics
    // Input results into the InverseKinematics which expects entry point and target point to be with respect to the zFrame
    Neuro_IK_outputs IK{};
    double XEntry = zFrameToEntry(0);
    double YEntry = zFrameToEntry(1);
    double ZEntry = zFrameToEntry(2);
    // The Lateral Translation is given by the desired distance in x
    IK.LateralTranslation = XEntry - Inverse._xInitialRCM - Inverse._robotToRCMOffset * sin(IK.PitchRotation) + _probe->_robotToEntry * sin(IK.PitchRotation);

    // Equations calculated through the symbolic equations for the Forward Kinematics
    // Substituting known values in the FK equations yields the value for Axial Head and Feet
    IK.AxialHeadTranslation = ZEntry - Inverse._initialAxialSeperation / 2 + Inverse._widthTrapezoidTop / 2 - Inverse._zInitialRCM + sqrt(8 * YEntry * Inverse._yInitialRCM - 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop - 4 * YEntry * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) + 4 * Inverse._yInitialRCM * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) - 4 * pow(YEntry, 2) + pow(Inverse._initialAxialSeperation, 2) + pow(Inverse._widthTrapezoidTop, 2) - 4 * pow(Inverse._yInitialRCM, 2) - 4 * pow(Inverse._robotToRCMOffset, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) - 4 * pow(_probe->_robotToEntry, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * Inverse._robotToRCMOffset * _probe->_robotToEntry * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * YEntry * Inverse._robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * YEntry * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * Inverse._robotToRCMOffset * Inverse._yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 8 * _probe->_robotToEntry * Inverse._yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 4 * Inverse._robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) - 4 * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2))) / 2 + Inverse._robotToRCMOffset * cos(IK.PitchRotation) * sin(IK.YawRotation) - _probe->_robotToEntry * cos(IK.PitchRotation) * sin(IK.YawRotation);
    IK.AxialFeetTranslation = ZEntry + Inverse._initialAxialSeperation / 2 - Inverse._widthTrapezoidTop / 2 - Inverse._zInitialRCM - sqrt(8 * YEntry * Inverse._yInitialRCM - 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop - 4 * YEntry * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) + 4 * Inverse._yInitialRCM * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) - 4 * pow(YEntry, 2) + pow(Inverse._initialAxialSeperation, 2) + pow(Inverse._widthTrapezoidTop, 2) - 4 * pow(Inverse._yInitialRCM, 2) - 4 * pow(Inverse._robotToRCMOffset, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) - 4 * pow(_probe->_robotToEntry, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * Inverse._robotToRCMOffset * _probe->_robotToEntry * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * YEntry * Inverse._robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * YEntry * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * Inverse._robotToRCMOffset * Inverse._yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 8 * _probe->_robotToEntry * Inverse._yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 4 * Inverse._robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) - 4 * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2))) / 2 + Inverse._robotToRCMOffset * cos(IK.PitchRotation) * sin(IK.YawRotation) - _probe->_robotToEntry * cos(IK.PitchRotation) * sin(IK.YawRotation);

    Neuro_IK_outputs IK_output = Inverse.InverseKinematics(zFrameToEntry, zFrameToTarget);
    _targetPose = _registration * (Eigen::Matrix4d() << IK_output.targetPose).finished();

    // Printing stuff
    cout << "Target Pose is : " << _targetPose << endl;
    cout << "IK_output.Target Pose is : " << IK_output.targetPose << endl;
    cout << "Desired Lateral translation : " << IK_output.LateralTranslation << endl;
    cout << "Desired AxialHead translation :  " << IK_output.AxialHeadTranslation << endl;
    cout << "Desired AxialFeet translation :  " << IK_output.AxialFeetTranslation << endl;
    cout << "Location of the selected point in the Robot's Cartesian Space : \n"
         << zFrameToEntry << endl;

    // cout << "Desired ProbeInsertion is : " << IK_output.ProbeInsertion << endl;
    // cout << "Desired Probe Rotation is : " << IK_output.ProbeRotation << endl;
    // cout << "Desired Yaw Rotation is : " << IK_output.YawRotation << endl;
    // cout << "Desired Pitch Rotation is : " << IK_output.PitchRotation << endl;

    cout << "======================FK Output=========================" << endl;

    AxialFeetTranslation = IK_output.AxialFeetTranslation;
    AxialHeadTranslation = IK_output.AxialHeadTranslation;
    LateralTranslation = IK_output.LateralTranslation;
    ProbeInsertion = IK_output.ProbeInsertion;

    Neuro_FK_outputs FK = Inverse.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);

    cout << "X Position :" << FK.zFrameToTreatment(0, 3) << endl;
    cout << "Y Position :" << FK.zFrameToTreatment(1, 3) << endl;
    cout << "Z Position :" << FK.zFrameToTreatment(2, 3) << endl;
    cout << "zFrameToTreatment :\n"
         << FK.zFrameToTreatment << endl;

    ProbeInsertion = 0;

    Neuro_FK_outputs FK1 = Inverse.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                     LateralTranslation, ProbeInsertion,
                                                     ProbeRotation, PitchRotation, YawRotation);

    cout << "X entry Position :" << FK1.zFrameToTreatment(0, 3) << endl;
    cout << "Y entry Position :" << FK1.zFrameToTreatment(1, 3) << endl;
    cout << "Z entry Position :" << FK1.zFrameToTreatment(2, 3) << endl;
    cout << "zFrameToTreatment :\n"
         << FK1.zFrameToTreatment << endl;
    // Axis_Setpoint_Validator();
    return 0;
}

// _yawRotation._setpoint = IK_output.YawRotation * _yawRotation._ticksPerUnit;
// _probeRotation._setpoint = IK_output.ProbeRotation * _probeRotation._ticksPerUnit;
// _pitchRotation._setpoint = IK_output.PitchRotation * _pitchRotation._ticksPerUnit;
// _probeInsertion._setpoint = IK_output.ProbeInsertion * _probeInsertion._ticksPerUnit;
// _lateralTranslation._setpoint = IK_output.LateralTranslation * _lateralTranslation._ticksPerUnit;
// _axialHeadTranslation._setpoint = IK_output.AxialHeadTranslation * _axialHeadTranslation._ticksPerUnit;
// _axialFeetTranslation._setpoint = IK_output.AxialFeetTranslation * _axialFeetTranslation._ticksPerUnit;

// Logger &log = Logger::GetInstance();
// log.Log("Inverse Kinematics -- Yaw Rotation: " + to_string(IK_output.YawRotation * (180 / 3.14)) +
//             " deg | Pitch Rotation: " + to_string(IK_output.PitchRotation * (180 / 3.14)) +
//             " deg | Probe Insertion: " + to_string(IK_output.ProbeInsertion) +
//             " mm | Lateral Translation: " + to_string(IK_output.LateralTranslation) +
//             " mm | Axial Head Translation: " + to_string(IK_output.AxialHeadTranslation) +
//             " mm |  Axial Feet Translation: " + to_string(IK_output.AxialFeetTranslation) + " mm",
//         LOG_LEVEL_INFO, true);

// log.Log("Cannula Length -- " + to_string(_probe._robotToTreatmentAtHome + IK_output.ProbeInsertion - _probe._cannulaToTreatment) + " mm", LOG_LEVEL_INFO, true);

// ===================================================
// This method checks if the a given set of IK outputs are valid
// void NeuroRobot::Axis_Setpoint_Validator()
// {
//     Logger &log = Logger::GetInstance();

//     //------------------------------------------------------------------------
//     // Lateral Validations
//     if (_lateralTranslation._setpoint > _lateralTranslation._maxTicks)
//     {
//         log.Log("Lateral Translation of " + to_string(_lateralTranslation._setpoint / _lateralTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_lateralTranslation._maxTicks / _lateralTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
//     }

//     if (_lateralTranslation._setpoint < _lateralTranslation._minTicks)
//     {
//         log.Log("Lateral Translation of " + to_string(_lateralTranslation._setpoint / _lateralTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_lateralTranslation._minTicks / _lateralTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
//     }

//     //------------------------------------------------------------------------
//     // Axial Distance Validations
//     double axialSeparation = 143 + _axialHeadTranslation._setpoint / _axialHeadTranslation._ticksPerUnit - _axialFeetTranslation._setpoint / _axialFeetTranslation._ticksPerUnit;
//     if (axialSeparation < 75.00)
//     {
//         log.Log("Axial Separation of " + to_string(axialSeparation) + " mm  will exceed the minimum allowed separation of 75 mm", LOG_LEVEL_WARNING, false);
//     }

//     //if( axialSeparation > 245){
//     if (axialSeparation > 146)
//     {
//         log.Log("Axial Separation of " + to_string(axialSeparation) + " mm  will exceed the maximum allowed separation of  146 mm", LOG_LEVEL_WARNING, false);
//     }

//     //------------------------------------------------------------------------
//     // Axial Head Validations
//     if (_axialHeadTranslation._setpoint > _axialHeadTranslation._maxTicks)
//     {
//         log.Log("Axial Head Translation of " + to_string(_axialHeadTranslation._setpoint / _axialHeadTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_axialHeadTranslation._maxTicks / _axialHeadTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
//     }

//     if (_axialHeadTranslation._setpoint < _axialHeadTranslation._minTicks)
//     {
//         log.Log("Axial Head Translation of " + to_string(_axialHeadTranslation._setpoint / _axialHeadTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_axialHeadTranslation._minTicks / _axialHeadTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
//     }

//     //------------------------------------------------------------------------
//     // Axial Feet Validations
//     if (_axialFeetTranslation._setpoint > _axialFeetTranslation._maxTicks)
//     {
//         log.Log("Axial Feet Translation of " + to_string(_axialFeetTranslation._setpoint / _axialFeetTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_axialFeetTranslation._maxTicks / _axialFeetTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
//     }

//     if (_axialFeetTranslation._setpoint < _axialFeetTranslation._minTicks)
//     {
//         log.Log("Axial Feet Translation of " + to_string(_axialFeetTranslation._setpoint / _axialFeetTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_axialFeetTranslation._minTicks / _axialFeetTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
//     }

//     //------------------------------------------------------------------------
//     // Probe Insertion Validations
//     if (_probeInsertion._setpoint > _probeInsertion._maxTicks)
//     {
//         log.Log("Probe Insertion of " + to_string(_probeInsertion._setpoint / _probeInsertion._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_probeInsertion._maxTicks / _probeInsertion._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
//     }

//     if (_probeInsertion._setpoint < _probeInsertion._minTicks)
//     {
//         log.Log("Probe Insertion of " + to_string(_probeInsertion._setpoint / _probeInsertion._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_probeInsertion._minTicks / _probeInsertion._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
//     }

//     //------------------------------------------------------------------------
//     // Yaw Validations
//     if (_yawRotation._setpoint > _yawRotation._maxTicks)
//     {
//         log.Log("Yaw Rotation of " + to_string(_yawRotation._setpoint / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the maximum allowed distance of " + to_string(_yawRotation._maxTicks / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
//     }

//     if (_yawRotation._setpoint < _yawRotation._minTicks)
//     {
//         log.Log("Yaw Rotation of " + to_string(_yawRotation._setpoint / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the minimum allowed distance of " + to_string(_yawRotation._minTicks / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
//     }

//     //------------------------------------------------------------------------
//     // Pitch Validations
//     if (_pitchRotation._setpoint > _pitchRotation._maxTicks)
//     {
//         log.Log("Pitch Rotation of " + to_string(_pitchRotation._setpoint / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the maximum allowed distance of " + to_string(_pitchRotation._maxTicks / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
//     }

//     if (_pitchRotation._setpoint < _pitchRotation._minTicks)
//     {
//         log.Log("Pitch Rotation of " + to_string(_pitchRotation._setpoint / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the minimum allowed distance of " + to_string(_pitchRotation._maxTicks / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
//     }
// }
