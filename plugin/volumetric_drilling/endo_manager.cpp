//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2025

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@jhu.edu>
    \author    Adnan Munawar

    \author    <hishida3@jhu.edu>
    \author    Hisashi Ishida

    \author    <aying2@jhu.edu>
    \author    Andrew Ying
*/
//==============================================================================

#include "endo_manager.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btVector3.h"
#include "afFramework.h"
#include "math/CTransform.h"
#include "math/CVector3d.h"
#include "ros_interface.h"
#include <boost/program_options.hpp>

EndoManager::EndoManager(){
    m_units_mmToSim = 0.001;
}

void EndoManager::cleanup()
{
    for(auto tool : m_toolCursorList)
        {
            tool->stop();
            delete tool;
        }
    m_burrCursor->stop();
    delete m_burrCursor;

    delete m_endoscope;

    delete m_deviceHandler;

    delete m_drillingPub;
}

int EndoManager::init(afWorldPtr a_worldPtr, CameraPanelManager* a_panelManager, p_opt::variables_map& var_map){
    // importing drill model

    int nt = var_map["nt"].as<int>();
    bool mute = var_map["mute"].as<bool>();

    m_mainCamera = a_worldPtr->getCamera("main_camera");
    m_panelManager = a_panelManager;

    if (nt > 0 && nt <= 8){
        m_toolCursorList.resize(nt);
    }
    else{
        cerr << "ERROR! VALID NUMBER OF TOOL CURSORS ARE BETWEEN 1 - 8. Specified value = " << nt << endl;
        return -1;
    }

    afRigidBodyPtr endoscopeRB = a_worldPtr->getRigidBody("Endoscope 35 degree in REMS");

    if (!endoscopeRB){
        cerr << "ERROR! FAILED TO FIND REFERENCE ENDOSCOPE RIGID BODY " << "Endoscope 35 degree in REMS" << endl;
        return -1;
    }

    // consider the endoscope as a dirll with no radius or removal for control purposes
    m_endoscope = new Endo();
    m_endoscope->m_name = "Endoscope 35 degree in REMS";
    m_endoscope->m_rigidBody = endoscopeRB;
    m_endoscope->m_radius = 0.001;
    m_endoscope->setVoxelRemvalThreshold(0);

    m_activeDrill = m_endoscope;

    m_dX = var_map["ds"].as<float>();

    // initializeLabels();


    // Set up voxels_removed publisher
    m_drillingPub = new ToolPublisher(a_worldPtr->getNamespace(), "/plugin/volumetric_drilling/endoscope", "MTML_base");

    string file_path = __FILE__;
    string current_filepath = file_path.substr(0, file_path.rfind("/"));

    // create a haptic device handler
    m_deviceHandler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    m_deviceHandler->getDevice(m_hapticDevice, 0);

    m_toolCursorRadii.push_back(0.001);
    m_toolCursorRadii.push_back(0.00065);
    m_toolCursorRadii.push_back(0.00075);
    m_toolCursorRadii.push_back(0.00085);
    m_toolCursorRadii.push_back(0.00095);
    m_toolCursorRadii.push_back(0.00105);
    m_toolCursorRadii.push_back(0.00115);
    m_toolCursorRadii.push_back(0.00125);

    // Initializing tool cursors
    toolCursorInit(a_worldPtr);

    m_burrCursor = new cToolCursor(a_worldPtr->getChaiWorld());
    a_worldPtr->addSceneObjectToWorld(m_burrCursor);
    m_burrCursor->setHapticDevice(m_hapticDevice);

    m_burrCursor->setWorkspaceRadius(m_units_mmToSim);
    m_burrCursor->setWaitForSmallForce(true);
    m_burrCursor->start();
    m_burrCursor->m_hapticPoint->m_sphereProxy->setShowFrame(false);

    m_burrCursor->m_name = "mastoidectomy_drill";
    m_burrCursor->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
    m_burrCursor->m_hapticPoint->m_sphereProxy->m_material->setBrown();
    m_burrCursor->m_hapticPoint->m_sphereGoal->m_material->setGray();
    // if the haptic device has a gripper, enable it as a user switch
    m_hapticDevice->setEnableGripperUserSwitch(true);
    m_burrCursor->setRadius(0.001);

    m_burrCursor->initialize();


    m_burrCursor->updateFromDevice();
    cTransform T_c_w = m_mainCamera->getLocalTransform();
    cMatrix3d offset(cVector3d(0, 0, 1), -.05);
    
    T_c_w.setLocalRot(T_c_w.getLocalRot() * offset);

    m_T_d_init = m_mainCamera->getLocalTransform() * endoscopeRB->getLocalTransform();
    m_T_i = m_burrCursor->getDeviceLocalTransform();
    m_T_d_init.setLocalRot(T_c_w.getLocalRot() * m_T_i.getLocalRot());
    
    m_T_d = m_T_d_init;

    m_burrCursor->setLocalTransform(m_T_d);
    m_endoscope->m_rigidBody->setLocalTransform(m_T_d);
    toolCursorsPosUpdate(m_activeDrill->m_rigidBody->getLocalTransform());

    m_rbForce.zero();
    m_rbTorque.zero();

    if (m_deviceHandler->getNumDevices() < 2) {
        m_deviceHandler->getDevice(m_hapticDevice, 1);
    }

    return 1;
}

void EndoManager::update(double dt)
{
    m_burrCursor->updateFromDevice();
    cTransform T_c_w = m_mainCamera->getLocalTransform();

    // If a valid haptic device is found, then it should be available
    if (getOverrideControl()){
        m_T_d = m_endoscope->m_rigidBody->getLocalTransform();
    }
    else if(m_hapticDevice->isDeviceAvailable()){
        m_T_i = m_burrCursor->getDeviceLocalTransform();
        cMatrix3d offset(cVector3d(0, 0, 1), -.05);
        

        T_c_w.setLocalRot(T_c_w.getLocalRot() * offset);

        m_V_i = T_c_w.getLocalRot() * m_burrCursor->getDeviceLocalLinVel();
        m_T_d.setLocalPos(m_T_d.getLocalPos() + (m_V_i * !m_deviceClutch * !m_camClutch));
        m_T_d.setLocalRot(T_c_w.getLocalRot() * m_T_i.getLocalRot());

        // set zero forces when manipulating objects
        if (m_deviceClutch || m_camClutch){
            if (m_camClutch){
                m_mainCamera->setView(T_c_w.getLocalPos() + m_V_i * !m_deviceClutch, m_mainCamera->getTargetPosLocal(), m_mainCamera->getUpVector());
            }
            m_burrCursor->setDeviceLocalForce(0.0, 0.0, 0.0);
        }
    }

    // not sure why using device local transform like FIVRS doesn't work
    // m_burrCursor->setDeviceLocalTransform(m_T_d);
    m_burrCursor->setLocalTransform(m_T_d);

    // updates position of drill mesh
    updatePoseFromCursors(dt);
    // m_activeDrill->m_rigidBody->setLocalTransform(m_T_d);

    toolCursorsPosUpdate(m_activeDrill->m_rigidBody->getLocalTransform());

    // check for shaft collision
    checkShaftCollision();

}

void EndoManager::setOverrideControl(bool val){
    m_overrideControl = val;
}

///
/// \brief This method initializes the tool cursors.
/// \param a_afWorld    A world that contains all objects of the virtual environment
/// \return
///
void EndoManager::toolCursorInit(const afWorldPtr a_afWorld){

    for(int i=0; i < m_toolCursorList.size(); i++)
    {
        m_toolCursorList[i] = new cToolCursor(a_afWorld->getChaiWorld());

        a_afWorld->addSceneObjectToWorld(m_toolCursorList[i]);

        if(i == 0)
        {
            m_toolCursorList[i]->m_hapticPoint->setShow(m_showGoalProxySpheres, m_showGoalProxySpheres);
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();
            m_toolCursorList[i]->setRadius(0.001); // Set the correct radius for the tip which is not from the list of cursor radii
        }
        else
        {
            m_toolCursorList[i]->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
            m_toolCursorList[i]->setRadius(m_toolCursorRadii[i]);
        }
     }

    // Initialize the start pose of the tool cursors
    toolCursorsPosUpdate(m_activeDrill->m_rigidBody->getLocalTransform());
    toolCursorsInitialize();
}


///
/// \brief incrementDevicePos
/// \param a_vel
///
void EndoManager::incrementDevicePos(cVector3d a_vel){
    m_T_d.setLocalPos(m_T_d.getLocalPos() + a_vel);
}


///
/// \brief incrementDeviceRot
/// \param a_rot
///
void EndoManager::incrementDeviceRot(cVector3d a_rot){
    cMatrix3d R_cmd;
    R_cmd.setExtrinsicEulerRotationDeg(a_rot(0), a_rot(1), a_rot(2), C_EULER_ORDER_XYZ);
    R_cmd = m_T_d.getLocalRot() * R_cmd;
    m_T_d.setLocalRot(R_cmd);
}

///
/// \brief afVolmetricDrillingPlugin::toolCursorsInitialize
///
void EndoManager::toolCursorsInitialize(){
    for (int i = 0 ;  i < m_toolCursorList.size() ; i++){
        m_toolCursorList[i]->initialize();
    }
}

///
/// \brief This method updates the position of the shaft tool cursors
/// which eventually updates the position of the whole tool.
///
void EndoManager::toolCursorsPosUpdate(cTransform a_targetPose){
    cVector3d n_x = a_targetPose.getLocalRot().getCol0() * m_dX;
    for (int i = 0 ; i < m_toolCursorList.size() ; i++){
        cVector3d P = a_targetPose.getLocalPos() + n_x * i;
        m_toolCursorList[i]->setDeviceLocalPos(P);
        m_toolCursorList[i]->setDeviceLocalRot(a_targetPose.getLocalRot());
    }
}

void EndoManager::reset(){
    m_burrCursor->setDeviceGlobalForce(cVector3d(0., 0., 0.));

    m_T_d = m_T_d_init;

    m_burrCursor->setLocalTransform(m_T_d);
    m_burrCursor->initialize();

    toolCursorsPosUpdate(m_activeDrill->m_rigidBody->getLocalTransform());
    toolCursorsInitialize();

    m_activeDrill->m_rigidBody->setLocalTransform(m_T_d);
}

///
/// \brief This method checks for collision between the tool shaft and the volume.
/// The error between the proxy and goal position of each of the shaft tool cursors is constantly
/// computed. The shaft tool cursor having the maximum error is set as g_targetToolCursor. Further, the
/// position of the drill mesh is set such that it follows the proxy position of the g_targetToolCursor.
/// If there's no collision, the drill mesh follows the proxy position of the shaft tool cursor which is
/// closest to the tip tool cursor.
///
void EndoManager::checkShaftCollision(){

    m_maxError = 0;
    m_targetToolCursor = m_toolCursorList[0];
    m_targetToolCursorIdx = 0;
    for(int i=0; i<m_toolCursorList.size(); i++)
    {

        m_currError = cDistance(m_toolCursorList[i]->m_hapticPoint->getLocalPosProxy(), m_toolCursorList[i]->m_hapticPoint->getLocalPosGoal());

        if(abs(m_currError) > abs(m_maxError + 0.00001))
        {
            m_maxError = m_currError;
            m_targetToolCursor = m_toolCursorList[i];
            m_targetToolCursorIdx = i;
        }
    }
}


///
/// \brief This method updates the position of the drill mesh.
/// After obtaining g_targetToolCursor, the drill mesh adjust it's position and rotation
/// such that it follows the proxy position of the g_targetToolCursor.
///
void EndoManager::updatePoseFromCursors(double dt){
    cMatrix3d newDrillRot = m_T_d.getLocalRot();
    cVector3d a_axis;
    double a_angle;
    newDrillRot.toAxisAngle(a_axis, a_angle);

    // not sure why using the proxy point doesn't work but the transform is wrong
    cVector3d newDrillPos = m_burrCursor->getLocalPos();

    afRigidBodyPtr afRBPtr = m_activeDrill->m_rigidBody;
    btRigidBody* btRBPtr = afRBPtr->m_bulletRigidBody;

    btVector3 cur_pos, cmd_pos;
    btQuaternion cmd_rot_quat = btQuaternion(btVector3(a_axis.x(), a_axis.y(), a_axis.z()), a_angle);

    btMatrix3x3 cur_rot, cmd_rot;
    btTransform b_trans;
    b_trans = afRBPtr->getCOMTransform();

    cur_pos = b_trans.getOrigin();
    cur_rot.setRotation(b_trans.getRotation());
    cmd_pos.setValue(newDrillPos.x(), newDrillPos.y(), newDrillPos.z());
    if( cmd_rot_quat.length() < 0.9 || cmd_rot_quat.length() > 1.1 ){
        cerr << "WARNING: BODY \"" << afRBPtr->getName() << "'s\" rotation quaternion command"
                                                " not normalized" << endl;
        if (cmd_rot_quat.length() < 0.1){
            cmd_rot_quat.setW(1.0); // Invalid Quaternion
        }
    }
    cmd_rot.setRotation(cmd_rot_quat);

    btVector3 pCommand, rCommand;
    // Use the internal Cartesian Position Controller to Compute Output
    pCommand = afRBPtr->m_controller.computeOutput<btVector3>(cur_pos, cmd_pos, dt);
    // Use the internal Cartesian Rotation Controller to Compute Output
    rCommand = afRBPtr->m_controller.computeOutput<btVector3>(cur_rot, cmd_rot, dt);

    if (afRBPtr->m_controller.m_positionOutputType == afControlType::FORCE){
        // IF PID GAINS WERE DEFINED, USE THE PID CONTROLLER
        // Use the internal Cartesian Position Controller
        btRBPtr->applyCentralForce(pCommand);
        btRBPtr->applyTorque(rCommand);
    }
    else{
        // ELSE USE THE VELOCITY INTERFACE
        btRBPtr->setLinearVelocity(pCommand);
        btRBPtr->setAngularVelocity(rCommand);
    }

    double const FC_force = 10.0;
    double const RC_force = 1.0 / (2 * C_PI * FC_force);
    double alpha_force = dt / (RC_force + dt);

    double const FC_torque = 2.5;
    double const RC_torque = 1.0 / (2 * C_PI * FC_torque);
    double alpha_torque = dt / (RC_torque + dt);

    cVector3d force = cTranspose(m_mainCamera->getLocalRot()) * -cVector3d(pCommand.x(), pCommand.y(), pCommand.z());
    cVector3d torque = cTranspose(m_mainCamera->getLocalRot()) * -cVector3d(rCommand.x(), rCommand.y(), rCommand.z());

    cVector3d force_low_pass = alpha_force * force + (1 - alpha_force) * m_rbForce;
    cVector3d torque_low_pass = alpha_torque * torque + (1 - alpha_torque) * m_rbTorque;
    m_rbForce = force_low_pass;
    m_rbTorque = torque_low_pass;
}