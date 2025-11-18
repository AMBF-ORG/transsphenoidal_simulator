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
*/
//==============================================================================
#ifndef TOOL_MANAGER_H
#define TOOL_MANAGER_H

#include <afFramework.h>
#include "camera_panel_manager.h"
#include "math/CVector3d.h"
#include "ros_interface.h"
#include "common.h"
#include "tools/CToolCursor.h"
#include <boost/circular_buffer.hpp>

struct Tool{
public:
    afRigidBodyPtr m_rigidBody;
    string m_name;
};

class ToolManager{
public:

    ToolManager();

    void cleanup();

    int init(afWorldPtr a_worldPtr, CameraPanelManager* a_panelManager, p_opt::variables_map& opts);

    void update(double dt);

    void initializeLabels();

    // Initialize tool cursors
    void toolCursorInit(const afWorldPtr);

    void incrementDevicePos(cVector3d a_pos);

    void incrementDeviceRot(cVector3d a_rot);

    void toolCursorsInitialize();

    // update position of shaft tool cursors
    void toolCursorsPosUpdate(cTransform a_devicePose);

    // check for shaft collision
    void checkShaftCollision(void);

    bool getOverrideControl(){return m_overrideControl;}

    void setOverrideControl(bool val);

    // update position of tool mesh
    void updatePoseFromCursors(double dt);

    void reset();

    // a haptic device handler
    cHapticDeviceHandler* m_deviceHandler;

    // a pointer to the current haptic device
    cGenericHapticDevicePtr m_hapticDevice;

    DrillingPublisher* m_drillingPub;

    std::vector<Tool*> m_tools;

    Tool* m_activeTool = nullptr;

    bool m_overrideControl = false;

    afRigidBodyPtr m_toolReferenceBody;

    // A map of drill burr names and their sizes in simulation
    double m_units_mmToSim;

    bool m_show = true;

    cTransform m_T_d, m_T_d_init; // Drills target pose
    cTransform m_T_d_hold; // Hold drill target pose when controlling endoscope

    cTransform m_T_i; // Input device transform

    cVector3d m_V_i; // Input device linear velocity

    // rigid (and soft body) force and torque using error
    // circular buffer for smoothing
    boost::circular_buffer<cVector3d> m_rbForce;
    boost::circular_buffer<cVector3d> m_rbTorque;

    // current and maximum distance between proxy and goal spheres
    double m_currError = 0;

    double m_maxError = 0;

    bool m_camClutch = false;

    bool m_deviceClutch = false;

    cLabel* m_sizeLabel;

    cLabel* m_controlModeLabel;

    cAudioSource* m_audioSource = nullptr;

    cAudioBuffer* m_audioBuffer = nullptr;

    cAudioDevice* m_audioDevice = nullptr;

    bool m_isOn;

    // toggles whether the drill mesh should move slowly towards the followSphere
    // or make a sudden jump
    bool m_suddenJump = true;

    // index of current drill size
    int m_activeDrillIdx = 0;

    bool m_showGoalProxySpheres = false;

    // list of tool cursors
    vector<cToolCursor*> m_toolCursorList;

    // radius of tool cursors
    vector<double> m_toolCursorRadii;

    // Local offset between shaft tool cursors
    double m_dX;

    // for storing index of follow sphere
    int m_targetToolCursorIdx = 0;

    cToolCursor* m_targetToolCursor;

    // cursor for the active drill to follow
    cToolCursor* m_tipCursor;

    afCameraPtr m_mainCamera;

    CameraPanelManager* m_panelManager;
};

#endif
