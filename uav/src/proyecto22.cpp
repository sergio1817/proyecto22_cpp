//  created:    2011/05/01  /   2022/11/20
//  filename:   proyecto22.cpp
//
//  author:     Guillaume Sanahuja / Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Proyecto 2022
//
//
/*********************************************************************/

#include "proyecto22.h"
#include "Linear.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <DoubleSpinBox.h>
#include <FrameworkManager.h>
#include <Matrix.h>
#include <Eigen/Dense>
#include <MatrixDescriptor.h>
#include <cmath>
#include <Tab.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <ComboBox.h>
#include <GroupBox.h>
#include <TabWidget.h>
#include <Tab.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;


proyecto22::proyecto22(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default) {
    Uav* uav=GetUav();
    
    GroupBox *groupbox = new GroupBox(GetButtonsLayout()->NewRow(), "Controles");
    
    
    Tab *lawTab2 = new Tab(getFrameworkManager()->GetTabWidget(), "control laws custom");
    TabWidget *tabWidget2 = new TabWidget(lawTab2->NewRow(), "laws");
    
    setupLawTab2 = new Tab(tabWidget2, "Setup Linear");
    setupLawTab3 = new Tab(tabWidget2, "Setup Nested");
    setupLawTab4 = new Tab(tabWidget2, "Setup Sliding");
    graphLawTab2 = new Tab(tabWidget2, "Graficas Linear");
    graphLawTab3 = new Tab(tabWidget2, "Graficas Nested");
    graphLawTab4 = new Tab(tabWidget2, "Graficas Sliding");
    
    
    control_select=new ComboBox(groupbox->NewRow(),"select control");
    control_select->AddItem("Linear");
    control_select->AddItem("Nested");
    control_select->AddItem("Sliding");
    
    start_prueba1=new PushButton(groupbox->NewRow(),"start control");
    stop_prueba1=new PushButton(groupbox->NewRow(),"stop control");
    
    
    
    
    u_linear = new Linear(setupLawTab2->At(0, 0), "u_lin");
    u_linear->UseDefaultPlot(graphLawTab2->At(0, 0));
    u_linear->UseDefaultPlot2(graphLawTab2->At(0, 1));
    u_linear->UseDefaultPlot3(graphLawTab2->At(0, 2));
    u_linear->UseDefaultPlot4(graphLawTab2->At(1, 2));
    

    AddDeviceToControlLawLog(u_linear);


}

proyecto22::~proyecto22() {
}

//this method is called by UavStateMachine::Run (main loop) when TorqueMode is Custom
void proyecto22::ComputeCustomTorques(Euler &torques) {
    ComputeDefaultTorques(torques);
    switch(control_select->CurrentIndex()) {
        case 0:
            Thread::Info("Linear\n");
            linear_ctrl(torques);
            break;
        
        case 1:
            Thread::Info("Nested\n");
            nested_ctrl(torques);
            break;
            
        case 2:
            Thread::Info("Sliding\n");
            nested_sliding(torques);
            break;
    }
    
}

float proyecto22::ComputeCustomThrust(void) {
    return thrust;
}

void proyecto22::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        control_select->setEnabled(false);
        break;
    case Event_t::Stopped:
        behaviourMode=BehaviourMode_t::Default;
        control_select->setEnabled(true);
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        control_select->setEnabled(true);
        break;
    }
}


void proyecto22::ExtraCheckPushButton(void) {
    if(start_prueba1->Clicked() && (behaviourMode!=BehaviourMode_t::control)) {
        Startproyecto22();
    }

    if(stop_prueba1->Clicked() && (behaviourMode==BehaviourMode_t::control)) {
        Stopproyecto22();
    }
}

void proyecto22::ExtraCheckJoystick(void) {
    //R1
    if(GetTargetController()->IsButtonPressed(9) && (behaviourMode!=BehaviourMode_t::control)) {
        Thread::Info("boton\n");
        Startproyecto22();
    }
    //L1
    if(GetTargetController()->IsButtonPressed(6) && (behaviourMode==BehaviourMode_t::control)) {
        Stopproyecto22();
    }
    
}

void proyecto22::Startproyecto22(void) {
    control_select->setEnabled(false);
    Thread::Info("fnc\n");
    //ask UavStateMachine to enter in custom torques
    if (SetTorqueMode(TorqueMode_t::Custom) && SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Info("proyecto22: start\n");
        u_linear->Reset();
    } else {
        Thread::Warn("proyecto22: could not start\n");
        return;
    }

    behaviourMode=BehaviourMode_t::control;
}

void proyecto22::Stopproyecto22(void) {
    control_select->setEnabled(true);
    //just ask to enter fail safe mode
    SetTorqueMode(TorqueMode_t::Default);
    SetThrustMode(ThrustMode_t::Default);
    behaviourMode=BehaviourMode_t::Default;
    EnterFailSafeMode();
}

void proyecto22::linear_ctrl(Euler &torques){
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);
        
    Euler refAngles = refQuaternion.ToEuler();

    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    
    Euler currentAngles = currentQuaternion.ToEuler();
    
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    float refAltitude, refVerticalVelocity;
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
    
    float z, zp;
    
    AltitudeValues(z,zp);
    
    float ze = z - refAltitude;
    
    u_linear->SetValues(ze,zp,currentAngles.yaw,currentAngularSpeed.z,0,0,currentAngles.pitch,currentAngularSpeed.y,0,0,currentAngles.roll,currentAngularSpeed.x);
    
    
    u_linear->Update(GetTime());
    
    Thread::Info("%f \t %f \t %f \t %f \n",u_linear->Output(0),u_linear->Output(1),u_linear->Output(2), u_linear->Output(3));
    
    torques.roll = u_linear->Output(0);
    torques.pitch = u_linear->Output(1);
    torques.yaw = u_linear->Output(2);
    thrust = u_linear->Output(3);
    
}

void proyecto22::nested_ctrl(Euler &torques){
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);
        
    Euler refAngles = refQuaternion.ToEuler();

    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    
    Euler currentAngles = currentQuaternion.ToEuler();
    
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    float refAltitude, refVerticalVelocity;
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
    
    float z, zp;
    
    AltitudeValues(z,zp);
    
    
    

}


void proyecto22::nested_sliding(Euler &torques){
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);
        
    Euler refAngles = refQuaternion.ToEuler();

    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    
    Euler currentAngles = currentQuaternion.ToEuler();
    
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    float refAltitude, refVerticalVelocity;
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
    
    float z, zp;
    
    AltitudeValues(z,zp);
    
    
    

}