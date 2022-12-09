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
#include "Nested.h"
#include "Sliding.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <DoubleSpinBox.h>
#include <Label.h>
#include <FrameworkManager.h>
#include <Matrix.h>
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
    control_select->AddItem("Sliding tracking");
    
    l2 = new Label(groupbox->LastRowLastCol(), "Control selec");
    l2->SetText("Control: off");
    
    start_prueba1=new PushButton(groupbox->NewRow(),"start control");
    stop_prueba1=new PushButton(groupbox->NewRow(),"stop control");
    
    
    
    
    u_linear = new Linear(setupLawTab2->At(0, 0), "u_lin");
    u_linear->UseDefaultPlot(graphLawTab2->At(0, 0));
    u_linear->UseDefaultPlot2(graphLawTab2->At(0, 1));
    u_linear->UseDefaultPlot3(graphLawTab2->At(0, 2));
    u_linear->UseDefaultPlot4(graphLawTab2->At(1, 2));
    
    u_nested = new Nested(setupLawTab3->At(0, 0), "u_nes");
    u_nested->UseDefaultPlot(graphLawTab3->At(0, 0));
    u_nested->UseDefaultPlot2(graphLawTab3->At(0, 1));
    u_nested->UseDefaultPlot3(graphLawTab3->At(0, 2));
    u_nested->UseDefaultPlot4(graphLawTab3->At(1, 2));
    
    u_sliding = new Sliding(setupLawTab4->At(0, 0), "u_smc");
    u_sliding->UseDefaultPlot(graphLawTab4->At(0, 0));
    u_sliding->UseDefaultPlot2(graphLawTab4->At(0, 1));
    u_sliding->UseDefaultPlot3(graphLawTab4->At(0, 2));
    u_sliding->UseDefaultPlot4(graphLawTab4->At(1, 2));
    
    GroupBox *seg = new GroupBox(setupLawTab4->At(1, 0), "Tracking");
    l = new Label(seg->NewRow(), "funcion");
    l->SetText("a*sin(b*t)");
    a = new DoubleSpinBox(seg->NewRow(), "a:", 0, 3, 0.01,3);
    b = new DoubleSpinBox(seg->LastRowLastCol(), "b:", 0, 10, 0.01, 3);
    
    customReferenceOrientation= new AhrsData(this,"reference");
    
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    //AddDataToControlLawLog(refAnglesC);
    //first_update = true;
    
    getFrameworkManager()->AddDeviceToLog(u_linear);
    getFrameworkManager()->AddDeviceToLog(u_nested);
    getFrameworkManager()->AddDeviceToLog(u_sliding);
    AddDeviceToControlLawLog(u_linear);
    AddDeviceToControlLawLog(u_nested);
    AddDeviceToControlLawLog(u_sliding);


}

proyecto22::~proyecto22() {
}

//this method is called by UavStateMachine::Run (main loop) when TorqueMode is Custom
void proyecto22::ComputeCustomTorques(Euler &torques) {
    ComputeDefaultTorques(torques);
    thrust = ComputeDefaultThrust();
    switch(control_select->CurrentIndex()) {
        case 0:
            linear_ctrl(torques);
            break;
        
        case 1:
            nested_ctrl(torques);
            break;
            
        case 2:
            sliding_ctrl(torques);
            break;
        
        case 3:
            sliding_track(torques);
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
        break;
    case Event_t::Stopped:
        l2->SetText("Control: off");
        control_select->setEnabled(true);
        //first_update==true;
        behaviourMode=BehaviourMode_t::Default;
        break;
    case Event_t::EnteringFailSafeMode:
        l2->SetText("Control: off");
        control_select->setEnabled(true);
        //first_update==true;
        behaviourMode=BehaviourMode_t::Default;
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
        Startproyecto22();
    }
    //L1
    if(GetTargetController()->IsButtonPressed(6) && (behaviourMode==BehaviourMode_t::control)) {
        Stopproyecto22();
    }
    
}

AhrsData *proyecto22::GetReferenceOrientation(void) {
    float tactual=double(GetTime())/1000000000-u_sliding->t0;
    //Thread::Info("%f\n",tactual);
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);
        
    Euler refAnglesC = refQuaternion.ToEuler();
    
    refAnglesC.yaw = a->Value()*sin(b->Value()*abs(tactual));
    
    refAngularRates.z = a->Value()*b->Value()*cos(b->Value()*abs(tactual));

    customReferenceOrientation->SetQuaternionAndAngularRates(refAnglesC.ToQuaternion(),refAngularRates);

    return customReferenceOrientation;
}

void proyecto22::Startproyecto22(void) {
    control_select->setEnabled(false);
    //ask UavStateMachine to enter in custom torques
    if (SetTorqueMode(TorqueMode_t::Custom) && SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Info("proyecto22: start\n");
        u_linear->Reset();
        u_nested->Reset();
        u_sliding->Reset();
    } else {
        Thread::Warn("proyecto22: could not start\n");
        l2->SetText("Control: err");
        control_select->setEnabled(true);
        return;
    }
    switch(control_select->CurrentIndex()) {
        case 0:
            l2->SetText("Control: Linear");
            Thread::Info("Linear\n");
            break;
        
        case 1:
            l2->SetText("Control: Nested");
            Thread::Info("Nested\n");
            break;
            
        case 2:
            l2->SetText("Control: Sliding");
            Thread::Info("Sliding\n");
            break;
        
        case 3:
            l2->SetText("Control: Sliding Tracking");
            Thread::Info("Sliding tracking\n");
//            if(first_update==true){
//                t0 = double(GetTime())/1000000000;
//                first_update==false;
//            }
            break;
    }

    behaviourMode=BehaviourMode_t::control;
}

void proyecto22::Stopproyecto22(void) {
    control_select->setEnabled(true);
    //just ask to enter fail safe mode
    l2->SetText("Control: off");
    //first_update==true;
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
    //float zpe = zp;
    
    u_linear->SetValues(ze,zp,currentAngles.yaw,currentAngularRates.z,0,0,currentAngles.pitch,currentAngularRates.y,0,0,currentAngles.roll,currentAngularRates.x);
    
    u_linear->Update(GetTime());
    
    //Thread::Info("%f \t %f \t %f\n",currentAngles.yaw,currentAngularSpeed.z, u_linear->Output(2));
    
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
    
    float ze = z - refAltitude;
    
    u_nested->SetValues(ze,zp,currentAngles.yaw,currentAngularRates.z,0,0,currentAngles.pitch,currentAngularRates.y,0,0,currentAngles.roll,currentAngularRates.x);
    
    u_nested->Update(GetTime());
    
    //Thread::Info("%f \t %f \t %f \t %f \n",u_nested->Output(0),u_nested->Output(1),u_nested->Output(2), u_nested->Output(3));
    
    torques.roll = u_nested->Output(0);
    torques.pitch = u_nested->Output(1);
    torques.yaw = u_nested->Output(2);
    thrust = u_nested->Output(3);
    
    

}


void proyecto22::sliding_ctrl(Euler &torques){
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);

    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    
    //Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    float refAltitude, refVerticalVelocity;
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
    
    float z, zp;
    
    AltitudeValues(z,zp);
    
    float ze = z - refAltitude;
    
    Vector3Df we = currentAngularRates - refAngularRates;
    
    u_sliding->SetValues(ze,zp,we.x,we.y,we.z,currentQuaternion.q0,currentQuaternion.q1,currentQuaternion.q2,currentQuaternion.q3,
                            refQuaternion.q0,refQuaternion.q1, refQuaternion.q2,refQuaternion.q3);
                         
    u_sliding->Update(GetTime());
    
    //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    
    torques.roll = u_sliding->Output(0);
    torques.pitch = u_sliding->Output(1);
    torques.yaw = u_sliding->Output(2);
    thrust = u_sliding->Output(3);
    //thrust = ComputeDefaultThrust();
    

}

void proyecto22::sliding_track(Euler &torques){
    const AhrsData *refOrientation = GetReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);

    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    
    //Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    float refAltitude, refVerticalVelocity;
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
    
    float z, zp;
    
    AltitudeValues(z,zp);
    
    float ze = z - refAltitude;
    
    Vector3Df we = currentAngularRates - refAngularRates;
    
    
    
    u_sliding->SetValues(ze,zp,we.x,we.y,we.z,currentQuaternion.q0,currentQuaternion.q1,currentQuaternion.q2,currentQuaternion.q3,
                            refQuaternion.q0,refQuaternion.q1, refQuaternion.q2,refQuaternion.q3);
                         
    u_sliding->Update(GetTime());
    
    //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    
    torques.roll = u_sliding->Output(0);
    torques.pitch = u_sliding->Output(1);
    torques.yaw = u_sliding->Output(2);
    thrust = u_sliding->Output(3);
    //thrust = ComputeDefaultThrust();
    

}