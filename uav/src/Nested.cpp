// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   Pid_impl.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a PID
//
//
/*********************************************************************/
#include "Nested.h"
#include <Eigen/Dense>
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

namespace flair {
namespace filter {

Nested::Nested(const LayoutPosition *position, string name): ControlLaw(position->getLayout(), name, 4){

    // init matrix
    input = new Matrix(this, 4, 3, floatType, name);
  
    MatrixDescriptor *desc = new MatrixDescriptor(4, 1);
    desc->SetElementName(0, 0, "u_roll");
    desc->SetElementName(1, 0, "u_pitch");
    desc->SetElementName(2, 0, "u_yaw");
    desc->SetElementName(3, 0, "u_z");
    state = new Matrix(this, desc, floatType, name);
    delete desc;


    GroupBox *reglages_groupbox = new GroupBox(position, name);
    k1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "k1:", -50000, 50000, 0.1, 3);
    k2 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "k2:", -50000, 50000, 0.1, 3);
    k3 = new DoubleSpinBox(reglages_groupbox->NewRow(), "k3:", -50000, 50000, 0.1, 3);
    k4 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "k4:", -50000, 50000, 0.1, 3);
    a1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "a1:", -50000, 50000, 0.1, 3);
    b1 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "b1:", -50000, 50000, 0.1, 3);
    c1 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "c1:", -50000, 50000, 0.1, 3);
    d1 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "d1:", -50000, 50000, 0.1, 3);
    a2 = new DoubleSpinBox(reglages_groupbox->NewRow(), "a2:", -50000, 50000, 0.1, 3);
    b2 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "b2:", -50000, 50000, 0.1, 3);
    c2 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "c2:", -50000, 50000, 0.1, 3);
    d2 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "d2:", -50000, 50000, 0.1, 3);
    sat_r = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat thrust:", 0, 1, 0.1);
    
    km = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "km:", -10, 10, 0.01, 6);
    
    
    //GroupBox *c_fisicas = new GroupBox(position->NewRow(), "Constantes Fisicas");
    
    J11 = new DoubleSpinBox(reglages_groupbox->NewRow(),"J11",-2000,2000,0.001,12);
    J22 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"J22",-2000,2000,0.001,12);
    J33 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"J33",-2000,2000,0.001,12);
    J12 = new DoubleSpinBox(reglages_groupbox->NewRow(),"J12",-2000,2000,0.001,12);
    J13 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"J13",-2000,2000,0.001,12);
    J23 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"J23",-2000,2000,0.001,12);
    
    m = new DoubleSpinBox(reglages_groupbox->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"g",-10,10,0.001,3);
    
    
    J(0,0) = J11->Value();
    J(0,1) = J12->Value();
    J(0,2) = J13->Value();
    J(1,1) = J22->Value();
    J(1,2) = J23->Value();
    J(2,2) = J33->Value();
    
    J(1,0) = J(0,1);
    J(2,0) = J(0,2);
    J(2,1) = J(1,2);
    
    
    Ji = J.inverse();
    
}

Nested::~Nested(void) {}

void Nested::Reset(void) {
//    pimpl_->i = 0;
//    pimpl_->first_update = true;
}

void Nested::SetValues(float z, float zp, float psi, float psip, float x1, float x2, float x3, float x4, float y1, float y2, float y3, float y4) {
  input->SetValue(0, 0, x1);
  input->SetValue(1, 0, x2);
  input->SetValue(2, 0, x3);
  input->SetValue(3, 0, x4);
  
  input->SetValue(0, 1, y1);
  input->SetValue(1, 1, y2);
  input->SetValue(2, 1, y3);
  input->SetValue(3, 1, y4);
  
  input->SetValue(0, 2, z);
  input->SetValue(1, 2, zp);
  input->SetValue(2, 2, psi);
  input->SetValue(3, 2, psip);
}

void Nested::UseDefaultPlot(const LayoutPosition *position) {
    DataPlot1D *rollg = new DataPlot1D(position, "u_roll", -1, 1);
    rollg->AddCurve(state->Element(0));
    
}

void Nested::UseDefaultPlot2(const LayoutPosition *position) {
    DataPlot1D *pitchg = new DataPlot1D(position, "u_pitch", -1, 1);
    pitchg->AddCurve(state->Element(1));
    
}

void Nested::UseDefaultPlot3(const LayoutPosition *position) {
    DataPlot1D *yawg = new DataPlot1D(position, "u_yaw", -1, 1);
    yawg->AddCurve(state->Element(2));
    
}

void Nested::UseDefaultPlot4(const LayoutPosition *position) {    
    DataPlot1D *uz = new DataPlot1D(position, "u_z", -1, 1);
    uz->AddCurve(state->Element(3));
    
}


void Nested::UpdateFrom(const io_data *data) {
    float T;
    Eigen::Vector3d tau(0,0,0);
    const Matrix* input = dynamic_cast<const Matrix*>(data);
  
    if (!input) {
        Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
        return;
    }


    input->GetMutex();
  
    float x = input->ValueNoMutex(0, 0);
    float xp = input->ValueNoMutex(1, 0);
    float th = input->ValueNoMutex(2, 0);
    float thp = input->ValueNoMutex(3, 0);
  
    float y = input->ValueNoMutex(0, 1);
    float yp = input->ValueNoMutex(1, 1);
    float phi = input->ValueNoMutex(2, 1);
    float phip = input->ValueNoMutex(3, 1);
    
    float ze = input->ValueNoMutex(0, 2);
    float zp = input->ValueNoMutex(1, 2);
    float psi = input->ValueNoMutex(2, 2);
    float psip = input->ValueNoMutex(3, 2);
    
    input->ReleaseMutex();
    
    Eigen::Matrix3d W = MatrixW(phi, th);
    Eigen::Matrix3d Wi = MatrixWi(phi, th);
    Eigen::Matrix3d Wp = MatrixWp(phi, th, phip, thp);
    
    
    Eigen::Vector3d w(phip, thp, psip);
    
    Eigen::Vector3d etap = Wi*w;
    
    float tau_th_t = Sat(thp + Sat(th + thp + Sat(2*th + thp - (1/g->Value())*xp + Sat(3*th + thp - (3/g->Value())*xp -(1/g->Value())*x ,d1->Value()) ,c1->Value()) ,b1->Value()) ,a1->Value());
    
    float tau_phi_t = Sat(phip + Sat(phi + phip + Sat(2*phi + phip - (1/g->Value())*yp + Sat(3*phi + phip - (3/g->Value())*yp -(1/g->Value())*y ,d2->Value()) ,c2->Value()) ,b2->Value()) ,a2->Value());;
    
    float tau_psi_t = -k3->Value()*psip - k4->Value()*psi;
    
    Eigen::Vector3d taub(tau_phi_t, tau_th_t, tau_psi_t);
    
    float tau_roll, tau_pitch, tau_yaw, Tr;
    
    
    tau = J*W*(Wi*Wp*etap + Wi*Ji*CPO(W*etap)*J*W*etap + taub);

    
    T = (m->Value()*(k1->Value()*zp + k2->Value()*ze + g->Value()))/(cos(th)*cos(phi));
    
    tau_roll = (float)tau(0)/km->Value();
    
    tau_pitch = (float)tau(1)/km->Value();
    
    tau_yaw = -(float)tau(2)/km->Value();
    
    Tr = (float)T/km->Value();
    
    tau_roll = Sat(tau_roll,sat_r->Value());
    tau_pitch = Sat(tau_pitch,sat_p->Value());
    tau_yaw = Sat(tau_yaw,sat_y->Value());
    Tr = Sat(Tr,sat_t->Value());
    
    state->GetMutex();
    state->SetValueNoMutex(0, 0, tau_roll);
    state->SetValueNoMutex(1, 0, tau_pitch);
    state->SetValueNoMutex(2, 0, tau_yaw);
    state->SetValueNoMutex(3, 0, Tr);
    state->ReleaseMutex();


    output->SetValue(0, 0, tau_roll);
    output->SetValue(1, 0, tau_pitch);
    output->SetValue(2, 0, tau_yaw);
    output->SetValue(3, 0, Tr);
    output->SetDataTime(data->DataTime());
    
    ProcessUpdate(output);
}


Eigen::Matrix3d Nested::MatrixW(float &roll, float &pitch){
    Eigen::Matrix3d W;
    
    W(0,0) = 1;
    W(0,1) = 0;
    W(0,2) = -sin(pitch);
    
    W(1,0) = 0;
    W(1,1) = cos(roll);
    W(1,2) = sin(roll)*cos(pitch);
    
    W(2,0) = 0;
    W(2,1) = -sin(roll);
    W(2,2) = cos(roll)*cos(pitch);
    
    return W;
}

Eigen::Matrix3d Nested::MatrixWi(float &roll, float &pitch){
    Eigen::Matrix3d Wi;
    
    
    Wi(0,0) = 1;
    Wi(0,1) = sin(roll)*tan(pitch);
    Wi(0,2) = tan(pitch)*cos(roll);
    
    Wi(1,0) = 0;
    Wi(1,1) = cos(roll);
    Wi(1,2) = -sin(pitch);
    
    Wi(2,0) = 0;
    Wi(2,1) = sin(roll)/cos(pitch);
    Wi(2,2) = cos(roll)/cos(pitch);
    
    return Wi;
}

Eigen::Matrix3d Nested::MatrixWp(float &roll, float &pitch, float &rollp, float &pitchp){
    Eigen::Matrix3d Wp;
    
    Wp(0,0) = 0;
    Wp(0,1) = 0;
    Wp(0,2) = -pitchp*cos(pitch);
    
    Wp(1,0) = 0;
    Wp(1,1) = -rollp*sin(roll);
    Wp(1,2) = rollp*cos(pitch)*cos(roll) - pitchp*sin(pitch)*sin(roll);
    
    Wp(2,0) = 0;
    Wp(2,1) = -roll*cos(roll);
    Wp(2,2) = -pitchp*cos(roll)*sin(pitch) - rollp*cos(pitch)*sin(roll);
    
    return Wp;
}


Eigen::Matrix3d Nested::CPO(Eigen::Vector3d aux){
    Eigen::Matrix3d S;
    
    S << 0, -aux(2), aux(1),
        aux(2), 0, -aux(0),
        -aux(1), aux(0), 0;
    
    return S;
}

float Nested::Sat(float value, float borne) {
  if (value < -borne)
    return -borne;
  if (value > borne)
    return borne;
  return value;
}


} // end namespace filter
} // end namespace flair
