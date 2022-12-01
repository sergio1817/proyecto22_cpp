//  created:    2011/05/01
//  filename:   proyecto22.h
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#ifndef PROYECTO22_H
#define PROYECTO22_H

#include <UavStateMachine.h>
#include "Linear.h"

namespace flair {
    namespace gui {
        class PushButton;
        class ComboBox;
        class Tab;
        class TabWidget;
        class DoubleSpinBox;
    }
    namespace filter {
        class ControlLaw;
        class TrajectoryGenerator1D;
        class Linear;
    }
    namespace sensor {
        class TargetController;
    }
}

class proyecto22 : public flair::meta::UavStateMachine {
    public:
        proyecto22(flair::sensor::TargetController *controller);
        ~proyecto22();

    private:

	enum class BehaviourMode_t {
            Default,
            control
        }clTabCtrl;

        BehaviourMode_t behaviourMode;

        void ExtraCheckPushButton(void);
        void ExtraCheckJoystick(void);
        void SignalEvent(Event_t event);
        void Startproyecto22(void);
        void Stopproyecto22(void);
        void ComputeCustomTorques(flair::core::Euler &torques);
        float ComputeCustomThrust(void);
        void linear_ctrl(flair::core::Euler &torques);
        void nested_ctrl(flair::core::Euler &torques);
        void nested_sliding(flair::core::Euler &torques);

        flair::filter::Linear *u_linear;

        float thrust;

        flair::gui::PushButton *start_prueba1,*stop_prueba1;
        flair::gui::ComboBox *control_select;   
        flair::gui::Tab *setupLawTab2, *graphLawTab2, *lawTab2, *setupLawTab3, *graphLawTab3, *setupLawTab4, *graphLawTab4;
        flair::gui::TabWidget *tabWidget2;
};

#endif // PROYECTO22_H
