// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file Linear_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef SLIDING_H
#define SLIDING_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
    }
}

/*! \class Linear
* \brief Class defining a Linear
*/

    
    
namespace flair {
    namespace filter {
    /*! \class Linear
    *
    * \brief Class defining a Linear
    */
        class Sliding : public ControlLaw {
    
    
public:
    Sliding(const flair::gui::LayoutPosition *position, std::string name);
    ~Sliding();
    void UpdateFrom(const flair::core::io_data *data);
    void Reset(void);
    
    /*!
  * \brief Set input values
  *
  * \param x1 x
  * \param x2 xp
  * \param x3 th
  * \param x4 thp
  * \param y1 y
  * \param y2 yp
  * \param y3 phi
  * \param y4 phip
  */
    void SetValues(float ze, float zp, float wex, float wey, float wez, float q0, float q1, float q2, float q3, float qd0, float qd1, float qd2, float qd3);
    
    void UseDefaultPlot(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot2(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot3(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot4(const flair::gui::LayoutPosition *position);
    
    float t0;

private:
    flair::core::Matrix *state;

    flair::gui::DoubleSpinBox *T, *k1, *k2, *gamma, *alpha, *k, *Kd, *sat_r, *sat_p, *sat_y, *sat_t, *m, *g, *km, *p;
    
    float Sat(float value, float borne);
    
    float delta_t;
    
    bool first_update;
    
    flair::core::Vector3Df sgnp, sgn;
    
    
};
} // end namespace filter
} // end namespace flair

#endif // LINEAR_H
