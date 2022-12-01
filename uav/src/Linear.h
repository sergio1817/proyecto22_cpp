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

#ifndef LINEAR_H
#define LINEAR_H

#include <Object.h>
#include <Eigen/Dense>
#include <ControlLaw.h>

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
        class Linear : public ControlLaw {
    
    
public:
    Linear(const flair::gui::LayoutPosition *position, std::string name);
    ~Linear();
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
  void SetValues(float z, float zp, float psi, float psip, float x1, float x2, float x3, float x4, float y1, float y2, float y3, float y4) ;
    
    void UseDefaultPlot(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot2(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot3(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot4(const flair::gui::LayoutPosition *position);

private:
    flair::core::Matrix *state;
    
    Eigen::Matrix3d MatrixW(float &roll, float &pitch);
    Eigen::Matrix3d MatrixWi(float &roll, float &pitch);
    Eigen::Matrix3d MatrixWp(float &roll, float &pitch, float &rollp, float &pitchp);
    
    Eigen::Matrix3d CPO(Eigen::Vector3d aux);

    flair::gui::DoubleSpinBox *k1, *k2, *k3, *k4, *k5, *k6, *k7, *k8, *k9, *k10, *k11, *k12, *sat;
    flair::gui::DoubleSpinBox *J11, *J22, *J33, *J12, *J13, *J23, *J21, *J31, *J32, *m, *g;
    Eigen::Matrix3d J;
    Eigen::Matrix3d Ji;
};
} // end namespace filter
} // end namespace flair

#endif // LINEAR_H
